/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_Corridor_config.h"

#if AP_CORRIDOR_ENABLED

#include "AP_Corridor.h"

#include <AP_HAL/AP_HAL.h>

AP_Corridor *AP_Corridor::_singleton;

AP_Corridor::AP_Corridor()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Corridor must be singleton");
    }
    _singleton = this;
}

bool AP_Corridor::set_graph(const Item *items, uint16_t count)
{
    uint16_t new_num_nodes = 0;
    for (uint16_t i=0; i<count; i++) {
        if (!items[i].is_edge) {
            new_num_nodes++;
        }
    }
    const uint16_t new_num_edges = count - new_num_nodes;

    Node *new_nodes = nullptr;
    Edge *new_edges = nullptr;
    uint16_t *new_first_edge = NEW_NOTHROW uint16_t[new_num_nodes+1];
    if (new_first_edge == nullptr) {
        return false;
    }
    if (new_num_nodes != 0) {
        new_nodes = NEW_NOTHROW Node[new_num_nodes];
        if (new_nodes == nullptr) {
            delete[] new_first_edge;
            return false;
        }
    }
    if (new_num_edges != 0) {
        new_edges = NEW_NOTHROW Edge[new_num_edges];
        if (new_edges == nullptr) {
            delete[] new_first_edge;
            delete[] new_nodes;
            return false;
        }
    }

    uint16_t n = 0;
    uint16_t e = 0;
    for (uint16_t i=0; i<count; i++) {
        if (items[i].is_edge) {
            new_edges[e++] = items[i].edge;
        } else {
            new_nodes[n++] = items[i].node;
        }
    }

    // edges are sorted by source node:
    uint16_t edge = 0;
    for (uint16_t node=0; node<=new_num_nodes; node++) {
        while (edge < new_num_edges && new_edges[edge].from < node) {
            edge++;
        }
        new_first_edge[node] = edge;
    }

    clear();
    _nodes = new_nodes;
    _edges = new_edges;
    _first_edge = new_first_edge;
    _num_nodes = new_num_nodes;
    _num_edges = new_num_edges;

    return true;
}

void AP_Corridor::clear()
{
    delete[] _nodes;
    delete[] _edges;
    delete[] _first_edge;
    _nodes = nullptr;
    _edges = nullptr;
    _first_edge = nullptr;
    _num_nodes = 0;
    _num_edges = 0;
}

bool AP_Corridor::get_node(uint16_t index, Node &node) const
{
    if (index >= _num_nodes) {
        return false;
    }
    node = _nodes[index];
    return true;
}

bool AP_Corridor::get_edge(uint16_t index, Edge &edge) const
{
    if (index >= _num_edges) {
        return false;
    }
    edge = _edges[index];
    return true;
}

bool AP_Corridor::get_item(uint16_t seq, Item &item) const
{
    if (seq < _num_nodes) {
        item.is_edge = false;
        item.node = _nodes[seq];
        return true;
    }
    item.is_edge = true;
    return get_edge(seq - _num_nodes, item.edge);
}

float AP_Corridor::edge_weight(const Edge &edge) const
{
    if (is_positive(edge.cost)) {
        return edge.cost;
    }
    const Node &from = _nodes[edge.from];
    const Node &to = _nodes[edge.to];
    const float horizontal = from.location().get_distance(to.location());
    const float vertical = (to.alt_cm - from.alt_cm) * 0.01f;
    return norm(horizontal, vertical);
}

// the point of segment a-b closest to the origin
static Vector3f closest_point_to_origin(const Vector3f &a, const Vector3f &b)
{
    const Vector3f ab = b - a;
    const float length_sq = ab.length_squared();
    if (!is_positive(length_sq)) {
        return a;
    }
    const float t = constrain_float(-(a * ab) / length_sq, 0, 1);
    return a + ab * t;
}

Vector3f AP_Corridor::node_offset_NEU(const Location &loc, uint16_t node) const
{
    const Node &n = _nodes[node];
    const Vector2f ne = loc.get_distance_NE(n.location());
    float loc_alt_cm = 0;
    Location loc_abs = loc;
    if (loc_abs.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
        loc_alt_cm = loc_abs.alt;
    }
    return Vector3f{ne.x, ne.y, (n.alt_cm - loc_alt_cm) * 0.01f};
}

bool AP_Corridor::plan_route(const Location &loc, Route &route) const
{
    route = {};
    if (_num_nodes == 0) {
        return false;
    }

    // find how far the graph is from loc: the closest point of any
    // edge, or any node (which may have no edges)
    float entry_distance = FLT_MAX;
    for (uint16_t i=0; i<_num_nodes; i++) {
        entry_distance = MIN(entry_distance, node_offset_NEU(loc, i).length());
    }
    for (uint16_t i=0; i<_num_edges; i++) {
        const Vector3f a = node_offset_NEU(loc, _edges[i].from);
        const Vector3f b = node_offset_NEU(loc, _edges[i].to);
        entry_distance = MIN(entry_distance, closest_point_to_origin(a, b).length());
    }

    // every way of joining the graph about that close is a starting
    // point for the search.  A node can be started from directly; an
    // edge is joined part way along, costing the rest of the edge to
    // reach its destination node.  A corridor flown both ways is two
    // edges, so this lets the search choose the direction.
    const float entry_tolerance_m = 1.0;
    float *dist = NEW_NOTHROW float[_num_nodes];
    uint16_t *prev = NEW_NOTHROW uint16_t[_num_nodes];
    bool *done = NEW_NOTHROW bool[_num_nodes];
    // the edge the route joins the graph on to reach each node, or
    // UINT16_MAX where it starts at a node
    uint16_t *entry_edge = NEW_NOTHROW uint16_t[_num_nodes];
    if (dist == nullptr || prev == nullptr || done == nullptr || entry_edge == nullptr) {
        delete[] dist;
        delete[] prev;
        delete[] done;
        delete[] entry_edge;
        return false;
    }
    for (uint16_t i=0; i<_num_nodes; i++) {
        dist[i] = FLT_MAX;
        prev[i] = UINT16_MAX;
        entry_edge[i] = UINT16_MAX;
    }
    for (uint16_t i=0; i<_num_nodes; i++) {
        if (node_offset_NEU(loc, i).length() <= entry_distance + entry_tolerance_m) {
            dist[i] = 0;
        }
    }
    for (uint16_t i=0; i<_num_edges; i++) {
        const Edge &e = _edges[i];
        const Vector3f a = node_offset_NEU(loc, e.from);
        const Vector3f b = node_offset_NEU(loc, e.to);
        const Vector3f p = closest_point_to_origin(a, b);
        if (p.length() > entry_distance + entry_tolerance_m) {
            continue;
        }
        const float length = (b - a).length();
        const float remaining = is_positive(length) ? (b - p).length() / length : 0;
        const float d = edge_weight(e) * remaining;
        if (d < dist[e.to]) {
            dist[e.to] = d;
            entry_edge[e.to] = i;
        }
    }

    // Dijkstra
    while (true) {
        uint16_t u = UINT16_MAX;
        for (uint16_t i=0; i<_num_nodes; i++) {
            if (!done[i] && dist[i] < FLT_MAX && (u == UINT16_MAX || dist[i] < dist[u])) {
                u = i;
            }
        }
        if (u == UINT16_MAX) {
            break;
        }
        done[u] = true;
        for (uint16_t i=_first_edge[u]; i<_first_edge[u+1]; i++) {
            const Edge &e = _edges[i];
            const float d = dist[u] + edge_weight(e);
            if (d < dist[e.to]) {
                dist[e.to] = d;
                prev[e.to] = u;
                entry_edge[e.to] = entry_edge[u];
            }
        }
    }

    // choose the landing site: the cheapest nest, else the cheapest
    // divert site, else the cheapest ditch site
    uint16_t site = UINT16_MAX;
    for (const NodeType type : { NodeType::NEST, NodeType::DIVERT_SITE, NodeType::DITCH_SITE }) {
        for (uint16_t i=0; i<_num_nodes; i++) {
            if (_nodes[i].type != type || !(dist[i] < FLT_MAX)) {
                continue;
            }
            if (site == UINT16_MAX || dist[i] < dist[site]) {
                site = i;
            }
        }
        if (site != UINT16_MAX) {
            break;
        }
    }

    bool ret = false;
    if (site != UINT16_MAX) {
        // count the nodes on the path, back from the site
        uint16_t count = 0;
        uint16_t first = site;
        for (uint16_t n=site; n!=UINT16_MAX; n=prev[n]) {
            count++;
            first = n;
        }
        // the route joins at a point part way along an edge unless
        // that point is the edge's destination
        Location entry;
        bool have_entry = false;
        const uint16_t join_edge = entry_edge[first];
        if (join_edge != UINT16_MAX) {
            const Edge &e = _edges[join_edge];
            const Vector3f a = node_offset_NEU(loc, e.from);
            const Vector3f b = node_offset_NEU(loc, e.to);
            const Vector3f p = closest_point_to_origin(a, b);
            if ((b - p).length() > entry_tolerance_m) {
                entry = loc;
                if (entry.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
                    entry.offset(p.x, p.y);
                    entry.alt += p.z * 100;
                    have_entry = true;
                }
            }
        }

        const uint16_t num_points = count + (have_entry ? 1 : 0);
        route.points = NEW_NOTHROW Location[num_points];
        if (route.points != nullptr) {
            uint16_t i = num_points;
            for (uint16_t n=site; n!=UINT16_MAX; n=prev[n]) {
                route.points[--i] = _nodes[n].location();
            }
            if (have_entry) {
                route.points[0] = entry;
            }
            route.num_points = num_points;
            route.site = site;
            route.site_type = _nodes[site].type;
            route.cost = dist[site];
            route.entry_distance_m = entry_distance;
            ret = true;
        }
    }

    delete[] dist;
    delete[] prev;
    delete[] done;
    delete[] entry_edge;
    return ret;
}

namespace AP {

AP_Corridor *corridor()
{
    return AP_Corridor::get_singleton();
}

};

#endif  // AP_CORRIDOR_ENABLED

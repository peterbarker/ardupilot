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
    if (new_num_nodes != 0) {
        new_nodes = NEW_NOTHROW Node[new_num_nodes];
        if (new_nodes == nullptr) {
            return false;
        }
    }
    if (new_num_edges != 0) {
        new_edges = NEW_NOTHROW Edge[new_num_edges];
        if (new_edges == nullptr) {
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

    clear();
    _nodes = new_nodes;
    _edges = new_edges;
    _num_nodes = new_num_nodes;
    _num_edges = new_num_edges;

    return true;
}

void AP_Corridor::clear()
{
    delete[] _nodes;
    delete[] _edges;
    _nodes = nullptr;
    _edges = nullptr;
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

namespace AP {

AP_Corridor *corridor()
{
    return AP_Corridor::get_singleton();
}

};

#endif  // AP_CORRIDOR_ENABLED

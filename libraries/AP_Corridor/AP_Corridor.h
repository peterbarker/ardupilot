/*
  AP_Corridor - storage for a flight corridor graph

  A corridor graph is a set of nodes (3D points, some of which are
  landing sites) connected by directed edges (approved flight
  segments).  Each node is a sphere of a given radius around its
  location and each edge is a cylinder of a given radius around the
  segment between its two nodes.

  The graph is transferred using the mission protocol with
  MAV_MISSION_TYPE_CORRIDOR; see MissionItemProtocol_Corridor.

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
#pragma once

#include "AP_Corridor_config.h"

#if AP_CORRIDOR_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>

class AP_Corridor
{
public:
    AP_Corridor();

    CLASS_NO_COPY(AP_Corridor);

    static AP_Corridor *get_singleton() { return _singleton; }

    // values match MAV_CORRIDOR_NODE_TYPE
    enum class NodeType : uint8_t {
        WAYPOINT = 0,
        NEST = 1,
        DIVERT_SITE = 2,
        DITCH_SITE = 3,
    };

    struct Node {
        int32_t lat;        // degE7
        int32_t lng;        // degE7
        int32_t alt_cm;     // AMSL
        float radius_m;     // sphere radius, 0 if unspecified
        NodeType type;

        Location location() const {
            return Location{lat, lng, alt_cm, Location::AltFrame::ABSOLUTE};
        }
    };

    struct Edge {
        uint16_t from;      // node index
        uint16_t to;        // node index
        float cost;         // 0 means use the distance between the nodes
        float radius_m;     // cylinder radius, 0 if unspecified
    };

    // one item of a graph in upload order; nodes precede edges
    struct Item {
        bool is_edge;
        union {
            Node node;
            Edge edge;
        };
    };

    // replace the stored graph with a copy of items, which must
    // already have been validated.  Returns false (leaving the
    // stored graph unchanged) if memory could not be allocated
    bool set_graph(const Item *items, uint16_t count) WARN_IF_UNUSED;

    // remove the stored graph
    void clear();

    uint16_t num_nodes() const { return _num_nodes; }
    uint16_t num_edges() const { return _num_edges; }
    uint16_t num_items() const { return _num_nodes + _num_edges; }
    uint16_t max_items() const { return AP_CORRIDOR_MAX_ITEMS; }

    bool get_node(uint16_t index, Node &node) const WARN_IF_UNUSED;
    bool get_edge(uint16_t index, Edge &edge) const WARN_IF_UNUSED;

    // get an item by its upload sequence number
    bool get_item(uint16_t seq, Item &item) const WARN_IF_UNUSED;

    // weight of an edge for routing: its cost if it has one, otherwise
    // the 3D distance between its nodes
    float edge_weight(const Edge &edge) const;

private:
    static AP_Corridor *_singleton;

    Node *_nodes;
    Edge *_edges;
    uint16_t _num_nodes;
    uint16_t _num_edges;
};

namespace AP {
    AP_Corridor *corridor();
};

#endif  // AP_CORRIDOR_ENABLED

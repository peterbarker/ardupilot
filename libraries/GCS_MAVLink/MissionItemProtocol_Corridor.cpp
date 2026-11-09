/*
  Implementation details for transferring a flight corridor graph
  using the MISSION_ITEM protocol to and from a GCS.

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

#include "GCS_config.h"
#include <AP_Corridor/AP_Corridor_config.h>

#if HAL_GCS_ENABLED && AP_CORRIDOR_ENABLED

#include "MissionItemProtocol_Corridor.h"

#include <AC_Fence/AC_Fence.h>
#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>

// true if f is a whole number which fits in a uint16_t
static bool is_uint16_index(float f)
{
    return f >= 0 && f <= UINT16_MAX && is_equal(f, floorf(f));
}

static bool is_valid_radius(float f)
{
    return isfinite(f) && f >= 0;
}

MAV_MISSION_RESULT MissionItemProtocol_Corridor::convert_MISSION_ITEM_INT_to_Item(const mavlink_mission_item_int_t &cmd, AP_Corridor::Item &ret)
{
    ret = {};

    switch (cmd.command) {
    case MAV_CMD_NAV_CORRIDOR_NODE: {
        if (cmd.frame != MAV_FRAME_GLOBAL) {
            return MAV_MISSION_UNSUPPORTED_FRAME;
        }
        if (!is_uint16_index(cmd.param1) ||
            cmd.param1 > float(AP_Corridor::NodeType::DITCH_SITE)) {
            return MAV_MISSION_INVALID_PARAM1;
        }
        if (!is_valid_radius(cmd.param2)) {
            return MAV_MISSION_INVALID_PARAM2;
        }
        if (!is_zero(cmd.param3)) {
            return MAV_MISSION_INVALID_PARAM3;
        }
        if (!is_zero(cmd.param4)) {
            return MAV_MISSION_INVALID_PARAM4;
        }
        if (!check_lat(cmd.x)) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (!check_lng(cmd.y)) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        // altitude is stored in centimetres in an int32_t
        if (!isfinite(cmd.z) || fabsf(cmd.z) > 1.0e6f) {
            return MAV_MISSION_INVALID_PARAM7;
        }
        ret.is_edge = false;
        ret.node.type = AP_Corridor::NodeType(uint8_t(cmd.param1));
        ret.node.radius_m = cmd.param2;
        ret.node.lat = cmd.x;
        ret.node.lng = cmd.y;
        ret.node.alt_cm = lroundf(cmd.z * 100);
        return MAV_MISSION_ACCEPTED;
    }

    case MAV_CMD_NAV_CORRIDOR_EDGE:
        // edges have no location, so the frame is not checked
        if (!is_uint16_index(cmd.param1)) {
            return MAV_MISSION_INVALID_PARAM1;
        }
        if (!is_uint16_index(cmd.param2)) {
            return MAV_MISSION_INVALID_PARAM2;
        }
        if (!isfinite(cmd.param3) || cmd.param3 < 0) {
            return MAV_MISSION_INVALID_PARAM3;
        }
        if (!is_valid_radius(cmd.param4)) {
            return MAV_MISSION_INVALID_PARAM4;
        }
        if (cmd.x != 0) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (cmd.y != 0) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        if (!is_zero(cmd.z)) {
            return MAV_MISSION_INVALID_PARAM7;
        }
        ret.is_edge = true;
        ret.edge.from = uint16_t(cmd.param1);
        ret.edge.to = uint16_t(cmd.param2);
        ret.edge.cost = cmd.param3;
        ret.edge.radius_m = cmd.param4;
        return MAV_MISSION_ACCEPTED;

    default:
        return MAV_MISSION_UNSUPPORTED;
    }
}

void MissionItemProtocol_Corridor::convert_Item_to_MISSION_ITEM_INT(const AP_Corridor::Item &item, uint16_t seq, mavlink_mission_item_int_t &ret)
{
    if (item.is_edge) {
        ret = {
            param1: float(item.edge.from),
            param2: float(item.edge.to),
            param3: item.edge.cost,
            param4: item.edge.radius_m,
            seq: seq,
            command: MAV_CMD_NAV_CORRIDOR_EDGE,
            frame: MAV_FRAME_MISSION,
            mission_type: MAV_MISSION_TYPE_CORRIDOR,
        };
        return;
    }
    ret = {
        param1: float(uint8_t(item.node.type)),
        param2: item.node.radius_m,
        x: item.node.lat,
        y: item.node.lng,
        z: item.node.alt_cm * 0.01f,
        seq: seq,
        command: MAV_CMD_NAV_CORRIDOR_NODE,
        frame: MAV_FRAME_GLOBAL,
        mission_type: MAV_MISSION_TYPE_CORRIDOR,
    };
}

/*
  plan structure checks.  Items must be supplied in upload order.
 */
MAV_MISSION_RESULT MissionItemProtocol_Corridor::Validator::check(const AP_Corridor::Item &item)
{
    if (!item.is_edge) {
        if (num_edges != 0) {
            // all nodes must precede all edges
            return MAV_MISSION_INVALID_SEQUENCE;
        }
        if (item.node.type == AP_Corridor::NodeType::NEST ||
            item.node.type == AP_Corridor::NodeType::DIVERT_SITE) {
            have_landing_site = true;
        }
        num_nodes++;
        return MAV_MISSION_ACCEPTED;
    }

    const AP_Corridor::Edge &edge = item.edge;
    if (edge.from >= num_nodes) {
        return MAV_MISSION_INVALID_PARAM1;
    }
    if (edge.to >= num_nodes || edge.to == edge.from) {
        return MAV_MISSION_INVALID_PARAM2;
    }
    if (num_edges != 0 && edge.from < last_from) {
        // edges must be sorted by source node
        return MAV_MISSION_INVALID_SEQUENCE;
    }
    // either all edges carry a cost or none of them do:
    const bool edge_has_cost = is_positive(edge.cost);
    if (num_edges == 0) {
        have_cost = edge_has_cost;
    } else if (edge_has_cost != have_cost) {
        return MAV_MISSION_INVALID_PARAM3;
    }
    last_from = edge.from;
    num_edges++;
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Corridor::Validator::finish() const
{
    if (!have_landing_site) {
        return MAV_MISSION_INVALID;
    }
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Corridor::validate_new_items(const GCS_MAVLINK &_link) const
{
    Validator validator {};
    for (uint16_t i=0; i<_new_items_count; i++) {
        const MAV_MISSION_RESULT ret = validator.check(_new_items[i]);
        if (ret != MAV_MISSION_ACCEPTED) {
            _link.send_text(MAV_SEVERITY_WARNING, "Corridor: item %u invalid", i);
            return ret;
        }
    }
    const MAV_MISSION_RESULT ret = validator.finish();
    if (ret != MAV_MISSION_ACCEPTED) {
        _link.send_text(MAV_SEVERITY_WARNING, "Corridor: no nest or divert site");
        return ret;
    }

#if AP_FENCE_ENABLED
    // every node must be inside the inclusion fences and outside the
    // exclusion fences (horizontally):
    const AC_Fence *fence = AP::fence();
    if (fence != nullptr && fence->polyfence().loaded()) {
        for (uint16_t i=0; i<_new_items_count; i++) {
            const AP_Corridor::Item &item = _new_items[i];
            if (item.is_edge) {
                break;
            }
            if (fence->polyfence().breached(item.node.location())) {
                _link.send_text(MAV_SEVERITY_WARNING, "Corridor: node %u outside fence", i);
                return MAV_MISSION_INVALID;
            }
        }
    }
#endif

    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Corridor::get_item(uint16_t seq, mavlink_mission_item_int_t &ret_packet)
{
    AP_Corridor::Item item;
    if (!corridor.get_item(seq, item)) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }
    convert_Item_to_MISSION_ITEM_INT(item, seq, ret_packet);
    return MAV_MISSION_ACCEPTED;
}

uint16_t MissionItemProtocol_Corridor::item_count() const
{
    if (receiving) {
        return _new_items_count;
    }
    return corridor.num_items();
}

uint16_t MissionItemProtocol_Corridor::max_items() const
{
    return corridor.max_items();
}

MAV_MISSION_RESULT MissionItemProtocol_Corridor::replace_item(const mavlink_mission_item_int_t &cmd)
{
    if (_new_items == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return MAV_MISSION_ERROR;
    }
    if (cmd.seq >= _new_items_count) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }

    AP_Corridor::Item item;
    MAV_MISSION_RESULT ret = convert_MISSION_ITEM_INT_to_Item(cmd, item);
    if (ret != MAV_MISSION_ACCEPTED) {
        return ret;
    }

    // items arriving in order are checked against those before them
    // straight away; the whole plan is checked again on completion
    if (cmd.seq == _upload_validator.next_index()) {
        ret = _upload_validator.check(item);
        if (ret != MAV_MISSION_ACCEPTED) {
            return ret;
        }
    }

    _new_items[cmd.seq] = item;
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Corridor::append_item(const mavlink_mission_item_int_t &cmd)
{
    return replace_item(cmd);
}

MAV_MISSION_RESULT MissionItemProtocol_Corridor::complete(const GCS_MAVLINK &_link)
{
    if (_new_items_count == 0) {
        corridor.clear();
        return MAV_MISSION_ACCEPTED;
    }

    const MAV_MISSION_RESULT ret = validate_new_items(_link);
    if (ret != MAV_MISSION_ACCEPTED) {
        return ret;
    }

    if (!corridor.set_graph(_new_items, _new_items_count)) {
        _link.send_text(MAV_SEVERITY_WARNING, "Corridor: out of memory");
        return MAV_MISSION_ERROR;
    }

    _link.send_text(MAV_SEVERITY_INFO, "Corridor: %u nodes, %u edges",
                    corridor.num_nodes(), corridor.num_edges());
    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_Corridor::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Corridor upload timeout");
}

bool MissionItemProtocol_Corridor::clear_all_items()
{
    corridor.clear();
    return true;
}

void MissionItemProtocol_Corridor::free_upload_resources()
{
    delete[] _new_items;
    _new_items = nullptr;
    _new_items_count = 0;
}

MAV_MISSION_RESULT MissionItemProtocol_Corridor::allocate_receive_resources(const uint16_t count)
{
    if (_new_items != nullptr) {
        // the base class should have called free_upload_resources first
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return MAV_MISSION_ERROR;
    }
    if (count != 0) {
        _new_items = NEW_NOTHROW AP_Corridor::Item[count];
        if (_new_items == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Out of memory for upload");
            return MAV_MISSION_ERROR;
        }
    }
    _new_items_count = count;
    _upload_validator = {};
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Corridor::allocate_update_resources()
{
    // a partial update starts from a copy of the stored graph:
    const uint16_t count = corridor.num_items();
    const MAV_MISSION_RESULT ret = allocate_receive_resources(count);
    if (ret != MAV_MISSION_ACCEPTED) {
        return ret;
    }
    for (uint16_t i=0; i<count; i++) {
        if (!corridor.get_item(i, _new_items[i])) {
            free_upload_resources();
            return MAV_MISSION_ERROR;
        }
    }
    return MAV_MISSION_ACCEPTED;
}

#endif  // HAL_GCS_ENABLED && AP_CORRIDOR_ENABLED

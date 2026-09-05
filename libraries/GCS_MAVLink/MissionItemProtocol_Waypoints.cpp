/*
  Implementation details for transfering waypoint information using
  the MISSION_ITEM protocol to and from a GCS.

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
#include <AP_Mission/AP_Mission_config.h>

#if HAL_GCS_ENABLED && AP_MISSION_ENABLED

#include "MissionItemProtocol_Waypoints.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Mission/AP_Mission.h>

#include "GCS.h"

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::append_item(const mavlink_mission_item_int_t &mission_item_int)
{
    // sanity check for DO_JUMP command
    AP_Mission::Mission_Command cmd {};

    const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    if (cmd.id == MAV_CMD_DO_JUMP && !jump_target_valid(cmd)) {
        return MAV_MISSION_ERROR;
    }

    if (!mission.add_cmd(cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

// the target of a DO_JUMP must be an existing item or one which will
// be uploaded before the current transfer is complete, and may not be
// home:
bool MissionItemProtocol_Waypoints::jump_target_valid(const AP_Mission::Mission_Command &cmd) const
{
    const uint16_t target = cmd.content.jump.target;
    if (target == 0) {
        return false;
    }
    if (target < item_count()) {
        return true;
    }
    // request_last is in the sequence numbering used on the link:
    return target <= request_last + seq_offset(*link);
}

uint16_t MissionItemProtocol_Waypoints::seq_offset(const GCS_MAVLINK &_link) const
{
    return _link.mission_seq_offset();
}

void MissionItemProtocol_Waypoints::apply_seq_offset(mavlink_mission_item_int_t &item, int16_t offset) const
{
    MissionItemProtocol::apply_seq_offset(item, offset);
    if (item.command == MAV_CMD_DO_JUMP) {
        item.param1 += offset;
    }
}

bool MissionItemProtocol_Waypoints::clear_all_items()
{
    return mission.clear();
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::complete(const GCS_MAVLINK &_link)
{
    _link.send_text(MAV_SEVERITY_INFO, "Flight plan received");
#if HAL_LOGGING_ENABLED
    AP::logger().Write_EntireMission();
#endif
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::get_item(uint16_t seq, mavlink_mission_item_int_t &ret_packet)
{
    if (seq != 0 && // always allow HOME to be read
        seq >= mission.num_commands()) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }

    AP_Mission::Mission_Command cmd;

    // retrieve mission from eeprom
    if (!mission.read_cmd_from_storage(seq, cmd)) {
        return MAV_MISSION_ERROR;
    }

    if (!AP_Mission::mission_cmd_to_mavlink_int(cmd, ret_packet)) {
        return MAV_MISSION_ERROR;
    }
    ret_packet.mission_type = MAV_MISSION_TYPE_MISSION;

    // set packet's current field to 1 if this is the command being executed
    if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
        ret_packet.current = 1;
    }

    return MAV_MISSION_ACCEPTED;
}

uint16_t MissionItemProtocol_Waypoints::item_count() const {
    return mission.num_commands();
}

uint16_t MissionItemProtocol_Waypoints::max_items() const {
    return mission.num_commands_max();
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::replace_item(const mavlink_mission_item_int_t &mission_item_int)
{
    AP_Mission::Mission_Command cmd {};

    const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    // sanity check for DO_JUMP command
    if (cmd.id == MAV_CMD_DO_JUMP && !jump_target_valid(cmd)) {
        return MAV_MISSION_ERROR;
    }
    if (!mission.replace_cmd(cmd.index, cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_Waypoints::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Mission upload timeout");
}

void MissionItemProtocol_Waypoints::truncate(uint16_t count)
{
    // new mission arriving, truncate mission to be the same length
    mission.truncate(count);
}

#endif  // HAL_GCS_ENABLED && AP_MISSION_ENABLED

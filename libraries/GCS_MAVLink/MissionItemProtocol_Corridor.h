#pragma once

#include <AP_Corridor/AP_Corridor.h>

#if AP_CORRIDOR_ENABLED

#include "MissionItemProtocol.h"

class MissionItemProtocol_Corridor : public MissionItemProtocol {
public:
    MissionItemProtocol_Corridor(class AP_Corridor &_corridor) :
        corridor(_corridor) {}

    MAV_MISSION_TYPE mission_type() const override {
        return MAV_MISSION_TYPE_CORRIDOR;
    }

    void truncate(const mavlink_mission_count_t &packet) override {}
    MAV_MISSION_RESULT complete(const GCS_MAVLINK &_link) override;
    void timeout() override;

    static MAV_MISSION_RESULT convert_MISSION_ITEM_INT_to_Item(const mavlink_mission_item_int_t &cmd, AP_Corridor::Item &ret) WARN_IF_UNUSED;
    static void convert_Item_to_MISSION_ITEM_INT(const AP_Corridor::Item &item, uint16_t seq, mavlink_mission_item_int_t &ret);

protected:

    ap_message next_item_ap_message_id() const override {
        return MSG_NEXT_MISSION_REQUEST_CORRIDOR;
    }
    bool clear_all_items() override WARN_IF_UNUSED;

private:
    AP_Corridor &corridor;

    uint16_t item_count() const override;
    uint16_t max_items() const override;

    MAV_MISSION_RESULT replace_item(const mavlink_mission_item_int_t&) override WARN_IF_UNUSED;
    MAV_MISSION_RESULT append_item(const mavlink_mission_item_int_t&) override WARN_IF_UNUSED;

    MAV_MISSION_RESULT get_item(uint16_t seq, mavlink_mission_item_int_t &ret_packet) override WARN_IF_UNUSED;

    void free_upload_resources() override;
    MAV_MISSION_RESULT allocate_receive_resources(const uint16_t count) override WARN_IF_UNUSED;
    MAV_MISSION_RESULT allocate_update_resources() override WARN_IF_UNUSED;

    // checks the structure of a plan item by item, in upload order
    class Validator {
    public:
        MAV_MISSION_RESULT check(const AP_Corridor::Item &item) WARN_IF_UNUSED;
        MAV_MISSION_RESULT finish() const WARN_IF_UNUSED;
        uint16_t next_index() const { return num_nodes + num_edges; }
    private:
        uint16_t num_nodes;
        uint16_t num_edges;
        uint16_t last_from;
        bool have_cost;
        bool have_landing_site;
    };

    MAV_MISSION_RESULT validate_new_items(const GCS_MAVLINK &_link) const WARN_IF_UNUSED;

    // items being uploaded; the stored graph is only replaced once
    // the whole plan has been received and validated
    AP_Corridor::Item *_new_items;
    uint16_t _new_items_count;

    // checks items as they arrive in order, so errors are reported
    // against the item which caused them:
    Validator _upload_validator;
};

#endif  // AP_CORRIDOR_ENABLED

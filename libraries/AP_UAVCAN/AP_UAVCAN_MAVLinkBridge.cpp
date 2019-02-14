#include "AP_UAVCAN_MAVLinkBridge.h"

#include <AP_UAVCAN/AP_UAVCAN.h>
#include <uavcan/protocol/NodeStatus.hpp>

#include <GCS_MAVLink/GCS.h>

UC_REGISTRY_BINDER(NodeStatusCb, uavcan::protocol::NodeStatus);

bool AP_UAVCAN_MAVLinkBridge::init(AP_UAVCAN* ap_uavcan)
{
    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::protocol::NodeStatus, NodeStatusCb> *listener =
        new uavcan::Subscriber<uavcan::protocol::NodeStatus, NodeStatusCb>(*node);
    if (listener->start(NodeStatusCb(ap_uavcan, &handle_nodestatus)) < 0) {
        AP_HAL::panic("subscriber start problem");
        return false;
    }
    return true;
}

void AP_UAVCAN_MAVLinkBridge::handle_nodestatus(class AP_UAVCAN* ap_uavcan,
                                                uint8_t node_id,
                                                const NodeStatusCb &cb)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "handle_notestatus (%u)", node_id);
    // translate into a mavlink heartbeat; in the future we may want
    // to send this when we receive more information from the node.
    // translate NodeStatus health into a mavlink health:

// uint2 HEALTH_OK         = 0     # The node is functioning properly.
// uint2 HEALTH_WARNING    = 1     # A critical parameter went out of range or the node encountered a minor failure.
// uint2 HEALTH_ERROR      = 2     # The node encountered a major failure.
// uint2 HEALTH_CRITICAL   = 3     # The node suffered a fatal malfunction.
// uint2 health

// uint3 MODE_OPERATIONAL      = 0         # Normal operating mode.
// uint3 MODE_INITIALIZATION   = 1         # Initialization is in progress; this mode is entered immediately after startup.
// uint3 MODE_MAINTENANCE      = 2         # E.g. calibration, the bootloader is running, etc.
// uint3 MODE_SOFTWARE_UPDATE  = 3         # New software/firmware is being loaded.
// uint3 MODE_OFFLINE          = 7         # The node is no longer available.
// uint3 mode

    MAV_STATE system_status;
    switch (cb.msg->health) {
    case uavcan::protocol::NodeStatus::HEALTH_OK:
    case uavcan::protocol::NodeStatus::HEALTH_WARNING:
        switch (cb.msg->mode) {
        case uavcan::protocol::NodeStatus::MODE_OPERATIONAL:
            system_status = MAV_STATE_ACTIVE;
            break;
        case uavcan::protocol::NodeStatus::MODE_INITIALIZATION:
        case uavcan::protocol::NodeStatus::MODE_MAINTENANCE:
        case uavcan::protocol::NodeStatus::MODE_SOFTWARE_UPDATE:
        case uavcan::protocol::NodeStatus::MODE_OFFLINE:
        default:
            system_status = MAV_STATE_BOOT;
            break;
        }
        break;
    case uavcan::protocol::NodeStatus::HEALTH_ERROR:
    case uavcan::protocol::NodeStatus::HEALTH_CRITICAL:
    default:
        system_status = MAV_STATE_CRITICAL;
        break;
    }

    // send a heartbeat packet (MAVProxy can send parameter-sets to
    // the component ID)
    const mavlink_heartbeat_t heartbeat_packet {
        MAV_TYPE_ONBOARD_CONTROLLER, // frame_type(),
        MAV_AUTOPILOT_INVALID,
        0, // base_mode
        0, // custom_mode,
        system_status
    };
    gcs().send_to_streaming_channels_from_system(MAVLINK_MSG_ID_HEARTBEAT,
                                                 (const char*)heartbeat_packet,
                                                 mavlink_system.sysid,
                                                 node_id);
}

void AP_UAVCAN_MAVLinkBridge::handle_message(const mavlink_message_t &msg)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "handle message %u", msg.msgid);
}

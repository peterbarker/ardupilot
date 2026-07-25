/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_CANManager_config.h"

#if AP_MAVLINKCAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/CANIface.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <GCS_MAVLink/GCS.h>

class AP_MAVLinkCAN {
public:
    AP_MAVLinkCAN() {}

    // Handle commands to forward CAN frames to GCS
    static bool handle_can_forward(mavlink_channel_t chan, const mavlink_command_int_t &packet, const mavlink_message_t &msg);

    // Handle received MAVLink CAN frames
    static void handle_can_frame(const mavlink_message_t &msg);

    // Handle CAN filter modification
    static void handle_can_filter_modify(const mavlink_message_t &msg);

private:
    // Callback for receiving CAN frames from CAN bus and sending to GCS
    void can_frame_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags);
    
    /*
     * Structure to maintain forwarding state
     */
    struct {
        mavlink_channel_t chan;
        uint8_t system_id;
        uint8_t component_id;
        uint8_t frame_counter;
        uint32_t last_callback_enable_ms;
        // bumped whenever the owning client changes; see BufferFrame::gen
        uint32_t gen;
        HAL_Semaphore sem;
        uint16_t num_filter_ids;
        uint16_t *filter_ids;
        uint8_t callback_id;
        uint8_t callback_bus;
    } can_forward;

    // Buffer for storing CAN frames to be sent
    struct BufferFrame {
        uint8_t bus;
        AP_HAL::CANFrame frame;
        // which forwarding ownership this frame was captured under;
        // stale generations are discarded rather than delivered to a
        // client they were not captured for
        uint32_t gen;
    };
    
    // Frame buffer for queuing frames
    HAL_Semaphore frame_buffer_sem;
    ObjectBuffer<BufferFrame> *frame_buffer;

    // frames captured from the bus awaiting forwarding to the GCS,
    // drained as transmit space becomes available so that bursts
    // larger than the instantaneous free space (e.g. a multi-frame
    // file Read response) are not dropped
    HAL_Semaphore fwd_frame_sem;
    ObjectBuffer<BufferFrame> *fwd_frames;
    uint32_t fwd_frames_dropped;

    static AP_MAVLinkCAN *ensure_singleton();

    // Process CAN frame forwarding
    void process_frame_buffer();

    // drain captured bus frames to the GCS as space allows
    void process_fwd_frames();

    // discard any forwarded-frame backlog.  Called when the
    // forwarding client changes: queued frames were captured for the
    // old client and must not be delivered to the new one
    void flush_fwd_frames();

    // Handle commands to forward CAN frames to GCS
    bool _handle_can_forward(mavlink_channel_t chan, const mavlink_command_int_t &packet, const mavlink_message_t &msg);

    // Handle received MAVLink CAN frames
    void _handle_can_frame(const mavlink_message_t &msg);

    // Handle CAN filter modification
    void _handle_can_filter_modify(const mavlink_message_t &msg);
};

#endif // AP_MAVLINKCAN_ENABLED

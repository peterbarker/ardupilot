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

#include "AP_MAVLinkCAN.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Common/sorting.h>
#include <AP_Common/ExpandingString.h>

#if AP_MAVLINKCAN_ENABLED

// buffer sizes for bus->GCS forwarding and GCS->bus injection; a full
// 256-byte file Read response is 38 classic CAN frames
#ifndef AP_MAVLINKCAN_FWD_BUFFER_SIZE
#define AP_MAVLINKCAN_FWD_BUFFER_SIZE 128
#endif
#ifndef AP_MAVLINKCAN_INJECT_BUFFER_SIZE
#define AP_MAVLINKCAN_INJECT_BUFFER_SIZE 64
#endif

extern const AP_HAL::HAL& hal;

static AP_MAVLinkCAN *singleton;

AP_MAVLinkCAN *AP_MAVLinkCAN::ensure_singleton()
{
    if (singleton == nullptr) {
        singleton = NEW_NOTHROW AP_MAVLinkCAN();
    }
    return singleton;
}

bool AP_MAVLinkCAN::handle_can_forward(mavlink_channel_t chan, const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    auto *s = ensure_singleton();
    if (s == nullptr) {
        return false;
    }
    return singleton->_handle_can_forward(chan, packet, msg);
}

void AP_MAVLinkCAN::handle_can_frame(const mavlink_message_t &msg)
{
    auto *s = ensure_singleton();
    if (s == nullptr) {
        return;
    }
    singleton->_handle_can_frame(msg);
}

void AP_MAVLinkCAN::handle_can_filter_modify(const mavlink_message_t &msg)
{
    auto *s = ensure_singleton();
    if (s == nullptr) {
        return;
    }
    singleton->_handle_can_filter_modify(msg);
}


/*
  handle MAV_CMD_CAN_FORWARD mavlink long command
 */
bool AP_MAVLinkCAN::_handle_can_forward(mavlink_channel_t chan, const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(can_forward.sem);
    const int8_t bus = int8_t(packet.param1)-1;

    if (bus == -1) {
        /*
          a request to stop forwarding
         */
        if (can_forward.callback_id != 0) {
            hal.can[can_forward.callback_bus]->unregister_frame_callback(can_forward.callback_id);
            can_forward.callback_id = 0;
        }
        can_forward.gen++;
        flush_fwd_frames();
        return true;
    }

    if (bus < 0 || bus >= HAL_NUM_CAN_IFACES || hal.can[bus] == nullptr) {
        return false;
    }

    if (can_forward.callback_id != 0 && can_forward.callback_bus != bus && can_forward.callback_bus < HAL_NUM_CAN_IFACES) {
        /*
          the client is changing which bus they are monitoring, unregister from the previous bus
         */
        hal.can[can_forward.callback_bus]->unregister_frame_callback(can_forward.callback_id);
        can_forward.callback_id = 0;
    }

    if (can_forward.callback_id == 0 &&
        !hal.can[bus]->register_frame_callback(
            FUNCTOR_BIND_MEMBER(&AP_MAVLinkCAN::can_frame_callback, void, uint8_t, const AP_HAL::CANFrame &, AP_HAL::CANIface::CanIOFlags), can_forward.callback_id)) {
        // failed to register the callback
        return false;
    }

    if (can_forward.callback_bus != bus ||
        can_forward.chan != chan ||
        can_forward.system_id != msg.sysid ||
        can_forward.component_id != msg.compid) {
        // the backlog belongs to the previous client; the periodic
        // re-enable from an unchanged client must not flush, as the
        // queue is busy precisely when that would drop frames.  The
        // generation stamp catches the frame a callback already
        // validated but has not yet pushed
        can_forward.gen++;
        flush_fwd_frames();
    }

    can_forward.callback_bus = bus;
    can_forward.last_callback_enable_ms = AP_HAL::millis();
    can_forward.chan = chan;
    can_forward.system_id = msg.sysid;
    can_forward.component_id = msg.compid;

    return true;
}

void AP_MAVLinkCAN::flush_fwd_frames()
{
    WITH_SEMAPHORE(fwd_frame_sem);
    if (fwd_frames != nullptr) {
        fwd_frames->clear();
    }
}

/*
  handle a CAN_FRAME packet
 */
void AP_MAVLinkCAN::_handle_can_frame(const mavlink_message_t &msg)
{
    if (frame_buffer == nullptr) {
        // allocate frame buffer; the default covers a pipelined burst
        // of multi-frame service requests (e.g. file Write requests
        // during an upload)
        uint16_t buffer_size = AP_MAVLINKCAN_INJECT_BUFFER_SIZE;
        WITH_SEMAPHORE(frame_buffer_sem);
        while (frame_buffer == nullptr && buffer_size > 0) {
            // we'd like 20 frames, but will live with less
            frame_buffer = NEW_NOTHROW ObjectBuffer<BufferFrame>(buffer_size);
            if (frame_buffer != nullptr && frame_buffer->get_size() != 0) {
                // register a callback for when frames can't be sent immediately
                hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_MAVLinkCAN::process_frame_buffer, void));
                break;
            }
            delete frame_buffer;
            frame_buffer = nullptr;
            buffer_size /= 2;
        }
        if (frame_buffer == nullptr) {
            // discard the frames
            return;
        }
    }

    fwd_stats.inject_frames++;
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_CAN_FRAME: {
        mavlink_can_frame_t p;
        mavlink_msg_can_frame_decode(&msg, &p);
        if (p.bus >= HAL_NUM_CAN_IFACES || hal.can[p.bus] == nullptr) {
            return;
        }
        struct BufferFrame frame {
            bus : p.bus,
            frame : AP_HAL::CANFrame(p.id, p.data, p.len)
        };
        {
            WITH_SEMAPHORE(frame_buffer_sem);
            frame_buffer->push(frame);
        }
        break;
    }
#if HAL_CANFD_SUPPORTED
    case MAVLINK_MSG_ID_CANFD_FRAME: {
        mavlink_canfd_frame_t p;
        mavlink_msg_canfd_frame_decode(&msg, &p);
        if (p.bus >= HAL_NUM_CAN_IFACES || hal.can[p.bus] == nullptr) {
            return;
        }
        struct BufferFrame frame {
            bus : p.bus,
            frame : AP_HAL::CANFrame(p.id, p.data, p.len, true)
        };
        {
            WITH_SEMAPHORE(frame_buffer_sem);
            frame_buffer->push(frame);
        }
        break;
    }
#endif
    }
    process_frame_buffer();
}

/*
  process the frame buffer
 */
void AP_MAVLinkCAN::process_frame_buffer()
{
    while (frame_buffer) {
        WITH_SEMAPHORE(frame_buffer_sem);
        struct BufferFrame frame;
        const uint16_t timeout_us = 2000;
        if (!frame_buffer->peek(frame)) {
            // no frames in the queue
            break;
        }
        const int16_t retcode = hal.can[frame.bus]->send(frame.frame,
                                                            AP_HAL::micros64() + timeout_us,
                                                            AP_HAL::CANIface::IsForwardedFrame);
        if (retcode == 0) {
            // no space in the CAN output slots, try again later
            break;
        }
        // retcode == 1 means sent, -1 means a frame that can't be
        // sent. Either way we should remove from the queue
        frame_buffer->pop();
    }
}

/*
  handle a CAN_FILTER_MODIFY packet
 */
void AP_MAVLinkCAN::_handle_can_filter_modify(const mavlink_message_t &msg)
{
    mavlink_can_filter_modify_t p;
    mavlink_msg_can_filter_modify_decode(&msg, &p);
    const int8_t bus = int8_t(p.bus)-1;
    if (bus < 0 || bus >= HAL_NUM_CAN_IFACES || hal.can[bus] == nullptr) {
        return;
    }
    if (p.num_ids > ARRAY_SIZE(p.ids)) {
        return;
    }
    uint16_t *new_ids = nullptr;
    uint16_t num_new_ids = 0;
    WITH_SEMAPHORE(can_forward.sem);

    // sort the list, so we can bisection search and the array
    // operations below are efficient
    insertion_sort_uint16(p.ids, p.num_ids);
    
    switch (p.operation) {
    case CAN_FILTER_REPLACE: {
        if (p.num_ids == 0) {
            can_forward.num_filter_ids = 0;
            delete[] can_forward.filter_ids;
            can_forward.filter_ids = nullptr;
            return;
        }
        if (p.num_ids == can_forward.num_filter_ids &&
            memcmp(p.ids, can_forward.filter_ids, p.num_ids*sizeof(uint16_t)) == 0) {
            // common case of replacing with identical list
            return;
        }
        new_ids = NEW_NOTHROW uint16_t[p.num_ids];
        if (new_ids != nullptr) {
            num_new_ids = p.num_ids;
            memcpy((void*)new_ids, (const void *)p.ids, p.num_ids*sizeof(uint16_t));
        }
        break;
    }
    case CAN_FILTER_ADD: {
        if (common_list_uint16(can_forward.filter_ids, can_forward.num_filter_ids,
                               p.ids, p.num_ids) == p.num_ids) {
            // nothing changing
            return;
        }
        new_ids = NEW_NOTHROW uint16_t[can_forward.num_filter_ids+p.num_ids];
        if (new_ids == nullptr) {
            return;
        }
        if (can_forward.num_filter_ids != 0) {
            memcpy(new_ids, can_forward.filter_ids, can_forward.num_filter_ids*sizeof(uint16_t));
        }
        memcpy(&new_ids[can_forward.num_filter_ids], p.ids, p.num_ids*sizeof(uint16_t));
        insertion_sort_uint16(new_ids, can_forward.num_filter_ids+p.num_ids);
        num_new_ids = remove_duplicates_uint16(new_ids, can_forward.num_filter_ids+p.num_ids);
        break;
    }
    case CAN_FILTER_REMOVE: {
        if (common_list_uint16(can_forward.filter_ids, can_forward.num_filter_ids,
                               p.ids, p.num_ids) == 0) {
            // nothing changing
            return;
        }
        can_forward.num_filter_ids = remove_list_uint16(can_forward.filter_ids, can_forward.num_filter_ids,
                                                        p.ids, p.num_ids);
        if (can_forward.num_filter_ids == 0) {
            delete[] can_forward.filter_ids;
            can_forward.filter_ids = nullptr;
        }
        break;
    }
    }
    if (new_ids != nullptr) {
        // handle common case of no change
        if (num_new_ids == can_forward.num_filter_ids &&
            memcmp(new_ids, can_forward.filter_ids, num_new_ids*sizeof(uint16_t)) == 0) {
            delete[] new_ids;
        } else {
            // put the new list in place
            delete[] can_forward.filter_ids;
            can_forward.filter_ids = new_ids;
            can_forward.num_filter_ids = num_new_ids;
        }
    }
}

/*
  handler for CAN frames from the registered callback, sending frames
  out as CAN_FRAME or CANFD_FRAME messages
 */
void AP_MAVLinkCAN::can_frame_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags)
{
    uint32_t gen;
    fwd_stats.cb_calls++;
    {
        WITH_SEMAPHORE(can_forward.sem);
        if (bus != can_forward.callback_bus) {
            // we are not registered for forwarding this bus, discard frame
            fwd_stats.cb_filtered++;
            return;
        }
        if (can_forward.frame_counter++ == 100) {
            // check every 100 frames for disabling CAN_FRAME send
            // we stop sending after 5s if the client stops
            // sending MAV_CMD_CAN_FORWARD requests
            if (can_forward.callback_id != 0 &&
                AP_HAL::millis() - can_forward.last_callback_enable_ms > 5000) {
                hal.can[bus]->unregister_frame_callback(can_forward.callback_id);
                can_forward.callback_id = 0;
                return;
            }
            can_forward.frame_counter = 0;
        }
        if (can_forward.filter_ids != nullptr) {
            // work out ID of this frame
            uint16_t id = 0;
            if ((frame.id&0xff) != 0) {
                // not anonymous
                if (frame.id & 0x80) {
                    // service message
                    id = uint8_t(frame.id>>16);
                } else {
                    // message frame
                    id = uint16_t(frame.id>>8);
                }
            }
            if (!bisect_search_uint16(can_forward.filter_ids, can_forward.num_filter_ids, id)) {
                fwd_stats.cb_filtered++;
                return;
            }
        }
        // remember whose forwarding session this frame belongs to;
        // ownership can change between here and the push below, and a
        // frame must never be delivered to a client it was not
        // captured for
        gen = can_forward.gen;
    }

    // the rest is run without the can_forward.sem.  Frames are queued
    // rather than sent directly: a burst of frames (e.g. a multi-frame
    // service response) can easily exceed the instantaneous free space
    // in the telemetry stream, and frames dropped here break the whole
    // DroneCAN transfer they belong to
    if (fwd_frames == nullptr) {
        WITH_SEMAPHORE(fwd_frame_sem);
        if (fwd_frames == nullptr) {
            uint16_t buffer_size = AP_MAVLINKCAN_FWD_BUFFER_SIZE;
            while (fwd_frames == nullptr && buffer_size > 0) {
                fwd_frames = NEW_NOTHROW ObjectBuffer<BufferFrame>(buffer_size);
                if (fwd_frames != nullptr && fwd_frames->get_size() != 0) {
                    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_MAVLinkCAN::process_fwd_frames, void));
                    fwd_stats.io_registered = 1;
                    break;
                }
                delete fwd_frames;
                fwd_frames = nullptr;
                buffer_size /= 2;
            }
            if (fwd_frames == nullptr) {
                return;
            }
        }
    }
    {
        WITH_SEMAPHORE(fwd_frame_sem);
        struct BufferFrame frame_copy {
            bus : bus,
            frame : frame,
            gen : gen
        };
        if (!fwd_frames->push(frame_copy)) {
            fwd_stats.push_drops++;
        } else {
            fwd_stats.pushed++;
        }
    }
    process_fwd_frames();
}

/*
  drain captured bus frames to the GCS as telemetry stream space
  becomes available
 */
void AP_MAVLinkCAN::process_fwd_frames()
{
    fwd_stats.drain_calls++;
    mavlink_channel_t chan;
    uint8_t system_id;
    uint8_t component_id;
    uint32_t gen;
    {
        WITH_SEMAPHORE(can_forward.sem);
        if (can_forward.callback_id == 0) {
            // forwarding is not active; discard any backlog
            WITH_SEMAPHORE(fwd_frame_sem);
            if (fwd_frames != nullptr) {
                struct BufferFrame frame;
                while (fwd_frames->pop(frame)) {
                }
            }
            return;
        }
        chan = can_forward.chan;
        system_id = can_forward.system_id;
        component_id = can_forward.component_id;
        gen = can_forward.gen;
    }

    while (true) {
        WITH_SEMAPHORE(fwd_frame_sem);
        struct BufferFrame frame;
        if (fwd_frames == nullptr || !fwd_frames->peek(frame)) {
            break;
        }
        if (frame.gen != gen) {
            // captured for a previous forwarding client; discard
            IGNORE_RETURN(fwd_frames->pop(frame));
            continue;
        }
        WITH_SEMAPHORE(comm_chan_lock(chan));
        const uint8_t data_len = AP_HAL::CANFrame::dlcToDataLength(frame.frame.dlc);
#if HAL_CANFD_SUPPORTED
        if (frame.frame.isCanFDFrame()) {
            if (!HAVE_PAYLOAD_SPACE(chan, CANFD_FRAME)) {
                fwd_stats.space_breaks++;
                break;
            }
            mavlink_msg_canfd_frame_send(chan, system_id, component_id,
                                         frame.bus, data_len, frame.frame.id,
                                         const_cast<uint8_t*>(frame.frame.data));
        } else
#endif
        {
            if (!HAVE_PAYLOAD_SPACE(chan, CAN_FRAME)) {
                fwd_stats.space_breaks++;
                break;
            }
            mavlink_msg_can_frame_send(chan, system_id, component_id,
                                       frame.bus, data_len, frame.frame.id,
                                       const_cast<uint8_t*>(frame.frame.data));
        }
        fwd_stats.sent++;
        fwd_frames->pop();
    }
}

/*
  dump forwarding diagnostics for @SYS/mavlinkcan.txt
 */
void AP_MAVLinkCAN::get_stats_text(ExpandingString &str)
{
    auto *s = singleton;
    if (s == nullptr) {
        str.printf("MAVLinkCAN not active\n");
        return;
    }
    uint32_t depth = 0;
    {
        WITH_SEMAPHORE(s->fwd_frame_sem);
        if (s->fwd_frames != nullptr) {
            depth = s->fwd_frames->available();
        }
    }
    uint32_t txspace = 0;
    uint8_t cb_id;
    uint8_t chan;
    {
        WITH_SEMAPHORE(s->can_forward.sem);
        cb_id = s->can_forward.callback_id;
        chan = (uint8_t)s->can_forward.chan;
        txspace = comm_get_txspace(s->can_forward.chan);
    }
    str.printf("callback_id:  %u\n"
               "chan:         %u\n"
               "txspace:      %u\n"
               "cb_calls:     %u\n"
               "cb_filtered:  %u\n"
               "pushed:       %u\n"
               "push_drops:   %u\n"
               "sent:         %u\n"
               "drain_calls:  %u\n"
               "space_breaks: %u\n"
               "inject_frames:%u\n"
               "io_registered:%u\n"
               "queue_depth:  %u\n",
               (unsigned)cb_id, (unsigned)chan, (unsigned)txspace,
               (unsigned)s->fwd_stats.cb_calls, (unsigned)s->fwd_stats.cb_filtered,
               (unsigned)s->fwd_stats.pushed, (unsigned)s->fwd_stats.push_drops,
               (unsigned)s->fwd_stats.sent, (unsigned)s->fwd_stats.drain_calls,
               (unsigned)s->fwd_stats.space_breaks, (unsigned)s->fwd_stats.inject_frames,
               (unsigned)s->fwd_stats.io_registered, (unsigned)depth);
}

#endif  // AP_MAVLINKCAN_ENABLED

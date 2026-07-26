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
/*
  client for the uavcan.protocol.file Read service, fetching a file
  from another node on a DroneCAN bus into the local filesystem.

  One read at a time is latency bound - a round trip per 256 bytes -
  so several are kept in flight at once.  Each one needs its own
  Canard::Client: a client matches a reply by transfer ID and holds
  only the one it last sent.  Transfer IDs come from a pool shared by
  every client of the same service and destination, so the reads which
  are in flight together are told apart.

  That sharing is also why a read which goes unanswered ends the
  transfer instead of being sent again - see the timeout in the
  implementation.
 */
#pragma once

#include "AP_DroneCAN_FileClient_config.h"

#if AP_DRONECAN_FILE_CLIENT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <dronecan_msgs.h>

class AP_DroneCAN;

class AP_DroneCAN_FileClient {
public:

    AP_DroneCAN_FileClient() {}

    CLASS_NO_COPY(AP_DroneCAN_FileClient);

    // maximum DSDL Path length plus nul terminator
    static const uint16_t PATH_MAX_LEN = 201;

    enum class State : uint8_t {
        IDLE = 0,
        RUNNING,
        DONE,
        FAILED,
    };

    // attach to a driver.  Must be called before start()
    bool init(AP_DroneCAN *dronecan);

    /*
      only one transfer runs at a time, so the first client to come up
      is the one scripting talks to.  A board with two DroneCAN drivers
      fetches over the first of them
     */
    static AP_DroneCAN_FileClient *get_singleton() { return _singleton; }

    /*
      begin fetching remote_path from node_id into local_path.
      Returns false if a transfer is already running, the paths do not
      fit, or the local file cannot be created
     */
    bool start(uint8_t node_id, const char *remote_path, const char *local_path);

    // abandon any transfer in progress
    void stop();

    // drive the transfer; called from the driver's thread
    void update();

    // these are called from other threads (e.g. scripting), while
    // the transfer runs in the DroneCAN thread, so take the semaphore
    State get_state() const {
        WITH_SEMAPHORE(sem);
        return state;
    }

    // the state as a plain integer, for scripting
    uint8_t get_status() const { return (uint8_t)get_state(); }

    uint32_t get_bytes() const {
        WITH_SEMAPHORE(sem);
        return bytes_written;
    }

private:

    // a read which has been sent and not yet answered
    struct Slot {
        AP_DroneCAN_FileClient *owner;
        Canard::Client<uavcan_protocol_file_ReadResponse> *client;
        uint32_t offset;
        uint32_t sent_ms;
        bool active;

        void handle_read_response(const CanardRxTransfer& transfer,
                                  const uavcan_protocol_file_ReadResponse& rsp);
    };

    // fill any free slots with the next reads which are needed
    void fill_pipeline();

    // remember an offset whose read went unanswered, to ask again once
    // the bus has been left quiet long enough
    void queue_redo(uint32_t offset);

    // send a read for offset in this slot
    bool send_read(Slot &slot, uint32_t offset);

    // give up, closing the file and leaving what was fetched behind
    void fail();

    // the transfer is finished; close the file
    void finish();

    AP_DroneCAN *ap_dronecan;

    Slot slots[AP_DRONECAN_FILE_CLIENT_DEPTH];
    Canard::ObjCallback<Slot, uavcan_protocol_file_ReadResponse> *callbacks[AP_DRONECAN_FILE_CLIENT_DEPTH];

    State state = State::IDLE;

    uint8_t target_node;
    char remote_path[PATH_MAX_LEN];

    int fd = -1;
    bool discard;
    uint32_t bytes_written;

    /*
      the offset of the next read to send, and where the file ends once
      a short reply has told us.  Replies can arrive out of order, so
      the end is remembered rather than acted on at once: reads already
      in flight beyond it are simply dropped
     */
    uint32_t next_offset;
    uint32_t eof_offset;
    bool eof_known;

    /*
      offsets whose reads were abandoned, and the time before which no
      new read may be sent.  A reply is matched to its read by transfer
      ID alone and those come round again every thirty two requests, so
      after abandoning one we stop sending until any reply to it must
      have arrived - otherwise its ID would be handed to another read
      and its data written at that read's offset
     */
    uint32_t redo[AP_DRONECAN_FILE_CLIENT_DEPTH];
    uint8_t redo_count;
    uint32_t quiet_until_ms;
    uint8_t timeouts;

    /*
      how many reads to have out at once.  Not every node can take the
      full depth: a small one runs out of memory to reassemble them in
      and simply drops what it cannot hold, which looks like a read
      going unanswered.  Back off when that happens rather than
      failing, and the transfer finishes at whatever depth suits.

      For the same reason this starts low and works up rather than
      opening at the full depth: a burst of reads to a node with a
      small pool loses most of them before anything has been learnt
      about what it can take
     */
    uint8_t depth;
    uint16_t good_streak;

    // mutable so the const getters above can take it
    mutable HAL_Semaphore sem;

    static AP_DroneCAN_FileClient *_singleton;
};

#endif  // AP_DRONECAN_FILE_CLIENT_ENABLED

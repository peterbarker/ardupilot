#include "AP_DroneCAN_FileClient.h"

#if AP_DRONECAN_FILE_CLIENT_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

#include <string.h>

// a Read reply carries at most this much; a shorter one is the end of
// the file
#define FILE_READ_CHUNK 256U

/*
  how long to wait for a read before giving up on the transfer.

  A read which goes unanswered ends the transfer rather than being
  sent again, and that is deliberate.  A reply is matched to its
  request by transfer ID alone, those are five bits, and every client
  of a service draws them from one pool - so an ID comes round again
  after thirty two requests, which at the rate these are sent is a few
  tens of milliseconds.  Send a read again and the reply to the first
  one can still arrive afterwards, by which time its ID belongs to a
  different read, whose reply it will be taken for.  The data then
  lands at the wrong offset in a transfer which goes on to report that
  it succeeded.

  So a read which goes unanswered is not simply sent again.  Nothing
  at all is sent for a while first, long enough that a reply to the
  abandoned read must already have arrived and been discarded, and
  only then is that offset asked for again.  Sending stops completely
  during that wait, so no transfer ID is handed out while an older use
  of it could still be answered.
 */
#define FILE_CLIENT_READ_TIMEOUT_MS 400

/*
  how long to send nothing after abandoning a read, and how many
  abandoned reads to put up with before giving up.

  The wait only has to outlast a reply which is merely late; one the
  far node dropped is never coming.  A node too small to reassemble
  what is asked of it drops a good many, so allow plenty of them - the
  depth comes down each time, and a transfer which crawls is better
  than one which gives up.
 */
#define FILE_CLIENT_QUIET_MS 300
#define FILE_CLIENT_MAX_TIMEOUTS 40

// reads in flight to begin with, and how many must be answered in a
// row before asking for one more at a time
#define FILE_CLIENT_START_DEPTH 2
#define FILE_CLIENT_GROW_AFTER 16

extern const AP_HAL::HAL& hal;

AP_DroneCAN_FileClient *AP_DroneCAN_FileClient::_singleton;

bool AP_DroneCAN_FileClient::init(AP_DroneCAN *dronecan)
{
    if (dronecan == nullptr || ap_dronecan != nullptr) {
        return false;
    }
    ap_dronecan = dronecan;

    for (uint8_t i = 0; i < ARRAY_SIZE(slots); i++) {
        slots[i].owner = this;
        callbacks[i] = NEW_NOTHROW Canard::ObjCallback<Slot, uavcan_protocol_file_ReadResponse>{
            &slots[i], &Slot::handle_read_response};
        if (callbacks[i] == nullptr) {
            return false;
        }
        slots[i].client = NEW_NOTHROW Canard::Client<uavcan_protocol_file_ReadResponse>{
            ap_dronecan->get_canard_iface(), *callbacks[i]};
        if (slots[i].client == nullptr) {
            return false;
        }
    }
    if (_singleton == nullptr) {
        _singleton = this;
    }
    return true;
}

bool AP_DroneCAN_FileClient::start(uint8_t node_id, const char *rpath, const char *lpath)
{
    WITH_SEMAPHORE(sem);

    if (ap_dronecan == nullptr || state == State::RUNNING) {
        return false;
    }
    if (node_id == 0 || rpath == nullptr || lpath == nullptr) {
        return false;
    }
    if (strlen(rpath) >= sizeof(remote_path)) {
        return false;
    }

    if (fd != -1) {
        AP::FS().close(fd);
        fd = -1;
    }
    /*
      an empty local path throws the data away as it arrives.  That is
      not useful for fetching a file, but it takes the local
      filesystem out of the measurement when working out what limits
      the transfer rate
     */
    discard = (lpath[0] == 0);
    if (!discard) {
        fd = AP::FS().open(lpath, O_WRONLY | O_CREAT | O_TRUNC);
        if (fd == -1) {
            return false;
        }
    }

    strncpy(remote_path, rpath, sizeof(remote_path)-1);
    remote_path[sizeof(remote_path)-1] = 0;
    target_node = node_id;
    bytes_written = 0;
    redo_count = 0;
    quiet_until_ms = 0;
    timeouts = 0;
    depth = FILE_CLIENT_START_DEPTH;
    good_streak = 0;
    next_offset = 0;
    eof_offset = 0;
    eof_known = false;
    for (uint8_t i = 0; i < ARRAY_SIZE(slots); i++) {
        slots[i].active = false;
    }
    state = State::RUNNING;

    fill_pipeline();
    return true;
}

void AP_DroneCAN_FileClient::stop()
{
    WITH_SEMAPHORE(sem);
    if (state == State::RUNNING) {
        fail();
    }
}

bool AP_DroneCAN_FileClient::send_read(Slot &slot, uint32_t offset)
{
    uavcan_protocol_file_ReadRequest req {};
    req.offset = offset;
    const size_t plen = strlen(remote_path);
    req.path.path.len = plen;
    memcpy(req.path.path.data, remote_path, plen);

    if (!slot.client->request(target_node, req)) {
        return false;
    }
    slot.offset = offset;
    slot.sent_ms = AP_HAL::millis();
    slot.active = true;
    return true;
}

void AP_DroneCAN_FileClient::queue_redo(uint32_t offset)
{
    if (redo_count < ARRAY_SIZE(redo)) {
        redo[redo_count++] = offset;
    }
}

void AP_DroneCAN_FileClient::fill_pipeline()
{
    if (AP_HAL::millis() < quiet_until_ms) {
        // waiting out a read we abandoned; sending now would reuse a
        // transfer ID which is still spoken for
        return;
    }
    uint8_t in_flight = 0;
    for (uint8_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (slots[i].active) {
            in_flight++;
        }
    }
    for (uint8_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (state != State::RUNNING || in_flight >= depth) {
            return;
        }
        if (slots[i].active) {
            continue;
        }
        uint32_t offset;
        if (redo_count > 0) {
            offset = redo[redo_count - 1];
        } else if (!eof_known || next_offset < eof_offset) {
            offset = next_offset;
        } else {
            // nothing left worth asking for
            continue;
        }
        if (!send_read(slots[i], offset)) {
            // the queue is full; try again on the next pass
            return;
        }
        in_flight++;
        if (redo_count > 0) {
            redo_count--;
        } else {
            next_offset += FILE_READ_CHUNK;
        }
    }
}

void AP_DroneCAN_FileClient::Slot::handle_read_response(const CanardRxTransfer& transfer,
                                                        const uavcan_protocol_file_ReadResponse& rsp)
{
    (void)transfer;
    AP_DroneCAN_FileClient &c = *owner;
    WITH_SEMAPHORE(c.sem);

    if (!active || c.state != State::RUNNING) {
        return;
    }
    active = false;

    if (rsp.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK) {
        c.fail();
        return;
    }

    if (rsp.data.len > 0) {
        if (!c.discard) {
            if (AP::FS().lseek(c.fd, offset, SEEK_SET) == -1) {
                c.fail();
                return;
            }
            const int32_t n = AP::FS().write(c.fd, rsp.data.data, rsp.data.len);
            if (n != (int32_t)rsp.data.len) {
                c.fail();
                return;
            }
        }
        c.bytes_written += rsp.data.len;
    }

    if (++c.good_streak >= (uint16_t)c.depth * FILE_CLIENT_GROW_AFTER) {
        c.good_streak = 0;
        if (c.depth < ARRAY_SIZE(c.slots)) {
            c.depth++;
        }
    }

    if (rsp.data.len < FILE_READ_CHUNK) {
        // a short reply is the end of the file.  Reads already in
        // flight past this point are still outstanding, so remember
        // where the end is rather than finishing here
        const uint32_t end = offset + rsp.data.len;
        if (!c.eof_known || end < c.eof_offset) {
            c.eof_offset = end;
            c.eof_known = true;
        }
    }
}

void AP_DroneCAN_FileClient::fail()
{
    if (fd != -1) {
        AP::FS().close(fd);
        fd = -1;
    }
    for (uint8_t i = 0; i < ARRAY_SIZE(slots); i++) {
        slots[i].active = false;
    }
    state = State::FAILED;
}

void AP_DroneCAN_FileClient::finish()
{
    // a chunk asked for twice is counted twice, so settle up here
    bytes_written = eof_offset;
    if (fd != -1) {
        AP::FS().close(fd);
        fd = -1;
    }
    state = State::DONE;
}

void AP_DroneCAN_FileClient::update()
{
    WITH_SEMAPHORE(sem);

    if (state != State::RUNNING) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    bool any_active = false;

    for (uint8_t i = 0; i < ARRAY_SIZE(slots); i++) {
        Slot &slot = slots[i];
        if (!slot.active) {
            continue;
        }
        if (eof_known && slot.offset >= eof_offset) {
            // this read is past the end of the file; whatever comes
            // back for it is of no use
            slot.active = false;
            continue;
        }
        if (now_ms - slot.sent_ms <= FILE_CLIENT_READ_TIMEOUT_MS) {
            any_active = true;
            continue;
        }
        /*
          abandon this read.  Its offset is asked for again, but only
          after everything has been quiet long enough that its reply
          cannot still turn up wearing a transfer ID which by then
          means a different read
         */
        slot.active = false;
        if (++timeouts > FILE_CLIENT_MAX_TIMEOUTS) {
            fail();
            return;
        }
        // the far node could not keep up with this many at once
        depth = (depth > 1) ? depth / 2 : 1;
        good_streak = 0;
        queue_redo(slot.offset);
        quiet_until_ms = now_ms + FILE_CLIENT_QUIET_MS;
    }

    fill_pipeline();

    for (uint8_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (slots[i].active) {
            any_active = true;
        }
    }

    if (eof_known && !any_active && redo_count == 0 &&
        next_offset >= eof_offset) {
        finish();
    }
}

#endif  // AP_DRONECAN_FILE_CLIENT_ENABLED

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
  command line client for the uavcan.protocol.file services, for
  listing and downloading files - notably dataflash logs - from a
  DroneCAN node.

  Talks either to an SLCAN serial port or to a MAVLink endpoint
  forwarding CAN traffic (MAVCAN).  Reads are pipelined, which matters:
  each request/response pair costs a round trip, so a serialised
  client runs several times slower than a pipelined one.

  See README.md for build and usage.
 */
#include <canard.h>

#include <arpa/inet.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>

#define UAVCAN_PROTOCOL_FILE_GETINFO_ID 45
#define UAVCAN_PROTOCOL_FILE_GETINFO_SIGNATURE 0x5004891EE8A27531ULL
#define UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_ID 46
#define UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_SIGNATURE 0x8C46E8AB568BDA79ULL
#define UAVCAN_PROTOCOL_FILE_DELETE_ID 47
#define UAVCAN_PROTOCOL_FILE_DELETE_SIGNATURE 0x78648C99170B47AAULL
#define UAVCAN_PROTOCOL_FILE_WRITE_ID 49
#define UAVCAN_PROTOCOL_FILE_WRITE_SIGNATURE 0x515AA1DC77E58429ULL
#define UAVCAN_PROTOCOL_FILE_READ_ID 48
#define UAVCAN_PROTOCOL_FILE_READ_SIGNATURE 0x8DCDCA939F33F678ULL

#define FILE_READ_CHUNK 256U
#define FILE_WRITE_CHUNK 192U
#define FILE_PATH_MAX 200U
#define ENTRY_TYPE_FLAG_DIRECTORY 2U

// transfer IDs are five bits on the wire, so a deeper pipeline than
// this cannot be matched to its requests
#define MAX_PIPELINE_DEPTH 30

// how the pipeline depth is chosen; see the controllers below
enum WindowAlgo {
    WINDOW_AIMD = 0,
    WINDOW_PROBE,
};

static struct {
    const char *uri;
    const char *remote_path;
    const char *local_path;
    uint8_t target_node;
    uint8_t local_node;
    int depth;
    bool adaptive;
    enum WindowAlgo algo;
    bool canfd;
    bool verbose;
} opts = {nullptr, nullptr, nullptr, 0, 100, 0, true, WINDOW_AIMD, false, false};

#if defined(USE_USER_HELPERS)
/*
  libcanard can be built expecting the application to serialise access
  to its memory pool.  This tool drives canard from a single thread, so
  these need do nothing
 */
extern "C" {
void canard_allocate_sem_take(CanardPoolAllocator *allocator) { (void)allocator; }
void canard_allocate_sem_give(CanardPoolAllocator *allocator) { (void)allocator; }
}
#endif

static CanardInstance canard;
static uint8_t canard_memory[131072];
static int transport_fd = -1;
static bool transport_is_mcast;

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */

/*
  monotonic microseconds.  Deliberately integer: seconds since the
  epoch do not fit in a float, and this is built with
  -fsingle-precision-constant, which would silently evaluate a
  floating point expression here at float precision - around two
  minutes of resolution - and break every timeout in this program
 */
static uint64_t micros64(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
}

static double now_s(void)
{
    // relative to the first call, so that the result stays small
    // enough to be exact
    static uint64_t start_us;
    const uint64_t us = micros64();
    if (start_us == 0) {
        start_us = us;
    }
    return (double)(us - start_us) * 1e-6;
}

static uint8_t dlc_to_len(uint8_t dlc)
{
    static const uint8_t lens[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return lens[dlc & 0xF];
}

static uint8_t len_to_dlc(uint8_t len)
{
    if (len <= 8) {
        return len;
    }
    static const uint8_t lens[7] = {12, 16, 20, 24, 32, 48, 64};
    for (uint8_t i = 0; i < 7; i++) {
        if (len <= lens[i]) {
            return 9 + i;
        }
    }
    return 15;
}

/* ------------------------------------------------------------------ */
/* SLCAN transport                                                     */
/* ------------------------------------------------------------------ */

static void serial_write_all(const char *buf, size_t len)
{
    size_t ofs = 0;
    while (ofs < len) {
        const ssize_t n = write(transport_fd, buf + ofs, len - ofs);
        if (n > 0) {
            ofs += (size_t)n;
        } else {
            usleep(200);
        }
    }
}

static void slcan_send_frame(const CanardCANFrame *f)
{
    // an FD frame needs 1 + 8 + 1 + 128 characters
    char line[160];
    int n;
#if CANARD_ENABLE_CANFD
    if (f->canfd) {
        n = snprintf(line, sizeof(line), "D%08X%X", (unsigned)(f->id & 0x1FFFFFFFU), len_to_dlc(f->data_len));
    } else
#endif
    {
        n = snprintf(line, sizeof(line), "T%08X%u", (unsigned)(f->id & 0x1FFFFFFFU), f->data_len);
    }
    for (uint8_t i = 0; i < f->data_len; i++) {
        n += snprintf(line + n, sizeof(line) - n, "%02X", f->data[i]);
    }
    line[n++] = '\r';
    serial_write_all(line, n);
}

static void slcan_handle_line(const char *line, size_t len)
{
    const bool is_fd = line[0] == 'D';
    if (len < 10 || (line[0] != 'T' && !is_fd)) {
        return;
    }
    CanardCANFrame f {};
    char idbuf[9];
    memcpy(idbuf, &line[1], 8);
    idbuf[8] = 0;
    f.id = (uint32_t)strtoul(idbuf, nullptr, 16) | CANARD_CAN_FRAME_EFF;
    const char dlcbuf[2] = {line[9], 0};
    const uint8_t dlc = (uint8_t)strtoul(dlcbuf, nullptr, 16);
    const uint8_t nbytes = is_fd ? dlc_to_len(dlc) : dlc;
    if ((!is_fd && dlc > 8) || len < 10 + 2U * nbytes) {
        return;
    }
#if CANARD_ENABLE_CANFD
    f.canfd = is_fd;
#endif
    f.data_len = nbytes;
    for (uint8_t i = 0; i < nbytes; i++) {
        const char b[3] = {line[10 + 2 * i], line[11 + 2 * i], 0};
        f.data[i] = (uint8_t)strtoul(b, nullptr, 16);
    }
    canardHandleRxFrame(&canard, &f, micros64());
}

static void slcan_pump_rx(void)
{
    // an FD frame line can exceed 138 characters; a short buffer here
    // silently discards every FD frame
    static char linebuf[300];
    static size_t linelen;
    char buf[4096];
    const ssize_t n = read(transport_fd, buf, sizeof(buf));
    for (ssize_t i = 0; i < n; i++) {
        const char c = buf[i];
        if (c == '\r' || c == '\n' || c == '\a') {
            if (linelen > 0) {
                linebuf[linelen] = 0;
                slcan_handle_line(linebuf, linelen);
                linelen = 0;
            }
        } else if (linelen < sizeof(linebuf) - 1) {
            linebuf[linelen++] = c;
        } else {
            linelen = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/* multicast transport, as used between SITL instances                 */
/* ------------------------------------------------------------------ */

#define MCAST_ADDRESS_BASE "239.65.82.0"
#define MCAST_PORT 57732U
#define MCAST_MAGIC 0x2934U
#define MCAST_FLAG_CANFD 0x0001
#define MCAST_MAX_PKT_LEN 74

struct __attribute__((packed)) mcast_pkt {
    uint16_t magic;
    uint16_t crc;
    uint16_t flags;
    uint32_t message_id;
    uint8_t data[MCAST_MAX_PKT_LEN - 10];
};

static int mcast_tx_fd = -1;
static struct sockaddr_in mcast_dest;

static uint16_t crc16_ccitt_sw(const uint8_t *buf, uint32_t len, uint16_t crc)
{
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

static void mcast_send_frame(const CanardCANFrame *f)
{
    struct mcast_pkt pkt {};
    pkt.magic = MCAST_MAGIC;
#if CANARD_ENABLE_CANFD
    if (f->canfd) {
        pkt.flags |= MCAST_FLAG_CANFD;
    }
#endif
    /*
      the flag bits above the 29 bit ID must be kept: the far end
      reconstructs the frame from this field and a missing extended
      frame flag leaves it looking like an 11 bit ID, which the
      DroneCAN stack ignores
     */
    pkt.message_id = f->id;
    memcpy(pkt.data, f->data, f->data_len);
    pkt.crc = crc16_ccitt_sw((uint8_t *)&pkt.flags, f->data_len + 6, 0xFFFFU);
    if (sendto(mcast_tx_fd, &pkt, f->data_len + 10, 0,
               (struct sockaddr *)&mcast_dest, sizeof(mcast_dest)) < 0) {
        static bool reported;
        if (!reported) {
            perror("multicast send");
            reported = true;
        }
    }
}

static void mcast_pump_rx(void)
{
    struct mcast_pkt pkt;
    while (true) {
        const ssize_t ret = recv(transport_fd, &pkt, sizeof(pkt), 0);
        if (ret < 10) {
            return;
        }
        if (pkt.magic != MCAST_MAGIC) {
            continue;
        }
        if (pkt.crc != crc16_ccitt_sw((uint8_t *)&pkt.flags, ret - 4, 0xFFFFU)) {
            continue;
        }
        CanardCANFrame f {};
        f.id = pkt.message_id | CANARD_CAN_FRAME_EFF;
        f.data_len = (uint8_t)(ret - 10);
#if CANARD_ENABLE_CANFD
        f.canfd = (pkt.flags & MCAST_FLAG_CANFD) != 0;
#endif
        memcpy(f.data, pkt.data, f.data_len);
        canardHandleRxFrame(&canard, &f, micros64());
    }
}

static bool mcast_init(uint8_t bus)
{
    char address[] = MCAST_ADDRESS_BASE;
    address[strlen(address) - 1] = '0' + bus;

    transport_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (transport_fd < 0) {
        perror("socket");
        return false;
    }
    int one = 1;
    setsockopt(transport_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in sa {};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(MCAST_PORT);
    sa.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(transport_fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
        perror("bind");
        return false;
    }
    struct ip_mreq mreq {};
    mreq.imr_multiaddr.s_addr = inet_addr(address);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(transport_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) != 0) {
        perror("IP_ADD_MEMBERSHIP");
        return false;
    }
    fcntl(transport_fd, F_SETFL, fcntl(transport_fd, F_GETFL, 0) | O_NONBLOCK);

    // send from a separate socket so that our source port differs from
    // the multicast port we are bound to
    mcast_tx_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (mcast_tx_fd < 0) {
        return false;
    }
    fcntl(mcast_tx_fd, F_SETFL, fcntl(mcast_tx_fd, F_GETFL, 0) | O_NONBLOCK);
    memset(&mcast_dest, 0, sizeof(mcast_dest));
    mcast_dest.sin_family = AF_INET;
    mcast_dest.sin_port = htons(MCAST_PORT);
    mcast_dest.sin_addr.s_addr = inet_addr(address);
    return true;
}

/* ------------------------------------------------------------------ */
/* transport dispatch                                                  */
/* ------------------------------------------------------------------ */

static void transport_send_frame(const CanardCANFrame *f)
{
    if (transport_is_mcast) {
        mcast_send_frame(f);
    } else {
        slcan_send_frame(f);
    }
}

static void transport_pump_rx(void)
{
    if (transport_is_mcast) {
        mcast_pump_rx();
    } else {
        slcan_pump_rx();
    }
}

static void pump_tx(void)
{
    const CanardCANFrame *f;
    while ((f = canardPeekTxQueue(&canard)) != nullptr) {
        transport_send_frame(f);
        canardPopTxQueue(&canard);
    }
}

/* ------------------------------------------------------------------ */
/* service requests                                                    */
/* ------------------------------------------------------------------ */

/*
  the tail array optimisation removes the length prefix of a trailing
  array, but it does not apply to CANFD transfers
 */
static bool tao_active(void)
{
    return !opts.canfd;
}

// append a Path to a request payload, honouring TAO
static uint16_t append_path(uint8_t *payload, uint16_t ofs, const char *path)
{
    const size_t plen = strlen(path);
    if (!tao_active()) {
        payload[ofs++] = (uint8_t)plen;
    }
    memcpy(&payload[ofs], path, plen);
    return ofs + (uint16_t)plen;
}

/*
  append a Path which is not the last field of its message.  The tail
  array optimisation only ever drops the length of the *last* field, so
  a Path with anything after it always carries its length
 */
static uint16_t append_sized_path(uint8_t *payload, uint16_t ofs, const char *path)
{
    const size_t plen = strlen(path);
    payload[ofs++] = (uint8_t)plen;
    memcpy(&payload[ofs], path, plen);
    return ofs + (uint16_t)plen;
}

/*
  a reply, as copied out of the transfer.  The transfer itself is only
  valid for the duration of the reception callback - canard frees its
  payload blocks as soon as that returns - so the bytes are copied out
  and everything afterwards works from this
 */
struct Reply {
    bool got;
    uint16_t payload_len;
    uint8_t payload[300];
};

// little endian field accessors over a copied payload
static uint64_t reply_uint(const Reply &r, uint16_t byte_ofs, uint8_t nbytes)
{
    uint64_t v = 0;
    for (uint8_t i = 0; i < nbytes; i++) {
        if (byte_ofs + i < r.payload_len) {
            v |= (uint64_t)r.payload[byte_ofs + i] << (8 * i);
        }
    }
    return v;
}

static int16_t reply_int16(const Reply &r, uint16_t byte_ofs)
{
    return (int16_t)reply_uint(r, byte_ofs, 2);
}

// a pending Read, tracked by the transfer ID seen on the wire
struct Pending {
    bool active;
    uint32_t chunk;
    uint8_t wire_tid;
    double sent_t;
    uint8_t retries;
};

static Pending pending[MAX_PIPELINE_DEPTH];
static uint8_t simple_transfer_id;

/*
  A Read reply carries no offset, so replies can only be matched to
  requests by the five bit transfer ID on the wire.  There are only 32
  of those, so if IDs are simply used in sequence a reply that arrives
  late - after we have given up and moved on - is indistinguishable
  from the reply to the request issued 32 requests later, and one
  chunk's data is silently stored in another chunk's place.
  (Observed in practice: chunk N holding the data of chunk N+32.)

  Transfer IDs are therefore owned: an ID is allocated to a request and
  is not reused until that request completes.  An ID belonging to a
  request we abandoned is held back for long enough that a late reply
  is certain to have arrived and been discarded before the ID can be
  issued again.
 */
#define TRANSFER_ID_COUNT 32
#define TRANSFER_ID_HOLDOFF_S 3.0

/*
  How many reads to keep in flight.  A distant or busy node answers
  slowly, and pushing more requests at it than it can service just
  produces timeouts and retried work - one board measured here services
  a read in 5ms, another in 54ms.  Unless a fixed depth is asked for,
  the depth follows what the link can actually carry.  Two controllers
  are available, chosen with -a:

  probe   time how long a run of chunks takes and step the depth in
          whichever direction last made that better.  This measures the
          quantity we actually want and needs no opinion about how many
          timeouts are too many.

  aimd    grow by one after a run of good replies, halve on a timeout.
          This is fine while the CAN wire is not the limit, but on a bus
          where it is, timeouts happen even at the best depth, so the
          depth never settles: it climbs until throughput collapses,
          halves to below the best depth, and climbs again.  Measured
          over a 1Mbit bus it returned about two thirds of the
          throughput of the best fixed depth.
 */
#define ADAPTIVE_START_DEPTH 2
#define ADAPTIVE_GROW_AFTER 8       // consecutive good replies per step

// how many chunks the probe controller times before it decides, and how
// much worse a run has to be before it turns around.  The margin keeps
// ordinary jitter from causing a reversal
#define PROBE_RUN_CHUNKS 64
#define PROBE_WORSE_RATIO 0.97
// how far below the best run we tolerate before going back to the depth
// which produced it, and how fast that best fades so that a depth which
// was good once does not hold us there forever
#define PROBE_FALLBACK_RATIO 0.9
#define PROBE_BEST_DECAY 0.99

static int window = ADAPTIVE_START_DEPTH;
static uint32_t good_streak;

static double probe_run_start;
static bool probe_running;
static uint32_t probe_run_chunks;
static double probe_prev_rate;
static double probe_best_rate;
static int probe_best_window = ADAPTIVE_START_DEPTH;
static int probe_dir = 1;

static int in_flight(void);

static void window_step(int delta)
{
    window += delta;
    if (window < 1) {
        window = 1;
    } else if (window > MAX_PIPELINE_DEPTH) {
        window = MAX_PIPELINE_DEPTH;
    }
}

static void window_reply_ok(void)
{
    if (!opts.adaptive) {
        return;
    }
    if (opts.algo == WINDOW_AIMD) {
        if (++good_streak >= (uint32_t)(window * ADAPTIVE_GROW_AFTER)) {
            good_streak = 0;
            window_step(1);
        }
        return;
    }

    if (!probe_running) {
        probe_running = true;
        probe_run_start = now_s();
    }
    if (++probe_run_chunks < PROBE_RUN_CHUNKS) {
        return;
    }
    const double now = now_s();
    const double dt = now - probe_run_start;
    const double rate = dt > 0 ? probe_run_chunks / dt : 0;
    if (rate > probe_best_rate) {
        probe_best_rate = rate;
        probe_best_window = window;
    }
    if (probe_best_rate > 0 && rate < probe_best_rate * PROBE_FALLBACK_RATIO) {
        /*
          this depth is well off the best we have managed.  Stepping one
          at a time from here just wanders, so go back to the depth that
          produced the best run and probe the other way from there
         */
        window = probe_best_window;
        probe_dir = -probe_dir;
    } else {
        if (probe_prev_rate > 0 && rate < probe_prev_rate * PROBE_WORSE_RATIO) {
            // the last step made things worse, so go back the other way
            probe_dir = -probe_dir;
        }
        window_step(probe_dir);
    }
    // let the best fade, so a depth which suited earlier conditions
    // does not pin the window there for the rest of the transfer
    probe_best_rate *= PROBE_BEST_DECAY;
    probe_prev_rate = rate;
    probe_run_chunks = 0;
    probe_run_start = now;
}

static void window_timeout(void)
{
    if (!opts.adaptive) {
        return;
    }
    if (opts.algo == WINDOW_AIMD) {
        good_streak = 0;
        window = window > 2 ? window / 2 : 1;
        return;
    }
    /*
      the probe controller deliberately does not react here.  A timeout
      on its own says nothing about the right depth - a busy bus times
      one out now and then even at its best depth - and treating each
      one as congestion is exactly what makes aimd oscillate.  A depth
      which really is too deep shows up as a slower run, and the timing
      above will turn around.
     */
}

enum TransferIDState {
    TID_FREE = 0,
    TID_INFLIGHT,
    TID_HELD,
};

static struct {
    TransferIDState state;
    int slot;
    double free_at;
} transfer_ids[TRANSFER_ID_COUNT];

/*
  allocate an unused transfer ID, or -1 if none are available.

  Allocation rotates rather than taking the lowest free ID: the
  receiving end treats a transfer whose ID repeats the previous one as
  a duplicate and discards it, so handing out 0, 0, 0 - which is what
  taking the lowest free ID does when only one request is in flight -
  means every request is ignored and only its retry gets through.
 */
static uint8_t transfer_id_next;

static int transfer_id_alloc(int slot)
{
    const double now = now_s();
    for (uint8_t i = 0; i < TRANSFER_ID_COUNT; i++) {
        if (transfer_ids[i].state == TID_HELD && now >= transfer_ids[i].free_at) {
            transfer_ids[i].state = TID_FREE;
        }
    }
    for (uint8_t n = 0; n < TRANSFER_ID_COUNT; n++) {
        const uint8_t i = (transfer_id_next + n) % TRANSFER_ID_COUNT;
        if (transfer_ids[i].state == TID_FREE) {
            transfer_ids[i].state = TID_INFLIGHT;
            transfer_ids[i].slot = slot;
            transfer_id_next = (i + 1) % TRANSFER_ID_COUNT;
            return i;
        }
    }
    return -1;
}

// state of the current download
static uint8_t *file_buf;
static bool *chunk_done;
static uint32_t num_chunks;
static uint32_t chunks_remaining;
static uint32_t next_chunk;
static uint32_t total_retries;
static uint32_t short_replies;
static uint32_t stale_replies;
static bool download_failed;
static uint64_t file_size;

// set once a reply proves the file ends before GetInfo said it would.
// See decode_read_response()
static bool size_unreliable;

// state of a simple (non pipelined) request
static Reply simple_reply;
static uint16_t simple_expect_id;

static void fill_pipeline(void);

// the lowest chunk we have not stored yet
static uint32_t lowest_missing_chunk(void)
{
    for (uint32_t c = 0; c < num_chunks; c++) {
        if (!chunk_done[c]) {
            return c;
        }
    }
    return num_chunks;
}

/*
  the file ends part way through this chunk: keep what came back and
  account for the chunks past the end, which will never be filled
 */
static void store_final_chunk(const CanardRxTransfer *transfer, uint32_t chunk,
                              uint16_t data_len, uint32_t bit_ofs)
{
    uint8_t *out = &file_buf[(uint64_t)chunk * FILE_READ_CHUNK];
    for (uint16_t i = 0; i < data_len; i++) {
        canardDecodeScalar(transfer, bit_ofs + 8U * i, 8, false, &out[i]);
    }
    file_size = (uint64_t)chunk * FILE_READ_CHUNK + data_len;
    for (uint32_t c = chunk; c < num_chunks; c++) {
        if (!chunk_done[c]) {
            chunk_done[c] = true;
            chunks_remaining--;
        }
    }
}

static void decode_read_response(const CanardRxTransfer *transfer, uint32_t chunk)
{
    int16_t err = 0;
    canardDecodeScalar(transfer, 0, 16, true, &err);
    if (err != 0) {
        fprintf(stderr, "read error %d at offset %u\n", err, (unsigned)(chunk * FILE_READ_CHUNK));
        download_failed = true;
        return;
    }
    uint16_t data_len;
    uint32_t bit_ofs;
    if (tao_active()) {
        // the trailing array runs to the end of the transfer
        data_len = transfer->payload_len > 2 ? transfer->payload_len - 2 : 0;
        bit_ofs = 16;
    } else {
        // explicit nine bit length prefix leaves the data unaligned
        uint16_t len = 0;
        canardDecodeScalar(transfer, 16, 9, false, &len);
        data_len = len;
        bit_ofs = 25;
    }
    if (data_len > FILE_READ_CHUNK) {
        data_len = FILE_READ_CHUNK;
    }
    /*
      every chunk but the last must be full.  A short reply here means
      the reply did not belong to this request - replies carry no
      offset, so a late reply whose transfer ID has been reused is
      indistinguishable until its length gives it away - so drop it and
      let the chunk be requested again
     */
    const uint64_t offset = (uint64_t)chunk * FILE_READ_CHUNK;
    const uint64_t expected = file_size - offset < FILE_READ_CHUNK ?
        file_size - offset : FILE_READ_CHUNK;
    if (data_len != expected) {
        short_replies++;
        /*
          a reply shorter than we asked for is either a file which ends
          before GetInfo said it would - the @SYS files report a fixed
          nominal size rather than their real one - or a stale reply
          from a reused transfer ID.  Replies carry no offset, so the
          two look alike while several reads are in flight.  Stop
          pipelining and ask again: with one read outstanding a short
          reply can only belong to the read we are waiting on, and the
          lowest chunk we still need is the only one we ask for
         */
        if (data_len < expected) {
            if (size_unreliable && chunk == lowest_missing_chunk()) {
                store_final_chunk(transfer, chunk, data_len, bit_ofs);
                return;
            }
            size_unreliable = true;
            opts.adaptive = false;
            opts.depth = 1;
            window = 1;
            // the chunk holding the end of the file is behind the
            // cursor, which only moves forward, so send it back
            next_chunk = 0;
        }
        return;
    }
    uint8_t *out = &file_buf[(uint64_t)chunk * FILE_READ_CHUNK];
    for (uint16_t i = 0; i < data_len; i++) {
        canardDecodeScalar(transfer, bit_ofs + 8U * i, 8, false, &out[i]);
    }
    if (!chunk_done[chunk]) {
        chunk_done[chunk] = true;
        chunks_remaining--;
    }
}

static void on_reception(CanardInstance *ins, CanardRxTransfer *transfer)
{
    (void)ins;
    if (transfer->data_type_id == simple_expect_id && simple_expect_id != 0) {
        simple_reply.got = true;
        simple_reply.payload_len = transfer->payload_len;
        const uint16_t n = transfer->payload_len < sizeof(simple_reply.payload) ?
            transfer->payload_len : (uint16_t)sizeof(simple_reply.payload);
        for (uint16_t i = 0; i < n; i++) {
            canardDecodeScalar(transfer, 8U * i, 8, false, &simple_reply.payload[i]);
        }
        return;
    }
    if (transfer->data_type_id != UAVCAN_PROTOCOL_FILE_READ_ID) {
        return;
    }
    const uint8_t tid = transfer->transfer_id & (TRANSFER_ID_COUNT - 1);
    if (transfer_ids[tid].state != TID_INFLIGHT) {
        // a reply to a request we abandoned
        stale_replies++;
        return;
    }
    const int slot = transfer_ids[tid].slot;
    if (slot < 0 || slot >= MAX_PIPELINE_DEPTH || !pending[slot].active || pending[slot].wire_tid != tid) {
        stale_replies++;
        return;
    }
    const uint32_t chunk = pending[slot].chunk;
    pending[slot].active = false;
    transfer_ids[tid].state = TID_FREE;
    window_reply_ok();
    decode_read_response(transfer, chunk);
    fill_pipeline();
}

static bool should_accept(const CanardInstance *ins, uint64_t *out_signature,
                          uint16_t data_type_id, CanardTransferType transfer_type,
                          uint8_t source_node_id)
{
    (void)ins;
    if (transfer_type != CanardTransferTypeResponse || source_node_id != opts.target_node) {
        return false;
    }
    switch (data_type_id) {
    case UAVCAN_PROTOCOL_FILE_READ_ID:
        *out_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_GETINFO_ID:
        *out_signature = UAVCAN_PROTOCOL_FILE_GETINFO_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_ID:
        *out_signature = UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_WRITE_ID:
        *out_signature = UAVCAN_PROTOCOL_FILE_WRITE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_DELETE_ID:
        *out_signature = UAVCAN_PROTOCOL_FILE_DELETE_SIGNATURE;
        return true;
    }
    return false;
}

static void send_request(uint64_t signature, uint8_t data_type_id, uint8_t *transfer_id,
                         const uint8_t *payload, uint16_t payload_len)
{
    // the optional arguments depend on how libcanard was configured
    const int16_t res = canardRequestOrRespond(&canard, opts.target_node,
                                               signature, data_type_id, transfer_id,
                                               CANARD_TRANSFER_PRIORITY_MEDIUM,
                                               CanardRequest, payload, payload_len
#if CANARD_ENABLE_DEADLINE
                                               , micros64() + 1000000U
#endif
#if CANARD_MULTI_IFACE
                                               , 0xFF
#endif
#if CANARD_ENABLE_CANFD
                                               , opts.canfd
#endif
                                               );
    if (res < 0) {
        fprintf(stderr, "failed to queue request: %d\n", res);
        exit(1);
    }
    pump_tx();
}

/*
  issue a request and wait for its reply; returns false once the
  attempts are used up.

  A reply can simply go missing - a frame dropped on the wire is
  answered by nobody - so what matters is asking again promptly rather
  than waiting a long time on each go.  Every request here can be sent
  again safely: a read has no side effect, and a write puts the same
  bytes at the same offset.
 */
static bool request_and_wait(uint64_t signature, uint8_t data_type_id,
                             const uint8_t *payload, uint16_t payload_len,
                             double timeout, Reply &reply, int attempts = 4)
{
    for (int attempt = 0; attempt < attempts; attempt++) {
        memset(&reply, 0, sizeof(reply));
        simple_reply = reply;
        simple_expect_id = data_type_id;
        send_request(signature, data_type_id, &simple_transfer_id, payload, payload_len);
        const double deadline = now_s() + timeout;
        while (now_s() < deadline) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(transport_fd, &rfds);
            struct timeval tv = {0, 2000};
            select(transport_fd + 1, &rfds, nullptr, nullptr, &tv);
            transport_pump_rx();
            pump_tx();
            if (simple_reply.got) {
                reply = simple_reply;
                simple_expect_id = 0;
                return true;
            }
        }
    }
    simple_expect_id = 0;
    return false;
}

/* ------------------------------------------------------------------ */
/* commands                                                            */
/* ------------------------------------------------------------------ */

struct FileInfo {
    bool ok;
    uint64_t size;
    bool is_directory;
    int16_t error;
};

static FileInfo cmd_getinfo(const char *path)
{
    FileInfo out {};
    uint8_t payload[1 + FILE_PATH_MAX];
    const uint16_t len = append_path(payload, 0, path);
    Reply reply;
    if (!request_and_wait(UAVCAN_PROTOCOL_FILE_GETINFO_SIGNATURE, UAVCAN_PROTOCOL_FILE_GETINFO_ID,
                          payload, len, 2.0, reply)) {
        fprintf(stderr, "GetInfo timed out for %s\n", path);
        return out;
    }
    // uint40 size, Error error, EntryType entry_type
    const uint64_t size = reply_uint(reply, 0, 5);
    const int16_t err = reply_int16(reply, 5);
    const uint8_t flags = (uint8_t)reply_uint(reply, 7, 1);
    out.error = err;
    out.ok = err == 0;
    out.size = size;
    out.is_directory = (flags & ENTRY_TYPE_FLAG_DIRECTORY) != 0;
    return out;
}

static int cmd_list(const char *dir)
{
    for (uint32_t index = 0; ; index++) {
        uint8_t payload[4 + 1 + FILE_PATH_MAX];
        for (int i = 0; i < 4; i++) {
            payload[i] = (index >> (8 * i)) & 0xFF;
        }
        const uint16_t len = append_path(payload, 4, dir);
        Reply reply;
        if (!request_and_wait(UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_SIGNATURE,
                              UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_ID,
                              payload, len, 2.0, reply)) {
            fprintf(stderr, "directory listing timed out at entry %u\n", index);
            return 1;
        }
        const int16_t err = reply_int16(reply, 0);
        const uint8_t flags = (uint8_t)reply_uint(reply, 2, 1);
        if (err != 0 || flags == 0) {
            break;                      // end of directory
        }
        uint16_t path_len;
        uint16_t path_ofs;
        if (tao_active()) {
            path_len = reply.payload_len > 3 ? reply.payload_len - 3 : 0;
            path_ofs = 3;
        } else {
            path_len = (uint16_t)reply_uint(reply, 3, 1);
            path_ofs = 4;
        }
        char path[FILE_PATH_MAX + 1] {};
        for (uint16_t i = 0; i < path_len && i < FILE_PATH_MAX; i++) {
            path[i] = (char)reply_uint(reply, path_ofs + i, 1);
        }
        const bool is_dir = (flags & ENTRY_TYPE_FLAG_DIRECTORY) != 0;
        if (is_dir) {
            printf("  %-40s <dir>\n", path);
        } else {
            const FileInfo info = cmd_getinfo(path);
            if (info.ok) {
                printf("  %-40s %10llu\n", path, (unsigned long long)info.size);
            } else {
                printf("  %-40s          ?\n", path);
            }
        }
    }
    return 0;
}

// returns false if no transfer ID is currently available
static bool send_read(uint32_t chunk, int slot)
{
    const int tid = transfer_id_alloc(slot);
    if (tid < 0) {
        return false;
    }
    uint8_t payload[5 + 1 + FILE_PATH_MAX];
    const uint64_t offset = (uint64_t)chunk * FILE_READ_CHUNK;
    for (int i = 0; i < 5; i++) {
        payload[i] = (offset >> (8 * i)) & 0xFF;
    }
    const uint16_t len = append_path(payload, 5, opts.remote_path);

    pending[slot].active = true;
    pending[slot].chunk = chunk;
    pending[slot].wire_tid = (uint8_t)tid;
    pending[slot].sent_t = now_s();
    uint8_t wire_tid = (uint8_t)tid;
    send_request(UAVCAN_PROTOCOL_FILE_READ_SIGNATURE, UAVCAN_PROTOCOL_FILE_READ_ID,
                 &wire_tid, payload, len);
    return true;
}

static int in_flight(void)
{
    int n = 0;
    for (int i = 0; i < MAX_PIPELINE_DEPTH; i++) {
        if (pending[i].active) {
            n++;
        }
    }
    return n;
}

static bool chunk_in_flight(uint32_t chunk)
{
    for (int i = 0; i < MAX_PIPELINE_DEPTH; i++) {
        if (pending[i].active && pending[i].chunk == chunk) {
            return true;
        }
    }
    return false;
}

/*
  the lowest chunk which is neither stored nor currently requested.
  The cursor is only a hint: a request which could not be issued, or
  one abandoned after too long, leaves a hole behind it, and every
  chunk must still be accounted for
 */
static bool next_needed_chunk(uint32_t *out)
{
    while (next_chunk < num_chunks &&
           (chunk_done[next_chunk] || chunk_in_flight(next_chunk))) {
        next_chunk++;
    }
    if (next_chunk < num_chunks) {
        *out = next_chunk;
        return true;
    }
    for (uint32_t c = 0; c < num_chunks; c++) {
        if (!chunk_done[c] && !chunk_in_flight(c)) {
            *out = c;
            return true;
        }
    }
    return false;
}

static void fill_pipeline(void)
{
    const int limit = opts.adaptive ? window : opts.depth;
    for (int i = 0; i < MAX_PIPELINE_DEPTH; i++) {
        if (in_flight() >= limit) {
            return;
        }
        if (pending[i].active) {
            continue;
        }
        uint32_t chunk;
        if (!next_needed_chunk(&chunk)) {
            return;             // everything is stored or in flight
        }
        pending[i].retries = 0;
        if (!send_read(chunk, i)) {
            return;             // no transfer ID free, try again later
        }
    }
}

/*
  upload a file.  Writes are sent one at a time: an upload is not the
  hot path here, and the node has to see them in order anyway
 */
static int cmd_put(const char *local_path, const char *remote_path)
{
    FILE *in = fopen(local_path, "rb");
    if (in == nullptr) {
        perror("fopen");
        return 1;
    }
    uint64_t offset = 0;
    const double t0 = now_s();
    while (true) {
        uint8_t chunk[FILE_WRITE_CHUNK];
        const size_t n = fread(chunk, 1, sizeof(chunk), in);
        uint8_t payload[5 + 1 + FILE_PATH_MAX + 1 + FILE_WRITE_CHUNK];
        uint16_t ofs = 0;
        for (int i = 0; i < 5; i++) {
            payload[ofs++] = (uint8_t)((offset >> (8 * i)) & 0xFF);
        }
        ofs = append_sized_path(payload, ofs, remote_path);
        if (!tao_active()) {
            payload[ofs++] = (uint8_t)n;
        }
        memcpy(&payload[ofs], chunk, n);
        ofs += (uint16_t)n;
        Reply reply;
        /*
          a node writing to a slow filesystem takes longer to answer
          than it does to read, so allow more than a read would; but
          keep it short enough that a dropped reply is asked for again
          quickly, and try plenty of times rather than few and slow
         */
        if (!request_and_wait(UAVCAN_PROTOCOL_FILE_WRITE_SIGNATURE, UAVCAN_PROTOCOL_FILE_WRITE_ID,
                              payload, ofs, 2.0, reply, 10)) {
            fprintf(stderr, "write timed out at offset %llu\n", (unsigned long long)offset);
            fclose(in);
            return 1;
        }
        const int16_t err = reply_int16(reply, 0);
        if (err != 0) {
            fprintf(stderr, "write error %d at offset %llu\n", err, (unsigned long long)offset);
            fclose(in);
            return 1;
        }
        offset += n;
        if (n == 0) {
            // an empty write is how the node is told the file is done
            break;
        }
    }
    fclose(in);
    const double dt = now_s() - t0;
    printf("%s -> %s: %llu bytes in %.2fs (%.0f bytes/sec)\n",
           local_path, remote_path, (unsigned long long)offset, dt,
           dt > 0 ? offset / dt : 0);
    return 0;
}

static int cmd_rm(const char *remote_path)
{
    uint8_t payload[1 + FILE_PATH_MAX];
    const uint16_t len = append_path(payload, 0, remote_path);
    Reply reply;
    if (!request_and_wait(UAVCAN_PROTOCOL_FILE_DELETE_SIGNATURE, UAVCAN_PROTOCOL_FILE_DELETE_ID,
                          payload, len, 2.0, reply)) {
        fprintf(stderr, "delete timed out for %s\n", remote_path);
        return 1;
    }
    const int16_t err = reply_int16(reply, 0);
    if (err != 0) {
        fprintf(stderr, "delete error %d for %s\n", err, remote_path);
        return 1;
    }
    printf("deleted %s\n", remote_path);
    return 0;
}

static int cmd_get(void)
{
    const FileInfo info = cmd_getinfo(opts.remote_path);
    if (!info.ok) {
        fprintf(stderr, "cannot stat %s (error %d)\n", opts.remote_path, info.error);
        return 1;
    }
    if (info.is_directory) {
        fprintf(stderr, "%s is a directory\n", opts.remote_path);
        return 1;
    }
    const uint64_t size = info.size;
    file_size = size;
    num_chunks = (uint32_t)((size + FILE_READ_CHUNK - 1) / FILE_READ_CHUNK);
    if (num_chunks == 0) {
        num_chunks = 1;
    }
    chunks_remaining = num_chunks;
    file_buf = (uint8_t *)calloc(num_chunks, FILE_READ_CHUNK);
    chunk_done = (bool *)calloc(num_chunks, sizeof(bool));
    if (file_buf == nullptr || chunk_done == nullptr) {
        fprintf(stderr, "out of memory for %llu bytes\n", (unsigned long long)size);
        return 1;
    }

    const double t0 = now_s();
    double last_cleanup = t0;
    double last_report = t0;
    fill_pipeline();
    while (chunks_remaining > 0 && !download_failed) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(transport_fd, &rfds);
        struct timeval tv = {0, 2000};
        select(transport_fd + 1, &rfds, nullptr, nullptr, &tv);
        transport_pump_rx();
        pump_tx();

        const double now = now_s();
        if (opts.verbose && now - last_report > 1.0) {
            last_report = now;
            fprintf(stderr, "  [%.0fs] window=%d inflight=%d remaining=%u cursor=%u retries=%u short=%u stale=%u\n",
                    now - t0, window, in_flight(), chunks_remaining, next_chunk,
                    total_retries, short_replies, stale_replies);
        }
        if (now - last_cleanup > 0.1) {
            canardCleanupStaleTransfers(&canard, micros64());
            last_cleanup = now;
        }
        for (int i = 0; i < MAX_PIPELINE_DEPTH; i++) {
            if (!pending[i].active || now - pending[i].sent_t <= 0.5) {
                continue;
            }
            if (++pending[i].retries > 8) {
                fprintf(stderr, "chunk at offset %llu timed out\n",
                        (unsigned long long)pending[i].chunk * FILE_READ_CHUNK);
                return 1;
            }
            total_retries++;
            window_timeout();
            // hold the abandoned ID back so a late reply cannot be
            // mistaken for the reply to a future request
            const uint8_t old_tid = pending[i].wire_tid;
            transfer_ids[old_tid].state = TID_HELD;
            transfer_ids[old_tid].free_at = now + TRANSFER_ID_HOLDOFF_S;
            const uint32_t chunk = pending[i].chunk;
            pending[i].active = false;
            send_read(chunk, i);
        }
        fill_pipeline();
    }
    if (download_failed) {
        return 1;
    }
    const double dt = now_s() - t0;

    FILE *out = fopen(opts.local_path, "wb");
    if (out == nullptr) {
        perror("fopen");
        return 1;
    }
    // file_size is the size we were told, less any truncation found
    // when the file turned out to end early
    const uint64_t written = file_size;
    if (fwrite(file_buf, 1, written, out) != written) {
        perror("fwrite");
        fclose(out);
        return 1;
    }
    fclose(out);
    printf("%s -> %s: %llu bytes in %.2fs (%.0f bytes/sec, %u retries, %u stale replies)\n",
           opts.remote_path, opts.local_path, (unsigned long long)written, dt,
           written / dt, total_retries, stale_replies + short_replies);
    if (opts.adaptive && opts.verbose) {
        printf("final window: %d\n", window);
    }
    return 0;
}

/* ------------------------------------------------------------------ */

static void usage(const char *prog)
{
    fprintf(stderr,
            "usage: %s [options] <uri> get <remote-path> [local-path]\n"
            "       %s [options] <uri> list <remote-dir>\n"
            "       %s [options] <uri> info <remote-path>\n"
            "       %s [options] <uri> put <local-path> <remote-path>\n"
            "       %s [options] <uri> rm <remote-path>\n"
            "\n"
            "uri:   /dev/ttyACM1            an SLCAN serial port\n"
            "       tcp:127.0.0.1:5772      an SLCAN port exposed over TCP\n"
            "       mcast:0                 SITL multicast CAN, bus 0\n"
            "\n"
            "options:\n"
            "  -n NODE   node ID to fetch from (required)\n"
            "  -N NODE   our own node ID (default 100)\n"
            "  -d DEPTH  fixed pipeline depth for reads, 1 to %u; without\n"
            "            this the depth adapts to how fast the node replies\n"
            "  -a ALGO   how the depth adapts, one of:\n"
            "              aimd   grow on good replies, halve on a timeout\n"
            "                     (default)\n"
            "              probe  experimental; time runs of chunks and follow\n"
            "                     whichever direction improves throughput\n"
            "  -f        use CANFD frames\n"
            "  -v        verbose\n",
            prog, prog, prog, prog, prog, MAX_PIPELINE_DEPTH);
}

int main(int argc, char **argv)
{
    int opt;
    while ((opt = getopt(argc, argv, "n:N:d:a:fvh")) != -1) {
        switch (opt) {
        case 'n':
            opts.target_node = (uint8_t)atoi(optarg);
            break;
        case 'N':
            opts.local_node = (uint8_t)atoi(optarg);
            break;
        case 'd':
            opts.depth = atoi(optarg);
            opts.adaptive = false;
            break;
        case 'a':
            if (strcmp(optarg, "probe") == 0) {
                opts.algo = WINDOW_PROBE;
            } else if (strcmp(optarg, "aimd") == 0) {
                opts.algo = WINDOW_AIMD;
            } else {
                fprintf(stderr, "unknown adaption algorithm %s\n", optarg);
                usage(argv[0]);
                return 1;
            }
            break;
        case 'f':
            opts.canfd = true;
            break;
        case 'v':
            opts.verbose = true;
            break;
        default:
            usage(argv[0]);
            return 1;
        }
    }
    if (argc - optind < 2 || opts.target_node == 0) {
        usage(argv[0]);
        return 1;
    }
    if (!opts.adaptive) {
        if (opts.depth < 1) {
            opts.depth = 1;
        }
        if (opts.depth > MAX_PIPELINE_DEPTH) {
            opts.depth = MAX_PIPELINE_DEPTH;
        }
    }
#if !CANARD_ENABLE_CANFD
    if (opts.canfd) {
        fprintf(stderr, "built without CANFD support\n");
        return 1;
    }
#endif

    opts.uri = argv[optind];
    const char *command = argv[optind + 1];
    const char *arg = argc - optind > 2 ? argv[optind + 2] : nullptr;

    transport_is_mcast = strncmp(opts.uri, "mcast:", 6) == 0;
    if (transport_is_mcast) {
        if (!mcast_init((uint8_t)atoi(opts.uri + 6))) {
            return 1;
        }
    } else if (strncmp(opts.uri, "tcp:", 4) == 0) {
        // an SLCAN port exposed over TCP
        char host[64];
        int port = 0;
        if (sscanf(opts.uri + 4, "%63[^:]:%d", host, &port) != 2) {
            fprintf(stderr, "expected tcp:ADDRESS:PORT\n");
            return 1;
        }
        transport_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (transport_fd < 0) {
            perror("socket");
            return 1;
        }
        struct sockaddr_in sa {};
        sa.sin_family = AF_INET;
        sa.sin_port = htons(port);
        sa.sin_addr.s_addr = inet_addr(host);
        if (connect(transport_fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
            perror("connect");
            return 1;
        }
        int one = 1;
        setsockopt(transport_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        fcntl(transport_fd, F_SETFL, fcntl(transport_fd, F_GETFL, 0) | O_NONBLOCK);
        serial_write_all("\rC\rS8\rO\r", 8);
        usleep(200000);
    } else {
        transport_fd = open(opts.uri, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (transport_fd < 0) {
            perror("open");
            return 1;
        }
        struct termios t;
        tcgetattr(transport_fd, &t);
        cfmakeraw(&t);
        tcsetattr(transport_fd, TCSANOW, &t);
        serial_write_all("\rC\rS8\rO\r", 8);
        usleep(200000);
        tcflush(transport_fd, TCIFLUSH);
    }

    canardInit(&canard, canard_memory, sizeof(canard_memory), on_reception, should_accept, nullptr);
    canardSetLocalNodeID(&canard, opts.local_node);

    if (strcmp(command, "get") == 0) {
        if (arg == nullptr) {
            usage(argv[0]);
            return 1;
        }
        opts.remote_path = arg;
        const char *slash = strrchr(arg, '/');
        opts.local_path = argc - optind > 3 ? argv[optind + 3] : (slash ? slash + 1 : arg);
        return cmd_get();
    }
    if (strcmp(command, "list") == 0) {
        return cmd_list(arg != nullptr ? arg : "/");
    }
    if (strcmp(command, "put") == 0) {
        if (arg == nullptr || argc - optind < 4) {
            usage(argv[0]);
            return 1;
        }
        return cmd_put(arg, argv[optind + 3]);
    }

    if (strcmp(command, "rm") == 0) {
        if (arg == nullptr) {
            usage(argv[0]);
            return 1;
        }
        return cmd_rm(arg);
    }

    if (strcmp(command, "info") == 0) {
        if (arg == nullptr) {
            usage(argv[0]);
            return 1;
        }
        const FileInfo info = cmd_getinfo(arg);
        if (!info.ok) {
            fprintf(stderr, "error %d\n", info.error);
            return 1;
        }
        printf("%s: %llu bytes%s\n", arg, (unsigned long long)info.size,
               info.is_directory ? " (directory)" : "");
        return 0;
    }
    usage(argv[0]);
    return 1;
}

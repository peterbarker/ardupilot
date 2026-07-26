# File transfer benchmarks

Reference throughput figures for the uavcan.protocol.file services, for
use as hardware regression baselines. A change which moves these
materially wants explaining.

These are all real hardware. SITL numbers are not comparable and are not
recorded here.

## Bench

Measured 2026-07-26.

| | |
|---|---|
| Serving node | Holybro Pixhawk6C, STM32H743, node 12, ArduCopter branch build |
| Bridge | ZeroOne X6, STM32H743, node 10, SLCAN on SERIAL8 (`-if02`) |
| Client | `dronecan_file_client` on the host, over the bridge's SLCAN port |
| Bus | CAN1, 1 Mbit arbitration, 8 Mbit FD data phase, 120R both ends |
| File | `/APM/LOGS/00000002.BIN`, 841859 bytes (3290 chunks of 256) |

Topology is host → SLCAN → X6 → **CAN wire** → Pixhawk6C. The serving
node is genuinely remote; the wire is in the path.

Both boards on `CAN_D1_UC_OPTION` 2308 for the CANFD runs and 2304 for
the classic runs. Note `CAN_P1_FDBITRATE` defaults differ per board -
see the warning at the bottom.

## CANFD, 8 Mbit data phase

| depth | throughput | retries |
|---|---|---|
| 1 | 53.5 kB/s | 0 |
| 2 | 98.7 kB/s | 0 |
| 4 | 151.8 kB/s | 0 |
| 8 | 179.5 kB/s | 0 |
| 16 | **184.8 kB/s** | 1 |
| adaptive (aimd) | 182.6 kB/s | 0 |

Sustained, 7491584 byte file, adaptive: **182.5 kB/s over 41.0 s**, 35
retries, 0 stale replies. The result parses as a valid dataflash log
(182568 messages, 58 types).

## Classic CAN, 1 Mbit

| depth | throughput | retries |
|---|---|---|
| 1 | 24.9 kB/s | 0 |
| 2 | 31.9 kB/s | 23 |
| 4 | **36.2 kB/s** | 92 |
| 8 | 27.7 kB/s | 266 |
| 16 | 22.5 kB/s | 340 |
| adaptive (aimd) | 22.8 kB/s | 107 |

The theoretical ceiling for classic CAN here is about 47 kB/s - a 256
byte chunk is 43 frames of roughly 125 bits at 1 Mbit - so depth 4
reaches about 77% of the wire.

**CANFD is roughly 5x classic** on this bench, 185 against 36 kB/s.

## Pipeline depth is not one-size-fits-all

Classic peaks at **depth 4** and collapses beyond it: past that the
window overruns the buffers on the wire and retried work costs more
than the extra concurrency gains. CANFD peaks at **depth 16** and is
flat from 8 upwards.

It looked at first as though the host SLCAN link was what capped CANFD
at around 185 kB/s. **It is not.** The in-firmware client reaches a
similar figure over the wire with no serial link anywhere in the path,
so what both are hitting is a per-chunk cost shared by the two nodes.
See "Where the time goes" below.

## Adaptive controllers

`-a` selects how the depth adapts. Repeated runs, same file:

| | classic | CANFD |
|---|---|---|
| `aimd` (default) | 23.8 / 22.4 / 23.4 kB/s | 182.6 kB/s |
| `probe` (experimental) | 27.9 / 23.1 / 29.3 kB/s | not yet measured |
| best fixed depth | **35.9 / 35.6 / 35.9 kB/s** (d4) | 184.8 kB/s (d16) |

**Known deficiency: both adaptive controllers are well short of a
well-chosen fixed depth on classic CAN** - `aimd` returns about 64% of
it, `probe` about 75%. Neither is a problem on CANFD, where `aimd`
reaches 99% of the best fixed depth.

The cause is that `aimd` treats every timeout as congestion and halves
the window. On a bus where the CAN wire is the constraint, timeouts
happen even at the best depth - depth 4 above still shows 92 of them -
so the window never settles: it climbs until throughput collapses,
halves to below the best depth, and climbs again. On CANFD the wire is
not the constraint, timeouts are rare, and the window simply climbs to
its cap and stays there, which is why it looks good.

`probe` times runs of chunks and steps the depth in whichever direction
improved throughput. It beats `aimd` on classic but still wanders,
because a single retry stall badly distorts a short measurement run.
It is left in as an option, not as the default, until someone does this
properly.

**If you care about throughput on a classic bus, pass `-d 4`.**

## The other two clients

Same bench, same file, same classic bus, so these are directly
comparable with the table above.

| client | throughput | reads in flight |
|---|---|---|
| this tool, `-d 4` | 36.2 kB/s | 4 |
| this tool, `-d 1` | 24.9 kB/s | 1 |
| `DroneCAN_log_download.lua` | 27.5 kB/s | 1 |
| dronecan_gui_tool | 10.9 kB/s | adaptive |

The Lua applet keeps a single read outstanding, and it lands within a
few percent of this tool held at depth 1. **The cost of the applet is
the serialised read, not the fact that it is Lua** - which is worth
knowing before anyone tries to make it faster by rewriting it in C++.
Pipelining it is the change that would pay.

dronecan_gui_tool is the slowest of the three despite pipelining,
which matches the per-frame cost of pydronecan seen elsewhere on this
branch. It is a GUI for occasional use rather than a bulk transfer
tool, so this is not alarming, but it does mean gui_tool timings
should not be used to judge the firmware.

Lua figure: timed across the transfer alone, X6 as client fetching
from node 12; the same script over CANFD reaches 35.2 kB/s, where this
tool is limited by the host SLCAN link rather than the bus. gui_tool figure: 841859 bytes, 77.5 and 76.9 s over two runs,
driven headlessly through the real `FileDownloadWindow` widget over
the same SLCAN port, output byte-identical to this tool's.

## The in-firmware client

`AP_DroneCAN_FileClient` fetches node to node with no serial link in
the path, eight reads in flight. Measured X6 fetching from the
Pixhawk6C over CANFD, 7491584 byte file:

| | throughput |
|---|---|
| fetching, writing the file locally | 113 kB/s |
| the same with the local write skipped | 167 kB/s |

That is about 3x what the Lua applet manages on the same link, which
was the point of writing it. It is *not* faster than the host tool
over SLCAN, which was the surprise.

## Where the time goes

Both ends are ours, so each end's filesystem can be taken out in turn:
an empty local path throws the data away as it arrives, and
`@SYS/storage.bin` on the far node is RAM backed. Depth 8, CANFD,
timing the transfer alone:

| server reads from | client writes to | throughput | per 256 byte chunk |
|---|---|---|---|
| SD | SD | 114 kB/s | 2.24 ms |
| SD | nothing | 167 kB/s | 1.53 ms |
| RAM | SD | 128 kB/s | 1.99 ms |
| RAM | nothing | **212 kB/s** | **1.21 ms** |

Subtracting across those:

| | ms per chunk | share of a normal transfer |
|---|---|---|
| client writing the chunk | ~0.75 | 33% |
| server reading the chunk | ~0.29 | 13% |
| everything else | ~1.21 | 54% |

The client's write is the largest single cost and the easiest to fix:
it seeks and writes for every 256 bytes, because replies can arrive
out of order. Buffering runs of chunks would recover most of it.

**The remaining 1.21 ms per chunk is not the bus and not the
filesystem, and pipelining does not hide it** - going from 8 reads in
flight to 16 gained under 5%. It is a per-chunk cost paid somewhere in
the two nodes' DroneCAN handling, and it is the same cost the host
tool runs into at 185 kB/s over SLCAN.

How much of the bus that leaves is worth knowing. The frame counters
in `@SYS/canN_stats.txt` on the serving node moved by 105691 sent and
22036 received over about thirty seconds of a transfer, near enough
4250 frames a second, which agrees with six frames per chunk at the
measured chunk rate. A full 64 byte CANFD frame here is roughly 107us
- about 35us of arbitration at 1Mbit and 69us of data at 8Mbit - so a
chunk occupies the wire for around 585us, and the fastest case above
works out at close to half the bus.

So the bus is neither the constraint nor irrelevant: there is
somewhere around a factor of two of headroom, not the factor of four
the round trip figure on its own suggests. That last sentence rests on
calculated frame timings rather than a measured saturation point; a
broadcast which simply streams as fast as the driver accepts would
replace the arithmetic with a number.

Two caveats on the RAM figures: `@SYS/storage.bin` is only 16 kbyte,
so it was fetched forty times over and each fetch pays a fresh open
which regenerates the buffer - 212 kB/s is if anything an
underestimate of what the protocol alone would manage.

## What the link actually carries

Both ends being ours, the request and response can be taken out of it
altogether: have one node broadcast as fast as the driver accepts and
count the frames arriving at the other. Nothing subscribes, so the
receiving interpreter is not in the measurement either. Pixhawk6C
sending, X6 receiving, CANFD:

| payload | transfers/s | throughput | per transfer |
|---|---|---|---|
| 64 B | 4940 | 316 kB/s | 202 us |
| 256 B | 1691 | 433 kB/s | 591 us |
| 512 B | 888 | 455 kB/s | 1126 us |
| 1000 B | 469 | **468 kB/s** | 2132 us |

Those fit a straight line closely: about **70us fixed per transfer and
2.06us per byte**. Predicting the two middle rows from the outer two
gives 598us and 1126us against 591 and 1126 measured.

At the top of that table the receiving driver counted 8120 frames a
second, which at roughly 107us of airtime a frame is **87% of the
bus**. So 468 kB/s is close to what this wire will carry, and the
2.06us per byte is the wire rather than the software.

## Reading the file transfer against it

Comparing like with like - a 256 byte payload either way:

| | per 256 bytes | throughput | of the link |
|---|---|---|---|
| broadcast, no round trip | 591 us | 433 kB/s | 100% |
| file transfer, no filesystem at either end | 1210 us | 212 kB/s | 49% |
| file transfer as it normally runs | 2240 us | 114 kB/s | 26% |

So a chunk costs 619us more than simply sending those bytes. Some of
that is real: a Read also sends a request, worth about 150us of wire
and fixed cost. The remaining ~470us is the round trip itself, and it
is the part pipelining ought to hide but does not.

Putting the whole 2.24ms of a normal chunk together:

| | us | share |
|---|---|---|
| client writing to its SD card | 750 | 33% |
| data bytes on the wire | 530 | 24% |
| server reading from its SD card | 290 | 13% |
| the request transfer | 150 | 7% |
| round trip not hidden by pipelining | 520 | 23% |

Only the second row is unavoidable at this chunk size. The first is a
seek and a write per 256 bytes and should mostly go away with
buffering; the last is worth understanding before anything else is
optimised.

## MAVCAN

Measured 2026-08-03, same two boards, using pydronecan's `mavcan`
driver over each board's first USB MAVLink interface (256 byte
chunks, fixed pipeline depth).  The `dronecan_file_client` tool does
not speak MAVCAN; use dronecan_gui_tool or pydronecan.

| path | framing | depth: rate |
|---|---|---|
| Pixhawk6C serving itself over its own USB | classic | 2: 18.6, 4: **25.5**, 8: 24.6 kB/s |
| Pixhawk6C serving itself | CANFD | 4: 46.5, 8: 80.5, 16: 108, 24: **144.8** kB/s |
| node 12 through the X6's tunnel and the wire | classic | 2: 10.1, 4: 10.4, 8: **11.5** kB/s |
| node 12 through the tunnel and the wire | CANFD | 4: 45.9, 8: **53.9** kB/s |

What the numbers say:

- **Ask for CANFD on the tunnel.**  A 256 byte chunk is 5
  `CANFD_FRAME` messages rather than 38 `CAN_FRAME` ones, and the
  cost is per message, not per byte: FD is 4-6x faster end to end.
  pydronecan takes `send_canfd=True`; the server answers in the
  framing it was asked in.
- Classic tops out near depth 4 and FD near depth 24; **depth 30
  collapsed** (88 kB/s, retries) as the window closes on the five bit
  transfer-ID space.  A client pipelining that deep needs the owned
  transfer-ID discipline this tool uses for SLCAN.
- The wire hop costs about half the rate (144.8 self against 53.9
  remote at the same framing): each request is store-and-forwarded
  through the bridge at ~20ms plus ~1ms per frame, so FD helps twice.
- Forwarding ownership is a single global slot re-claimed at 1Hz;
  a second MAVCAN client steals it and the two fight.  A depth 8 FD
  transfer survived three deliberate steals at full rate with zero
  retries - the forwarding queue is flushed and generation-stamped
  across ownership changes - but two clients working the same
  autopilot still halve each other's service, so don't.

## Reproducing

```
# list what the remote node has
dronecan_file_client -n 12 -f /dev/serial/by-id/usb-ArduPilot_ZeroOneX6_...-if02 list /APM/LOGS

# one depth
dronecan_file_client -n 12 -f -d 4 <bridge-slcan> get /APM/LOGS/00000002.BIN out.bin

# drop -f for classic; -a probe|aimd to pick the controller; -v to
# watch the window move
```

The node-to-node figures, and the streaming ones above, come from two
scripts kept beside this file's numbers rather than in it:

```
libraries/AP_Scripting/tests/dronecan_file_bench.lua    # what limits a fetch
libraries/AP_Scripting/tests/dronecan_stream_bench.lua  # what the link carries
```

with dronecan_bench.md next to them for how to drive each one.

Confirm the bus is healthy before believing any number:

```
dronecan_file_client -n 12 -f <bridge-slcan> get @SYS/can0_stats.txt -
```

Want non-zero `tx_success` and `rx_received`, zero `rx_errors`, and no
bus-off. A non-zero but *falling* `ECR` is a recently cleared fault
rather than a live one - the transmit error counter decrements on each
success - so read it twice before drawing conclusions.

## Trap: CAN_Pn_FDBITRATE defaults differ between boards

The default is `HAL_CANFD_SUPPORTED`, which the hwdef generator sets to
**4 for every STM32H7 unless that board's hwdef overrides it**. Only a
handful do, mostly to 8. So a ZeroOne X6 defaults to 8 Mbit and a
Pixhawk6C to 4 Mbit, and with CANFD enabled on both they arbitrate
happily at 1 Mbit but cannot decode each other's data phase. The
symptom is two nodes which cannot see each other, with nothing pointing
at the bitrate. Check it first when a CANFD bus will not come up.

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
| Bridge | ZeroOne X6, STM32H743, node 10, SLCAN on SERIAL7 (`-if02`) |
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
at around 185 kB/s. **It is not.** Streaming over the same wire
reaches 468 kB/s with no request and response in the way, so the link
has plenty left; what caps a transfer is the cost of a chunk at each
end. See "What the link actually carries" below.

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

## Reproducing

```
# list what the remote node has
dronecan_file_client -n 12 -f /dev/serial/by-id/usb-ArduPilot_ZeroOneX6_...-if02 list /APM/LOGS

# one depth
dronecan_file_client -n 12 -f -d 4 <bridge-slcan> get /APM/LOGS/00000002.BIN out.bin

# drop -f for classic; -a probe|aimd to pick the controller; -v to
# watch the window move
```

The streaming figures above come from a script kept beside this
file's numbers rather than in it:

```
libraries/AP_Scripting/tests/dronecan_stream_bench.lua  # what the link carries
```

with dronecan_bench.md next to it for how to drive it.

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

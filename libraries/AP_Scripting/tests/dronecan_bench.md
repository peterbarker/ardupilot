# DroneCAN file transfer benchmarks

A script for finding out what a DroneCAN link carries.  It is not an
applet and is not useful in flight: it exists so the figures in
`libraries/AP_DroneCAN/tools/file_client/BENCHMARKS.md` can be
reproduced and re-checked when something changes.

It wants two nodes on a bus, both running ArduPilot.  Set
`SCR_ENABLE=1` and drop the script in the scripts directory as
usual. The figures below were taken between a ZeroOne X6 and a Holybro
Pixhawk6C, both STM32H743, over CANFD with an 8 Mbit data phase.

## Warning: the streaming bench disturbs other tools on the bus

`dronecan_stream_bench.lua` broadcasts a data type nothing else knows,
as fast as the wire will take it. Anything else listening on that bus
will try to make sense of those frames and fail: dronecan_gui_tool
raises a CRC mismatch on every one of them, and gives up altogether
after a thousand, which looks exactly like the tool crashing.

**Stop other tools on the bus, or accept that they will fall over.**
Set `DCST_RUN` back to zero when finished.

## dronecan_stream_bench.lua

Broadcasts as fast as the driver accepts, with nothing subscribed at
the far end. Runs on the *sending* node. This answers what the link
carries when there is no round trip at all, which is the number the
file transfer figures should be read against.

| parameter | meaning |
|---|---|
| `DCST_SIZE` | payload bytes per broadcast, 1 to 1000 |
| `DCST_RUN` | 1 to stream, 0 to stop |

The script reports what it managed every two seconds: `DCST: 1000B
x937 469/s 468.5kB/s`.

Nothing subscribes to the data type it sends, deliberately - the
frames still arrive and are counted by the receiving driver, so the
receiving interpreter is not in the measurement. To see the frame rate
rather than the byte rate, read `@SYS/can0_stats.txt` from the
receiving node before and after a run; `dronecan_file_client` in
`libraries/AP_DroneCAN/tools/file_client` will fetch it over the bus.

Sweeping `DCST_SIZE` separates the fixed cost of a transfer from the
cost of its bytes:

| payload | transfers/s | throughput |
|---|---|---|
| 64 B | 4940 | 316 kB/s |
| 256 B | 1691 | 433 kB/s |
| 512 B | 888 | 455 kB/s |
| 1000 B | 469 | 468 kB/s |

which fits about 70us per transfer and 2.06us per byte. At the top of
that sweep the receiving driver counted 8120 frames a second, near
enough 87% of the bus, so 468 kB/s is close to what the wire carries
rather than a software limit.

## Leaving the bench alone

The script starts work as soon as `DCST_RUN` is non-zero.  Set it back
to zero, or reboot, before walking away.

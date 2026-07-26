# DroneCAN file transfer benchmarks

Two scripts for finding out what limits a DroneCAN file transfer.
Neither is an applet and neither is useful in flight: they exist so
the figures in
`libraries/AP_DroneCAN/tools/file_client/BENCHMARKS.md` can be
reproduced and re-checked when something changes.

Both want two nodes on a bus, both running ArduPilot, with the file
server enabled on the far one - bit 11 (2048) of `CAN_Dn_UC_OPTION`.
Set `SCR_ENABLE=1` and drop the script in the scripts directory as
usual. The figures below were taken between a ZeroOne X6 and a Holybro
Pixhawk6C, both STM32H743, over CANFD with an 8 Mbit data phase.

## dronecan_file_bench.lua

Fetches a file with `AP_DroneCAN_FileClient`, with either end's
filesystem able to be taken out of the measurement. Runs on the
*fetching* node.

| parameter | meaning |
|---|---|
| `DCBM_NODE` | node to fetch from; cleared when the fetch ends |
| `DCBM_MODE` | 0 writes the file locally, 1 discards it as it arrives |
| `DCBM_SRC` | 0 fetches `REMOTE_PATH`, 1 fetches a RAM backed file |

`DCBM_MODE=1` passes an empty local path, which
`AP_DroneCAN_FileClient` takes as "throw the data away" - that removes
the local SD card. `DCBM_SRC=1` fetches `@SYS/storage.bin`, which is
RAM backed on the far node, so its SD card goes too; it is only 16
kbyte so it is fetched forty times over, and each of those pays a
fresh open, which makes that figure slightly pessimistic.

**`REMOTE_PATH` at the top of the script is hardcoded** to a log which
happened to be on the bench and will need changing for yours. Pick
something a few megabytes long so the transfer is long enough to time.

The result arrives as a STATUSTEXT: `DCBM: sd/write 7491584B 65.5s
114.4kB/s`.

Setting all four combinations gives, per 256 byte chunk, how much is
the client writing, how much is the server reading, and how much is
neither:

| server reads from | client writes to | measured |
|---|---|---|
| SD | SD | 114 kB/s |
| SD | nothing | 167 kB/s |
| RAM | SD | 128 kB/s |
| RAM | nothing | 212 kB/s |

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

Both scripts start work as soon as their node parameter is non-zero,
and `dronecan_file_bench.lua` will happily fill the local card. Set
`DCBM_NODE` or `DCST_RUN` back to zero, or reboot, before walking
away.

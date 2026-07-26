# DroneCAN Log Fetch

This script downloads dataflash logs from another node on a DroneCAN
bus. It decides what to fetch - listing the remote log directory and
skipping anything already here - and leaves the transfer itself to
AP_DroneCAN_FileClient, which is C++ and keeps several reads in flight.

DroneCAN_log_download.lua does the same job with the transfer written
in Lua. That one is simpler and needs nothing but the scripting
DroneCAN bindings; this one is about three times quicker. Both can be
installed at once - they use different parameters and save under
different names - which is how the figures below were compared.

The remote node must be running a DroneCAN file server:

- ArduPilot main firmware: set the EnableFileServer bit (bit 11) in
  CAN_Dn_UC_OPTION on the remote node
- AP_Periph: build with AP_PERIPH_FILE_SERVER_ENABLED

# Parameters

## DCLF_NODE

The node ID to fetch logs from. Setting this to a non-zero value
starts a fetch. The script collects all .BIN logs from the remote
node's log directory which are not already present locally, then
resets this parameter to zero.

## DCLF_CANDRV

DroneCAN driver index to use; 0 is the first driver.

## DCLF_DEBUG

Set to 1 for verbose progress output.

# Operation

Install the script in the scripts directory on the flight controller
and set SCR_ENABLE to 1.

To fetch logs, set DCLF_NODE to the node ID of the remote node. The
script probes for the remote log directory (/APM/LOGS on ChibiOS
boards, logs on SITL), lists it, and fetches each .BIN file to the
root of the local filesystem, named dcf_<node>_<name> - for example
log 00000012.BIN from node 12 is saved as dcf_12_00000012.BIN.

Files already fetched are skipped, so setting DCLF_NODE again only
collects new logs. Each file's size, time and rate are reported as
STATUSTEXT messages as it completes.

# Performance

Measured between two STM32H743 boards over CANFD with an 8 Mbit data
phase, a ZeroOne X6 fetching from a Holybro Pixhawk6C:

| | throughput |
|---|---|
| this script | 113 kbyte/s |
| DroneCAN_log_download.lua, same link | 35 kbyte/s |

That is worth putting against what the link itself carries. Streaming
broadcasts between the same two boards, with no request and response
in the way, reaches:

| payload | throughput |
|---|---|
| 256 bytes | 433 kbyte/s |
| 1000 bytes | 468 kbyte/s |

At the top of that the receiving driver counts 8120 frames a second,
about 87% of the bus, so 468 kbyte/s is close to what this wire will
carry rather than a software limit.

So this script runs at roughly a quarter of what the link can do. Of
the 2.24ms a 256 byte chunk costs, about 750us is writing the chunk to
the local SD card, 530us is the bytes on the wire, 290us is the remote
node reading it, and the rest is the round trip. Only the 530us is
unavoidable at this chunk size.

See BENCHMARKS.md alongside libraries/AP_DroneCAN/tools/file_client
for the full breakdown and for how to reproduce any of it.

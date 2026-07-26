# DroneCAN Log Download

This script downloads dataflash logs from another node on a DroneCAN
bus, using the standard uavcan.protocol.file services (GetInfo,
GetDirectoryEntryInfo and Read).

The remote node must be running a DroneCAN file server:

- ArduPilot main firmware: set the EnableFileServer bit (bit 11) in
  CAN_Dn_UC_OPTION on the remote node
- AP_Periph: build with AP_PERIPH_FILE_SERVER_ENABLED

# Parameters

## DCLD_NODE

The node ID to download logs from. Setting this to a non-zero value
starts a download. The script fetches all .BIN logs from the remote
node's log directory which are not already present locally, then
resets this parameter to zero.

## DCLD_CANDRV

DroneCAN driver index to use; 0 is the first driver.

## DCLD_DEBUG

Set to 1 for verbose progress output.

# Operation

Install the script in the scripts directory on the flight controller
and set SCR_ENABLE to 1.

To download logs, set DCLD_NODE to the node ID of the remote node.
The script probes for the remote log directory (/APM/LOGS on ChibiOS
boards, logs on SITL), lists it, and downloads each .BIN file to the
root of the local filesystem, named dcl_<node>_<name> - for example
log 00000012.BIN from node 125 is saved as dcl_125_00000012.BIN.

Files already downloaded (matching name and size) are skipped, so
setting DCLD_NODE again only fetches new logs.

Progress and completion are reported as STATUSTEXT messages.

Only one read is outstanding at a time, so the rate is set by the round
trip to the remote node rather than by the bus: 256 bytes are moved per
request, and a node which takes 50ms to answer will yield about 5
kbyte/s however fast the bus is.

Downloading from a simulated peripheral measures 4 to 6 kbyte/s.

Against real hardware, fetching from a Pixhawk6C with the script
running on a ZeroOne X6, timed across the transfer itself:

  classic CAN, 1Mbit    27 kbyte/s    9.3 ms per 256 byte read
  CANFD, 8Mbit          35 kbyte/s    7.3 ms per 256 byte read

Enumerating the remote directory first costs under a second and is not
included above.

Only one read is outstanding at a time, so what sets these figures is
the round trip, not the bus: at 256 bytes a read, 7.3 ms per round
trip is 35 kbyte/s however wide the bus is. Of that 7.3 ms about 2.8
ms is this script shifting the reply into place - CANFD carries an
explicit array length which leaves the data a bit out of step with the
bytes holding it - and about 4.5 ms is the round trip proper. Classic
frames need no such shifting but spend about 5 ms of every read
putting 43 frames on a 1Mbit wire, which is why they come out slower
despite the cheaper decode.

Pipelining reads would be the change that matters. Streaming
broadcasts between the same two boards, with no request and response
in the way, carries 433 kbyte/s at a 256 byte payload and 468 kbyte/s
at 1000 bytes - at which point the receiving driver counts 8120 frames
a second, about 87% of the bus. So this script runs at under a tenth
of what the link will carry, and the gap is round trips rather than
bandwidth.

Pipelining this script would need the scripting DroneCAN bindings to
allow more than one request outstanding on a handle, which they
presently do not.

That is within a few percent of what the C++ client in
libraries/AP_DroneCAN/tools/file_client manages when it is held to one
read at a time, so the limit is the serialised read rather than the
cost of running in Lua. The same client pipelining four reads reaches
36 kbyte/s on the same bus. See BENCHMARKS.md alongside that tool.

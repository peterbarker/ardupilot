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

Transfers use a single outstanding Read request at a time; expect
roughly 15-20 kbyte/s on a 1Mbit classic CAN bus.

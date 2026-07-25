# DroneCAN file client

A command line client for the `uavcan.protocol.file` services, for
listing and downloading files - notably dataflash logs - from a
DroneCAN node running a file server.

The server side is `AP_DroneCAN_FileServer`, enabled on a vehicle with
the `EnableFileServer` bit in `CAN_Dn_UC_OPTION`, or on an AP_Periph
build with `AP_PERIPH_FILE_SERVER_ENABLED`.

## Building

    make

The client links against the libcanard in `modules/DroneCAN` and has no
other dependencies. `make NOCANFD=1` builds without CANFD support.

## Usage

    dronecan_file_client [options] <uri> get <remote-path> [local-path]
    dronecan_file_client [options] <uri> list <remote-dir>
    dronecan_file_client [options] <uri> info <remote-path>

The uri is an SLCAN port - a serial device or the same protocol over
TCP - or a SITL multicast CAN bus:

    dronecan_file_client -n 10 /dev/ttyACM1 list /APM/LOGS
    dronecan_file_client -n 10 /dev/ttyACM1 get /APM/LOGS/00000001.BIN
    dronecan_file_client -n 10 mcast:0 get logs/00000001.BIN

Configure the autopilot for SLCAN first, for example on a second USB
interface:

    SERIAL8_PROTOCOL 22
    CAN_SLCAN_CPORT 1
    CAN_SLCAN_SERNUM 8
    CAN_SLCAN_TIMOUT 0

Options:

    -n NODE   node ID to fetch from (required)
    -N NODE   our own node ID (default 100)
    -d DEPTH  pipeline depth for reads, 1 to 30 (default 8)
    -f        use CANFD frames
    -v        verbose

## Speed

Reads are pipelined because each request and reply is a round trip; a
serialised client is several times slower. Depth 4 to 16 is usually
enough to saturate the far end, and deeper pipelines eventually make
throughput worse as replies begin to time out.

CANFD makes by far the largest difference, because a 256 byte reply
needs about five FD frames rather than 38 classic ones. Measured
against a ZeroOne X6 over SLCAN, downloading the same 290 kB file:

| frames  | depth | throughput  |
|---------|-------|-------------|
| classic | 1     | 33 kB/s     |
| classic | 4     | 57 kB/s     |
| CANFD   | 16    | 188 kB/s    |

Note that a physical 1 Mbit classic CAN bus tops out near 47 kB/s, so
those classic numbers are only reachable over a diagnostic link with no
cable attached.

## A note on transfer IDs

A Read reply carries no offset, so replies can only be matched to
requests by the five bit transfer ID on the wire. With only 32 IDs, a
late reply - one for a request we already gave up on - is
indistinguishable from the reply to the request issued 32 requests
later, and its data would be stored in the wrong place. This was
observed in practice as chunk N holding the data of chunk N+32.

The client therefore treats transfer IDs as owned: an ID is allocated
to a request and not reused until that request completes, and an ID
belonging to an abandoned request is held back for several seconds
before it can be issued again. Any client which pipelines these
services needs to do something equivalent.

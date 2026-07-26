#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <AP_Scripting/AP_Scripting_config.h>

/*
  client for the uavcan.protocol.file services, fetching files from
  another node on the bus into the local filesystem.

  OFF BY DEFAULT, and not to be turned on yet.  A reply is matched to
  its read by transfer ID, those are five bits, and every client of a
  service draws them from one pool - so an ID is handed out again
  after thirty two requests, tens of milliseconds at the rate these go
  out.  A read which is answered late, after its own read was given up
  on, is then taken for the reply to whichever read now holds that ID,
  and its data is written at that read's offset.  A transfer damaged
  that way still reports that it succeeded, which is the worst way for
  it to be wrong.

  Waiting quietly after abandoning a read narrows the window but does
  not close it.  Closing it means allocating the transfer IDs here,
  holding an abandoned one out of use until its reply cannot still
  arrive, which is what the host tool in
  libraries/AP_DroneCAN/tools/file_client does and which Canard::Client
  gives no way to do.
 */
#ifndef AP_DRONECAN_FILE_CLIENT_ENABLED
#define AP_DRONECAN_FILE_CLIENT_ENABLED 0
#endif

// how many reads to keep in flight.  A read costs a round trip, so a
// serialised client is limited by latency rather than by the bus; on a
// pair of H743s a round trip is around 4.5ms, which one read at a time
// turns into 55 kbyte/s however fast the bus runs
#ifndef AP_DRONECAN_FILE_CLIENT_DEPTH
#define AP_DRONECAN_FILE_CLIENT_DEPTH 8
#endif

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
  transport-agnostic server for the uavcan.protocol.file services,
  serving the local filesystem (via AP_Filesystem) to other nodes on a
  DroneCAN bus.  Users of this class are expected to decode the
  incoming service request, call the relevant handler to fill in a
  response structure, then encode and send that response.
 */
#pragma once

#include "AP_DroneCAN_FileServer_config.h"

#if AP_DRONECAN_FILE_SERVER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <dronecan_msgs.h>

class AP_DroneCAN_FileServer {
public:

    AP_DroneCAN_FileServer() {}

    CLASS_NO_COPY(AP_DroneCAN_FileServer);

    // fill in a response to a uavcan.protocol.file.Read request
    void handle_read_request(const uavcan_protocol_file_ReadRequest &req, uavcan_protocol_file_ReadResponse &rsp);

    // fill in a response to a uavcan.protocol.file.GetInfo request
    void handle_getinfo_request(const uavcan_protocol_file_GetInfoRequest &req, uavcan_protocol_file_GetInfoResponse &rsp);

    // fill in a response to a uavcan.protocol.file.GetDirectoryEntryInfo request
    void handle_getdirectoryentryinfo_request(const uavcan_protocol_file_GetDirectoryEntryInfoRequest &req, uavcan_protocol_file_GetDirectoryEntryInfoResponse &rsp);

    // fill in a response to a uavcan.protocol.file.Write request.
    // src_node_id identifies the writing client; only one node may
    // have an upload in progress at a time
    void handle_write_request(const uavcan_protocol_file_WriteRequest &req, uint8_t src_node_id, uavcan_protocol_file_WriteResponse &rsp);

    // fill in a response to a uavcan.protocol.file.Delete request
    void handle_delete_request(const uavcan_protocol_file_DeleteRequest &req, uavcan_protocol_file_DeleteResponse &rsp);

    // periodic housekeeping; closes the cached file descriptor if the
    // client has gone away mid-transfer
    void update();

private:

    // extract a nul-terminated path from a DSDL Path structure.
    // Returns false if the path does not fit or contains a nul
    static bool extract_path(const uavcan_protocol_file_Path &path, char *buf, uint16_t buflen);

    // map errno to a uavcan.protocol.file.Error value
    static int16_t errno_to_error();

    void close_cached_fd();
    void close_write_fd();

    // remove path and, if it is a directory, everything beneath it.
    // path is a scratch buffer of buflen bytes holding the target;
    // it is extended in place while recursing and restored before
    // returning.  Returns 0 on success, -1 with errno set on failure
    static int remove_tree(char *path, uint16_t buflen, uint8_t depth, uint16_t &budget);

    // true if filepath is dirpath itself or lies beneath it
    static bool path_is_or_is_under(const char *dirpath, const char *filepath);

    // true if the given path may be written or deleted; the
    // generated filesystems are not writeable
    static bool path_is_writeable(const char *path);

    // cached open file to avoid an open/seek/close cycle per 256-byte
    // read.  Only one file is served at a time; a Read for a
    // different path closes this one
    int fd = -1;
    uint32_t fd_ofs;
    char fd_path[201];  // maximum DSDL Path length plus nul terminator
    uint32_t last_read_ms;

    // separate cached file for writes, so an upload does not disturb
    // a concurrent download
    int wfd = -1;
    uint32_t wfd_ofs;
    char wfd_path[201];
    uint32_t last_write_ms;
    // the node whose upload this is; a competing writer is refused
    // rather than have two nodes interleave through one descriptor
    uint8_t wfd_node;
    // whether completion must truncate: false only while the upload
    // has run in order from offset zero, where O_TRUNC at open plus
    // the writes themselves already make the length exact
    bool wfd_needs_truncate;

    HAL_Semaphore sem;
};

#endif  // AP_DRONECAN_FILE_SERVER_ENABLED

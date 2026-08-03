#include "AP_DroneCAN_FileServer.h"

#if AP_DRONECAN_FILE_SERVER_ENABLED

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

#include <errno.h>
#include <stdio.h>

// close the cached file if no Read request has arrived recently
#define FILE_SERVER_IDLE_CLOSE_MS 3000

bool AP_DroneCAN_FileServer::extract_path(const uavcan_protocol_file_Path &path, char *buf, uint16_t buflen)
{
    if (path.path.len >= buflen) {
        return false;
    }
    memcpy(buf, path.path.data, path.path.len);
    buf[path.path.len] = 0;
    // reject paths containing embedded nuls:
    return strlen(buf) == path.path.len;
}

int16_t AP_DroneCAN_FileServer::errno_to_error()
{
    switch (errno) {
    case ENOENT:
        return UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND;
    case EIO:
        return UAVCAN_PROTOCOL_FILE_ERROR_IO_ERROR;
    case EACCES:
        return UAVCAN_PROTOCOL_FILE_ERROR_ACCESS_DENIED;
    case EISDIR:
        return UAVCAN_PROTOCOL_FILE_ERROR_IS_DIRECTORY;
    case EINVAL:
        return UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE;
    case EFBIG:
        return UAVCAN_PROTOCOL_FILE_ERROR_FILE_TOO_LARGE;
    case ENOSPC:
        return UAVCAN_PROTOCOL_FILE_ERROR_OUT_OF_SPACE;
    default:
        return UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR;
    }
}

void AP_DroneCAN_FileServer::close_cached_fd()
{
    if (fd != -1) {
        AP::FS().close(fd);
        fd = -1;
    }
}

void AP_DroneCAN_FileServer::handle_read_request(const uavcan_protocol_file_ReadRequest &req, uavcan_protocol_file_ReadResponse &rsp)
{
    char path[sizeof(fd_path)];
    if (!extract_path(req.path, path, sizeof(path))) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE;
        return;
    }

    WITH_SEMAPHORE(sem);

    if (req.offset > INT32_MAX) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_FILE_TOO_LARGE;
        return;
    }

    if (fd != -1 && strcmp(path, fd_path) != 0) {
        close_cached_fd();
    }
    if (fd == -1) {
        fd = AP::FS().open(path, O_RDONLY);
        if (fd == -1) {
            rsp.error.value = errno_to_error();
            return;
        }
        strncpy(fd_path, path, sizeof(fd_path)-1);
        fd_path[sizeof(fd_path)-1] = 0;
        fd_ofs = 0;
    }

    if (req.offset != fd_ofs) {
        if (AP::FS().lseek(fd, req.offset, SEEK_SET) == -1) {
            if (errno == EINVAL) {
                // the two backends differ here.  FATFS is the one
                // which follows POSIX: an offset at or past the end of
                // a file is legal, and the read which follows returns
                // no bytes.  @SYS deliberately departs from that and
                // refuses any offset past its end with EINVAL.  As far
                // as this protocol is concerned neither is an error -
                // an empty read is how a client is told where the file
                // ends - so answer the same way for both.
                rsp.data.len = 0;
                rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_OK;
                last_read_ms = AP_HAL::millis();
                return;
            }
            rsp.error.value = errno_to_error();
            close_cached_fd();
            return;
        }
        fd_ofs = req.offset;
    }

    const int32_t n = AP::FS().read(fd, rsp.data.data, sizeof(rsp.data.data));
    if (n < 0) {
        rsp.error.value = errno_to_error();
        close_cached_fd();
        return;
    }
    rsp.data.len = n;
    rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_OK;
    fd_ofs += n;
    last_read_ms = AP_HAL::millis();

    // a short read signals EOF to the client.  We leave the file open
    // for the idle timer to reap: a client with several reads in
    // flight will ask for offsets past the end, and reopening for each
    // of those would regenerate the content of a @SYS file, which both
    // costs time and can hand back a different snapshot each time.
}

void AP_DroneCAN_FileServer::handle_getinfo_request(const uavcan_protocol_file_GetInfoRequest &req, uavcan_protocol_file_GetInfoResponse &rsp)
{
    char path[sizeof(fd_path)];
    if (!extract_path(req.path, path, sizeof(path))) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE;
        return;
    }

    AP_Filesystem::stat_t st;
    if (!AP::FS().stat(path, st)) {
        rsp.error.value = errno_to_error();
        return;
    }
    rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_OK;
    if (st.is_directory()) {
        rsp.entry_type.flags = UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY | UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE;
        rsp.size = 0;
    } else {
        rsp.entry_type.flags = UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE | UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE;
        rsp.size = st.size;
    }
}

void AP_DroneCAN_FileServer::handle_getdirectoryentryinfo_request(const uavcan_protocol_file_GetDirectoryEntryInfoRequest &req, uavcan_protocol_file_GetDirectoryEntryInfoResponse &rsp)
{
    char path[sizeof(fd_path)];
    if (!extract_path(req.directory_path, path, sizeof(path))) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE;
        return;
    }

    auto *d = AP::FS().opendir(path);
    if (d == nullptr) {
        rsp.error.value = errno_to_error();
        return;
    }

    // no error unless we say otherwise; flags remaining zero indicates
    // no-such-entry to the client
    rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_OK;

    uint32_t index = 0;
    for (struct dirent *entry = AP::FS().readdir(d); entry != nullptr; entry = AP::FS().readdir(d)) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        if (index++ != req.entry_index) {
            continue;
        }
        char fullpath[sizeof(fd_path)];
        // avoid doubling the separator if the client passed a trailing one:
        const bool trailing_sep = (strlen(path) > 0 && path[strlen(path)-1] == '/');
        const int len = snprintf(fullpath, sizeof(fullpath), trailing_sep ? "%s%s" : "%s/%s", path, entry->d_name);
        if (len < 0 || len >= (int)sizeof(fullpath)) {
            rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE;
            break;
        }
        if (entry->d_type == DT_DIR) {
            rsp.entry_type.flags = UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY | UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE;
        } else {
            rsp.entry_type.flags = UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE | UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE;
        }
        rsp.entry_full_path.path.len = len;
        memcpy(rsp.entry_full_path.path.data, fullpath, len);
        break;
    }
    AP::FS().closedir(d);

    if (rsp.entry_type.flags == 0 && rsp.error.value == UAVCAN_PROTOCOL_FILE_ERROR_OK) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND;
    }
}

void AP_DroneCAN_FileServer::close_write_fd()
{
    if (wfd != -1) {
        AP::FS().close(wfd);
        wfd = -1;
    }
}

void AP_DroneCAN_FileServer::handle_write_request(const uavcan_protocol_file_WriteRequest &req, uint8_t src_node_id, uavcan_protocol_file_WriteResponse &rsp)
{
    char path[sizeof(wfd_path)];
    if (!extract_path(req.path, path, sizeof(path))) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE;
        return;
    }

    WITH_SEMAPHORE(sem);

    if (req.offset > INT32_MAX) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_FILE_TOO_LARGE;
        return;
    }

    // one upload at a time.  There is one cached descriptor, so a
    // second node writing - to the same path or any other - would
    // silently interleave with the first through it; refuse instead.
    // A wedged writer gives the slot up when the idle timer reaps
    // the descriptor
    if (wfd != -1 && src_node_id != wfd_node) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_ACCESS_DENIED;
        return;
    }

    if (wfd != -1 && strcmp(path, wfd_path) != 0) {
        close_write_fd();
    }

    if (req.data.len == 0) {
        // an empty write signals the upload is complete, and requires
        // us to truncate the file at the supplied offset.  An empty
        // write to a file we never opened is a zero-length upload or
        // a bare truncation; open the file so the same path serves
        if (wfd == -1) {
            wfd = AP::FS().open(path, O_WRONLY | O_CREAT);
            if (wfd == -1) {
                rsp.error.value = errno_to_error();
                return;
            }
            // the file's existing length is unknown here
            wfd_needs_truncate = true;
        }
        // a sequential upload from offset zero opened with O_TRUNC
        // and wrote every byte in order, so the file is already
        // exactly this long and there is nothing to truncate.  That
        // matters beyond economy: the virtual backends which apply an
        // upload when it is closed (@PARAM, @MISSION) support only
        // that shape and have no truncate, and must not draw an error
        // here for an upload the close below will apply
        if ((wfd_needs_truncate || req.offset != wfd_ofs) &&
            AP::FS().ftruncate(wfd, req.offset) == -1) {
            rsp.error.value = errno_to_error();
            close_write_fd();
            return;
        }
        close_write_fd();
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_OK;
        return;
    }

    if (wfd == -1) {
        // O_TRUNC when starting at offset zero so an upload shorter
        // than any existing content replaces it entirely
        const int flags = O_WRONLY | O_CREAT | (req.offset == 0 ? O_TRUNC : 0);
        wfd = AP::FS().open(path, flags);
        if (wfd == -1) {
            rsp.error.value = errno_to_error();
            return;
        }
        strncpy(wfd_path, path, sizeof(wfd_path)-1);
        wfd_path[sizeof(wfd_path)-1] = 0;
        wfd_ofs = 0;
        wfd_node = src_node_id;
        // beginning anywhere but zero leaves existing content in
        // place, which only a truncate at completion clears up
        wfd_needs_truncate = (req.offset != 0);
    }

    if (req.offset != wfd_ofs) {
        if (AP::FS().lseek(wfd, req.offset, SEEK_SET) == -1) {
            rsp.error.value = errno_to_error();
            close_write_fd();
            return;
        }
        wfd_ofs = req.offset;
        // no longer a straight run from zero; only a truncate at
        // completion makes the final length certain
        wfd_needs_truncate = true;
    }

    const int32_t n = AP::FS().write(wfd, req.data.data, req.data.len);
    if (n < 0) {
        rsp.error.value = errno_to_error();
        close_write_fd();
        return;
    }
    if ((uint16_t)n != req.data.len) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_OUT_OF_SPACE;
        close_write_fd();
        return;
    }
    wfd_ofs += n;
    last_write_ms = AP_HAL::millis();
    rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_OK;
}

void AP_DroneCAN_FileServer::handle_delete_request(const uavcan_protocol_file_DeleteRequest &req, uavcan_protocol_file_DeleteResponse &rsp)
{
    char path[sizeof(fd_path)];
    if (!extract_path(req.path, path, sizeof(path))) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE;
        return;
    }

    WITH_SEMAPHORE(sem);

    // drop any cached descriptor on the file being removed
    if (fd != -1 && strcmp(path, fd_path) == 0) {
        close_cached_fd();
    }
    if (wfd != -1 && strcmp(path, wfd_path) == 0) {
        close_write_fd();
    }

    // note: no recursive directory removal, unlike the DSDL
    // suggestion; a directory must be emptied first
    if (AP::FS().unlink(path) != 0) {
        rsp.error.value = errno_to_error();
        return;
    }
    rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_OK;
}

void AP_DroneCAN_FileServer::update()
{
    WITH_SEMAPHORE(sem);
    const uint32_t now_ms = AP_HAL::millis();
    if (fd != -1 && now_ms - last_read_ms > FILE_SERVER_IDLE_CLOSE_MS) {
        close_cached_fd();
    }
    if (wfd != -1 && now_ms - last_write_ms > FILE_SERVER_IDLE_CLOSE_MS) {
        close_write_fd();
    }
}

#endif  // AP_DRONECAN_FILE_SERVER_ENABLED

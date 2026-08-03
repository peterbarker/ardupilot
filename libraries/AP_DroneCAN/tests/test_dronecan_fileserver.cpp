/*
  tests for the uavcan.protocol.file server

  The server is deliberately transport agnostic - it fills in a
  response structure and knows nothing about how the request arrived -
  so it can be driven directly here, with no bus and no peripheral.
  That reaches the error paths, which are awkward to provoke over a
  real link.
 */
#include <AP_gtest.h>

#include <AP_DroneCAN_FileServer/AP_DroneCAN_FileServer.h>

#if AP_DRONECAN_FILE_SERVER_ENABLED

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

#include <string.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define TEST_DIR "dronecan_fileserver_test"

class FileServerTest : public ::testing::Test {
protected:
    AP_DroneCAN_FileServer server;

    // the node id most requests arrive from, and a competitor
    static constexpr uint8_t NODE_A = 10;
    static constexpr uint8_t NODE_B = 11;

    void SetUp() override {
        remove_test_dir();
        AP::FS().mkdir(TEST_DIR);
    }

    void TearDown() override {
        remove_test_dir();
    }

    void remove_test_dir() {
        // no recursive remove in AP_Filesystem, so name what we make
        static const char *names[] = {
            "hello.bin", "empty.bin", "written.bin", "doomed.bin", "big.bin",
        };
        for (const char *n : names) {
            AP::FS().unlink(path_in_dir(n));
        }
        AP::FS().unlink(TEST_DIR);
    }

    // returns a pointer to a static buffer; one at a time
    const char *path_in_dir(const char *name) {
        static char buf[128];
        snprintf(buf, sizeof(buf), "%s/%s", TEST_DIR, name);
        return buf;
    }

    void write_file(const char *name, const uint8_t *data, uint32_t len) {
        const int fd = AP::FS().open(path_in_dir(name), O_WRONLY | O_CREAT | O_TRUNC);
        ASSERT_NE(-1, fd);
        if (len > 0) {
            ASSERT_EQ((int32_t)len, AP::FS().write(fd, data, len));
        }
        AP::FS().close(fd);
    }

    // fill in a DSDL Path from a C string
    static void set_path(uavcan_protocol_file_Path &p, const char *s) {
        const size_t len = strlen(s);
        p.path.len = len;
        memcpy(p.path.data, s, len);
    }

    uavcan_protocol_file_ReadResponse read_at(const char *name, uint64_t offset) {
        uavcan_protocol_file_ReadRequest req {};
        req.offset = offset;
        set_path(req.path, path_in_dir(name));
        uavcan_protocol_file_ReadResponse rsp {};
        server.handle_read_request(req, rsp);
        return rsp;
    }
};

/*
  a plain read of a file shorter than one chunk
 */
TEST_F(FileServerTest, ReadWholeFile)
{
    const uint8_t content[] = {1, 2, 3, 4, 5};
    write_file("hello.bin", content, sizeof(content));

    const auto rsp = read_at("hello.bin", 0);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);
    ASSERT_EQ(sizeof(content), rsp.data.len);
    EXPECT_EQ(0, memcmp(content, rsp.data.data, sizeof(content)));
}

/*
  reading at the end of a file, and past it, both have to come back as
  a success carrying nothing.  That is how a client is told where the
  file ends; an error here stops a transfer which should have finished
 */
TEST_F(FileServerTest, ReadAtAndPastEndOfFile)
{
    const uint8_t content[] = {1, 2, 3, 4, 5};
    write_file("hello.bin", content, sizeof(content));

    auto rsp = read_at("hello.bin", sizeof(content));
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);
    EXPECT_EQ(0, rsp.data.len);

    rsp = read_at("hello.bin", sizeof(content) + 1000);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);
    EXPECT_EQ(0, rsp.data.len);
}

/*
  a read spanning more than one chunk comes back full, then short
 */
TEST_F(FileServerTest, ReadIsChunkedAndEndsShort)
{
    uint8_t big[600];
    for (uint16_t i = 0; i < sizeof(big); i++) {
        big[i] = (uint8_t)(i & 0xFF);
    }
    write_file("big.bin", big, sizeof(big));

    auto rsp = read_at("big.bin", 0);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);
    EXPECT_EQ(256, rsp.data.len);
    EXPECT_EQ(0, memcmp(big, rsp.data.data, 256));

    rsp = read_at("big.bin", 512);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);
    EXPECT_EQ(sizeof(big) - 512, rsp.data.len);
    EXPECT_EQ(0, memcmp(&big[512], rsp.data.data, sizeof(big) - 512));
}

/*
  the @SYS files are generated when opened and report a fixed nominal
  size rather than their real one, so a client reading to that size
  runs off the end.  That backend refuses the seek where a file on
  disk simply reads nothing, and the difference must not reach the
  client as an error - a transfer which should have ended would fail
  instead.

  This is the path a plain file cannot reach: POSIX is happy to seek
  past the end, so ReadAtAndPastEndOfFile above does not exercise it -
  disabling the EINVAL handling in the server leaves that test passing.

  @SYS is not generally readable from a bare test binary, so this
  usually skips.  What does cover the path is Copter.DroneCANFileClient,
  which fetches @SYS/uarts.txt from a running vehicle with a pipeline
  deep enough to be certain of reading past the end; that one does fail
  when the handling is removed.
 */
TEST_F(FileServerTest, ReadPastEndOfGeneratedFileIsEndOfFile)
{
    uavcan_protocol_file_ReadRequest req {};
    set_path(req.path, "@SYS/memory.txt");
    uavcan_protocol_file_ReadResponse rsp {};

    server.handle_read_request(req, rsp);
    if (rsp.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK || rsp.data.len == 0) {
        GTEST_SKIP() << "@SYS/memory.txt is not readable in this build";
    }

    // stat() reports a nominal size far beyond the real content, so a
    // client working from it asks well past the end
    req.offset = 90000;
    uavcan_protocol_file_ReadResponse past {};
    server.handle_read_request(req, past);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, past.error.value);
    EXPECT_EQ(0, past.data.len);
}

TEST_F(FileServerTest, ReadOfMissingFileIsNotFound)
{
    const auto rsp = read_at("nosuch.bin", 0);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND, rsp.error.value);
}

/*
  an offset which will not fit in the seek is refused rather than
  wrapped round into something valid
 */
TEST_F(FileServerTest, ReadBeyondSeekRangeIsRefused)
{
    const uint8_t content[] = {1, 2, 3};
    write_file("hello.bin", content, sizeof(content));

    const auto rsp = read_at("hello.bin", uint64_t(INT32_MAX) + 1);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_FILE_TOO_LARGE, rsp.error.value);
}

/*
  a Path carrying an embedded nul is not the path it appears to be, so
  it is refused rather than silently truncated
 */
TEST_F(FileServerTest, PathWithEmbeddedNulIsRejected)
{
    uavcan_protocol_file_ReadRequest req {};
    const char nasty[] = "a\0b";
    req.path.path.len = sizeof(nasty) - 1;
    memcpy(req.path.path.data, nasty, sizeof(nasty) - 1);
    uavcan_protocol_file_ReadResponse rsp {};
    server.handle_read_request(req, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE, rsp.error.value);
}

TEST_F(FileServerTest, GetInfoReportsSizeAndType)
{
    const uint8_t content[] = {1, 2, 3, 4, 5, 6, 7};
    write_file("hello.bin", content, sizeof(content));

    uavcan_protocol_file_GetInfoRequest req {};
    set_path(req.path, path_in_dir("hello.bin"));
    uavcan_protocol_file_GetInfoResponse rsp {};
    server.handle_getinfo_request(req, rsp);

    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);
    EXPECT_EQ(sizeof(content), rsp.size);
    EXPECT_TRUE(rsp.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE);
    EXPECT_FALSE(rsp.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY);
}

TEST_F(FileServerTest, GetInfoOnDirectorySaysDirectory)
{
    uavcan_protocol_file_GetInfoRequest req {};
    set_path(req.path, TEST_DIR);
    uavcan_protocol_file_GetInfoResponse rsp {};
    server.handle_getinfo_request(req, rsp);

    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);
    EXPECT_TRUE(rsp.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY);
}

TEST_F(FileServerTest, GetInfoOnMissingFileIsNotFound)
{
    uavcan_protocol_file_GetInfoRequest req {};
    set_path(req.path, path_in_dir("nosuch.bin"));
    uavcan_protocol_file_GetInfoResponse rsp {};
    server.handle_getinfo_request(req, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND, rsp.error.value);
}

/*
  a write, then the empty write which says the upload is over, and the
  file has to hold exactly what was sent
 */
TEST_F(FileServerTest, WriteThenCloseLeavesTheContent)
{
    const uint8_t content[] = {9, 8, 7, 6, 5, 4};

    uavcan_protocol_file_WriteRequest req {};
    set_path(req.path, path_in_dir("written.bin"));
    req.offset = 0;
    req.data.len = sizeof(content);
    memcpy(req.data.data, content, sizeof(content));
    uavcan_protocol_file_WriteResponse rsp {};
    server.handle_write_request(req, NODE_A, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);

    // an empty write closes the file
    req.offset = sizeof(content);
    req.data.len = 0;
    server.handle_write_request(req, NODE_A, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);

    const auto rd = read_at("written.bin", 0);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rd.error.value);
    ASSERT_EQ(sizeof(content), rd.data.len);
    EXPECT_EQ(0, memcmp(content, rd.data.data, sizeof(content)));
}

/*
  an upload which is shorter than what is already there replaces it
  entirely rather than leaving a tail of the old content behind
 */
TEST_F(FileServerTest, WriteFromZeroTruncates)
{
    const uint8_t old_content[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    write_file("written.bin", old_content, sizeof(old_content));

    const uint8_t fresh[] = {2, 2};
    uavcan_protocol_file_WriteRequest req {};
    set_path(req.path, path_in_dir("written.bin"));
    req.offset = 0;
    req.data.len = sizeof(fresh);
    memcpy(req.data.data, fresh, sizeof(fresh));
    uavcan_protocol_file_WriteResponse rsp {};
    server.handle_write_request(req, NODE_A, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);

    req.offset = sizeof(fresh);
    req.data.len = 0;
    server.handle_write_request(req, NODE_A, rsp);

    const auto rd = read_at("written.bin", 0);
    EXPECT_EQ(sizeof(fresh), rd.data.len);
}

/*
  an empty write to a file which was never opened is a zero length
  upload, and has to leave an empty file rather than nothing
 */
TEST_F(FileServerTest, EmptyUploadCreatesAnEmptyFile)
{
    uavcan_protocol_file_WriteRequest req {};
    set_path(req.path, path_in_dir("empty.bin"));
    req.offset = 0;
    req.data.len = 0;
    uavcan_protocol_file_WriteResponse rsp {};
    server.handle_write_request(req, NODE_A, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);

    const auto rd = read_at("empty.bin", 0);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rd.error.value);
    EXPECT_EQ(0, rd.data.len);
}

TEST_F(FileServerTest, DeleteRemovesTheFile)
{
    const uint8_t content[] = {1, 2, 3};
    write_file("doomed.bin", content, sizeof(content));

    uavcan_protocol_file_DeleteRequest req {};
    set_path(req.path, path_in_dir("doomed.bin"));
    uavcan_protocol_file_DeleteResponse rsp {};
    server.handle_delete_request(req, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);

    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND, read_at("doomed.bin", 0).error.value);
}

TEST_F(FileServerTest, DeleteOfMissingFileIsNotFound)
{
    uavcan_protocol_file_DeleteRequest req {};
    set_path(req.path, path_in_dir("nosuch.bin"));
    uavcan_protocol_file_DeleteResponse rsp {};
    server.handle_delete_request(req, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND, rsp.error.value);
}

AP_GTEST_MAIN()

#else

AP_GTEST_MAIN()

#endif  // AP_DRONECAN_FILE_SERVER_ENABLED

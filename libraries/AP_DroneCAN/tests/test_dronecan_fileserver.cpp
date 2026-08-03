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
        remove_tree(TEST_DIR);
    }

    // recursively remove a path so a failed test cannot leave litter
    // that upsets the next one
    static void remove_tree(const char *target) {
        if (AP::FS().unlink(target) == 0) {
            return;
        }
        auto *d = AP::FS().opendir(target);
        if (d == nullptr) {
            return;
        }
        char child[256];
        while (true) {
            child[0] = 0;
            for (struct dirent *entry = AP::FS().readdir(d); entry != nullptr; entry = AP::FS().readdir(d)) {
                if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                    continue;
                }
                // bounded so the compiler can see truncation is impossible
                snprintf(child, sizeof(child), "%.120s/%.120s", target, entry->d_name);
                break;
            }
            if (child[0] == 0) {
                break;
            }
            remove_tree(child);
            // rescan from the top; deleting while iterating is not
            // dependable
            AP::FS().closedir(d);
            d = AP::FS().opendir(target);
            if (d == nullptr) {
                return;
            }
        }
        AP::FS().closedir(d);
        AP::FS().unlink(target);
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

    // send one Write request and return the error it drew
    int16_t write_at(const char *name, uint64_t offset, const uint8_t *data, uint16_t len, uint8_t node = NODE_A) {
        uavcan_protocol_file_WriteRequest req {};
        set_path(req.path, path_in_dir(name));
        req.offset = offset;
        req.data.len = len;
        if (len > 0) {
            memcpy(req.data.data, data, len);
        }
        uavcan_protocol_file_WriteResponse rsp {};
        server.handle_write_request(req, node, rsp);
        return rsp.error.value;
    }

    // read the whole file back and check it holds exactly this content
    void expect_content(const char *name, const uint8_t *data, uint16_t len) {
        const auto rd = read_at(name, 0);
        EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rd.error.value);
        ASSERT_EQ(len, rd.data.len);
        if (len > 0) {
            EXPECT_EQ(0, memcmp(data, rd.data.data, len));
        }
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

/*
  resuming into an existing longer file and finishing early: the
  completing write carries the final size and everything beyond it
  must go.  Closing without truncating leaves a stale tail which reads
  back as part of the upload
 */
TEST_F(FileServerTest, WriteCompletionTruncatesAStaleTail)
{
    const uint8_t old_content[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    write_file("written.bin", old_content, sizeof(old_content));

    // patch two bytes in the middle, then declare the file complete
    // at six bytes
    const uint8_t patch[] = {7, 7};
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("written.bin", 4, patch, sizeof(patch)));
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("written.bin", 6, nullptr, 0));

    const uint8_t expected[] = {1, 1, 1, 1, 7, 7};
    expect_content("written.bin", expected, sizeof(expected));
}

/*
  chunks landing out of order - an offset four write before the offset
  zero one - must still leave exactly the declared bytes.  The first
  write not being at zero means no O_TRUNC, so this leans entirely on
  the completion truncating
 */
TEST_F(FileServerTest, WriteOutOfOrderChunksLeaveExactlyTheContent)
{
    const uint8_t old_content[] = {9, 9, 9, 9, 9, 9, 9, 9, 9};
    write_file("written.bin", old_content, sizeof(old_content));

    const uint8_t tail[] = {5, 6};
    const uint8_t head[] = {1, 2, 3, 4};
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("written.bin", 4, tail, sizeof(tail)));
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("written.bin", 0, head, sizeof(head)));
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("written.bin", 6, nullptr, 0));

    const uint8_t expected[] = {1, 2, 3, 4, 5, 6};
    expect_content("written.bin", expected, sizeof(expected));
}

/*
  a completion with no preceding writes is a bare truncation of an
  existing file, reaching the descriptor-less path
 */
TEST_F(FileServerTest, BareCompletionTruncatesAnExistingFile)
{
    const uint8_t old_content[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    write_file("written.bin", old_content, sizeof(old_content));

    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("written.bin", 3, nullptr, 0));

    expect_content("written.bin", old_content, 3);
}

/*
  a second node writing while an upload is in progress is refused
  rather than allowed to interleave through the shared descriptor;
  once the first upload completes the second node gets its turn
 */
TEST_F(FileServerTest, CompetingWriterIsRefusedUntilTheFirstFinishes)
{
    const uint8_t a[] = {1, 2, 3};
    const uint8_t b[] = {4, 5, 6};

    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("written.bin", 0, a, sizeof(a), NODE_A));
    // same path and another path both refused
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_ACCESS_DENIED, write_at("written.bin", 0, b, sizeof(b), NODE_B));
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_ACCESS_DENIED, write_at("other.bin", 0, b, sizeof(b), NODE_B));
    // the owner is unaffected
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("written.bin", sizeof(a), nullptr, 0, NODE_A));

    expect_content("written.bin", a, sizeof(a));

    // and with that upload complete the other node may write
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("other.bin", 0, b, sizeof(b), NODE_B));
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, write_at("other.bin", sizeof(b), nullptr, 0, NODE_B));
    expect_content("other.bin", b, sizeof(b));
}

/*
  deleting a directory removes what is beneath it, as the service
  contract requires
 */
TEST_F(FileServerTest, DeleteRemovesADirectoryTree)
{
    const uint8_t content[] = {1, 2, 3};
    ASSERT_EQ(0, AP::FS().mkdir(path_in_dir("sub")));
    write_file("sub/one.bin", content, sizeof(content));
    write_file("sub/two.bin", content, sizeof(content));
    ASSERT_EQ(0, AP::FS().mkdir(path_in_dir("sub/deeper")));
    write_file("sub/deeper/three.bin", content, sizeof(content));

    uavcan_protocol_file_DeleteRequest req {};
    set_path(req.path, path_in_dir("sub"));
    uavcan_protocol_file_DeleteResponse rsp {};
    server.handle_delete_request(req, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);

    AP_Filesystem::stat_t st;
    EXPECT_FALSE(AP::FS().stat(path_in_dir("sub"), st));
}

/*
  the recursion has a depth limit; a tree deeper than it draws an
  error rather than a runaway
 */
TEST_F(FileServerTest, DeleteOfTooDeepATreeIsRefused)
{
    char deep[128];
    snprintf(deep, sizeof(deep), "%s", path_in_dir("deep"));
    ASSERT_EQ(0, AP::FS().mkdir(deep));
    // nine levels below the target exceeds the limit of eight
    for (uint8_t i = 0; i < 9; i++) {
        const size_t len = strlen(deep);
        snprintf(&deep[len], sizeof(deep)-len, "/d%u", i);
        ASSERT_EQ(0, AP::FS().mkdir(deep));
    }

    uavcan_protocol_file_DeleteRequest req {};
    set_path(req.path, path_in_dir("deep"));
    uavcan_protocol_file_DeleteResponse rsp {};
    server.handle_delete_request(req, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE, rsp.error.value);
}

/*
  files a client may write have to say so; service discovery data
  which contradicts the server's own behaviour sends conforming
  clients away
 */
TEST_F(FileServerTest, GetInfoReportsWriteability)
{
    const uint8_t content[] = {1, 2, 3};
    write_file("hello.bin", content, sizeof(content));

    uavcan_protocol_file_GetInfoRequest req {};
    set_path(req.path, path_in_dir("hello.bin"));
    uavcan_protocol_file_GetInfoResponse rsp {};
    server.handle_getinfo_request(req, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_OK, rsp.error.value);
    EXPECT_TRUE(rsp.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE);
    EXPECT_TRUE(rsp.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_WRITEABLE);

    // the generated filesystems are not writeable and must not claim
    // to be
    uavcan_protocol_file_GetInfoRequest sysreq {};
    set_path(sysreq.path, "@SYS/memory.txt");
    uavcan_protocol_file_GetInfoResponse sysrsp {};
    server.handle_getinfo_request(sysreq, sysrsp);
    if (sysrsp.error.value == UAVCAN_PROTOCOL_FILE_ERROR_OK) {
        EXPECT_FALSE(sysrsp.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_WRITEABLE);
    }
}

/*
  one Delete request may only do so much work; a directory holding
  more entries than the budget draws an error instead of stalling the
  CAN thread for as long as the tree is deep and wide
 */
TEST_F(FileServerTest, DeleteOfTooManyEntriesIsRefused)
{
    ASSERT_EQ(0, AP::FS().mkdir(path_in_dir("many")));
    const uint8_t content[] = {1};
    for (uint8_t i = 0; i < 70; i++) {
        char name[32];
        snprintf(name, sizeof(name), "many/f%03u.bin", i);
        write_file(name, content, sizeof(content));
    }

    uavcan_protocol_file_DeleteRequest req {};
    set_path(req.path, path_in_dir("many"));
    uavcan_protocol_file_DeleteResponse rsp {};
    server.handle_delete_request(req, rsp);
    EXPECT_EQ(UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE, rsp.error.value);
}

AP_GTEST_MAIN()

#else

AP_GTEST_MAIN()

#endif  // AP_DRONECAN_FILE_SERVER_ENABLED

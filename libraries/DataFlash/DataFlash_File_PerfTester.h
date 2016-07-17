#ifndef _DATAFLASH_FILE_PERFTESTS_H
#define _DATAFLASH_FILE_PERFTESTS_H

#define DATAFLASH_FILE_PERFTESTER 1

#include <stdint.h>
#include "DataFlash_Backend.h"

#if DATAFLASH_FILE_PERFTESTER
class DataFlash_File_PerfTester {
public:

    DataFlash_File_PerfTester(class DataFlash_File &dataflash_file, const char *directory) :
        _dataflash_file(dataflash_file),
        _directory(directory)
        { }
    virtual ~DataFlash_File_PerfTester() { }
    bool run();

private:

    char *_result_file_name();
    bool _remove_results_file();
    int _open_result_fd();

    void _append_rate_result(uint32_t desired_kps, double actual_kps, uint32_t blocks, uint32_t bad_blocks);

    char *_ratetest_file_name(const uint16_t rate) const;

    // bool _run_write(const uint8_t *block, const uint16_t len, const uint16_t rate);
    // bool _run_writes();

    bool _run_logging_rate(DataFlash_Backend *logger, const uint8_t * const block, const uint16_t blocklen, const uint32_t desired_rate);
    bool _run_logging();

    void internal_error();

    // const uint32_t write_sample_size = 1 * 1024*1024;
    uint64_t each_test_duration = 20000000; // microseconds

    class DataFlash_File &_dataflash_file;
    const char *_directory;
};

#endif

#endif

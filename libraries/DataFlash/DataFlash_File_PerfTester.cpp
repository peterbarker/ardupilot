#include "DataFlash_File_PerfTester.h"

#include "stdio.h" // for asprintf
#include "stdlib.h" // for free
#include "unistd.h" // for unlink
#include "fcntl.h" // for O_CREAT and friends
#include "errno.h" // for errno global

#include <GCS_MAVLink/GCS.h> // for send_statustext_all
#include <AP_HAL/AP_HAL.h> 

#include "DataFlash_File.h"

extern const AP_HAL::HAL& hal;

/*
  construct a rate-test-output filepath given a rate
  Note: Caller must free.
 */
char *DataFlash_File_PerfTester::_ratetest_file_name(const uint16_t rate) const
{
    char *buf = NULL;
    if (asprintf(&buf, "%s/%u-kBps.DAT", _directory, (unsigned)rate) == 0) {
        return NULL;
    }
    return buf;
}

/*
  construct filepath to hold results of test
  Note: Caller must free.
 */
char *DataFlash_File_PerfTester::_result_file_name()
{
    char *buf = NULL;
    if (asprintf(&buf, "%s/DFRESULT.TXT", _directory) == 0) {
        return NULL;
    }
    return buf;
}

/*
  remove the result file
*/
bool DataFlash_File_PerfTester::_remove_results_file()
{
    char *filename = _result_file_name();
    if (filename == NULL) {
        // eep!
        return false;
    }
    if (unlink(filename) == -1) {
        if (errno != ENOENT) {
            return false;
        }
    }
    return true;
}

/*
  provide an open filehandle to the results file
*/
int DataFlash_File_PerfTester::_open_result_fd()
{
    char *filename = _result_file_name();
    if (filename == NULL) {
        free(filename);
        return -1;
    }
    int fd = open(filename, O_WRONLY|O_APPEND|O_CREAT, 0777);
    if (fd == -1) {
        hal.console->printf("Open (%s): %s\n", filename, strerror(errno));
    }
    free(filename);
    return fd;
}

/*
  Append a rate result to the results file
 */
void DataFlash_File_PerfTester::_append_rate_result(uint32_t desired_rate, double actual_rate, uint32_t blocks, uint32_t dropped_blocks)
{
    int fd = _open_result_fd();
    if (fd == -1) {
        return;
    }
    dprintf(fd, "desired-rate=%d actual-rate=%f blocks=%u dropped-blocks=%u\n",
            desired_rate, (double)actual_rate, blocks, dropped_blocks);
    close(fd);
}

/*
  Run a write test for a specific rate
*/
// bool DataFlash_File_PerfTester::_run_write(const uint8_t * const block, const uint16_t blocklen, const uint16_t desired_rate)
// {
//     bool ret = true;
//     char *readblock[blocklen];

//     // sync(); // not available on NuttX.  With a 1-block buffer it
//     // hardly matters

//     char *filename = _ratetest_file_name(desired_rate);
//     if (filename == NULL) {
//         return false;
//     }

//     int fd = open(filename, O_WRONLY|O_CREAT|O_TRUNC, 0777);
//     if (fd == -1) {
//         hal.console->printf("Open (%s): %s\n", filename, strerror(errno));
//         free(filename);
//         return false;
//     }

//     const uint32_t interval = 1000000/(desired_rate*1024);
//     const uint32_t count = write_sample_size/blocklen;

//     const uint64_t write_start_time = AP_HAL::micros64();
//     for(uint16_t i=0; i<count; i++) {
//         ssize_t res = write(fd, block, blocklen);
//         if (res == -1) {
//             hal.console->printf("Write (%s): %s\n", filename, strerror(errno));
//             ret = false;
//             break;
//         }

//         if (res < blocklen) {
//             // short write.  Should not happen with current
//             // implementation.  Ignore it and count the block as
//             // failed later
//         }

//         uint64_t sleep_til = write_start_time + i*interval;
//         uint64_t now = AP_HAL::micros64();
//         if (sleep_til < now) {
//             // write() is choking, apparently....
//         } else {
//             hal.scheduler->delay_microseconds(sleep_til - now);
//         }
//     }

//     const uint64_t write_stop_time = AP_HAL::micros64();

//     close(fd);

//     if (ret == false) {
//         unlink(filename);
//         free(filename);
//         return false;
//     }

//     // read back the file in block-sized chunks, make sure 
//     fd = open(filename, O_RDONLY);
//     if (fd == -1) {
//         hal.console->printf("Open (%s): %s\n", filename, strerror(errno));
//         free(filename);
//         return false;
//     }

//     uint32_t bad_blocks = 0;
//     for(uint16_t i=0; i<count; i++) {
//         ssize_t res = read(fd, readblock, blocklen);
//         if (res == -1) {
//             hal.console->printf("Read (%s): %s\n", filename, strerror(errno));
//             ret = false;
//             break;
//         }
//         if (res != blocklen) {
//             bad_blocks++; // ?!x
//             continue;
//         }
//         if (memcmp(readblock, block, blocklen)) {
//             bad_blocks++;
//         }
//     }

//     close(fd);

//     if (ret == true) {
//         // we assume stop_time != start_time (or, I guess, that
//         // division by zero is OK by everyone....)
//         const double actual_rate = ((double)count*blocklen)/(write_stop_time - write_start_time);
//         _append_rate_result(desired_rate, actual_rate, count, bad_blocks);
//     }

//     return ret;
// }

/*
  run write tests at various rates to test how many blocks actually
  make it out.  This is supposed to emulate the normal code writing at
  specific rates, which works with the curent ringbuffer
  implementation of DataFlash_File
*/
// bool DataFlash_File_PerfTester::_run_writes()
// {
//     uint8_t block[1024];

//     // fill the block with known data
//     for (uint16_t i=0; i< sizeof(block); i++) {
//         block[i] = i % 256;
//     }
    
//     // run write at e.g. 10k, 20k, 40k, ...
//     for (uint16_t i=1; i<=32; i<<=1) {
//         if (!_run_write(block, sizeof(block), i*10)) {
//             return false;
//         }
//     }
//     return true;
// }


enum {
    DFPLOG_LRTE_MESSAGE = 37
};

struct PACKED dfp_rate_Format {
    LOG_PACKET_HEADER;
    uint64_t TimeUS;
    uint32_t rate; // bytes/second
    uint64_t attempted;
    uint64_t dropped; // count of packets dropped
};

/*
  Run the logging system to see what happens...
 */
bool DataFlash_File_PerfTester::_run_logging_rate(DataFlash_Backend *logger, const uint8_t * const block, const uint16_t blocklen, const uint32_t desired_rate)
{

    const uint64_t interval = 1000000*blocklen / desired_rate;
    // const uint32_t count = write_sample_size/blocklen;

    _dataflash_file.start_new_log();

    // ensure the startup messages have all been set to go out:
    uint8_t loop_limit = 0;
    while (!_dataflash_file._startup_messagewriter->finished()) {
        if (loop_limit++ > 100) {
            // startup messages should not take too long to go out!
            internal_error();
            return false;
        }
        _dataflash_file.WriteMoreStartupMessages();
        // n.b. SITL scheduler is responsible for running IO, NOT a
        // separate thread.  This delay triggers IO for SITL:
        hal.scheduler->delay_microseconds(10000);
    }

    // ensure the ringbuffer is empty:
    _dataflash_file.flush();

    const uint64_t write_start_time = AP_HAL::micros64();

    uint32_t count = 0;
    uint32_t unwritten = 0;
    while (AP_HAL::micros64() - write_start_time < each_test_duration) {
        if (!_dataflash_file.WritePrioritisedBlock(block, blocklen, true)) {
            ::fprintf(stderr, "Lost block %d\n", count);
            unwritten++;
        }
        count++;

        uint64_t sleep_til = write_start_time + count*interval;
        uint64_t now = AP_HAL::micros64();
        if (sleep_til < now) {
            // write() is choking, apparently....
        } else {
            // ::fprintf(stderr, "interval=%ld now=%ld sleep_til=%ld sleeping %ld\n", interval, now, sleep_til, sleep_til - now);
            uint64_t delay = sleep_til - now;
            /* sigh.  Sometimes autpilots do need more than a little nap: */
            #define MAX_UINT16 65535
            while (delay > MAX_UINT16)  {
                hal.scheduler->delay_microseconds(MAX_UINT16);
                delay -= MAX_UINT16;
            }
            hal.scheduler->delay_microseconds(delay);
        }
    }

    const uint64_t write_stop_time = AP_HAL::micros64();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    _dataflash_file.flush();
#endif
    _dataflash_file.stop_logging();

    // we assume stop_time != start_time (or, I guess, that
    // division by zero is OK by everyone....)
    double actual_rate = ((double)count*blocklen*1000000)/(write_stop_time - write_start_time);
    _append_rate_result(desired_rate, actual_rate, count, unwritten);

    struct dfp_rate_Format results = {
        LOG_PACKET_HEADER_INIT(DFPLOG_LRTE_MESSAGE),
        TimeUS: AP_HAL::micros64(),
        rate: desired_rate,
        attempted: count,
        dropped: unwritten
    };

    ::fprintf(stderr, "Writing something to logger\n");
    if (!logger->WriteBlock(&results, sizeof(results))) {
        // FIXME: whinge, moan and groan here
        internal_error();
        return false;
    }

    return true;
}


void DataFlash_File_PerfTester::internal_error()
{
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "TEST: Error");
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        abort();
#endif
}


bool DataFlash_File_PerfTester::_run_logging()
{
    uint8_t block[1024];

    // fill the block with known data
    for (uint16_t i=0; i< sizeof(block); i++) {
        block[i] = i % 256;
    }

    // Create a logging object to output results
    const struct LogStructure _structures[] = {
        { LOG_FORMAT_MSG, sizeof(log_Format),
          "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns" },
        { DFPLOG_LRTE_MESSAGE, sizeof(dfp_rate_Format),
                    "LRTE", "QIQQ", "TimeUS,Rate,Attempted,Dropped" }
    };
    // avoid stack-allocating DataFlash_Class as it does not
    // initialise its members
    DataFlash_Class *logging_class = new DataFlash_Class("Bad Firmware String", false);
    if (logging_class == nullptr) {
        internal_error();
        return false;
    }
    logging_class->set_LogStructures(_structures, sizeof(_structures)/sizeof(_structures[0]));

    DFMessageWriter_WriteFormats formatwriter;
    // DataFlash_File does not init its own members; do not stack-allocate it
    DataFlash_File *logging_file = new DataFlash_File(*logging_class, &formatwriter, HAL_BOARD_LOG_DIRECTORY, 4096);
    if (logging_file == NULL) {
        delete logging_class;
        internal_error();
        return false;
    }
    logging_file->Init();
    if (!logging_file->initialised()) {
        delete logging_file;
        delete logging_class;
        return false;
    }
    logging_file->start_new_log();

    // run write at e.g. 1k, 2k, 4k, ...
    // for (uint16_t i=1; i<=512; i<<=1) {
    //     GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "TEST: Rate=%lukB/s", i*1024);
    //     if (!_run_logging_rate(logging_file, block, sizeof(block), i*1024)) {
    //         delete logging_file;
    //         delete logging_class;
    //         return false;
    //     }
    // }

    // find the highest rate this card can support
    uint16_t low = 1;
    uint16_t current = 256;
    uint16_t high = 512;
    uint32_t last_dropped_count = _dataflash_file.num_dropped();
    while (true) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "TEST: Rate=%lukB/s", current*1024);
        if (!_run_logging_rate(logging_file, block, sizeof(block), current*1024)) {
            delete logging_file;
            delete logging_class;
            return false;
        }
        uint32_t new_dropped = _dataflash_file.num_dropped();
        uint16_t new_current;
        if (new_dropped > last_dropped_count) {
            // we dropped packets; bisect down (watch for integer rounding)
            new_current = low + (current - low)/2;
        } else {
            // no packets dropped this round; bisect up
            new_current = high - (high-current)/2;
        }
        if (new_current == current) {
            // we've reached our result
            break;
        }
        current = new_current;
    }

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "TEST: Best good rate: %lukB/s", current);

    logging_file->stop_logging();
    delete logging_class;
    delete logging_file;

    return true;
}

/*
  Run a series of read and write tests against the SD card to get an
  idea of its speed and reliability
*/
bool DataFlash_File_PerfTester::run()
{
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "TEST: Start");
    if (!_remove_results_file()) {
        internal_error();
        return false;
    }
    // if (!_run_writes()) {
    //     GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "TEST: Error");
    //     return false;
    // }

    if (!_run_logging()) {
        internal_error();
        return false;
    }

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "TEST: Done");
    return true;
}

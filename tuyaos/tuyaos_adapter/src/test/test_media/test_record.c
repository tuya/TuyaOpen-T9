

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_fs.h"
#include "tkl_media_buf.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_thread.h"

#define TEST_REC_WRITER_STACK (4U * 1024U)
#define TEST_REC_POLL_MS      10U
/* Generous: one contiguous span can be the whole ring, and a slow card write is
 * still a write in progress, not a hang. */
#define TEST_REC_STOP_WAIT_MS 5000U

#define TEST_RECORD_PATH_MAX     128U
#define TEST_RECORD_TAG_MAX      12U
/** Board-measured peaks ranged 3.5%-26.6% of this ring with **no correlation to
 *  the data rate** - the peak is set by how long the card stalls, not by how
 *  fast the producer runs. Do NOT shrink it on the strength of a low reading:
 *  `dropped_frames` is the signal that matters, `peak_used` only says how much
 *  margin that one run happened to need. */
#define TEST_RECORD_RING_DEFAULT (2U * 1024U * 1024U)

typedef struct {
    BOOL_T   capturing;      /**< still accepting frames */
    BOOL_T   writer_running; /**< writer thread has not exited yet */
    UINT32_T queued_frames;  /**< frames copied into the ring */
    UINT32_T dropped_frames; /**< ring had no room for the whole frame */
    UINT32_T write_errors;
    UINT64_T written_bytes;
    UINT32_T peak_used;      /**< high-water bytes; == ring_bytes means it filled */
    UINT32_T ring_bytes;
} TEST_RECORD_STAT_T;

typedef struct {
    volatile bool capturing;
    volatile bool stop_req;
    volatile bool writer_running;
    volatile bool writer_done;
    volatile bool file_ready;
    volatile bool open_failed;
    TUYA_FILE file;
    TKL_THREAD_HANDLE writer_task;
    uint32_t deadline_ms;
    uint64_t written_bytes;
    uint32_t queued_frames;
    uint32_t dropped_frames;
    uint32_t write_errors;
    uint32_t peak_used;
    uint32_t ring_bytes;
    char tag[TEST_RECORD_TAG_MAX];
    char path[TEST_RECORD_PATH_MAX];
    uint8_t *ring;
    volatile uint32_t head; /**< write offset */
    volatile uint32_t tail; /**< read offset */
    volatile uint32_t used; /**< bytes in the ring */
} test_rec_t;

static test_rec_t s_rec;

/* mounts the card if it is not mounted yet; same helper ximg and xvenc use */
extern INT_T test_fs_mount(CONST CHAR_T *path, FS_DEV_TYPE_T dev_type);

/**
 * @brief Release the ring.
 */
static void test_record_ring_free(void)
{
    if (s_rec.ring != NULL) {
        tkl_media_buf_free(TKL_MEDIA_BUF_OWNER_TEST, s_rec.ring);
        s_rec.ring = NULL;
    }
}

/**
 * @brief Copy one frame into the ring. Runs on the producer's worker thread.
 * @param[in] data bitstream bytes, borrowed
 * @param[in] size byte count
 */
static void test_record_enqueue(const uint8_t *data, uint32_t size)
{
    uint32_t flags;
    uint32_t head;
    uint32_t first;

    if (size == 0U || size > s_rec.ring_bytes) {
        s_rec.dropped_frames++;
        return;
    }

    flags = rtos_disable_int();
    if ((s_rec.ring_bytes - s_rec.used) < size) {
        rtos_enable_int(flags);
        s_rec.dropped_frames++;
        return;
    }
    head = s_rec.head;
    /* conditional subtraction rather than %, now that the size is a runtime
     * value: size <= ring_bytes is guaranteed above, so one subtraction wraps */
    s_rec.head = head + size;
    if (s_rec.head >= s_rec.ring_bytes) {
        s_rec.head -= s_rec.ring_bytes;
    }
    s_rec.used += size;
    if (s_rec.used > s_rec.peak_used) {
        s_rec.peak_used = s_rec.used;
    }
    rtos_enable_int(flags);

    /* the reserved span is ours now; copying outside the critical section keeps
     * interrupts masked for a handful of instructions rather than a 7 KB memcpy */
    first = s_rec.ring_bytes - head;
    if (first >= size) {
        os_memcpy(&s_rec.ring[head], data, size);
    } else {
        os_memcpy(&s_rec.ring[head], data, first);
        os_memcpy(&s_rec.ring[0], data + first, size - first);
    }

    s_rec.queued_frames++;
}

VOID_T test_record_feed(CONST UINT8_T *data, UINT32_T size)
{
    if (s_rec.capturing != true || data == NULL) {
        return;
    }

    if ((uint32_t)tkl_system_get_millisecond() >= s_rec.deadline_ms) {
        /* stop producing; the writer drains what is left and closes */
        s_rec.capturing = false;
        s_rec.stop_req = true;
        return;
    }

    test_record_enqueue((const uint8_t *)data, (uint32_t)size);
}

/**
 * @brief Writer thread: own the file, drain the ring.
 * @param[in] arg unused
 */
static void test_record_writer_main(void *arg)
{
    TKL_THREAD_HANDLE self;

    (void)arg;

    if (test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD) != 0) {
        tkl_log_output("%s record: mount %s failed\r\n", s_rec.tag, TEST_SDCARD_MOUNT_POINT);
        s_rec.open_failed = true;
        goto done;
    }

    s_rec.file = tkl_fopen(s_rec.path, "wb");
    if (s_rec.file == NULL) {
        tkl_log_output("%s record open failed: %s\r\n", s_rec.tag, s_rec.path);
        s_rec.open_failed = true;
        goto done;
    }
    s_rec.file_ready = true;

    while (true) {
        uint32_t flags;
        uint32_t used;
        uint32_t tail;
        uint32_t span;

        flags = rtos_disable_int();
        used = s_rec.used;
        tail = s_rec.tail;
        rtos_enable_int(flags);

        if (used == 0U) {
            /* nothing buffered: leave only once production has stopped, so the
             * tail of the capture is not thrown away */
            if (s_rec.stop_req) {
                break;
            }
            tkl_system_sleep(TEST_REC_POLL_MS);
            continue;
        }

        /* as much as is contiguous - one call for up to the whole ring */
        span = s_rec.ring_bytes - tail;
        if (span > used) {
            span = used;
        }

        if (tkl_fwrite((VOID_T *)&s_rec.ring[tail], (INT_T)span, s_rec.file) != (INT_T)span) {
            s_rec.write_errors++;
        } else {
            s_rec.written_bytes += span;
        }

        flags = rtos_disable_int();
        s_rec.tail = tail + span;
        if (s_rec.tail >= s_rec.ring_bytes) {
            s_rec.tail -= s_rec.ring_bytes;
        }
        s_rec.used -= span;
        rtos_enable_int(flags);
    }

    (void)tkl_fsync((INT_T)s_rec.file);
    (void)tkl_fclose(s_rec.file);
    s_rec.file = NULL;

    tkl_log_output("%s record done: %s, %u frames, %llu bytes, %u dropped, "
                   "%u write errors, peak %uKB/%uKB\r\n",
                   s_rec.tag, s_rec.path,
                   (unsigned int)s_rec.queued_frames,
                   (unsigned long long)s_rec.written_bytes,
                   (unsigned int)s_rec.dropped_frames,
                   (unsigned int)s_rec.write_errors,
                   (unsigned int)(s_rec.peak_used / 1024U),
                   (unsigned int)(s_rec.ring_bytes / 1024U));

done:
    /* take the handle before publishing writer_done: once that flag is up,
     * test_record_stop() may return and a following start() memsets the struct */
    self = s_rec.writer_task;
    s_rec.capturing = false;
    s_rec.writer_running = false;
    s_rec.writer_done = true;
    tkl_thread_release(self);
}

OPERATE_RET test_record_start(CONST CHAR_T *tag, CONST CHAR_T *path, UINT32_T sec,
                              UINT32_T ring_bytes)
{
    OPERATE_RET ret;

    if (path == NULL || sec == 0U) {
        return OPRT_INVALID_PARM;
    }
    if (s_rec.capturing || s_rec.writer_running) {
        tkl_log_output("record already running: %s\r\n", s_rec.path);
        return OPRT_RESOURCE_NOT_READY;
    }

    test_record_ring_free();
    os_memset(&s_rec, 0, sizeof(s_rec));

    s_rec.ring_bytes = (ring_bytes != 0U) ? ring_bytes : TEST_RECORD_RING_DEFAULT;

    /* CODED slab, not the psram general heap: with the pipeline up that heap has
     * well under a megabyte left, while the coded pool this bitstream came from
     * is 14.25 MB. */
    s_rec.ring = (uint8_t *)tkl_media_buf_alloc(TKL_BUF_USE_STREAM, s_rec.ring_bytes,
                                                TKL_MEDIA_BUF_OWNER_TEST);
    if (s_rec.ring == NULL) {
        tkl_log_output("record: ring alloc failed (%u bytes)\r\n",
                       (unsigned int)s_rec.ring_bytes);
        return OPRT_MALLOC_FAILED;
    }

    snprintf(s_rec.tag, sizeof(s_rec.tag), "%s", (tag != NULL) ? tag : "rec");
    snprintf(s_rec.path, sizeof(s_rec.path), "%s", path);
    s_rec.deadline_ms = (uint32_t)tkl_system_get_millisecond() + (sec * 1000U);
    s_rec.writer_running = true;
    s_rec.writer_done = false;

    ret = tkl_thread_create(&s_rec.writer_task, "test_rec", TEST_REC_WRITER_STACK,
                            TKL_THREAD_PRI_NORMAL, test_record_writer_main, NULL);
    if (ret != OPRT_OK) {
        tkl_log_output("%s record: writer task failed: %d\r\n", s_rec.tag, ret);
        s_rec.writer_running = false;
        s_rec.writer_done = true;
        test_record_ring_free();
        return ret;
    }

    /* wait for the file before letting the callback enqueue, so a failed open is
     * reported here instead of silently dropping every frame */
    while (!s_rec.file_ready && !s_rec.open_failed) {
        tkl_system_sleep(TEST_REC_POLL_MS);
    }
    if (s_rec.open_failed) {
        /* open_failed is published before the writer clears writer_running, so
         * wait it out rather than leaving test_record_busy() briefly lying */
        while (s_rec.writer_running) {
            tkl_system_sleep(TEST_REC_POLL_MS);
        }
        test_record_ring_free();
        return OPRT_COM_ERROR;
    }

    s_rec.capturing = true;
    tkl_log_output("%s record: %us -> %s (ring %uKB)\r\n", s_rec.tag,
                   (unsigned int)sec, s_rec.path,
                   (unsigned int)(s_rec.ring_bytes / 1024U));
    return OPRT_OK;
}

VOID_T test_record_stop(VOID_T)
{
    uint32_t waited = 0U;

    if (s_rec.capturing != true && s_rec.writer_running != true) {
        test_record_ring_free();
        return;
    }

    s_rec.capturing = false;
    s_rec.stop_req = true;

    /* This wait is also what makes freeing the ring safe against a feeder that
     * had already passed the capturing check and is mid-memcpy: whenever
     * capturing was true on entry the writer is running too (it exits only on
     * stop_req), so the loop below always sleeps at least one poll interval
     * before it can see writer_running clear. Do not "optimise" that away. */
    while (s_rec.writer_running && waited < TEST_REC_STOP_WAIT_MS) {
        tkl_system_sleep(TEST_REC_POLL_MS);
        waited += TEST_REC_POLL_MS;
    }

    if (s_rec.writer_running) {
        /* the writer still holds the ring; freeing it now would pull the buffer
         * out from under a live fwrite. Leak it and say so - a leaked 2 MB is
         * recoverable by a reboot, a corrupted slab is not. */
        tkl_log_output("%s record: writer did not exit in %ums, ring leaked\r\n",
                       s_rec.tag, (unsigned int)TEST_REC_STOP_WAIT_MS);
        s_rec.ring = NULL;
        return;
    }

    test_record_ring_free();
}

BOOL_T test_record_busy(VOID_T)
{
    return (s_rec.capturing || s_rec.writer_running) ? TRUE : FALSE;
}

STATIC VOID_T test_record_stat(TEST_RECORD_STAT_T *out)
{
    if (out == NULL) {
        return;
    }

    out->capturing = s_rec.capturing ? TRUE : FALSE;
    out->writer_running = s_rec.writer_running ? TRUE : FALSE;
    out->queued_frames = s_rec.queued_frames;
    out->dropped_frames = s_rec.dropped_frames;
    out->write_errors = s_rec.write_errors;
    out->written_bytes = s_rec.written_bytes;
    out->peak_used = s_rec.peak_used;
    out->ring_bytes = s_rec.ring_bytes;
}

VOID_T test_record_status_dump(VOID_T)
{
    if (s_rec.ring_bytes == 0U) {
        return;
    }

    tkl_log_output("  record: %s %s, %u frames %llu bytes, dropped %u, errors %u, "
                   "peak %uKB/%uKB\r\n",
                   s_rec.path,
                   s_rec.capturing ? "capturing" : (s_rec.writer_running ? "draining" : "idle"),
                   (unsigned int)s_rec.queued_frames,
                   (unsigned long long)s_rec.written_bytes,
                   (unsigned int)s_rec.dropped_frames,
                   (unsigned int)s_rec.write_errors,
                   (unsigned int)(s_rec.peak_used / 1024U),
                   (unsigned int)(s_rec.ring_bytes / 1024U));
}

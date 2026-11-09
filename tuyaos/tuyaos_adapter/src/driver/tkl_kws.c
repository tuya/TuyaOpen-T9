/**
 * @file tkl_kws.c
 * @brief T9 (BK7259) keyword spotting, backed by the tal_audio_subsys KWS module
 * @version 1.0
 * @date 2026-11-06
 * @copyright Copyright (c) 2025-2026 Tuya Inc. All Rights Reserved.
 *
 * tkl_audio_frontend.c pushes 20 ms mono blocks in from the audio task; a
 * dedicated thread drains them so keyword inference never stalls capture.
 * The model is embedded in the library, so there is no filesystem access.
 *
 * IMPORTANT: the wake word is registered in s_kws_keywords below by CTC
 * token IDs; the mapping table decides which TKL_KWS_WAKEUP_WORD_E the upper
 * layer receives.
 */

#include "tkl_kws.h"

#include <string.h>

#include "tkl_memory.h"
#include "tkl_mutex.h"
#include "tkl_semaphore.h"
#include "tkl_thread.h"
#include "tkl_system.h"
#include "tkl_output.h"
#include "tuya_ringbuf.h"

#include "tkl_audio_frontend.h"

#if defined(TKL_AF_WITH_KWS) && (TKL_AF_WITH_KWS == 1)

#include "kws/audio_subsys_kws.h"

/***********************************************************
************************macro define************************
***********************************************************/
#define TKL_KWS_FRAME_BYTES  (TKL_AF_OUT_BYTES)         /* 640 */
#define TKL_KWS_BUFSZ        (TKL_KWS_FRAME_BYTES * 50) /* 1 s */
#define TKL_KWS_THREAD_STACK (6 * 1024)
#define TKL_KWS_THREAD_PRIO  (4)

/* Temporary bring-up diagnostics: every 5 s of 20 ms frames, print how many
 * frames were fed, dropped and actually processed.  Without this there is no
 * way to tell "KWS never sees audio" from "KWS hears audio but never fires".
 * Set to 0 to silence it. */
#define TKL_KWS_DIAG_LOG     (1)
#define TKL_KWS_DIAG_PERIOD  (250)

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef struct {
    BOOL_T            init;
    BOOL_T            enable;
    BOOL_T            running;
    TUYA_RINGBUFF_T   rb;
    TKL_MUTEX_HANDLE  mutex;
    TKL_SEM_HANDLE    sem;
    TKL_THREAD_HANDLE thread;
    uint8_t          *chunk;
} TKL_KWS_CTRL_T;

/***********************************************************
***********************variable define**********************
***********************************************************/
static TKL_KWS_CTRL_T    s_kws_mgr;
static TKL_KWS_WAKEUP_CB s_wakeup_cb = NULL;
static void             *s_kws_self  = NULL;

#if TKL_KWS_DIAG_LOG
static volatile uint32_t s_diag_fed;
static volatile uint32_t s_diag_drop;
static volatile uint32_t s_diag_proc;
static volatile uint32_t s_diag_skip;
#endif

/* Token IDs come from the embedded model's 40-entry CTC vocabulary. The model
 * is the upstream WeKWS demo "hey tino" (fsmn_tino_epoch66), whose documented
 * sequence is 27,8,22,4,23,10 (h ey t i n ow); see the library README. The
 * threshold is the mean of the per-token peak probabilities: the README's
 * reference recording scores 0.61-0.80, so 0.6 wakes 6/6 with no false hits,
 * while 0.8 misses every one. */
static const int s_tino_ids[] = {27, 8, 22, 4, 23, 10}; /* hey tino */

static const audio_subsys_kws_keyword_t s_kws_keywords[] = {
    {"tino", s_tino_ids, (int)(sizeof(s_tino_ids) / sizeof(s_tino_ids[0])), 0.6f},
};

/* Maps a detected keyword name to the enum the upper layer understands.
 * A name with no row here is reported as UNKNOWN. */
static const struct {
    const char           *name;
    TKL_KWS_WAKEUP_WORD_E word;
} s_wakeup_word_map[] = {
    {"tino", TKL_KWS_WAKEUP_TINO},
};

/***********************************************************
***********************function define**********************
***********************************************************/
static TKL_KWS_WAKEUP_WORD_E __tkl_kws_lookup_word(const char *name)
{
    uint32_t i;

    for (i = 0; i < sizeof(s_wakeup_word_map) / sizeof(s_wakeup_word_map[0]); i++) {
        if (strcmp(s_wakeup_word_map[i].name, name) == 0) {
            return s_wakeup_word_map[i].word;
        }
    }

    return TKL_KWS_WAKEUP_WORD_UNKNOWN;
}

static OPERATE_RET __tkl_kws_create(void)
{
    audio_subsys_kws_config_t cfg;

    if (s_kws_self) {
        return OPRT_OK;
    }

    audio_subsys_kws_config_default(&cfg);
    cfg.keywords      = s_kws_keywords;
    cfg.keyword_count = (int)(sizeof(s_kws_keywords) / sizeof(s_kws_keywords[0]));

    if (audio_subsys_kws_create(&s_kws_self, &cfg) != 0) {
        s_kws_self = NULL;
        tkl_log_output("tkl_kws: create failed\r\n");
        return OPRT_MALLOC_FAILED;
    }

    tkl_log_output("tkl_kws: engine ready\r\n");

    return OPRT_OK;
}

static void __tkl_kws_task(void *args)
{
    audio_subsys_kws_result_t result;
    int                       rc;

    (void)args;

    if (__tkl_kws_create() != OPRT_OK) {
        s_kws_mgr.running = FALSE;
        return;
    }

    while (s_kws_mgr.running) {
        if (tkl_semaphore_wait(s_kws_mgr.sem, 200) != OPRT_OK) {
            continue;
        }

        while (s_kws_mgr.running) {
            uint32_t got;

            tkl_mutex_lock(s_kws_mgr.mutex);
            got = tuya_ring_buff_read(s_kws_mgr.rb, s_kws_mgr.chunk, TKL_KWS_FRAME_BYTES);
            tkl_mutex_unlock(s_kws_mgr.mutex);

            if (got != TKL_KWS_FRAME_BYTES) {
                break;
            }

            rc = audio_subsys_kws_process(s_kws_self, s_kws_mgr.chunk, TKL_KWS_FRAME_BYTES, &result, sizeof(result));
#if TKL_KWS_DIAG_LOG
            s_diag_proc++;
#endif
            if (rc < 0) {
                /* Stream state is undefined after a failed frame. */
                tkl_log_output("tkl_kws: process failed %d, recreating\r\n", rc);
                audio_subsys_kws_destroy(s_kws_self);
                s_kws_self = NULL;
                if (__tkl_kws_create() != OPRT_OK) {
                    s_kws_mgr.running = FALSE;
                    break;
                }
                continue;
            }

            if (rc == (int)sizeof(result)) {
                tkl_log_output("tkl_kws: '%s' score %.3f frames %d..%d\r\n", result.name, result.score,
                               result.start_frame, result.end_frame);
                if (s_wakeup_cb) {
                    s_wakeup_cb(__tkl_kws_lookup_word(result.name));
                }
                audio_subsys_kws_ioctl(s_kws_self, AUDIO_SUBSYS_KWS_RESET, NULL);
            }
        }
    }

    tkl_log_output("tkl_kws: task exit\r\n");
}

static OPERATE_RET __tkl_kws_alloc(void)
{
#if defined(ENABLE_EXT_RAM) && (ENABLE_EXT_RAM == 1)
    s_kws_mgr.chunk = tkl_system_psram_malloc(TKL_KWS_FRAME_BYTES);
#else
    s_kws_mgr.chunk = tkl_system_malloc(TKL_KWS_FRAME_BYTES);
#endif
    if (s_kws_mgr.chunk == NULL) {
        return OPRT_MALLOC_FAILED;
    }

#if defined(ENABLE_EXT_RAM) && (ENABLE_EXT_RAM == 1)
    if (tuya_ring_buff_create(TKL_KWS_BUFSZ, OVERFLOW_PSRAM_STOP_TYPE, &s_kws_mgr.rb) != OPRT_OK) {
#else
    if (tuya_ring_buff_create(TKL_KWS_BUFSZ, OVERFLOW_STOP_TYPE, &s_kws_mgr.rb) != OPRT_OK) {
#endif
        tkl_system_free(s_kws_mgr.chunk);
        s_kws_mgr.chunk = NULL;
        return OPRT_MALLOC_FAILED;
    }

    return OPRT_OK;
}

static void __tkl_kws_free(void)
{
    if (s_kws_mgr.rb) {
        tuya_ring_buff_free(s_kws_mgr.rb);
        s_kws_mgr.rb = NULL;
    }

    if (s_kws_mgr.chunk) {
        tkl_system_free(s_kws_mgr.chunk);
        s_kws_mgr.chunk = NULL;
    }
}

OPERATE_RET tkl_kws_init(void)
{
    if (s_kws_mgr.init) {
        return OPRT_OK;
    }

    memset(&s_kws_mgr, 0, sizeof(s_kws_mgr));

    if (__tkl_kws_alloc() != OPRT_OK) {
        __tkl_kws_free();
        return OPRT_MALLOC_FAILED;
    }

    if (tkl_semaphore_create_init(&s_kws_mgr.sem, 0, 1) != OPRT_OK) {
        __tkl_kws_free();
        return OPRT_COM_ERROR;
    }

    if (tkl_mutex_create_init(&s_kws_mgr.mutex) != OPRT_OK) {
        tkl_semaphore_release(s_kws_mgr.sem);
        s_kws_mgr.sem = NULL;
        __tkl_kws_free();
        return OPRT_COM_ERROR;
    }

    s_kws_mgr.running = TRUE;
    if (tkl_thread_create(&s_kws_mgr.thread, "kws_task", TKL_KWS_THREAD_STACK, TKL_KWS_THREAD_PRIO, __tkl_kws_task,
                          NULL) != OPRT_OK) {
        s_kws_mgr.running = FALSE;
        tkl_mutex_release(s_kws_mgr.mutex);
        s_kws_mgr.mutex = NULL;
        tkl_semaphore_release(s_kws_mgr.sem);
        s_kws_mgr.sem = NULL;
        __tkl_kws_free();
        return OPRT_COM_ERROR;
    }

    s_kws_mgr.init = TRUE;

    return OPRT_OK;
}

OPERATE_RET tkl_kws_reg_wakeup_cb(TKL_KWS_WAKEUP_CB wakeup_cb)
{
    s_wakeup_cb = wakeup_cb;
    return OPRT_OK;
}

OPERATE_RET tkl_kws_enable(void)
{
    if (!s_kws_mgr.init) {
        return OPRT_RESOURCE_NOT_READY;
    }

    tkl_mutex_lock(s_kws_mgr.mutex);
    tuya_ring_buff_reset(s_kws_mgr.rb);
    tkl_mutex_unlock(s_kws_mgr.mutex);

    s_kws_mgr.enable = TRUE;

    tkl_log_output("tkl_kws: enabled\r\n");

    return OPRT_OK;
}

OPERATE_RET tkl_kws_disable(void)
{
    if (!s_kws_mgr.init) {
        return OPRT_RESOURCE_NOT_READY;
    }

    s_kws_mgr.enable = FALSE;

    tkl_log_output("tkl_kws: disabled\r\n");

    tkl_mutex_lock(s_kws_mgr.mutex);
    tuya_ring_buff_reset(s_kws_mgr.rb);
    tkl_mutex_unlock(s_kws_mgr.mutex);

    return OPRT_OK;
}

OPERATE_RET tkl_kws_deinit(void)
{
    if (!s_kws_mgr.init) {
        return OPRT_OK;
    }

    s_kws_mgr.enable  = FALSE;
    s_kws_mgr.running = FALSE;
    tkl_semaphore_post(s_kws_mgr.sem);

    if (s_kws_mgr.thread) {
        tkl_thread_release(s_kws_mgr.thread);
        s_kws_mgr.thread = NULL;
    }

    if (s_kws_self) {
        audio_subsys_kws_destroy(s_kws_self);
        s_kws_self = NULL;
    }

    if (s_kws_mgr.mutex) {
        tkl_mutex_release(s_kws_mgr.mutex);
        s_kws_mgr.mutex = NULL;
    }

    if (s_kws_mgr.sem) {
        tkl_semaphore_release(s_kws_mgr.sem);
        s_kws_mgr.sem = NULL;
    }

    __tkl_kws_free();

    s_kws_mgr.init = FALSE;

    return OPRT_OK;
}

OPERATE_RET tkl_kws_feed_with_vad(uint8_t *data, uint16_t datalen, uint8_t vadflag)
{
    uint32_t written;

    (void)vadflag;

    if (!s_kws_mgr.init || !s_kws_mgr.enable) {
#if TKL_KWS_DIAG_LOG
        if (((++s_diag_skip) % TKL_KWS_DIAG_PERIOD) == 1) {
            tkl_log_output("tkl_kws: not enabled (init %d, enable %d), %u frames skipped\r\n", s_kws_mgr.init,
                           s_kws_mgr.enable, (unsigned)s_diag_skip);
        }
#endif
        return OPRT_RESOURCE_NOT_READY;
    }

    if (data == NULL || datalen != TKL_KWS_FRAME_BYTES) {
        return OPRT_INVALID_PARM;
    }

    tkl_mutex_lock(s_kws_mgr.mutex);
    written = tuya_ring_buff_write(s_kws_mgr.rb, data, datalen);
    tkl_mutex_unlock(s_kws_mgr.mutex);

    if (written != datalen) {
        tkl_log_output("tkl_kws: buffer full, dropped %u bytes\r\n", datalen - written);
#if TKL_KWS_DIAG_LOG
        s_diag_drop++;
#endif
    } else {
        tkl_semaphore_post(s_kws_mgr.sem);
#if TKL_KWS_DIAG_LOG
        if (((++s_diag_fed) % TKL_KWS_DIAG_PERIOD) == 0) {
            /* fed grows once per 20 ms of audio; proc lags it if the KWS
             * thread cannot keep up. */
            tkl_log_output("tkl_kws: diag fed %u, dropped %u, processed %u\r\n", (unsigned)s_diag_fed,
                           (unsigned)s_diag_drop, (unsigned)s_diag_proc);
        }
#endif
    }

    return OPRT_OK;
}

#else /* TKL_AF_WITH_KWS */

OPERATE_RET tkl_kws_init(void)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_kws_reg_wakeup_cb(TKL_KWS_WAKEUP_CB wakeup_cb)
{
    (void)wakeup_cb;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_kws_enable(void)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_kws_disable(void)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_kws_deinit(void)
{
    return OPRT_OK;
}

#endif /* TKL_AF_WITH_KWS */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_aud_dac.h"
#include "tkl_memory.h"
#include "tkl_output.h"
#include "tkl_system.h"

#define DAC_TEST_DEFAULT_PORT            TUYA_AUDIO_DAC_PORT_0
#define DAC_TEST_DEFAULT_SAMPLE_RATE     16000U
#define DAC_TEST_DEFAULT_SAMPLE_BITS     TUYA_AUDIO_SAMPLE_BITS_16
#define DAC_TEST_DEFAULT_CHAN_NUM        1U
#define DAC_TEST_DEFAULT_FRAME_TIME_MS   20U
#define DAC_TEST_DEFAULT_VOLUME          70U
#define DAC_TEST_DEFAULT_PLAY_MS         2000U
#define DAC_TEST_DEFAULT_FREQ            1000U
#define DAC_TEST_BUSY_SLEEP_MS           2U
#define DAC_TEST_MAX_BUSY_RETRY          500U
#define DAC_TEST_INIT_RETRY_CNT          8U
#define DAC_TEST_INIT_RETRY_SLEEP_MS     20U
#define DAC_TEST_DEINIT_RETRY_CNT        8U
#define DAC_TEST_SQUARE_AMP              12000

typedef struct {
    bool initialized;
    bool started;
    TUYA_AUDIO_DAC_PORT_E port;
    uint32_t sample_rate;
    uint8_t sample_bits;
    uint8_t chan_num;
    uint32_t frame_time_ms;
    uint8_t volume;
    uint32_t frame_size;
    uint32_t tx_complete_cnt;
    uint32_t wave_sample_pos;
} test_dac_ctx_t;

static test_dac_ctx_t s_dac_ctx = {
    .initialized = false,
    .started = false,
    .port = DAC_TEST_DEFAULT_PORT,
    .sample_rate = DAC_TEST_DEFAULT_SAMPLE_RATE,
    .sample_bits = DAC_TEST_DEFAULT_SAMPLE_BITS,
    .chan_num = DAC_TEST_DEFAULT_CHAN_NUM,
    .frame_time_ms = DAC_TEST_DEFAULT_FRAME_TIME_MS,
    .volume = DAC_TEST_DEFAULT_VOLUME,
    .frame_size = 0U,
    .tx_complete_cnt = 0U,
    .wave_sample_pos = 0U,
};

static uint32_t test_dac_calc_frame_size(uint32_t sample_rate, uint8_t sample_bits, uint8_t chan_num, uint32_t frame_ms)
{
    if (sample_rate == 0U || sample_bits == 0U || chan_num == 0U || frame_ms == 0U) {
        return 0U;
    }

    return ((sample_rate * (sample_bits / 8U) * chan_num) / 1000U) * frame_ms;
}

static void test_dac_runtime_reset(void)
{
    s_dac_ctx.initialized = false;
    s_dac_ctx.started = false;
    s_dac_ctx.port = DAC_TEST_DEFAULT_PORT;
    s_dac_ctx.sample_rate = DAC_TEST_DEFAULT_SAMPLE_RATE;
    s_dac_ctx.sample_bits = DAC_TEST_DEFAULT_SAMPLE_BITS;
    s_dac_ctx.chan_num = DAC_TEST_DEFAULT_CHAN_NUM;
    s_dac_ctx.frame_time_ms = DAC_TEST_DEFAULT_FRAME_TIME_MS;
    s_dac_ctx.volume = DAC_TEST_DEFAULT_VOLUME;
    s_dac_ctx.frame_size = 0U;
    s_dac_ctx.tx_complete_cnt = 0U;
    s_dac_ctx.wave_sample_pos = 0U;
}

static void test_dac_frame_cb(TUYA_AUDIO_DAC_FRAME_EVT_E event, void *args)
{
    (void)args;

    if (event == TUYA_AUDIO_DAC_FRAME_EVENT_TX_COMPLETE) {
        s_dac_ctx.tx_complete_cnt++;
    }
}

static OPERATE_RET test_dac_set_volume(uint8_t volume)
{
    OPERATE_RET ret;

    if (s_dac_ctx.initialized != true) {
        return OPRT_COM_ERROR;
    }

    ret = tkl_aud_dac_set_volume(s_dac_ctx.port, volume);
    if (ret == OPRT_OK) {
        s_dac_ctx.volume = (volume > 100U) ? 100U : volume;
    }

    return ret;
}

static OPERATE_RET test_dac_init(uint32_t sample_rate, uint32_t frame_time_ms, uint8_t volume)
{
    OPERATE_RET ret;
    TKL_AUD_DAC_CFG_T cfg;
    uint32_t retry = 0U;

    if (sample_rate == 0U || frame_time_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_dac_ctx.started == true) {
        tkl_log_output("dac test is running, stop before reinit\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_dac_ctx.initialized == true) {
        ret = tkl_aud_dac_deinit(s_dac_ctx.port);
        if (ret != OPRT_OK && ret != OPRT_COM_ERROR) {
            tkl_log_output("dac deinit before reinit failed: %d\r\n", ret);
            return ret;
        }
        test_dac_runtime_reset();
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.volume = volume;
    cfg.chan_num = DAC_TEST_DEFAULT_CHAN_NUM;
    cfg.sample_bits = DAC_TEST_DEFAULT_SAMPLE_BITS;
    cfg.sample_rate = sample_rate;
    cfg.frame_time_ms = frame_time_ms;
    cfg.frame_cb = test_dac_frame_cb;
    cfg.args = NULL;

    for (retry = 0U; retry < DAC_TEST_INIT_RETRY_CNT; retry++) {
        ret = tkl_aud_dac_init(DAC_TEST_DEFAULT_PORT, &cfg);
        if (ret == OPRT_OK) {
            break;
        }

        if (ret != OPRT_COM_ERROR) {
            break;
        }

        /*
         * Recovery path: local test context may be reset while DAC driver
         * instance is still alive, or another test flow is holding DAC.
         */
        (void)tkl_aud_dac_stop(DAC_TEST_DEFAULT_PORT);
        (void)tkl_aud_dac_deinit(DAC_TEST_DEFAULT_PORT);
        tkl_system_sleep(DAC_TEST_INIT_RETRY_SLEEP_MS);
    }

    if (ret != OPRT_OK) {
        tkl_log_output("dac init failed: %d\r\n", ret);
        return ret;
    }

    s_dac_ctx.initialized = true;
    s_dac_ctx.started = false;
    s_dac_ctx.port = DAC_TEST_DEFAULT_PORT;
    s_dac_ctx.sample_rate = sample_rate;
    s_dac_ctx.sample_bits = DAC_TEST_DEFAULT_SAMPLE_BITS;
    s_dac_ctx.chan_num = DAC_TEST_DEFAULT_CHAN_NUM;
    s_dac_ctx.frame_time_ms = frame_time_ms;
    s_dac_ctx.volume = (volume > 100U) ? 100U : volume;
    s_dac_ctx.tx_complete_cnt = 0U;
    s_dac_ctx.wave_sample_pos = 0U;
    s_dac_ctx.frame_size = tkl_aud_dac_get_frame_size(s_dac_ctx.port);
    if (s_dac_ctx.frame_size == 0U) {
        s_dac_ctx.frame_size = test_dac_calc_frame_size(
                s_dac_ctx.sample_rate,
                s_dac_ctx.sample_bits,
                s_dac_ctx.chan_num,
                s_dac_ctx.frame_time_ms);
    }

    return OPRT_OK;
}

static OPERATE_RET test_dac_start(void)
{
    OPERATE_RET ret;

    if (s_dac_ctx.initialized != true) {
        tkl_log_output("dac test is not initialized, use: xdac init [rate] [frame_ms] [volume]\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_dac_ctx.started == true) {
        return OPRT_OK;
    }

    ret = tkl_aud_dac_start(s_dac_ctx.port);
    if (ret != OPRT_OK) {
        tkl_log_output("dac start failed: %d\r\n", ret);
        return ret;
    }

    s_dac_ctx.started = true;
    return OPRT_OK;
}

static OPERATE_RET test_dac_stop(void)
{
    OPERATE_RET ret;

    if (s_dac_ctx.initialized != true) {
        return OPRT_OK;
    }

    if (s_dac_ctx.started != true) {
        return OPRT_OK;
    }

    ret = tkl_aud_dac_stop(s_dac_ctx.port);
    if (ret != OPRT_OK) {
        tkl_log_output("dac stop failed: %d\r\n", ret);
        return ret;
    }

    s_dac_ctx.started = false;
    return OPRT_OK;
}

static OPERATE_RET test_dac_deinit(void)
{
    OPERATE_RET ret;
    TUYA_AUDIO_DAC_PORT_E port = s_dac_ctx.initialized ? s_dac_ctx.port : DAC_TEST_DEFAULT_PORT;
    uint32_t retry = 0U;

    /*
     * Always try to release the underlying DAC instance.
     * Local test context may not reflect actual driver state.
     */
    ret = OPRT_COM_ERROR;
    for (retry = 0U; retry < DAC_TEST_DEINIT_RETRY_CNT; retry++) {
        (void)tkl_aud_dac_stop(port);
        ret = tkl_aud_dac_deinit(port);
        if (ret == OPRT_OK || ret == OPRT_COM_ERROR) {
            tkl_system_sleep(1);
            continue;
        }

        tkl_log_output("dac deinit failed: %d\r\n", ret);
        return ret;
    }

    test_dac_runtime_reset();
    return OPRT_OK;
}

static void test_dac_fill_square_16bit(uint8_t *buf, uint32_t len, uint32_t freq_hz)
{
    int16_t *pcm = (int16_t *)buf;
    uint32_t sample_cnt = len / sizeof(int16_t);
    uint32_t half_period;
    uint32_t i;

    if (buf == NULL || len < sizeof(int16_t) || s_dac_ctx.sample_rate == 0U || freq_hz == 0U) {
        return;
    }

    half_period = s_dac_ctx.sample_rate / (freq_hz * 2U);
    if (half_period == 0U) {
        half_period = 1U;
    }

    for (i = 0U; i < sample_cnt; i++) {
        bool high_level = ((s_dac_ctx.wave_sample_pos / half_period) & 0x1U) ? true : false;
        pcm[i] = high_level ? DAC_TEST_SQUARE_AMP : -DAC_TEST_SQUARE_AMP;
        s_dac_ctx.wave_sample_pos++;
    }
}

static OPERATE_RET test_dac_write_with_retry(uint8_t *data, uint32_t len)
{
    OPERATE_RET ret = OPRT_OK;
    uint32_t retry = 0U;

    while (retry < DAC_TEST_MAX_BUSY_RETRY) {
        ret = tkl_aud_dac_write(s_dac_ctx.port, data, len);
        if (ret == OPRT_OK) {
            return OPRT_OK;
        }

        if (ret != OPRT_OS_ADAPTER_DAC_BUSY) {
            return ret;
        }

        retry++;
        tkl_system_sleep(DAC_TEST_BUSY_SLEEP_MS);
    }

    return ret;
}

static OPERATE_RET test_dac_play_square(uint32_t play_ms, uint32_t freq_hz)
{
    OPERATE_RET ret = OPRT_OK;
    uint8_t *frame = NULL;
    uint32_t total_bytes;
    uint32_t sent_bytes = 0U;
    uint32_t frame_size;

    if (play_ms == 0U || freq_hz == 0U) {
        return OPRT_INVALID_PARM;
    }

    /* One-key play: auto init + start if not already running. */
    if (s_dac_ctx.initialized != true) {
        ret = test_dac_init(DAC_TEST_DEFAULT_SAMPLE_RATE,
                            DAC_TEST_DEFAULT_FRAME_TIME_MS,
                            DAC_TEST_DEFAULT_VOLUME);
        if (ret != OPRT_OK) {
            return ret;
        }
    }
    if (s_dac_ctx.started != true) {
        ret = test_dac_start();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    frame_size = s_dac_ctx.frame_size;
    if (frame_size < sizeof(int16_t)) {
        frame_size = test_dac_calc_frame_size(
                s_dac_ctx.sample_rate,
                s_dac_ctx.sample_bits,
                s_dac_ctx.chan_num,
                s_dac_ctx.frame_time_ms);
    }
    if (frame_size < sizeof(int16_t)) {
        return OPRT_INVALID_PARM;
    }

    total_bytes = ((s_dac_ctx.sample_rate * (s_dac_ctx.sample_bits / 8U) * s_dac_ctx.chan_num) / 1000U) * play_ms;
    if (total_bytes < sizeof(int16_t)) {
        return OPRT_INVALID_PARM;
    }
    total_bytes &= ~(uint32_t)(sizeof(int16_t) - 1U);

    frame = (uint8_t *)tkl_system_malloc(frame_size);
    if (frame == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    while (sent_bytes < total_bytes) {
        uint32_t chunk = total_bytes - sent_bytes;
        if (chunk > frame_size) {
            chunk = frame_size;
        }

        chunk &= ~(uint32_t)(sizeof(int16_t) - 1U);
        if (chunk == 0U) {
            break;
        }

        test_dac_fill_square_16bit(frame, chunk, freq_hz);
        ret = test_dac_write_with_retry(frame, chunk);
        if (ret != OPRT_OK) {
            tkl_log_output("dac write failed: %d, sent=%u/%u\r\n",
                    ret,
                    (unsigned int)sent_bytes,
                    (unsigned int)total_bytes);
            break;
        }

        sent_bytes += chunk;
    }

    tkl_system_free(frame);
    return ret;
}

void cli_tuya_dac_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint32_t sample_rate = DAC_TEST_DEFAULT_SAMPLE_RATE;
    uint32_t frame_time_ms = DAC_TEST_DEFAULT_FRAME_TIME_MS;
    uint32_t play_ms = DAC_TEST_DEFAULT_PLAY_MS;
    uint32_t freq_hz = DAC_TEST_DEFAULT_FREQ;
    uint32_t volume = DAC_TEST_DEFAULT_VOLUME;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xdac <init [rate] [frame_ms] [volume]|start|play [ms] [freq]|vol [0-100]|stop|deinit|close|status>");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "init") == 0) {
        if (argc >= 3) {
            sample_rate = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            frame_time_ms = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            volume = os_strtoul(argv[4], NULL, 10);
        }
        if (volume > 100U) {
            volume = 100U;
        }

        ret = test_dac_init(sample_rate, frame_time_ms, (uint8_t)volume);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdac init %s (%d), rate=%u frame_ms=%u volume=%u frame_size=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)sample_rate,
                 (unsigned int)frame_time_ms,
                 (unsigned int)volume,
                 (unsigned int)s_dac_ctx.frame_size);
    } else if (os_strcmp(cmd, "start") == 0) {
        ret = test_dac_start();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdac start %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "play") == 0 || os_strcmp(cmd, "tone") == 0) {
        if (argc >= 3) {
            play_ms = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            freq_hz = os_strtoul(argv[3], NULL, 10);
        }

        ret = test_dac_play_square(play_ms, freq_hz);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdac %s %s (%d), play_ms=%u freq=%u tx_cb=%u\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)play_ms,
                 (unsigned int)freq_hz,
                 (unsigned int)s_dac_ctx.tx_complete_cnt);
    } else if (os_strcmp(cmd, "vol") == 0 || os_strcmp(cmd, "volume") == 0) {
        if (argc >= 3) {
            volume = os_strtoul(argv[2], NULL, 10);
        }
        if (volume > 100U) {
            volume = 100U;
        }

        ret = test_dac_set_volume((uint8_t)volume);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdac vol %s (%d), volume=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)volume);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = test_dac_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdac stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "deinit") == 0 || os_strcmp(cmd, "close") == 0) {
        ret = test_dac_deinit();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdac %s %s (%d)\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "status") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdac status init=%u start=%u port=%u rate=%u bits=%u chan=%u frame_ms=%u frame_size=%u volume=%u tx_cb=%u\r\n",
                 (unsigned int)s_dac_ctx.initialized,
                 (unsigned int)s_dac_ctx.started,
                 (unsigned int)s_dac_ctx.port,
                 (unsigned int)s_dac_ctx.sample_rate,
                 (unsigned int)s_dac_ctx.sample_bits,
                 (unsigned int)s_dac_ctx.chan_num,
                 (unsigned int)s_dac_ctx.frame_time_ms,
                 (unsigned int)s_dac_ctx.frame_size,
                 (unsigned int)s_dac_ctx.volume,
                 (unsigned int)s_dac_ctx.tx_complete_cnt);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xdac <init [rate] [frame_ms] [volume]|start|play [ms] [freq]|vol [0-100]|stop|deinit|close|status>");
    }
}

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_output.h"
#include "tkl_pwm.h"
#include "tkl_system.h"

#define PWM_TEST_DEFAULT_CH            TUYA_PWM_NUM_0
#define PWM_TEST_DEFAULT_FREQ_HZ       1000U
#define PWM_TEST_DEFAULT_DUTY          5000U
#define PWM_TEST_DEFAULT_CYCLE         10000U
#define PWM_TEST_DEFAULT_POLARITY      TUYA_PWM_POSITIVE
#define PWM_TEST_DEFAULT_RUN_MS        2000U

typedef struct {
    bool initialized;
    bool started;
    bool capture_mode;
    TUYA_PWM_NUM_E channel;
    TUYA_PWM_BASE_CFG_T cfg;
    uint32_t cap_irq_cnt;
    TUYA_PWM_CAPTURE_DATA_T last_cap;
} test_pwm_ctx_t;

static test_pwm_ctx_t s_pwm_ctx = {
    .initialized = false,
    .started = false,
    .capture_mode = false,
    .channel = PWM_TEST_DEFAULT_CH,
    .cfg = {
        .polarity = PWM_TEST_DEFAULT_POLARITY,
        .count_mode = TUYA_PWM_CNT_UP,
        .duty = PWM_TEST_DEFAULT_DUTY,
        .cycle = PWM_TEST_DEFAULT_CYCLE,
        .frequency = PWM_TEST_DEFAULT_FREQ_HZ,
    },
    .cap_irq_cnt = 0U,
    .last_cap = {0},
};

static void test_pwm_runtime_reset(void)
{
    (void)os_memset(&s_pwm_ctx, 0, sizeof(s_pwm_ctx));
    s_pwm_ctx.channel = PWM_TEST_DEFAULT_CH;
    s_pwm_ctx.cfg.polarity = PWM_TEST_DEFAULT_POLARITY;
    s_pwm_ctx.cfg.count_mode = TUYA_PWM_CNT_UP;
    s_pwm_ctx.cfg.duty = PWM_TEST_DEFAULT_DUTY;
    s_pwm_ctx.cfg.cycle = PWM_TEST_DEFAULT_CYCLE;
    s_pwm_ctx.cfg.frequency = PWM_TEST_DEFAULT_FREQ_HZ;
}

static OPERATE_RET test_pwm_validate_cfg(uint32_t ch, uint32_t freq_hz, uint32_t duty, uint32_t cycle, uint32_t polarity)
{
    if (ch >= TUYA_PWM_NUM_MAX || freq_hz == 0U || cycle == 0U) {
        return OPRT_INVALID_PARM;
    }
    if (duty > cycle) {
        return OPRT_INVALID_PARM;
    }
    if (polarity > (uint32_t)TUYA_PWM_POSITIVE) {
        return OPRT_INVALID_PARM;
    }
    return OPRT_OK;
}

static OPERATE_RET test_pwm_deinit(void)
{
    OPERATE_RET ret = OPRT_OK;
    TUYA_PWM_NUM_E ch = s_pwm_ctx.initialized ? s_pwm_ctx.channel : PWM_TEST_DEFAULT_CH;

    if (s_pwm_ctx.capture_mode) {
        (void)tkl_pwm_cap_stop(ch);
    } else {
        (void)tkl_pwm_stop(ch);
    }

    ret = tkl_pwm_deinit(ch);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm deinit failed: %d\r\n", ret);
        return ret;
    }

    test_pwm_runtime_reset();
    return OPRT_OK;
}

static OPERATE_RET test_pwm_init(uint32_t ch, uint32_t freq_hz, uint32_t duty, uint32_t cycle, uint32_t polarity)
{
    OPERATE_RET ret;
    TUYA_PWM_BASE_CFG_T cfg;

    ret = test_pwm_validate_cfg(ch, freq_hz, duty, cycle, polarity);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_pwm_ctx.initialized) {
        ret = test_pwm_deinit();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.polarity = (TUYA_PWM_POLARITY_E)polarity;
    cfg.count_mode = TUYA_PWM_CNT_UP;
    cfg.duty = duty;
    cfg.cycle = cycle;
    cfg.frequency = freq_hz;

    ret = tkl_pwm_init((TUYA_PWM_NUM_E)ch, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm init failed: %d\r\n", ret);
        return ret;
    }

    s_pwm_ctx.initialized = true;
    s_pwm_ctx.started = false;
    s_pwm_ctx.capture_mode = false;
    s_pwm_ctx.channel = (TUYA_PWM_NUM_E)ch;
    s_pwm_ctx.cfg = cfg;
    s_pwm_ctx.cap_irq_cnt = 0U;
    (void)os_memset(&s_pwm_ctx.last_cap, 0, sizeof(s_pwm_ctx.last_cap));
    return OPRT_OK;
}

static OPERATE_RET test_pwm_start(void)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_RESOURCE_NOT_READY;
    }
    if (s_pwm_ctx.started) {
        return OPRT_OK;
    }

    ret = tkl_pwm_start(s_pwm_ctx.channel);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm start failed: %d\r\n", ret);
        return ret;
    }

    s_pwm_ctx.started = true;
    return OPRT_OK;
}

static OPERATE_RET test_pwm_stop(void)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_OK;
    }
    if (!s_pwm_ctx.started) {
        return OPRT_OK;
    }

    ret = tkl_pwm_stop(s_pwm_ctx.channel);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm stop failed: %d\r\n", ret);
        return ret;
    }

    s_pwm_ctx.started = false;
    return OPRT_OK;
}

static OPERATE_RET test_pwm_set_duty(uint32_t duty)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (duty > s_pwm_ctx.cfg.cycle) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_pwm_duty_set(s_pwm_ctx.channel, duty);
    if (ret == OPRT_OK) {
        s_pwm_ctx.cfg.duty = duty;
    }

    return ret;
}

static OPERATE_RET test_pwm_set_frequency(uint32_t freq_hz)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (freq_hz == 0U) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_pwm_frequency_set(s_pwm_ctx.channel, freq_hz);
    if (ret == OPRT_OK) {
        s_pwm_ctx.cfg.frequency = freq_hz;
    }

    return ret;
}

static OPERATE_RET test_pwm_set_polarity(uint32_t polarity)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (polarity > (uint32_t)TUYA_PWM_POSITIVE) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_pwm_polarity_set(s_pwm_ctx.channel, (TUYA_PWM_POLARITY_E)polarity);
    if (ret == OPRT_OK) {
        s_pwm_ctx.cfg.polarity = (TUYA_PWM_POLARITY_E)polarity;
    }

    return ret;
}

static void test_pwm_cap_cb(TUYA_PWM_NUM_E port, TUYA_PWM_CAPTURE_DATA_T data, VOID_T *arg)
{
    (void)port;
    (void)arg;

    s_pwm_ctx.cap_irq_cnt++;
    s_pwm_ctx.last_cap = data;
}

static OPERATE_RET test_pwm_cap_start(uint32_t ch, uint32_t edge, uint32_t mode, uint32_t clk)
{
    OPERATE_RET ret;
    TUYA_PWM_CAP_IRQ_T cfg;

    if (ch >= TUYA_PWM_NUM_MAX || edge > (uint32_t)TUYA_PWM_POSITIVE || mode > 1U) {
        return OPRT_INVALID_PARM;
    }

    if (s_pwm_ctx.initialized) {
        ret = test_pwm_deinit();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.cap_mode = (mode == 0U) ? TUYA_PWM_CAPTURE_MODE_ONCE : TUYA_PWM_CAPTURE_MODE_PERIOD;
    cfg.trigger_level = (TUYA_PWM_POLARITY_E)edge;
    cfg.clk = clk;
    cfg.cb = test_pwm_cap_cb;
    cfg.arg = NULL;

    ret = tkl_pwm_cap_start((TUYA_PWM_NUM_E)ch, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm cap start failed: %d\r\n", ret);
        return ret;
    }

    s_pwm_ctx.initialized = true;
    s_pwm_ctx.started = true;
    s_pwm_ctx.capture_mode = true;
    s_pwm_ctx.channel = (TUYA_PWM_NUM_E)ch;
    s_pwm_ctx.cap_irq_cnt = 0U;
    (void)os_memset(&s_pwm_ctx.last_cap, 0, sizeof(s_pwm_ctx.last_cap));
    return OPRT_OK;
}

static OPERATE_RET test_pwm_cap_stop(void)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || !s_pwm_ctx.capture_mode) {
        return OPRT_OK;
    }

    ret = tkl_pwm_cap_stop(s_pwm_ctx.channel);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm cap stop failed: %d\r\n", ret);
        return ret;
    }

    test_pwm_runtime_reset();
    return OPRT_OK;
}

static OPERATE_RET test_pwm_run(uint32_t run_ms, uint32_t freq_hz, uint32_t duty, uint32_t cycle)
{
    OPERATE_RET ret;

    if (run_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    ret = test_pwm_init((uint32_t)PWM_TEST_DEFAULT_CH, freq_hz, duty, cycle, (uint32_t)PWM_TEST_DEFAULT_POLARITY);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = test_pwm_start();
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_system_sleep(run_ms);
    return test_pwm_stop();
}

void cli_pwm_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint32_t ch = (uint32_t)PWM_TEST_DEFAULT_CH;
    uint32_t freq_hz = PWM_TEST_DEFAULT_FREQ_HZ;
    uint32_t duty = PWM_TEST_DEFAULT_DUTY;
    uint32_t cycle = PWM_TEST_DEFAULT_CYCLE;
    uint32_t polarity = (uint32_t)PWM_TEST_DEFAULT_POLARITY;
    uint32_t run_ms = PWM_TEST_DEFAULT_RUN_MS;
    uint32_t edge = (uint32_t)TUYA_PWM_POSITIVE;
    uint32_t mode = (uint32_t)TUYA_PWM_CAPTURE_MODE_PERIOD;
    uint32_t clk = 0U;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xpwm <init [ch] [freq] [duty] [cycle] [pol]|start|duty [v]|freq [hz]|pol [0|1]|run [ms] [freq] [duty] [cycle]|cap_start [ch] [edge] [mode] [clk]|cap_stop|stop|deinit|status>");
        return;
    }

    cmd = argv[1];
    if (os_strcmp(cmd, "init") == 0) {
        if (argc >= 3) {
            ch = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            freq_hz = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            duty = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            cycle = os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            polarity = os_strtoul(argv[6], NULL, 10);
        }

        ret = test_pwm_init(ch, freq_hz, duty, cycle, polarity);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm init %s (%d), ch=%u freq=%u duty=%u cycle=%u pol=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)ch,
                 (unsigned int)freq_hz,
                 (unsigned int)duty,
                 (unsigned int)cycle,
                 (unsigned int)polarity);
    } else if (os_strcmp(cmd, "start") == 0) {
        ret = test_pwm_start();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm start %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "duty") == 0) {
        if (argc >= 3) {
            duty = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_pwm_set_duty(duty);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm duty %s (%d), duty=%u cycle=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)duty,
                 (unsigned int)s_pwm_ctx.cfg.cycle);
    } else if (os_strcmp(cmd, "freq") == 0) {
        if (argc >= 3) {
            freq_hz = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_pwm_set_frequency(freq_hz);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm freq %s (%d), freq=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)freq_hz);
    } else if (os_strcmp(cmd, "pol") == 0 || os_strcmp(cmd, "polarity") == 0) {
        if (argc >= 3) {
            polarity = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_pwm_set_polarity(polarity);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm pol %s (%d), polarity=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)polarity);
    } else if (os_strcmp(cmd, "run") == 0) {
        if (argc >= 3) {
            run_ms = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            freq_hz = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            duty = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            cycle = os_strtoul(argv[5], NULL, 10);
        }

        ret = test_pwm_run(run_ms, freq_hz, duty, cycle);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm run %s (%d), ms=%u freq=%u duty=%u cycle=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)run_ms,
                 (unsigned int)freq_hz,
                 (unsigned int)duty,
                 (unsigned int)cycle);
    } else if (os_strcmp(cmd, "cap_start") == 0) {
        if (argc >= 3) {
            ch = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            edge = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            mode = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            clk = os_strtoul(argv[5], NULL, 10);
        }

        ret = test_pwm_cap_start(ch, edge, mode, clk);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm cap_start %s (%d), ch=%u edge=%u mode=%u clk=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)ch,
                 (unsigned int)edge,
                 (unsigned int)mode,
                 (unsigned int)clk);
    } else if (os_strcmp(cmd, "cap_stop") == 0) {
        ret = test_pwm_cap_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm cap_stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = test_pwm_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "deinit") == 0 || os_strcmp(cmd, "close") == 0) {
        ret = test_pwm_deinit();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm %s %s (%d)\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "status") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm status init=%u start=%u cap=%u ch=%u freq=%u duty=%u cycle=%u pol=%u cap_irq=%u last_cap=%u last_edge=%u\r\n",
                 (unsigned int)s_pwm_ctx.initialized,
                 (unsigned int)s_pwm_ctx.started,
                 (unsigned int)s_pwm_ctx.capture_mode,
                 (unsigned int)s_pwm_ctx.channel,
                 (unsigned int)s_pwm_ctx.cfg.frequency,
                 (unsigned int)s_pwm_ctx.cfg.duty,
                 (unsigned int)s_pwm_ctx.cfg.cycle,
                 (unsigned int)s_pwm_ctx.cfg.polarity,
                 (unsigned int)s_pwm_ctx.cap_irq_cnt,
                 (unsigned int)s_pwm_ctx.last_cap.cap_value,
                 (unsigned int)s_pwm_ctx.last_cap.cap_edge);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xpwm <init [ch] [freq] [duty] [cycle] [pol]|start|duty [v]|freq [hz]|pol [0|1]|run [ms] [freq] [duty] [cycle]|cap_start [ch] [edge] [mode] [clk]|cap_stop|stop|deinit|status>");
    }
}


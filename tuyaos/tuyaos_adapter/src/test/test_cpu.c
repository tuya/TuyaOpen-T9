#include "cli.h"
#include "cli_tuya_test.h"

#include <stdint.h>

#include <os/os.h>
#include <os/str.h>

#include "bk_rtos_debug.h"

static uint32_t s_xcpu_top_enable;
static beken_thread_t s_xcpu_top_thread;

static void cpu_test_usage(void)
{
    bk_printf("Usage:\r\n");
    bk_printf("  xcpu help\r\n");
    bk_printf("  xcpu list\r\n");
    bk_printf("  xcpu load\r\n");
    bk_printf("  xcpu info\r\n");
    bk_printf("  xcpu backtrace\r\n");
    bk_printf("  xcpu stack\r\n");
#if CONFIG_FREERTOS_HISTORY_CPU_PERCENT
    bk_printf("  xcpu top {1|5|10}\r\n");
    bk_printf("  xcpu top stop\r\n");
#endif
}

#if CONFIG_FREERTOS_HISTORY_CPU_PERCENT
static int cpu_test_parse_period(const char *arg, uint32_t *period_s, BK_CpuLoadTime *period_enum)
{
    if ((arg == NULL) || (period_s == NULL) || (period_enum == NULL)) {
        return -1;
    }

    if (os_strcmp(arg, "1") == 0) {
        *period_s = 1;
        *period_enum = BK_GetOneSec;
        return 0;
    }

    if (os_strcmp(arg, "5") == 0) {
        *period_s = 5;
        *period_enum = BK_GetFiveSec;
        return 0;
    }

    if (os_strcmp(arg, "10") == 0) {
        *period_s = 10;
        *period_enum = BK_GetTenSec;
        return 0;
    }

    return -1;
}

static void cpu_test_top_task(void *arg)
{
    BK_CpuLoadTime period_enum = (BK_CpuLoadTime)(uintptr_t)arg;
    uint32_t period_s = 1;

    switch (period_enum) {
        case BK_GetFiveSec:
            period_s = 5;
            break;
        case BK_GetTenSec:
            period_s = 10;
            break;
        case BK_GetOneSec:
        default:
            period_s = 1;
            break;
    }

    while (s_xcpu_top_enable) {
        rtos_dump_task_history_runtime_stats(period_enum);
        rtos_delay_milliseconds(period_s * 1000);
    }

    s_xcpu_top_thread = NULL;
    rtos_delete_thread(NULL);
}

static void cpu_test_top_start(BK_CpuLoadTime period_enum)
{
    bk_err_t ret;

    if ((s_xcpu_top_enable == 1U) || (s_xcpu_top_thread != NULL)) {
        bk_printf("xcpu top is already running\r\n");
        return;
    }

    s_xcpu_top_enable = 1;
    ret = rtos_create_thread(&s_xcpu_top_thread,
                             BEKEN_DEFAULT_WORKER_PRIORITY,
                             "xcpu_top",
                             (beken_thread_function_t)cpu_test_top_task,
                             1024,
                             (void *)(uintptr_t)period_enum);
    if (ret != BK_OK) {
        s_xcpu_top_enable = 0;
        s_xcpu_top_thread = NULL;
        bk_printf("xcpu top start failed: %d\r\n", ret);
    }
}

static void cpu_test_top_stop(void)
{
    if ((s_xcpu_top_enable == 0U) && (s_xcpu_top_thread == NULL)) {
        bk_printf("xcpu top is not running\r\n");
        return;
    }

    s_xcpu_top_enable = 0;
    bk_printf("xcpu top stop requested\r\n");
}
#endif

void cli_cpu_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if ((argc < 2) || (argv == NULL) || (argv[1] == NULL)) {
        cpu_test_usage();
        return;
    }

    if ((os_strcmp(argv[1], "help") == 0) || (os_strcmp(argv[1], "?") == 0)) {
        cpu_test_usage();
        return;
    }

    if (os_strcmp(argv[1], "list") == 0) {
        rtos_dump_task_list();
        return;
    }

    if (os_strcmp(argv[1], "load") == 0) {
        rtos_dump_task_runtime_stats();
        return;
    }

    if (os_strcmp(argv[1], "info") == 0) {
        rtos_dump_task_list();
        rtos_dump_task_runtime_stats();
        return;
    }

    if (os_strcmp(argv[1], "backtrace") == 0) {
        rtos_dump_backtrace();
        return;
    }

    if (os_strcmp(argv[1], "stack") == 0) {
        rtos_dump_stack_memory_usage();
        return;
    }

#if CONFIG_FREERTOS_HISTORY_CPU_PERCENT
    if (os_strcmp(argv[1], "top") == 0) {
        uint32_t period_s = 0;
        BK_CpuLoadTime period_enum = BK_GetOneSec;

        if ((argc < 3) || (argv[2] == NULL)) {
            cpu_test_usage();
            return;
        }

        if (os_strcmp(argv[2], "stop") == 0) {
            cpu_test_top_stop();
            return;
        }

        if (cpu_test_parse_period(argv[2], &period_s, &period_enum) != 0) {
            bk_printf("invalid top period: %s\r\n", argv[2]);
            cpu_test_usage();
            return;
        }

        (void)period_s;
        cpu_test_top_start(period_enum);
        return;
    }
#else
    if (os_strcmp(argv[1], "top") == 0) {
        bk_printf("xcpu top is not enabled, set CONFIG_FREERTOS_HISTORY_CPU_PERCENT=y\r\n");
        return;
    }
#endif

    bk_printf("unknown xcpu subcmd: %s\r\n", argv[1]);
    cpu_test_usage();
}

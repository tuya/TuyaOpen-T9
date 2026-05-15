#include "cli.h"
#include "cli_tuya_test.h"
#include "sdkconfig.h"

#include <stdint.h>

#include <components/system.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>

#include "bk_rtos_debug.h"

static void memory_cli_usage(void)
{
    bk_printf("Usage:\r\n");
    bk_printf("  xmem show\r\n");
    bk_printf("  xmem dump [seconds_since_malloc] [start_tick] [task]\r\n");
    bk_printf("  xmem sum [func|task] [seconds_since_malloc] [start_tick] [task]\r\n");
    bk_printf("  xmem mark [name]\r\n");
    bk_printf("  xmem diff\r\n");
    bk_printf("  xmem psram\r\n");
    bk_printf("  xmem stack\r\n");
    bk_printf("  xmem task\r\n");
    bk_printf("  xmem all\r\n");
}

typedef struct {
    uint8_t valid;
    char name[16];
    uint32_t tick;
    uint32_t sram_free_bytes;
    uint32_t sram_min_free_bytes;
    uint32_t psram_free_bytes;
    uint32_t psram_min_free_bytes;
    uint32_t hsram_free_bytes;
    uint32_t hsram_min_free_bytes;
} memory_snapshot_t;

static memory_snapshot_t s_memory_snapshot;

static void memory_print_heap_line(const char *name, uint32_t total, uint32_t free, uint32_t min_free)
{
    uint32_t used = total - free;
    uint32_t peak = total - min_free;

    bk_printf("%-8s total=%-8u free=%-8u min=%-8u used=%-8u peak=%-8u\r\n",
              name, total, free, min_free, used, peak);
}

static void memory_read_snapshot(memory_snapshot_t *snapshot, const char *name)
{
    os_memset(snapshot, 0, sizeof(memory_snapshot_t));
    snapshot->valid = 1;
    snapshot->tick = rtos_get_tick_count();
    os_strlcpy(snapshot->name, name ? name : "mark", sizeof(snapshot->name));
    snapshot->sram_free_bytes = (uint32_t)rtos_get_free_heap_size();
    snapshot->sram_min_free_bytes = (uint32_t)rtos_get_minimum_free_heap_size();

#if CONFIG_PSRAM_AS_SYS_MEMORY
    snapshot->psram_free_bytes = (uint32_t)rtos_get_psram_free_heap_size();
    snapshot->psram_min_free_bytes = (uint32_t)rtos_get_psram_minimum_free_heap_size();
#endif

#if (CONFIG_CPU_INDEX == 1) && defined(CONFIG_AP_HSRAM_HEAP_ADDR)
    snapshot->hsram_free_bytes = (uint32_t)rtos_get_hsram_free_heap_size();
    snapshot->hsram_min_free_bytes = (uint32_t)rtos_get_hsram_minimum_free_heap_size();
#endif
}

static void memory_show_heap(void)
{
    bk_printf("runtime tick: %u\r\n", rtos_get_tick_count());
    memory_print_heap_line("sram",
                           (uint32_t)rtos_get_total_heap_size(),
                           (uint32_t)rtos_get_free_heap_size(),
                           (uint32_t)rtos_get_minimum_free_heap_size());

#if CONFIG_PSRAM_AS_SYS_MEMORY
    memory_print_heap_line("psram",
                           (uint32_t)rtos_get_psram_total_heap_size(),
                           (uint32_t)rtos_get_psram_free_heap_size(),
                           (uint32_t)rtos_get_psram_minimum_free_heap_size());
    bk_printf("psram used count: %u\r\n", bk_psram_heap_get_used_count());
#endif

#if (CONFIG_CPU_INDEX == 1) && defined(CONFIG_AP_HSRAM_HEAP_ADDR)
    memory_print_heap_line("hsram",
                           (uint32_t)rtos_get_hsram_total_heap_size(),
                           (uint32_t)rtos_get_hsram_free_heap_size(),
                           (uint32_t)rtos_get_hsram_minimum_free_heap_size());
#endif
}

static void memory_mark(const char *name)
{
    memory_read_snapshot(&s_memory_snapshot, name);
    bk_printf("memory mark saved: %s tick=%u\r\n", s_memory_snapshot.name, s_memory_snapshot.tick);
    memory_show_heap();
}

static void memory_print_diff_line(const char *name, uint32_t before_free, uint32_t current_free,
                                   uint32_t before_min, uint32_t current_min)
{
    bk_printf("%-8s used_delta=%-8d free_before=%-8u free_now=%-8u min_before=%-8u min_now=%-8u\r\n",
              name, (int32_t)(before_free - current_free), before_free, current_free,
              before_min, current_min);
}

static void memory_diff(void)
{
    memory_snapshot_t current;

    if (!s_memory_snapshot.valid) {
        bk_printf("no memory mark. run: xmem mark [name]\r\n");
        return;
    }

    memory_read_snapshot(&current, "current");
    bk_printf("memory diff from %s tick=%u to tick=%u\r\n",
              s_memory_snapshot.name, s_memory_snapshot.tick, current.tick);
    memory_print_diff_line("sram", s_memory_snapshot.sram_free_bytes, current.sram_free_bytes,
                           s_memory_snapshot.sram_min_free_bytes, current.sram_min_free_bytes);

#if CONFIG_PSRAM_AS_SYS_MEMORY
    memory_print_diff_line("psram", s_memory_snapshot.psram_free_bytes, current.psram_free_bytes,
                           s_memory_snapshot.psram_min_free_bytes, current.psram_min_free_bytes);
#endif

#if (CONFIG_CPU_INDEX == 1) && defined(CONFIG_AP_HSRAM_HEAP_ADDR)
    memory_print_diff_line("hsram", s_memory_snapshot.hsram_free_bytes, current.hsram_free_bytes,
                           s_memory_snapshot.hsram_min_free_bytes, current.hsram_min_free_bytes);
#endif
}

#if CONFIG_FREERTOS && CONFIG_MEM_DEBUG
static void memory_dump_allocations(int argc, char **argv, int first_arg, const char *group_hint)
{
    uint32_t start_tick = 0;
    uint32_t ticks_since_malloc = 0;
    uint32_t seconds_since_malloc = 0;
    char *task_name = NULL;
    int arg = first_arg;

    if (arg < argc) {
        if ((os_strcmp(argv[arg], "func") == 0) || (os_strcmp(argv[arg], "task") == 0)) {
            group_hint = argv[arg];
            arg++;
        }
    }

    if (arg < argc) {
        seconds_since_malloc = os_strtoul(argv[arg], NULL, 10);
        ticks_since_malloc = bk_get_ticks_per_second() * seconds_since_malloc;
        arg++;
    }

    if (arg < argc) {
        start_tick = os_strtoul(argv[arg], NULL, 10);
        arg++;
    }

    if (arg < argc) {
        task_name = argv[arg];
    }

    bk_printf("xmem dump mode=%s age_s=%u start_tick=%u task=%s\r\n",
              group_hint ? group_hint : "func", seconds_since_malloc, start_tick,
              task_name ? task_name : "all");
    bk_printf("note: aggregate the %s column from this dump; board-side aggregation needs a lower-layer heap iterator.\r\n",
              group_hint ? group_hint : "func");
    os_dump_memory_stats(start_tick, ticks_since_malloc, task_name);
}
#endif

void cli_memory_test_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if (argc < 2) {
        memory_cli_usage();
        return;
    }

    if (os_strcmp(argv[1], "show") == 0) {
        memory_show_heap();
        return;
    }

    if (os_strcmp(argv[1], "mark") == 0) {
        memory_mark((argc > 2) ? argv[2] : "mark");
        return;
    }

    if (os_strcmp(argv[1], "diff") == 0) {
        memory_diff();
        return;
    }

    if (os_strcmp(argv[1], "psram") == 0) {
        bk_psram_heap_get_used_state();
        return;
    }

    if (os_strcmp(argv[1], "stack") == 0) {
        rtos_dump_stack_memory_usage();
        return;
    }

    if (os_strcmp(argv[1], "task") == 0) {
        rtos_dump_task_list();
        rtos_dump_task_runtime_stats();
        return;
    }

#if CONFIG_FREERTOS && CONFIG_MEM_DEBUG
    if (os_strcmp(argv[1], "dump") == 0) {
        memory_dump_allocations(argc, argv, 2, "func");
        return;
    }

    if (os_strcmp(argv[1], "sum") == 0) {
        memory_dump_allocations(argc, argv, 2, "func");
        return;
    }

    if (os_strcmp(argv[1], "all") == 0) {
        memory_show_heap();
        rtos_dump_task_list();
        rtos_dump_task_runtime_stats();
        rtos_dump_stack_memory_usage();
        bk_psram_heap_get_used_state();
        memory_dump_allocations(argc, argv, 2, "func");
        return;
    }
#else
    if ((os_strcmp(argv[1], "dump") == 0) || (os_strcmp(argv[1], "sum") == 0) ||
        (os_strcmp(argv[1], "all") == 0)) {
        if (os_strcmp(argv[1], "all") == 0) {
            memory_show_heap();
            rtos_dump_task_list();
            rtos_dump_task_runtime_stats();
            rtos_dump_stack_memory_usage();
        }
        bk_printf("CONFIG_MEM_DEBUG is disabled.\r\n");
        return;
    }
#endif

    memory_cli_usage();
}

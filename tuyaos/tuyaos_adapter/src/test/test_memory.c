#include "cli.h"
#include "cli_tuya_test.h"
#include "sdkconfig.h"
#include "ram_regions.h"
#include <soc/soc.h>

#include <stdint.h>
#include <stddef.h>

#include <components/system.h>
#include <driver/aon_rtc.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>

#include "bk_rtos_debug.h"

#if (CONFIG_CPU_INDEX == 1) && defined(CONFIG_FRAME_BUFFER) && CONFIG_FRAME_BUFFER
#define MEMORY_ENABLE_SLAB_STATS 1
#include <components/bk_frame_buffer.h>
#include "ram_regions.h"
#else
#define MEMORY_ENABLE_SLAB_STATS 0
#endif

#if MEMORY_ENABLE_SLAB_STATS
#define MEM_SLAB_ALIGN_BYTES         (64)
#define MEM_SLAB_LIST_PATTERN        (0xA55AA55A)
#define MEM_SLAB_ALLOCATED_PATTERN   (0x83388338)
#define MEM_SLAB_FREE_PATTERN        (0xF00FF00F)

#ifndef CONFIG_PSRAM_MEM_SLAB_UNCODED_ADDR
#define CONFIG_PSRAM_MEM_SLAB_UNCODED_ADDR 0
#endif

#ifndef CONFIG_PSRAM_MEM_SLAB_UNCODED_SIZE
#define CONFIG_PSRAM_MEM_SLAB_UNCODED_SIZE 0
#endif

#ifndef CONFIG_PSRAM_MEM_SLAB_CODED_ADDR
#define CONFIG_PSRAM_MEM_SLAB_CODED_ADDR 0
#endif

#ifndef CONFIG_PSRAM_MEM_SLAB_CODED_SIZE
#define CONFIG_PSRAM_MEM_SLAB_CODED_SIZE 0
#endif

typedef struct memory_slab_free_block {
    uint32_t corrupt_check;
    uint32_t free_size;
    struct memory_slab_free_block *next;
    struct memory_slab_free_block *previous;
#if MEM_SLAB_MEM_DEBUG
    uint32_t reserved[11];
    uint32_t head_end_check;
#else
    uint32_t reserved[12];
#endif
} __attribute__((aligned(MEM_SLAB_ALIGN_BYTES))) memory_slab_free_block_t;

typedef struct __attribute__((aligned(MEM_SLAB_ALIGN_BYTES))) {
    uint32_t corrupt_check;
#if MEM_SLAB_MEM_DEBUG
    const char *func;
    uint32_t line;
#endif
    uint32_t size;
#if MEM_SLAB_MEM_DEBUG
    uint32_t user_size;
    uint32_t reserved[10];
    uint32_t head_end_check;
#else
    uint32_t reserved[14];
#endif
} memory_slab_used_block_t;

typedef struct {
    const char *name;
    uint32_t addr;
    uint32_t size;
} memory_slab_region_t;

typedef struct {
    uint32_t total;
    uint32_t used;
    uint32_t user_used;
    uint32_t free;
    uint32_t largest_free;
    uint32_t alloc_count;
    uint32_t free_count;
    uint32_t unknown_count;
    uint32_t scanned;
} memory_slab_stat_t;

static const memory_slab_region_t s_memory_slab_regions[] = {
    {"coded", CONFIG_PSRAM_MEM_SLAB_CODED_ADDR, CONFIG_PSRAM_MEM_SLAB_CODED_SIZE},
    {"uncoded", CONFIG_PSRAM_MEM_SLAB_UNCODED_ADDR, CONFIG_PSRAM_MEM_SLAB_UNCODED_SIZE},
};
#endif

#if (CONFIG_CPU_INDEX == 1) && defined(CONFIG_AP_EXT_SRAM_HEAP_ADDR)
extern size_t xPortGetExtSramTotalHeapSize(void);
extern size_t xPortGetExtSramFreeHeapSize(void);
extern size_t xPortGetExtSramMinimumFreeHeapSize(void);
#define MEMORY_EXT_SRAM_ENABLED 1
#else
#define MEMORY_EXT_SRAM_ENABLED 0
#endif

#define MEMORY_ALLOC_TEST_DEFAULT_SIZE      (4096U)
#define MEMORY_ALLOC_TEST_DEFAULT_COUNT     (1000U)
#define MEMORY_FALLBACK_MAX_BLOCKS          (256U)

typedef void *(*memory_alloc_func_t)(size_t size);
typedef void (*memory_free_func_t)(void *ptr);

static void *s_memory_os_test_ptr;
#if MEMORY_EXT_SRAM_ENABLED
static void *s_memory_ext_test_ptr;
static void *s_memory_fallback_ptrs[MEMORY_FALLBACK_MAX_BLOCKS];
#endif

static void memory_cli_usage(void)
{
    bk_printf("Usage:\r\n");
    bk_printf("  xmem show\r\n");
    bk_printf("  xmem ext\r\n");
    bk_printf("  xmem ext_malloc [bytes]\r\n");
    bk_printf("  xmem ext_free\r\n");
    bk_printf("  xmem ext_loop [bytes] [count]\r\n");
    bk_printf("  xmem os_malloc [bytes]\r\n");
    bk_printf("  xmem os_free\r\n");
    bk_printf("  xmem os_loop [bytes] [count]\r\n");
    bk_printf("  xmem os_fallback [block_bytes] [max_blocks]\r\n");
    bk_printf("  xmem dump [seconds_since_malloc] [start_tick] [task]\r\n");
    bk_printf("  xmem sum [func|task] [seconds_since_malloc] [start_tick] [task]\r\n");
    bk_printf("  xmem mark [name]\r\n");
    bk_printf("  xmem diff\r\n");
    bk_printf("  xmem psram\r\n");
#if MEMORY_ENABLE_SLAB_STATS
    bk_printf("  xmem slab [coded|uncoded|dump [coded|uncoded]|check]\r\n");
#endif
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
    uint32_t ext_sram_free_bytes;
    uint32_t ext_sram_min_free_bytes;
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

#if MEMORY_EXT_SRAM_ENABLED
    snapshot->ext_sram_free_bytes = (uint32_t)xPortGetExtSramFreeHeapSize();
    snapshot->ext_sram_min_free_bytes = (uint32_t)xPortGetExtSramMinimumFreeHeapSize();
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

#if MEMORY_EXT_SRAM_ENABLED
    memory_print_heap_line("ext_sram",
                           (uint32_t)xPortGetExtSramTotalHeapSize(),
                           (uint32_t)xPortGetExtSramFreeHeapSize(),
                           (uint32_t)xPortGetExtSramMinimumFreeHeapSize());
    bk_printf("ext_sram range: [0x%08x, 0x%08x)\r\n",
              (uint32_t)SOC_SRAM_CPU_ADDR(CONFIG_AP_EXT_SRAM_HEAP_ADDR),
              (uint32_t)SOC_SRAM_CPU_ADDR(CONFIG_AP_EXT_SRAM_HEAP_ADDR + CONFIG_AP_EXT_SRAM_HEAP_SIZE));
#endif
}

#if MEMORY_ENABLE_SLAB_STATS
static uint32_t memory_slab_align_up(uint32_t value)
{
    return (value + (MEM_SLAB_ALIGN_BYTES - 1)) & ~(MEM_SLAB_ALIGN_BYTES - 1);
}

static uint32_t memory_slab_align_down(uint32_t value)
{
    return value & ~(MEM_SLAB_ALIGN_BYTES - 1);
}

static int memory_slab_read_stat(const memory_slab_region_t *region, memory_slab_stat_t *stat)
{
    uint32_t heap_start = memory_slab_align_up(region->addr);
    uint32_t heap_size = memory_slab_align_down(region->size - (heap_start - region->addr));
    uint32_t heap_end = heap_start + heap_size;
    uint32_t current = heap_start;

    os_memset(stat, 0, sizeof(*stat));
    stat->total = heap_size;

    if ((region->addr == 0) || (region->size == 0) || (heap_size < sizeof(memory_slab_free_block_t))) {
        return -1;
    }

    while (current < heap_end) {
        memory_slab_used_block_t *used = (memory_slab_used_block_t *)(uintptr_t)current;
        uint32_t remain = heap_end - current;

        if (remain < sizeof(memory_slab_used_block_t)) {
            stat->unknown_count++;
            break;
        }

        if (used->corrupt_check == MEM_SLAB_ALLOCATED_PATTERN) {
            uint32_t block_size = used->size;

            if ((block_size < sizeof(memory_slab_used_block_t)) || (block_size > remain)) {
                stat->unknown_count++;
                break;
            }

            stat->alloc_count++;
            stat->used += block_size;
#if MEM_SLAB_MEM_DEBUG
            stat->user_used += used->user_size;
#else
            stat->user_used += block_size - sizeof(memory_slab_used_block_t);
#endif
            current += block_size;
        } else if (used->corrupt_check == MEM_SLAB_LIST_PATTERN) {
            memory_slab_free_block_t *free_block = (memory_slab_free_block_t *)(uintptr_t)current;
            uint32_t free_size = free_block->free_size;

            if ((free_size < sizeof(memory_slab_free_block_t)) || (free_size > remain)) {
                stat->unknown_count++;
                break;
            }

            stat->free_count++;
            stat->free += free_size;
            if (free_size > stat->largest_free) {
                stat->largest_free = free_size;
            }
            current += free_size;
        } else if (used->corrupt_check == MEM_SLAB_FREE_PATTERN) {
            uint32_t block_size = used->size;

            if ((block_size < sizeof(memory_slab_used_block_t)) || (block_size > remain)) {
                stat->unknown_count++;
                break;
            }

            current += block_size;
        } else {
            stat->unknown_count++;
            break;
        }
    }

    stat->scanned = current - heap_start;
    return (stat->unknown_count == 0) ? 0 : -2;
}

static void memory_print_slab_line(const memory_slab_region_t *region)
{
    memory_slab_stat_t stat;
    int ret = memory_slab_read_stat(region, &stat);

    if (ret == -1) {
        bk_printf("%-8s not configured\r\n", region->name);
        return;
    }

    if ((stat.alloc_count == 0) && (stat.free_count == 0)) {
        bk_printf("%-8s not initialized or invalid heap head, addr=0x%08x size=%u\r\n",
                  region->name, region->addr, region->size);
        return;
    }

    bk_printf("%-8s total=%-9u used=%-9u user=%-9u free=%-9u largest=%-9u alloc=%-5u free_blk=%-5u",
              region->name, stat.total, stat.used, stat.user_used, stat.free,
              stat.largest_free, stat.alloc_count, stat.free_count);

    if (ret != 0) {
        bk_printf(" warn=scan_incomplete scanned=%u unknown=%u", stat.scanned, stat.unknown_count);
    }

    bk_printf("\r\n");
}

static void memory_show_slab(void)
{
    bk_printf("mem slab usage, used includes allocator headers, user is requested bytes\r\n");
    for (uint32_t i = 0; i < sizeof(s_memory_slab_regions) / sizeof(s_memory_slab_regions[0]); i++) {
        memory_print_slab_line(&s_memory_slab_regions[i]);
    }
}

static void memory_show_slab_heap(uint8_t heap_type)
{
    if (heap_type >= (sizeof(s_memory_slab_regions) / sizeof(s_memory_slab_regions[0]))) {
        bk_printf("invalid slab heap type: %u\r\n", heap_type);
        return;
    }

    bk_printf("mem slab usage, used includes allocator headers, user is requested bytes\r\n");
    memory_print_slab_line(&s_memory_slab_regions[heap_type]);
}

static int memory_slab_heap_type_from_name(const char *name)
{
    if (name == NULL) {
        return -1;
    }

    if ((os_strcmp(name, "coded") == 0) || (os_strcmp(name, "0") == 0)) {
        return MEM_SLAB_HEAP_CODED;
    }

    if ((os_strcmp(name, "uncoded") == 0) || (os_strcmp(name, "1") == 0)) {
        return MEM_SLAB_HEAP_UNCODED;
    }

    return -1;
}

static void memory_dump_slab_heap(const char *heap_name)
{
    int heap_type = memory_slab_heap_type_from_name(heap_name);

    if (heap_name == NULL) {
        bk_mem_slab_dump_all_heaps();
        return;
    }

    if (heap_type < 0) {
        bk_printf("invalid slab heap: %s, use coded or uncoded\r\n", heap_name);
        return;
    }

    bk_mem_slab_dump_heap((uint8_t)heap_type);
}
#endif

static uint32_t memory_arg_u32(int argc, char **argv, int index, uint32_t default_value)
{
    if ((index < argc) && (argv[index] != NULL)) {
        return os_strtoul(argv[index], NULL, 10);
    }

    return default_value;
}

static void memory_touch_block(void *ptr, uint32_t size)
{
    volatile uint8_t *buf = (volatile uint8_t *)ptr;

    if ((buf == NULL) || (size == 0U)) {
        return;
    }

    buf[0] = 0x5a;
    buf[size - 1U] = 0xa5;
}

static void memory_os_free_wrapper(void *ptr)
{
    os_free(ptr);
}

static void *memory_os_malloc_wrapper(size_t size)
{
    return os_malloc(size);
}

static void memory_print_timing(const char *name, uint32_t size, uint32_t count,
                                uint32_t ok_count, uint32_t fail_count, uint64_t elapsed_us)
{
    uint64_t avg_x100 = (count == 0U) ? 0U : ((elapsed_us * 100U) / count);

    bk_printf("%s size=%u count=%u ok=%u fail=%u elapsed=%llu us avg=%llu.%02llu us/op\r\n",
              name, size, count, ok_count, fail_count,
              elapsed_us, avg_x100 / 100U, avg_x100 % 100U);
}

static void memory_run_alloc_loop(const char *name, memory_alloc_func_t alloc_fn,
                                  memory_free_func_t free_fn, uint32_t size, uint32_t count)
{
    uint32_t ok_count = 0;
    uint32_t fail_count = 0;
    uint64_t start_us;
    uint64_t elapsed_us;

    if ((alloc_fn == NULL) || (free_fn == NULL) || (count == 0U)) {
        bk_printf("invalid %s test parameter\r\n", name);
        return;
    }

    start_us = bk_aon_rtc_get_us();
    for (uint32_t i = 0; i < count; i++) {
        void *ptr = alloc_fn(size);

        if (ptr == NULL) {
            fail_count++;
            continue;
        }

        memory_touch_block(ptr, size);
        free_fn(ptr);
        ok_count++;
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;

    memory_print_timing(name, size, count, ok_count, fail_count, elapsed_us);
    memory_show_heap();
}

static void memory_test_single_alloc(const char *name, memory_alloc_func_t alloc_fn,
                                     memory_free_func_t free_fn, void **saved_ptr, uint32_t size)
{
    uint64_t start_us;
    uint64_t elapsed_us;
    void *ptr;

    if ((alloc_fn == NULL) || (free_fn == NULL) || (saved_ptr == NULL)) {
        bk_printf("invalid %s malloc parameter\r\n", name);
        return;
    }

    if (*saved_ptr != NULL) {
        free_fn(*saved_ptr);
        *saved_ptr = NULL;
    }

    start_us = bk_aon_rtc_get_us();
    ptr = alloc_fn(size);
    elapsed_us = bk_aon_rtc_get_us() - start_us;

    memory_touch_block(ptr, size);
    *saved_ptr = ptr;

    bk_printf("%s malloc size=%u ptr=%p elapsed=%llu us\r\n", name, size, ptr, elapsed_us);
    memory_show_heap();
}

static void memory_test_single_free(const char *name, memory_free_func_t free_fn, void **saved_ptr)
{
    uint64_t start_us;
    uint64_t elapsed_us;

    if ((free_fn == NULL) || (saved_ptr == NULL)) {
        bk_printf("invalid %s free parameter\r\n", name);
        return;
    }

    if (*saved_ptr == NULL) {
        bk_printf("%s free skipped: no saved pointer\r\n", name);
        return;
    }

    start_us = bk_aon_rtc_get_us();
    free_fn(*saved_ptr);
    elapsed_us = bk_aon_rtc_get_us() - start_us;

    bk_printf("%s free ptr=%p elapsed=%llu us\r\n", name, *saved_ptr, elapsed_us);
    *saved_ptr = NULL;
    memory_show_heap();
}

#if MEMORY_EXT_SRAM_ENABLED
static void *memory_ext_sram_malloc_wrapper(size_t size)
{
    return ext_sram_malloc(size);
}

static void memory_ext_sram_free_wrapper(void *ptr)
{
    ext_sram_free(ptr);
}

static uint8_t memory_ptr_in_ext_sram(const void *ptr)
{
    uintptr_t addr = (uintptr_t)ptr;
    uintptr_t start = (uintptr_t)SOC_SRAM_CPU_ADDR(CONFIG_AP_EXT_SRAM_HEAP_ADDR);
    uintptr_t end = (uintptr_t)SOC_SRAM_CPU_ADDR(CONFIG_AP_EXT_SRAM_HEAP_ADDR + CONFIG_AP_EXT_SRAM_HEAP_SIZE);

    return ((addr >= start) && (addr < end)) ? 1U : 0U;
}

static void memory_test_os_fallback(uint32_t block_size, uint32_t max_blocks)
{
    uint32_t alloc_count = 0;
    uint32_t ext_count = 0;
    uint32_t ext_index = UINT32_MAX;
    uint32_t fail_index = UINT32_MAX;
    uint64_t start_us;
    uint64_t elapsed_us;
    uint32_t before_sram = (uint32_t)rtos_get_free_heap_size();
    uint32_t before_ext = (uint32_t)xPortGetExtSramFreeHeapSize();

    if (block_size == 0U) {
        block_size = MEMORY_ALLOC_TEST_DEFAULT_SIZE;
    }

    if ((max_blocks == 0U) || (max_blocks > MEMORY_FALLBACK_MAX_BLOCKS)) {
        max_blocks = MEMORY_FALLBACK_MAX_BLOCKS;
    }

    os_memset(s_memory_fallback_ptrs, 0, sizeof(s_memory_fallback_ptrs));

    start_us = bk_aon_rtc_get_us();
    for (uint32_t i = 0; i < max_blocks; i++) {
        void *ptr = os_malloc(block_size);

        if (ptr == NULL) {
            fail_index = i;
            break;
        }

        memory_touch_block(ptr, block_size);
        s_memory_fallback_ptrs[i] = ptr;
        alloc_count++;

        if (memory_ptr_in_ext_sram(ptr)) {
            ext_count++;
            if (ext_index == UINT32_MAX) {
                ext_index = i;
            }
        }
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;

    bk_printf("os_fallback block=%u max=%u alloc=%u ext_alloc=%u first_ext_index=%d fail_index=%d elapsed=%llu us\r\n",
              block_size, max_blocks, alloc_count, ext_count,
              (ext_index == UINT32_MAX) ? -1 : (int32_t)ext_index,
              (fail_index == UINT32_MAX) ? -1 : (int32_t)fail_index, elapsed_us);
    bk_printf("before: sram_free=%u ext_free=%u\r\n", before_sram, before_ext);
    bk_printf("during: sram_free=%u ext_free=%u\r\n",
              (uint32_t)rtos_get_free_heap_size(), (uint32_t)xPortGetExtSramFreeHeapSize());

    for (uint32_t i = 0; i < alloc_count; i++) {
        os_free(s_memory_fallback_ptrs[i]);
        s_memory_fallback_ptrs[i] = NULL;
    }

    bk_printf("after:  sram_free=%u ext_free=%u\r\n",
              (uint32_t)rtos_get_free_heap_size(), (uint32_t)xPortGetExtSramFreeHeapSize());
    if (ext_index == UINT32_MAX) {
        bk_printf("os_fallback result: ext_sram was not used. Try larger block or max_blocks.\r\n");
    } else if (fail_index != UINT32_MAX) {
        bk_printf("os_fallback result: os_malloc used AP_EXT_SRAM_HEAP, then failed when both heaps could not satisfy block.\r\n");
    } else {
        bk_printf("os_fallback result: os_malloc used AP_EXT_SRAM_HEAP, max_blocks reached before allocation failure.\r\n");
    }
}
#endif

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

#if MEMORY_EXT_SRAM_ENABLED
    memory_print_diff_line("ext_sram", s_memory_snapshot.ext_sram_free_bytes, current.ext_sram_free_bytes,
                           s_memory_snapshot.ext_sram_min_free_bytes, current.ext_sram_min_free_bytes);
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

    if ((argc < 2) || (os_strcmp(argv[1], "help") == 0)) {
        memory_cli_usage();
        return;
    }

    if (os_strcmp(argv[1], "show") == 0) {
        memory_show_heap();
        return;
    }

    if ((os_strcmp(argv[1], "ext") == 0) || (os_strcmp(argv[1], "extra") == 0)) {
#if MEMORY_EXT_SRAM_ENABLED
        memory_print_heap_line("ext_sram",
                               (uint32_t)xPortGetExtSramTotalHeapSize(),
                               (uint32_t)xPortGetExtSramFreeHeapSize(),
                               (uint32_t)xPortGetExtSramMinimumFreeHeapSize());
        bk_printf("ext_sram range: [0x%08x, 0x%08x)\r\n",
                  (uint32_t)SOC_SRAM_CPU_ADDR(CONFIG_AP_EXT_SRAM_HEAP_ADDR),
                  (uint32_t)SOC_SRAM_CPU_ADDR(CONFIG_AP_EXT_SRAM_HEAP_ADDR + CONFIG_AP_EXT_SRAM_HEAP_SIZE));
#else
        bk_printf("AP_EXT_SRAM_HEAP is not configured.\r\n");
#endif
        return;
    }

    if (os_strcmp(argv[1], "ext_malloc") == 0) {
#if MEMORY_EXT_SRAM_ENABLED
        memory_test_single_alloc("ext_sram", memory_ext_sram_malloc_wrapper, memory_ext_sram_free_wrapper,
                                 &s_memory_ext_test_ptr,
                                 memory_arg_u32(argc, argv, 2, MEMORY_ALLOC_TEST_DEFAULT_SIZE));
#else
        bk_printf("AP_EXT_SRAM_HEAP is not configured.\r\n");
#endif
        return;
    }

    if (os_strcmp(argv[1], "ext_free") == 0) {
#if MEMORY_EXT_SRAM_ENABLED
        memory_test_single_free("ext_sram", memory_ext_sram_free_wrapper, &s_memory_ext_test_ptr);
#else
        bk_printf("AP_EXT_SRAM_HEAP is not configured.\r\n");
#endif
        return;
    }

    if (os_strcmp(argv[1], "ext_loop") == 0) {
#if MEMORY_EXT_SRAM_ENABLED
        memory_run_alloc_loop("ext_sram malloc/free",
                              memory_ext_sram_malloc_wrapper,
                              memory_ext_sram_free_wrapper,
                              memory_arg_u32(argc, argv, 2, MEMORY_ALLOC_TEST_DEFAULT_SIZE),
                              memory_arg_u32(argc, argv, 3, MEMORY_ALLOC_TEST_DEFAULT_COUNT));
#else
        bk_printf("AP_EXT_SRAM_HEAP is not configured.\r\n");
#endif
        return;
    }

    if (os_strcmp(argv[1], "os_malloc") == 0) {
        memory_test_single_alloc("os", memory_os_malloc_wrapper, memory_os_free_wrapper,
                                 &s_memory_os_test_ptr,
                                 memory_arg_u32(argc, argv, 2, MEMORY_ALLOC_TEST_DEFAULT_SIZE));
        return;
    }

    if (os_strcmp(argv[1], "os_free") == 0) {
        memory_test_single_free("os", memory_os_free_wrapper, &s_memory_os_test_ptr);
        return;
    }

    if (os_strcmp(argv[1], "os_loop") == 0) {
        memory_run_alloc_loop("os malloc/free",
                              memory_os_malloc_wrapper,
                              memory_os_free_wrapper,
                              memory_arg_u32(argc, argv, 2, MEMORY_ALLOC_TEST_DEFAULT_SIZE),
                              memory_arg_u32(argc, argv, 3, MEMORY_ALLOC_TEST_DEFAULT_COUNT));
        return;
    }

    if (os_strcmp(argv[1], "os_fallback") == 0) {
#if MEMORY_EXT_SRAM_ENABLED
#if defined(CONFIG_OS_MALLOC_EXT_SRAM_FALLBACK)
        memory_test_os_fallback(memory_arg_u32(argc, argv, 2, MEMORY_ALLOC_TEST_DEFAULT_SIZE),
                                memory_arg_u32(argc, argv, 3, 128U));
#else
        bk_printf("CONFIG_OS_MALLOC_EXT_SRAM_FALLBACK is disabled.\r\n");
#endif
#else
        bk_printf("AP_EXT_SRAM_HEAP is not configured.\r\n");
#endif
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

#if MEMORY_ENABLE_SLAB_STATS
    if (os_strcmp(argv[1], "slab") == 0) {
        if ((argc > 2) && (os_strcmp(argv[2], "dump") == 0)) {
            memory_dump_slab_heap((argc > 3) ? argv[3] : NULL);
        } else if ((argc > 2) && (os_strcmp(argv[2], "check") == 0)) {
            bk_mem_slab_check_all_heaps();
        } else if (argc > 2) {
            int heap_type = memory_slab_heap_type_from_name(argv[2]);

            if (heap_type < 0) {
                bk_printf("invalid slab argument: %s\r\n", argv[2]);
                memory_cli_usage();
            } else {
                memory_show_slab_heap((uint8_t)heap_type);
            }
        } else {
            memory_show_slab();
        }
        return;
    }
#endif

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
#if MEMORY_ENABLE_SLAB_STATS
        memory_show_slab();
#endif
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

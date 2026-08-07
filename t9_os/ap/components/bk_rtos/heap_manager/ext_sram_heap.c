#ifdef CONFIG_AP_EXT_SRAM_HEAP_ADDR
#include <stdint.h>
#include <stdbool.h>
#include "bk_heap.h"
#include "bk_heap/port/port_heap.h"
#include "bk_heap_impl/heap_4_1.h"
#include "os_heap_debug.h"
#include <soc/soc.h>

// Modified by TUYA Start
/* Match hsram_heap.c: with CONFIG_SRAM_DIRECT_ADDR, AP must use the 0x2c alias
 * (SOC_SRAM_CPU_ADDR). Raw 0x28 peri base faults on memory_touch / CPU access. */
#define EXT_SRAM_HEAP_START_ADDRESS    (uint32_t)SOC_SRAM_CPU_ADDR(CONFIG_AP_EXT_SRAM_HEAP_ADDR)
#define EXT_SRAM_HEAP_END_ADDRESS      (uint32_t)SOC_SRAM_CPU_ADDR(CONFIG_AP_EXT_SRAM_HEAP_ADDR + CONFIG_AP_EXT_SRAM_HEAP_SIZE)
// Modified by TUYA End
#define EXT_SRAM_HEAP_SIZE             CONFIG_AP_EXT_SRAM_HEAP_SIZE

static bk_heap_region_id_t s_ext_sram_region_id = BK_HEAP_INVALID_REGION_ID;
static Heap_4_1_Data_t s_ext_sram_heap_data;

static void ext_sram_heap_init(void)
{
    if (s_ext_sram_region_id != BK_HEAP_INVALID_REGION_ID) {
        return;
    }
    s_ext_sram_region_id = bk_heap_add_region(EXT_SRAM_HEAP_START_ADDRESS, EXT_SRAM_HEAP_SIZE,
                                              &heap_4_1_ops, &s_ext_sram_heap_data);
    BK_ASSERT(s_ext_sram_region_id != BK_HEAP_INVALID_REGION_ID);
#if CONFIG_MEM_DEBUG
    bk_heap_ext_sram_debug_init();
#endif
}

static void insure_ext_sram_heap_init(void)
{
    if (s_ext_sram_region_id == BK_HEAP_INVALID_REGION_ID) {
        port_heap_enter_critical();
        ext_sram_heap_init();
        port_heap_exit_critical();
    }
}

void *ext_sram_malloc_impl(size_t size)
{
    insure_ext_sram_heap_init();
    return bk_heap_malloc(s_ext_sram_region_id, size);
}

void ext_sram_free_impl(void *ptr)
{
    bk_heap_free(s_ext_sram_region_id, ptr);
}

bool ptr_is_ext_sram_heap(void *ptr)
{
    return (ptr >= (void *)EXT_SRAM_HEAP_START_ADDRESS && ptr < (void *)EXT_SRAM_HEAP_END_ADDRESS);
}

#if CONFIG_MEM_DEBUG
size_t ext_sram_get_allocated_size(void *ptr)
{
    return bk_heap_get_allocated_size(s_ext_sram_region_id, ptr);
}
#endif

size_t xPortGetExtSramTotalHeapSize(void)
{
    return EXT_SRAM_HEAP_SIZE;
}

size_t xPortGetExtSramFreeHeapSize(void)
{
    if (s_ext_sram_region_id == BK_HEAP_INVALID_REGION_ID) {
        return EXT_SRAM_HEAP_SIZE;
    }
    return bk_heap_get_free_size(s_ext_sram_region_id);
}

size_t xPortGetExtSramMinimumFreeHeapSize(void)
{
    if (s_ext_sram_region_id == BK_HEAP_INVALID_REGION_ID) {
        return EXT_SRAM_HEAP_SIZE;
    }
    return bk_heap_get_min_free_size(s_ext_sram_region_id);
}

#endif

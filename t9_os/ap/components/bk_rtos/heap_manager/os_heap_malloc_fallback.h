// Copyright 2026 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <stddef.h>

/**
 * Scheme C: when primary os heap (Kconfig OS_HEAP_TYPE) is exhausted, fall back
 * to AP_EXT_SRAM_HEAP. Requires CONFIG_OS_MALLOC_EXT_SRAM_FALLBACK and
 * CONFIG_AP_EXT_SRAM_HEAP_ADDR from ram_regions.csv.
 */
#if defined(CONFIG_OS_MALLOC_EXT_SRAM_FALLBACK) && defined(CONFIG_AP_EXT_SRAM_HEAP_ADDR)
#define OS_MALLOC_EXT_SRAM_FALLBACK_ENABLED    1
#else
#define OS_MALLOC_EXT_SRAM_FALLBACK_ENABLED    0
#endif

#if defined(CONFIG_OS_MALLOC_EXT_SRAM_FALLBACK) && !defined(CONFIG_AP_EXT_SRAM_HEAP_ADDR)
#error "CONFIG_OS_MALLOC_EXT_SRAM_FALLBACK requires AP_EXT_SRAM_HEAP in ram_regions.csv"
#endif

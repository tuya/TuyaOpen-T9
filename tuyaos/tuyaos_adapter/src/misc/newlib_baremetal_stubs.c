/**
 * @file newlib_baremetal_stubs.c
 * @brief Newlib symbols missing when linking without full CRT (crti/crtn)
 * @version 1.0
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */

/* ---------------------------------------------------------------------------
 * Function implementations
 * --------------------------------------------------------------------------- */

/**
 * @brief Finalization hook invoked by newlib __libc_fini_array
 * @return none
 * @note On hosted Linux, _fini comes from crtn.o. Bare-metal images that still
 *       pull libc.a (e.g. thumb/v8-m) must provide a Thumb _fini or the linker
 *       reports undefined reference and may emit dangerous relocation if _fini
 *       is satisfied with a non-function symbol.
 */
void _fini(void)
{
}

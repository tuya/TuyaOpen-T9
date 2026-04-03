/**
 * @file tkl_hci.c
 * @brief Auto-generated TKL adapter stub. Candidate HCI adapter for T9.
 * @version 0.1
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_hci.h"

OPERATE_RET tkl_hci_init(VOID)
{
    /* TODO: map `tkl_hci_init` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_hci_deinit(VOID)
{
    /* TODO: map `tkl_hci_deinit` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_hci_reset(VOID)
{
    /* TODO: map `tkl_hci_reset` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_hci_cmd_packet_send(CONST UCHAR_T *p_buf, USHORT_T buf_len)
{
    (void)p_buf;
    (void)buf_len;

    /* TODO: map `tkl_hci_cmd_packet_send` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_hci_acl_packet_send(CONST UCHAR_T *p_buf, USHORT_T buf_len)
{
    (void)p_buf;
    (void)buf_len;

    /* TODO: map `tkl_hci_acl_packet_send` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_hci_callback_register(CONST TKL_HCI_FUNC_CB hci_evt_cb, CONST TKL_HCI_FUNC_CB acl_pkt_cb)
{
    (void)hci_evt_cb;
    (void)acl_pkt_cb;

    /* TODO: map `tkl_hci_callback_register` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

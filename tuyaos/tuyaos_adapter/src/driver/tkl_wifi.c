/**
 * @file tkl_wifi.c
 * @brief Auto-generated TKL adapter stub. Candidate Wifi adapter for T9.
 * @version 0.1
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_wifi.h"

OPERATE_RET tkl_wifi_init(WIFI_EVENT_CB cb)
{
    (void)cb;

    /* TODO: map `tkl_wifi_init` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_scan_ap(CONST SCHAR_T *ssid, AP_IF_S **ap_ary, UINT_T *num)
{
    (void)ssid;
    (void)ap_ary;
    (void)num;

    /* TODO: map `tkl_wifi_scan_ap` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_release_ap(AP_IF_S *ap)
{
    (void)ap;

    /* TODO: map `tkl_wifi_release_ap` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_start_ap(CONST WF_AP_CFG_IF_S *cfg)
{
    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)cfg;

    /* TODO: map `tkl_wifi_start_ap` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_stop_ap(VOID_T)
{
    /* TODO: map `tkl_wifi_stop_ap` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_set_cur_channel(CONST UCHAR_T chan)
{
    (void)chan;

    /* TODO: map `tkl_wifi_set_cur_channel` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_get_cur_channel(UCHAR_T *chan)
{
    (void)chan;

    /* TODO: map `tkl_wifi_get_cur_channel` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_set_sniffer(CONST BOOL_T en, CONST SNIFFER_CALLBACK cb)
{
    (void)en;
    (void)cb;

    /* TODO: map `tkl_wifi_set_sniffer` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_get_ip(CONST WF_IF_E wf, NW_IP_S *ip)
{
    (void)wf;
    (void)ip;

    /* TODO: map `tkl_wifi_get_ip` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_get_ipv6(CONST WF_IF_E wf, NW_IP_TYPE type, NW_IP_S *ip)
{
    (void)wf;
    (void)type;
    (void)ip;

    /* TODO: map `tkl_wifi_get_ipv6` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_set_ip(CONST WF_IF_E wf, NW_IP_S *ip)
{
    (void)wf;
    (void)ip;

    /* TODO: map `tkl_wifi_set_ip` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_set_mac(CONST WF_IF_E wf, CONST NW_MAC_S *mac)
{
    (void)wf;
    (void)mac;

    /* TODO: map `tkl_wifi_set_mac` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_get_mac(CONST WF_IF_E wf, NW_MAC_S *mac)
{
    (void)wf;
    (void)mac;

    /* TODO: map `tkl_wifi_get_mac` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_set_work_mode(CONST WF_WK_MD_E mode)
{
    (void)mode;

    /* TODO: map `tkl_wifi_set_work_mode` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_get_work_mode(WF_WK_MD_E *mode)
{
    (void)mode;

    /* TODO: map `tkl_wifi_get_work_mode` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_get_connected_ap_info(FAST_WF_CONNECTED_AP_INFO_T **fast_ap_info)
{
    (void)fast_ap_info;

    /* TODO: map `tkl_wifi_get_connected_ap_info` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_get_bssid(UCHAR_T *mac)
{
    (void)mac;

    /* TODO: map `tkl_wifi_get_bssid` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_set_country_code(CONST COUNTRY_CODE_E ccode)
{
    (void)ccode;

    /* TODO: map `tkl_wifi_set_country_code` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_set_rf_calibrated(VOID_T)
{
    /* TODO: map `tkl_wifi_set_rf_calibrated` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_set_lp_mode(CONST BOOL_T enable, CONST UCHAR_T dtim)
{
    (void)enable;
    (void)dtim;

    /* TODO: map `tkl_wifi_set_lp_mode` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_station_fast_connect(CONST FAST_WF_CONNECTED_AP_INFO_T *fast_ap_info)
{
    (void)fast_ap_info;

    /* TODO: map `tkl_wifi_station_fast_connect` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_station_connect(CONST SCHAR_T *ssid, CONST SCHAR_T *passwd)
{
    (void)ssid;
    (void)passwd;

    /* TODO: map `tkl_wifi_station_connect` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_station_disconnect(VOID_T)
{
    /* TODO: map `tkl_wifi_station_disconnect` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_station_get_conn_ap_rssi(SCHAR_T *rssi)
{
    (void)rssi;

    /* TODO: map `tkl_wifi_station_get_conn_ap_rssi` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_station_get_status(WF_STATION_STAT_E *stat)
{
    (void)stat;

    /* TODO: map `tkl_wifi_station_get_status` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_send_mgnt(CONST UCHAR_T *buf, CONST UINT_T len)
{
    if (buf == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)buf;
    (void)len;

    /* TODO: map `tkl_wifi_send_mgnt` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_register_recv_mgnt_callback(CONST BOOL_T enable, CONST WIFI_REV_MGNT_CB recv_cb)
{
    (void)enable;
    (void)recv_cb;

    /* TODO: map `tkl_wifi_register_recv_mgnt_callback` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_ioctl(WF_IOCTL_CMD_E cmd, VOID *args)
{
    (void)cmd;
    (void)args;

    /* TODO: map `tkl_wifi_ioctl` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_get_country_code(UCHAR_T *ccode)
{
    (void)ccode;

    /* TODO: map `tkl_wifi_get_country_code` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_wifi_set_country_code_v2(CONST UCHAR_T *ccode)
{
    (void)ccode;

    /* TODO: map `tkl_wifi_set_country_code_v2` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

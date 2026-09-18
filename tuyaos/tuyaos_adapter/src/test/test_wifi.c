/*
 * test_wifi.c
 * Copyright (C) 2026 Tuya
 *
 * Distributed under terms of the MIT license.
 */

#include "cli.h"
#include "cli_tuya_test.h"
#include "tkl_system.h"
#include "tkl_wifi.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_svc_devos.h"
#include "gw_intf.h"
#include "modules/wifi.h"

#include <os/mem.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WIFI_TEST_DEFAULT_WAIT_MS      20000U
#define WIFI_TEST_WAIT_POLL_MS         200U
#define WIFI_TEST_BENCH_POLL_MS        50U
#define WIFI_TEST_DEFAULT_AP_CHANNEL   6U
#define WIFI_TEST_FAST_TIMEOUT_MS      5000U
#define WIFI_TEST_FAST_TARGET_MS       1500U
#define WIFI_TEST_MIN_PASS_PCT         95U
#define WIFI_TEST_DISCONNECT_WAIT_MS   1000U
#define WIFI_TEST_RSSI_INTERVAL_MS     1000U
#define WIFI_TEST_BURST_INTERVAL_MS    100U

static FAST_WF_CONNECTED_AP_INFO_T *s_wifi_fast_info = NULL;
static volatile uint32_t s_wifi_event_connected = 0;
static volatile uint32_t s_wifi_event_connect_failed = 0;
static volatile uint32_t s_wifi_event_disconnected = 0;
static volatile WF_EVENT_E s_wifi_last_event = WFE_DISCONNECTED;
static volatile WF_DISCONN_REASON_E s_wifi_last_reason = TUYA_WLAN_REASON_NONE;

static const char *wifi_test_event_name(WF_EVENT_E event)
{
    switch (event) {
        case WFE_CONNECTED:
            return "connected";
        case WFE_CONNECT_FAILED:
            return "connect_failed";
        case WFE_DISCONNECTED:
            return "disconnected";
        default:
            return "unknown";
    }
}

static const char *wifi_test_status_name(WF_STATION_STAT_E status)
{
    switch (status) {
        case WSS_IDLE:
            return "idle";
        case WSS_CONNECTING:
            return "connecting";
        case WSS_PASSWD_WRONG:
            return "passwd_wrong";
        case WSS_NO_AP_FOUND:
            return "no_ap_found";
        case WSS_CONN_FAIL:
            return "conn_fail";
        case WSS_CONN_SUCCESS:
            return "conn_success";
        case WSS_GOT_IP:
            return "got_ip";
        case WSS_DHCP_FAIL:
            return "dhcp_fail";
        default:
            return "unknown";
    }
}

static const char *wifi_test_mode_name(WF_WK_MD_E mode)
{
    switch (mode) {
        case WWM_POWERDOWN:
            return "pd";
        case WWM_SNIFFER:
            return "sniffer";
        case WWM_STATION:
            return "sta";
        case WWM_SOFTAP:
            return "ap";
        case WWM_STATIONAP:
            return "staap";
        default:
            return "unknown";
    }
}

static WF_WK_MD_E wifi_test_parse_mode(const char *mode)
{
    if (mode == NULL) {
        return WWM_UNKNOWN;
    }

    if ((strcmp(mode, "pd") == 0) || (strcmp(mode, "powerdown") == 0)) {
        return WWM_POWERDOWN;
    }
    if (strcmp(mode, "sniffer") == 0) {
        return WWM_SNIFFER;
    }
    if ((strcmp(mode, "sta") == 0) || (strcmp(mode, "station") == 0)) {
        return WWM_STATION;
    }
    if ((strcmp(mode, "ap") == 0) || (strcmp(mode, "softap") == 0)) {
        return WWM_SOFTAP;
    }
    if ((strcmp(mode, "staap") == 0) || (strcmp(mode, "stationap") == 0)) {
        return WWM_STATIONAP;
    }

    return WWM_UNKNOWN;
}

static const char *wifi_test_devos_state_name(DEVOS_STATE_E state)
{
    switch (state) {
        case DEVOS_STATE_INIT:
            return "init";
        case DEVOS_STATE_UNREGISTERED:
            return "unregistered";
        case DEVOS_STATE_REGISTERING:
            return "registering";
        case DEVOS_STATE_ACTIVATED:
            return "activated";
        case DEVOS_STATE_UPGRADING:
            return "upgrading";
        default:
            return "unknown";
    }
}

static const char *wifi_test_gw_reg_state_name(GW_WORK_STAT_T state)
{
    switch (state) {
        case UNREGISTERED:
            return "unregistered";
        case REGISTERED:
            return "registered";
        case ACTIVATED:
            return "activated";
        case PROXY_ACTIVING:
            return "proxy_activating";
        case PROXY_ACTIVATED:
            return "proxy_activated";
        default:
            return "unknown";
    }
}

static const char *wifi_test_nc_type_name(GW_WF_NWC_STAT_T nc_tp)
{
    switch (nc_tp) {
        case GWNS_LOWPOWER:
            return "lowpower";
        case GWNS_UNCFG_SMC:
            return "ez";
        case GWNS_UNCFG_AP:
            return "ap";
        case GWNS_TY_SMARTCFG:
            return "ez_cfgd";
        case GWNS_TY_AP:
            return "ap_cfgd";
        case GWNS_WECHAT_AK:
            return "wechat";
        case GWNS_OTHER_CFGED:
            return "other_cfgd";
        case GWNS_OTHER_UNCFG:
            return "other_uncfg";
        case GWNS_TY_SMART_AP_CFG:
            return "ap_ez_cfgd";
        case GWNS_UNCFG_SMC_AP:
            return "ap_ez";
        case GWNS_PROXY_ACTIVED:
            return "proxy_activated";
        case GWNS_PROXY_CONFIG_WF:
            return "proxy_cfg_wifi";
        case GWNS_MATTER_COMMISSIONED:
            return "matter";
        default:
            return "unknown";
    }
}

static const char *wifi_test_fast_nc_type_name(GW_WF_NWC_FAST_STAT_T nc_tp)
{
    switch (nc_tp) {
        case GWNS_FAST_LOWPOWER:
            return "lowpower";
        case GWNS_FAST_UNCFG_SMC:
            return "ez";
        case GWNS_FAST_UNCFG_AP:
            return "ap";
        case GWNS_FAST_UNCFG_NORMAL:
            return "normal";
        case GWNS_FAST_UNCFG_SMC_AP:
            return "ap_ez";
        default:
            return "unknown";
    }
}

static const char *wifi_test_gw_md_name(GW_WF_MD_T mode)
{
    switch (mode) {
        case GWM_NORMAL:
            return "normal";
        case GWM_SPECIAL_SMT_CFG:
            return "special_ez";
        case GWM_SPECIAL_AP_CFG:
            return "special_ap";
        case GWM_SPECIAL_SMT_AP_CFG:
            return "special_ap_ez";
        case GWM_SPECIAL_MATTER_CFG:
            return "special_matter";
        default:
            return "unknown";
    }
}

static void wifi_test_event_cb(WF_EVENT_E event, VOID_T *arg)
{
    s_wifi_last_event = event;

    if (event == WFE_CONNECTED) {
        s_wifi_event_connected++;
        s_wifi_last_reason = TUYA_WLAN_REASON_NONE;
    } else if (event == WFE_CONNECT_FAILED) {
        s_wifi_event_connect_failed++;
        if (arg != NULL) {
            s_wifi_last_reason = *(WF_DISCONN_REASON_E *)arg;
        }
    } else if (event == WFE_DISCONNECTED) {
        s_wifi_event_disconnected++;
        if (arg != NULL) {
            s_wifi_last_reason = *(WF_DISCONN_REASON_E *)arg;
        }
    }

    bk_printf("[xwifi][event] %s reason=%d c=%u f=%u d=%u\r\n",
              wifi_test_event_name(event), s_wifi_last_reason,
              (unsigned int)s_wifi_event_connected,
              (unsigned int)s_wifi_event_connect_failed,
              (unsigned int)s_wifi_event_disconnected);
}

static void wifi_test_usage(void)
{
    bk_printf("xwifi help\r\n");
    bk_printf("xwifi init\r\n");
    bk_printf("xwifi scan [ssid]\r\n");
    bk_printf("xwifi scanex <ssid|-> <all|2g|5g> [channel]\r\n");
    bk_printf("xwifi connect <ssid> [passwd]\r\n");
    bk_printf("xwifi connectburst <ssid> <passwd|-> <count> [interval_ms] [timeout_ms]\r\n");
    bk_printf("xwifi wait [timeout_ms]\r\n");
    bk_printf("xwifi connect_wait <ssid> <passwd|-> [timeout_ms]\r\n");
    bk_printf("xwifi disconnect\r\n");
    bk_printf("xwifi disconnect_wait [timeout_ms]\r\n");
    bk_printf("xwifi status\r\n");
    bk_printf("xwifi reason\r\n");
    bk_printf("xwifi rssi\r\n");
    bk_printf("xwifi rssimon <count> [interval_ms]\r\n");
    bk_printf("xwifi bssid\r\n");
    bk_printf("xwifi ip <sta|ap>\r\n");
    bk_printf("xwifi mode [pd|sniffer|sta|ap|staap]\r\n");
    bk_printf("xwifi country [CC]\r\n");
    bk_printf("xwifi fastsave\r\n");
    bk_printf("xwifi fastconnect\r\n");
    bk_printf("xwifi fastbench <count> [timeout_ms] [target_ms] [min_pass_pct]\r\n");
    bk_printf("xwifi fastclear\r\n");
    bk_printf("xwifi provinfo\r\n");
    bk_printf("xwifi provcfg <ssid> <passwd|-> [token|-]\r\n");
    bk_printf("xwifi provreinit\r\n");
    bk_printf("xwifi provswitch <ap|ez>\r\n");
    bk_printf("xwifi apstart <ssid> <passwd|-> [chan]\r\n");
    bk_printf("xwifi apstop\r\n");
    bk_printf("xwifi stress <ssid> <passwd|-> <count> [timeout_ms]\r\n");
    bk_printf("xwifi events\r\n");
    bk_printf("xwifi eventclr\r\n");
    bk_printf("xwifi reboot [delay_ms]\r\n");
}

static int wifi_test_parse_u32(const char *s, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed = 0;

    if ((s == NULL) || (value == NULL) || (s[0] == '\0')) {
        return -1;
    }

    parsed = strtoul(s, &end, 10);
    if ((end == NULL) || (*end != '\0')) {
        return -1;
    }

    *value = (uint32_t)parsed;
    return 0;
}

static const char *wifi_test_null_if_dash(const char *s)
{
    if ((s == NULL) || (strcmp(s, "-") == 0)) {
        return NULL;
    }

    return s;
}

static const char *wifi_test_empty_if_dash(const char *s)
{
    if ((s == NULL) || (strcmp(s, "-") == 0)) {
        return "";
    }

    return s;
}

static uint32_t wifi_test_percent(uint32_t part, uint32_t total)
{
    if (total == 0U) {
        return 0U;
    }

    return (part * 100U) / total;
}

static OPERATE_RET wifi_test_ensure_init(void)
{
    OPERATE_RET ret = tkl_wifi_init(wifi_test_event_cb);

    if (ret != OPRT_OK) {
        bk_printf("[xwifi] init failed ret=%d\r\n", ret);
    }

    return ret;
}

static void wifi_test_dump_ip(WF_IF_E wf)
{
    OPERATE_RET ret;
    NW_IP_S ip;

    memset(&ip, 0, sizeof(ip));
    ret = tkl_wifi_get_ip(wf, &ip);
    if (ret != OPRT_OK) {
        bk_printf("[xwifi] get ip fail if=%s ret=%d\r\n",
                  (wf == WF_STATION) ? "sta" : "ap", ret);
        return;
    }

    bk_printf("[xwifi] %s ip=%s mask=%s gw=%s dns=%s dhcp=%d\r\n",
              (wf == WF_STATION) ? "sta" : "ap",
              ip.ip, ip.mask, ip.gw, ip.dns, ip.dhcpen);
}

static void wifi_test_dump_status(void)
{
    OPERATE_RET ret;
    WF_STATION_STAT_E status = WSS_IDLE;
    WF_DISCONN_REASON_E reason = TUYA_WLAN_REASON_NONE;
    WF_WK_MD_E mode = WWM_UNKNOWN;

    ret = tkl_wifi_station_get_status(&status);
    if (ret != OPRT_OK) {
        bk_printf("[xwifi] status get fail ret=%d cached=%s\r\n",
                  ret, wifi_test_status_name(status));
    } else {
        bk_printf("[xwifi] status=%s(%d)\r\n", wifi_test_status_name(status), status);
    }

    ret = tkl_wifi_ioctl(WFI_GET_LAST_DISCONN_REASON, &reason);
    if (ret == OPRT_OK) {
        bk_printf("[xwifi] last_reason=%d\r\n", reason);
    }

    ret = tkl_wifi_get_work_mode(&mode);
    if (ret == OPRT_OK) {
        bk_printf("[xwifi] mode=%s(%d)\r\n", wifi_test_mode_name(mode), mode);
    }
}

static void wifi_test_scan(const char *ssid)
{
    OPERATE_RET ret;
    AP_IF_S *aps = NULL;
    UINT_T num = 0;

    ret = tkl_wifi_scan_ap((const SCHAR_T *)ssid, &aps, &num);
    if (ret != OPRT_OK) {
        bk_printf("[xwifi] scan fail ret=%d ssid=%s\r\n", ret, (ssid != NULL) ? ssid : "<all>");
        return;
    }

    bk_printf("[xwifi] scan ok num=%u\r\n", (unsigned int)num);
    for (UINT_T i = 0; i < num; i++) {
        AP_IF_S *ap = (AP_IF_S *)((uint8_t *)aps + (i * sizeof(AP_IF_S)));

        bk_printf("[%u] ssid=%s rssi=%d chan=%u sec=%u bssid=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                  (unsigned int)i,
                  ap->ssid,
                  ap->rssi,
                  (unsigned int)ap->channel,
                  (unsigned int)ap->security,
                  ap->bssid[0], ap->bssid[1], ap->bssid[2],
                  ap->bssid[3], ap->bssid[4], ap->bssid[5]);
    }

    tkl_wifi_release_ap(aps);
}

/**
 * @brief Exercise tkl_wifi_scan_ap_ex with band / channel / ssid scope.
 *
 * Usage: xwifi scanex <ssid|-> <all|2g|5g> [channel]
 *   - "-" means NULL ssid (scan all APs under the band/channel scope)
 *   - band: all / 2g / 5g, maps to WF_BAND_ALL / WF_BAND_2_4G / WF_BAND_5G;
 *     T9 does not support 5G scan, band=5g returns OPRT_NOT_SUPPORTED (ret=-4)
 *   - channel: 0 or omitted means all channels in the band scope; the
 *     channel is passed to firmware as-is (firmware owns channel validity,
 *     an unsupported channel fails after the scan wait times out)
 */
static void wifi_test_scan_ex(const char *ssid, const char *band, const char *chan)
{
    WF_SCAN_PARAM_T param;
    AP_IF_S *aps = NULL;
    UINT_T num = 0;
    uint32_t channel = 0;
    OPERATE_RET ret;

    memset(&param, 0, sizeof(param));
    param.ssid = (CONST SCHAR_T *)wifi_test_null_if_dash(ssid);
    param.band = WF_BAND_ALL;

    if ((band == NULL) || (strcmp(band, "all") == 0)) {
        param.band = WF_BAND_ALL;
    } else if (strcmp(band, "2g") == 0) {
        param.band = WF_BAND_2_4G;
    } else if (strcmp(band, "5g") == 0) {
        param.band = WF_BAND_5G;
    } else {
        bk_printf("[xwifi] scanex invalid band: %s (all|2g|5g)\r\n", band);
        return;
    }

    if ((chan != NULL) && (wifi_test_parse_u32(chan, &channel) != 0)) {
        bk_printf("[xwifi] invalid channel: %s\r\n", chan);
        return;
    }
    param.channel = (UCHAR_T)channel;

    ret = tkl_wifi_scan_ap_ex(&param, &aps, &num);
    if (ret != OPRT_OK) {
        bk_printf("[xwifi] scanex fail ret=%d band=%d chan=%u ssid=%s\r\n",
                  ret, param.band, (unsigned int)param.channel,
                  (param.ssid != NULL) ? param.ssid : "<all>");
        return;
    }

    bk_printf("[xwifi] scanex ok num=%u band=%d chan=%u\r\n",
              (unsigned int)num, param.band, (unsigned int)param.channel);
    for (UINT_T i = 0; i < num; i++) {
        AP_IF_S *ap = (AP_IF_S *)((uint8_t *)aps + (i * sizeof(AP_IF_S)));

        bk_printf("[%u] ssid=%s rssi=%d chan=%u sec=%u bssid=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                  (unsigned int)i,
                  ap->ssid,
                  ap->rssi,
                  (unsigned int)ap->channel,
                  (unsigned int)ap->security,
                  ap->bssid[0], ap->bssid[1], ap->bssid[2],
                  ap->bssid[3], ap->bssid[4], ap->bssid[5]);
    }

    tkl_wifi_release_ap(aps);
}

static int wifi_test_wait_for_result_ex(uint32_t timeout_ms, uint32_t poll_ms, int dump_final,
                                        uint32_t *elapsed_ms, WF_STATION_STAT_E *final_status)
{
    WF_STATION_STAT_E status = WSS_IDLE;
    OPERATE_RET ret;
    SYS_TIME_T start_ms = tkl_system_get_millisecond();

    while (1) {
        SYS_TIME_T now_ms = tkl_system_get_millisecond();
        uint32_t waited = (uint32_t)(now_ms - start_ms);

        ret = tkl_wifi_station_get_status(&status);
        if (ret == OPRT_OK) {
            if (status == WSS_GOT_IP) {
                if (elapsed_ms != NULL) {
                    *elapsed_ms = waited;
                }
                if (final_status != NULL) {
                    *final_status = status;
                }
                if (dump_final != 0) {
                    bk_printf("[xwifi] wait success status=%s time=%u\r\n",
                              wifi_test_status_name(status), (unsigned int)waited);
                    wifi_test_dump_ip(WF_STATION);
                }
                return 0;
            }

            if ((status == WSS_PASSWD_WRONG) || (status == WSS_NO_AP_FOUND) ||
                (status == WSS_CONN_FAIL) || (status == WSS_DHCP_FAIL)) {
                if (elapsed_ms != NULL) {
                    *elapsed_ms = waited;
                }
                if (final_status != NULL) {
                    *final_status = status;
                }
                if (dump_final != 0) {
                    bk_printf("[xwifi] wait fail status=%s time=%u\r\n",
                              wifi_test_status_name(status), (unsigned int)waited);
                }
                return -1;
            }
        }

        if (waited >= timeout_ms) {
            if (elapsed_ms != NULL) {
                *elapsed_ms = waited;
            }
            if (final_status != NULL) {
                *final_status = status;
            }
            if (dump_final != 0) {
                bk_printf("[xwifi] wait timeout after %u ms last=%s\r\n",
                          (unsigned int)timeout_ms, wifi_test_status_name(status));
            }
            return -2;
        }

        tkl_system_sleep((poll_ms == 0U) ? WIFI_TEST_WAIT_POLL_MS : poll_ms);
    }
}

static int wifi_test_wait_for_result(uint32_t timeout_ms, int dump_final)
{
    return wifi_test_wait_for_result_ex(timeout_ms, WIFI_TEST_WAIT_POLL_MS, dump_final, NULL, NULL);
}

static void wifi_test_connect(const char *ssid, const char *passwd, int do_wait, uint32_t timeout_ms)
{
    OPERATE_RET ret;

    ret = tkl_wifi_station_connect((const SCHAR_T *)ssid, (const SCHAR_T *)passwd);
    if (ret != OPRT_OK) {
        bk_printf("[xwifi] connect fail ret=%d ssid=%s\r\n", ret, ssid);
        return;
    }

    bk_printf("[xwifi] connect start ssid=%s passwd=%s\r\n",
              ssid, (passwd != NULL) ? passwd : "<open>");

    if (do_wait != 0) {
        (void)wifi_test_wait_for_result(timeout_ms, 1);
    }
}

static void wifi_test_fast_clear(void)
{
    if (s_wifi_fast_info != NULL) {
        os_free(s_wifi_fast_info);
        s_wifi_fast_info = NULL;
    }

    bk_printf("[xwifi] fast info cleared\r\n");
}

static void wifi_test_fast_save(void)
{
    OPERATE_RET ret;

    if (s_wifi_fast_info != NULL) {
        os_free(s_wifi_fast_info);
        s_wifi_fast_info = NULL;
    }

    ret = tkl_wifi_get_connected_ap_info(&s_wifi_fast_info);
    if (ret != OPRT_OK) {
        bk_printf("[xwifi] fastsave fail ret=%d\r\n", ret);
        return;
    }

    bk_printf("[xwifi] fastsave ok len=%u\r\n", (unsigned int)s_wifi_fast_info->len);
}

static void wifi_test_fast_connect(uint32_t timeout_ms)
{
    OPERATE_RET ret;
    WF_STATION_STAT_E status = WSS_IDLE;
    uint32_t elapsed_ms = 0;
    int wait_ret;

    if (s_wifi_fast_info == NULL) {
        bk_printf("[xwifi] no fast info, run xwifi fastsave first\r\n");
        return;
    }

    ret = tkl_wifi_station_fast_connect(s_wifi_fast_info);
    if (ret != OPRT_OK) {
        bk_printf("[xwifi] fastconnect fail ret=%d\r\n", ret);
        return;
    }

    bk_printf("[xwifi] fastconnect start\r\n");
    wait_ret = wifi_test_wait_for_result_ex(timeout_ms, WIFI_TEST_BENCH_POLL_MS, 1, &elapsed_ms, &status);
    bk_printf("[xwifi] fastconnect result=%s elapsed=%u target=%u\r\n",
              (wait_ret == 0) ? ((elapsed_ms <= WIFI_TEST_FAST_TARGET_MS) ? "PASS" : "SLOW") : "FAIL",
              (unsigned int)elapsed_ms, (unsigned int)WIFI_TEST_FAST_TARGET_MS);
}

static void wifi_test_fast_disconnect_prepare(void)
{
    WF_STATION_STAT_E status = WSS_IDLE;

    if (tkl_wifi_station_get_status(&status) != OPRT_OK) {
        return;
    }

    if ((status == WSS_CONNECTING) || (status == WSS_CONN_SUCCESS) || (status == WSS_GOT_IP)) {
        OPERATE_RET ret = tkl_wifi_station_disconnect();

        bk_printf("[xwifi] pre-disconnect ret=%d status=%s\r\n", ret, wifi_test_status_name(status));
        tkl_system_sleep(WIFI_TEST_DISCONNECT_WAIT_MS);
    }
}

static void wifi_test_fast_bench(uint32_t count, uint32_t timeout_ms, uint32_t target_ms, uint32_t min_pass_pct)
{
    uint32_t pass = 0;
    uint32_t speed_pass = 0;
    uint32_t min_ms = 0;
    uint32_t max_ms = 0;
    uint32_t sum_ms = 0;

    if (s_wifi_fast_info == NULL) {
        bk_printf("[xwifi] no fast info, run xwifi fastsave first\r\n");
        return;
    }

    bk_printf("[xwifi] fastbench start count=%u timeout=%u target=%u passline=%u%%\r\n",
              (unsigned int)count, (unsigned int)timeout_ms,
              (unsigned int)target_ms, (unsigned int)min_pass_pct);

    for (uint32_t i = 0; i < count; i++) {
        WF_STATION_STAT_E status = WSS_IDLE;
        uint32_t elapsed_ms = 0;
        int wait_ret;
        OPERATE_RET ret;

        wifi_test_fast_disconnect_prepare();

        ret = tkl_wifi_station_fast_connect(s_wifi_fast_info);
        if (ret != OPRT_OK) {
            bk_printf("[xwifi][fastbench] round=%u start fail ret=%d\r\n",
                      (unsigned int)(i + 1U), ret);
            continue;
        }

        wait_ret = wifi_test_wait_for_result_ex(timeout_ms, WIFI_TEST_BENCH_POLL_MS, 0, &elapsed_ms, &status);
        if (wait_ret == 0) {
            pass++;
            sum_ms += elapsed_ms;
            if ((min_ms == 0U) || (elapsed_ms < min_ms)) {
                min_ms = elapsed_ms;
            }
            if (elapsed_ms > max_ms) {
                max_ms = elapsed_ms;
            }
            if (elapsed_ms <= target_ms) {
                speed_pass++;
            }

            bk_printf("[xwifi][fastbench] round=%u PASS time=%u speed=%s\r\n",
                      (unsigned int)(i + 1U), (unsigned int)elapsed_ms,
                      (elapsed_ms <= target_ms) ? "ok" : "slow");
        } else {
            bk_printf("[xwifi][fastbench] round=%u FAIL status=%s time=%u\r\n",
                      (unsigned int)(i + 1U), wifi_test_status_name(status), (unsigned int)elapsed_ms);
        }
    }

    bk_printf("[xwifi][fastbench] summary success=%u/%u success_rate=%u%% speed=%u/%u speed_rate=%u%% target=%ums avg=%u min=%u max=%u overall=%s\r\n",
              (unsigned int)pass, (unsigned int)count,
              (unsigned int)wifi_test_percent(pass, count),
              (unsigned int)speed_pass, (unsigned int)count,
              (unsigned int)wifi_test_percent(speed_pass, count),
              (unsigned int)target_ms,
              (unsigned int)((pass == 0U) ? 0U : (sum_ms / pass)),
              (unsigned int)min_ms,
              (unsigned int)max_ms,
              ((wifi_test_percent(pass, count) >= min_pass_pct) &&
               (wifi_test_percent(speed_pass, count) >= min_pass_pct)) ? "PASS" : "FAIL");
}

static void wifi_test_prov_info(void)
{
    DEVOS_STATE_E devos_state = tuya_svc_devos_get_state();
    GW_CNTL_S *gw_cntl = get_gw_cntl();
    GW_WF_NWC_FAST_STAT_T fast_nc_tp = 0;
    CHAR_T ap_ssid[WIFI_SSID_LEN + 1] = {0};
    CHAR_T ap_passwd[WIFI_PASSWD_LEN + 1] = {0};
    CHAR_T *netcfg_stat = NULL;
    OPERATE_RET ret;

    bk_printf("[xwifi] devos=%s(%d)\r\n",
              wifi_test_devos_state_name(devos_state), devos_state);

    ret = tuya_iot_wf_fast_get_nc_type(&fast_nc_tp);
    bk_printf("[xwifi] fast_nc_type ret=%d value=%s(%d)\r\n",
              ret, wifi_test_fast_nc_type_name(fast_nc_tp), fast_nc_tp);

    ret = tuya_iot_get_dev_ap_if(ap_ssid, ap_passwd);
    bk_printf("[xwifi] dev_ap ret=%d ssid=%s passwd=%s\r\n",
              ret, ap_ssid, (ap_passwd[0] != '\0') ? "****************" : "<open>");

    netcfg_stat = tuya_svc_devos_get_netcfg_stat(1);
    bk_printf("[xwifi] netcfg_stat=%s\r\n", (netcfg_stat != NULL) ? netcfg_stat : "<null>");

    if (gw_cntl == NULL) {
        bk_printf("[xwifi] gw cntl is null\r\n");
        return;
    }

    bk_printf("[xwifi] gw is_init=%d\r\n", gw_cntl->is_init);
    bk_printf("[xwifi] gw stat=%s(%d) nc_tp=%s(%d) md=%s(%d)\r\n",
              wifi_test_gw_reg_state_name(gw_cntl->gw_wsm.stat), gw_cntl->gw_wsm.stat,
              wifi_test_nc_type_name(gw_cntl->gw_wsm.nc_tp), gw_cntl->gw_wsm.nc_tp,
              wifi_test_gw_md_name(gw_cntl->gw_wsm.md), gw_cntl->gw_wsm.md);
    bk_printf("[xwifi] gw ssid=%s passwd=%s token=%s region=%s\r\n",
              gw_cntl->gw_wsm.ssid,
              (gw_cntl->gw_wsm.passwd[0] != '\0') ? "****************" : "<open>",
              gw_cntl->gw_wsm.token,
              gw_cntl->gw_wsm.region);
}

static void wifi_test_prov_cfg(const char *ssid, const char *passwd, const char *token)
{
    OPERATE_RET ret = tuya_iot_gw_wf_user_cfg(ssid, passwd, token);

    bk_printf("[xwifi] provcfg ret=%d ssid=%s passwd=%s token=%s\r\n",
              ret, ssid,
              (passwd[0] != '\0') ? "****************" : "<open>",
              (token != NULL) ? token : "<null>");
}

static void wifi_test_prov_reinit(void)
{
    OPERATE_RET ret = tuya_iot_netcfg_reinit();

    bk_printf("[xwifi] provreinit ret=%d\r\n", ret);
}

static void wifi_test_prov_switch(const char *mode)
{
    OPERATE_RET ret;
    GW_WF_START_MODE start_mode;

    if (strcmp(mode, "ap") == 0) {
        start_mode = WF_START_AP_FIRST;
    } else if (strcmp(mode, "ez") == 0) {
        start_mode = WF_START_SMART_FIRST;
    } else {
        bk_printf("[xwifi] invalid provswitch mode: %s\r\n", mode);
        return;
    }

    ret = iot_wf_gw_unactive_custom_mode(start_mode);
    bk_printf("[xwifi] provswitch ret=%d mode=%s note=device may reset/restart\r\n", ret, mode);
}

static void wifi_test_ap_start(const char *ssid, const char *passwd, uint32_t channel)
{
    OPERATE_RET ret;
    WF_AP_CFG_IF_S cfg;

    memset(&cfg, 0, sizeof(cfg));

    cfg.s_len = (UCHAR_T)strlen(ssid);
    memcpy(cfg.ssid, ssid, cfg.s_len);

    if (passwd != NULL) {
        cfg.p_len = (UCHAR_T)strlen(passwd);
        memcpy(cfg.passwd, passwd, cfg.p_len);
        cfg.md = WAAM_WPA2_PSK;
    } else {
        cfg.p_len = 0;
        cfg.md = WAAM_OPEN;
    }

    cfg.chan = (UCHAR_T)channel;
    cfg.ssid_hidden = 0;
    cfg.max_conn = 3;
    snprintf(cfg.ip.ip, sizeof(cfg.ip.ip), "192.168.50.1");
    snprintf(cfg.ip.mask, sizeof(cfg.ip.mask), "255.255.255.0");
    snprintf(cfg.ip.gw, sizeof(cfg.ip.gw), "192.168.50.1");
    snprintf(cfg.ip.dns, sizeof(cfg.ip.dns), "192.168.50.1");
    cfg.ip.dhcpen = FALSE;

    ret = tkl_wifi_start_ap(&cfg);
    if (ret != OPRT_OK) {
        bk_printf("[xwifi] apstart fail ret=%d\r\n", ret);
        return;
    }

    bk_printf("[xwifi] apstart ok ssid=%s chan=%u auth=%s\r\n",
              ssid, (unsigned int)channel, (passwd != NULL) ? "wpa2" : "open");
    wifi_test_dump_ip(WF_AP);
}

static void wifi_test_events_dump(void)
{
    bk_printf("[xwifi] events connected=%u connect_failed=%u disconnected=%u last_event=%s last_reason=%d\r\n",
              (unsigned int)s_wifi_event_connected,
              (unsigned int)s_wifi_event_connect_failed,
              (unsigned int)s_wifi_event_disconnected,
              wifi_test_event_name(s_wifi_last_event),
              s_wifi_last_reason);
}

static void wifi_test_events_clear(void)
{
    s_wifi_event_connected = 0;
    s_wifi_event_connect_failed = 0;
    s_wifi_event_disconnected = 0;
    s_wifi_last_event = WFE_DISCONNECTED;
    s_wifi_last_reason = TUYA_WLAN_REASON_NONE;

    bk_printf("[xwifi] events cleared\r\n");
}

static void wifi_test_disconnect_wait(uint32_t timeout_ms)
{
    OPERATE_RET ret;
    SYS_TIME_T start_ms;
    WF_STATION_STAT_E status = WSS_IDLE;

    ret = tkl_wifi_station_disconnect();
    if (ret != OPRT_OK) {
        bk_printf("[xwifi] disconnect ret=%d\r\n", ret);
        return;
    }

    start_ms = tkl_system_get_millisecond();
    while (1) {
        SYS_TIME_T now_ms = tkl_system_get_millisecond();
        uint32_t waited = (uint32_t)(now_ms - start_ms);

        ret = tkl_wifi_station_get_status(&status);
        if ((ret == OPRT_OK) && ((status == WSS_CONN_FAIL) || (status == WSS_IDLE))) {
            bk_printf("[xwifi] disconnect done status=%s time=%u\r\n",
                      wifi_test_status_name(status), (unsigned int)waited);
            return;
        }

        if (waited >= timeout_ms) {
            bk_printf("[xwifi] disconnect timeout after %u ms last=%s ret=%d\r\n",
                      (unsigned int)timeout_ms, wifi_test_status_name(status), ret);
            return;
        }

        tkl_system_sleep(WIFI_TEST_WAIT_POLL_MS);
    }
}

static void wifi_test_rssi_monitor(uint32_t count, uint32_t interval_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        SCHAR_T rssi = 0;
        OPERATE_RET ret = tkl_wifi_station_get_conn_ap_rssi(&rssi);

        bk_printf("[xwifi][rssimon] round=%u ret=%d rssi=%d\r\n",
                  (unsigned int)(i + 1U), ret, rssi);

        if ((i + 1U) < count) {
            tkl_system_sleep(interval_ms);
        }
    }
}

static void wifi_test_connect_burst(const char *ssid, const char *passwd,
                                    uint32_t count, uint32_t interval_ms, uint32_t timeout_ms)
{
    bk_printf("[xwifi] connectburst start ssid=%s count=%u interval=%u timeout=%u\r\n",
              ssid, (unsigned int)count, (unsigned int)interval_ms, (unsigned int)timeout_ms);

    for (uint32_t i = 0; i < count; i++) {
        OPERATE_RET ret = tkl_wifi_station_connect((const SCHAR_T *)ssid, (const SCHAR_T *)passwd);

        bk_printf("[xwifi][connectburst] round=%u ret=%d\r\n", (unsigned int)(i + 1U), ret);

        if (((i + 1U) < count) && (interval_ms > 0U)) {
            tkl_system_sleep(interval_ms);
        }
    }

    (void)wifi_test_wait_for_result(timeout_ms, 1);
}

static void wifi_test_reboot(uint32_t delay_ms)
{
    bk_printf("[xwifi] reboot in %u ms\r\n", (unsigned int)delay_ms);

    if (delay_ms > 0U) {
        tkl_system_sleep(delay_ms);
    }

    tkl_system_reset();
}

static void wifi_test_stress(const char *ssid, const char *passwd, uint32_t count, uint32_t timeout_ms)
{
    uint32_t pass = 0;
    uint32_t fail = 0;

    for (uint32_t i = 0; i < count; i++) {
        OPERATE_RET ret = tkl_wifi_station_connect((const SCHAR_T *)ssid, (const SCHAR_T *)passwd);

        if (ret != OPRT_OK) {
            fail++;
            bk_printf("[xwifi][stress] round=%u connect ret=%d\r\n", (unsigned int)(i + 1), ret);
            continue;
        }

        if (wifi_test_wait_for_result(timeout_ms, 0) == 0) {
            pass++;
            bk_printf("[xwifi][stress] round=%u PASS\r\n", (unsigned int)(i + 1));
        } else {
            fail++;
            bk_printf("[xwifi][stress] round=%u FAIL\r\n", (unsigned int)(i + 1));
        }

        (void)tkl_wifi_station_disconnect();
        tkl_system_sleep(1000);
    }

    bk_printf("[xwifi][stress] summary pass=%u fail=%u\r\n",
              (unsigned int)pass, (unsigned int)fail);
}

void cli_wifi_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint32_t value = 0;

    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if ((argc < 2) || (argv[1] == NULL)) {
        wifi_test_usage();
        return;
    }

    if ((strcmp(argv[1], "help") == 0) || (strcmp(argv[1], "list") == 0)) {
        wifi_test_usage();
        return;
    }

    if (wifi_test_ensure_init() != OPRT_OK) {
        return;
    }

    if (strcmp(argv[1], "init") == 0) {
        bk_printf("[xwifi] init ok\r\n");
        return;
    }

    if (strcmp(argv[1], "scan") == 0) {
        wifi_test_scan(argv[2]);
        return;
    }

    if (strcmp(argv[1], "scanex") == 0) {
        wifi_test_scan_ex(argv[2], argv[3], argv[4]);
        return;
    }

    if (strcmp(argv[1], "connect") == 0) {
        if (argv[2] == NULL) {
            bk_printf("[xwifi] no ssid\r\n");
            return;
        }
        wifi_test_connect(argv[2], wifi_test_null_if_dash(argv[3]), 0, WIFI_TEST_DEFAULT_WAIT_MS);
        return;
    }

    if (strcmp(argv[1], "connectburst") == 0) {
        const char *passwd = NULL;
        uint32_t count = 0;
        uint32_t interval_ms = WIFI_TEST_BURST_INTERVAL_MS;
        uint32_t timeout_ms = WIFI_TEST_DEFAULT_WAIT_MS;

        if ((argv[2] == NULL) || (argv[4] == NULL)) {
            bk_printf("[xwifi] usage: xwifi connectburst <ssid> <passwd|-> <count> [interval_ms] [timeout_ms]\r\n");
            return;
        }

        passwd = wifi_test_null_if_dash(argv[3]);
        if (wifi_test_parse_u32(argv[4], &count) != 0) {
            bk_printf("[xwifi] invalid count: %s\r\n", argv[4]);
            return;
        }
        if (count == 0U) {
            bk_printf("[xwifi] count must be > 0\r\n");
            return;
        }
        if ((argv[5] != NULL) && (wifi_test_parse_u32(argv[5], &interval_ms) != 0)) {
            bk_printf("[xwifi] invalid interval: %s\r\n", argv[5]);
            return;
        }
        if ((argv[6] != NULL) && (wifi_test_parse_u32(argv[6], &timeout_ms) != 0)) {
            bk_printf("[xwifi] invalid timeout: %s\r\n", argv[6]);
            return;
        }

        wifi_test_connect_burst(argv[2], passwd, count, interval_ms, timeout_ms);
        return;
    }

    if (strcmp(argv[1], "connect_wait") == 0) {
        uint32_t timeout_ms = WIFI_TEST_DEFAULT_WAIT_MS;

        if (argv[2] == NULL) {
            bk_printf("[xwifi] no ssid\r\n");
            return;
        }
        if ((argv[4] != NULL) && (wifi_test_parse_u32(argv[4], &timeout_ms) != 0)) {
            bk_printf("[xwifi] invalid timeout: %s\r\n", argv[4]);
            return;
        }

        wifi_test_connect(argv[2], wifi_test_null_if_dash(argv[3]), 1, timeout_ms);
        return;
    }

    if (strcmp(argv[1], "wait") == 0) {
        uint32_t timeout_ms = WIFI_TEST_DEFAULT_WAIT_MS;

        if ((argv[2] != NULL) && (wifi_test_parse_u32(argv[2], &timeout_ms) != 0)) {
            bk_printf("[xwifi] invalid timeout: %s\r\n", argv[2]);
            return;
        }

        (void)wifi_test_wait_for_result(timeout_ms, 1);
        return;
    }

    if (strcmp(argv[1], "disconnect") == 0) {
        OPERATE_RET ret = tkl_wifi_station_disconnect();
        bk_printf("[xwifi] disconnect ret=%d\r\n", ret);
        return;
    }

    if (strcmp(argv[1], "disconnect_wait") == 0) {
        uint32_t timeout_ms = WIFI_TEST_DEFAULT_WAIT_MS;

        if ((argv[2] != NULL) && (wifi_test_parse_u32(argv[2], &timeout_ms) != 0)) {
            bk_printf("[xwifi] invalid timeout: %s\r\n", argv[2]);
            return;
        }

        wifi_test_disconnect_wait(timeout_ms);
        return;
    }

    if (strcmp(argv[1], "status") == 0) {
        wifi_test_dump_status();
        return;
    }

    if (strcmp(argv[1], "reason") == 0) {
        WF_DISCONN_REASON_E reason = TUYA_WLAN_REASON_NONE;
        OPERATE_RET ret = tkl_wifi_ioctl(WFI_GET_LAST_DISCONN_REASON, &reason);

        bk_printf("[xwifi] reason ret=%d value=%d\r\n", ret, reason);
        return;
    }

    if (strcmp(argv[1], "rssi") == 0) {
        SCHAR_T rssi = 0;
        OPERATE_RET ret = tkl_wifi_station_get_conn_ap_rssi(&rssi);

        bk_printf("[xwifi] rssi ret=%d value=%d\r\n", ret, rssi);
        return;
    }

    if (strcmp(argv[1], "rssimon") == 0) {
        uint32_t count = 0;
        uint32_t interval_ms = WIFI_TEST_RSSI_INTERVAL_MS;

        if (argv[2] == NULL) {
            bk_printf("[xwifi] usage: xwifi rssimon <count> [interval_ms]\r\n");
            return;
        }
        if (wifi_test_parse_u32(argv[2], &count) != 0) {
            bk_printf("[xwifi] invalid count: %s\r\n", argv[2]);
            return;
        }
        if (count == 0U) {
            bk_printf("[xwifi] count must be > 0\r\n");
            return;
        }
        if ((argv[3] != NULL) && (wifi_test_parse_u32(argv[3], &interval_ms) != 0)) {
            bk_printf("[xwifi] invalid interval: %s\r\n", argv[3]);
            return;
        }

        wifi_test_rssi_monitor(count, interval_ms);
        return;
    }

    if (strcmp(argv[1], "bssid") == 0) {
        UCHAR_T mac[6] = {0};
        OPERATE_RET ret = tkl_wifi_get_bssid(mac);

        if (ret != OPRT_OK) {
            bk_printf("[xwifi] bssid fail ret=%d\r\n", ret);
            return;
        }

        bk_printf("[xwifi] bssid=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return;
    }

    if (strcmp(argv[1], "ip") == 0) {
        if ((argv[2] == NULL) || (strcmp(argv[2], "sta") == 0)) {
            wifi_test_dump_ip(WF_STATION);
            return;
        }
        if (strcmp(argv[2], "ap") == 0) {
            wifi_test_dump_ip(WF_AP);
            return;
        }

        bk_printf("[xwifi] invalid if: %s\r\n", argv[2]);
        return;
    }

    if (strcmp(argv[1], "mode") == 0) {
        if (argv[2] == NULL) {
            WF_WK_MD_E mode = WWM_UNKNOWN;
            OPERATE_RET ret = tkl_wifi_get_work_mode(&mode);

            bk_printf("[xwifi] mode ret=%d value=%s(%d)\r\n",
                      ret, wifi_test_mode_name(mode), mode);
            return;
        } else {
            WF_WK_MD_E mode = wifi_test_parse_mode(argv[2]);
            OPERATE_RET ret;

            if (mode == WWM_UNKNOWN) {
                bk_printf("[xwifi] invalid mode: %s\r\n", argv[2]);
                return;
            }

            ret = tkl_wifi_set_work_mode(mode);
            bk_printf("[xwifi] set mode ret=%d value=%s(%d)\r\n",
                      ret, wifi_test_mode_name(mode), mode);
            return;
        }
    }

    if (strcmp(argv[1], "country") == 0) {
        if (argv[2] == NULL) {
            UCHAR_T ccode[3] = {0};
            OPERATE_RET ret = tkl_wifi_get_country_code(ccode);

            bk_printf("[xwifi] country ret=%d value=%s\r\n", ret, ccode);
            return;
        } else {
            UCHAR_T ccode[3] = {0};
            OPERATE_RET ret;

            if (strlen(argv[2]) < 2) {
                bk_printf("[xwifi] invalid country: %s\r\n", argv[2]);
                return;
            }

            ccode[0] = (UCHAR_T)argv[2][0];
            ccode[1] = (UCHAR_T)argv[2][1];
            ccode[2] = '\0';

            ret = tkl_wifi_set_country_code_v2(ccode);
            bk_printf("[xwifi] set country ret=%d value=%s\r\n", ret, ccode);
            return;
        }
    }

    if (strcmp(argv[1], "fastsave") == 0) {
        wifi_test_fast_save();
        return;
    }

    if (strcmp(argv[1], "fastconnect") == 0) {
        uint32_t timeout_ms = WIFI_TEST_DEFAULT_WAIT_MS;

        if ((argv[2] != NULL) && (wifi_test_parse_u32(argv[2], &timeout_ms) != 0)) {
            bk_printf("[xwifi] invalid timeout: %s\r\n", argv[2]);
            return;
        }

        wifi_test_fast_connect(timeout_ms);
        return;
    }

    if (strcmp(argv[1], "fastbench") == 0) {
        uint32_t count = 0;
        uint32_t timeout_ms = WIFI_TEST_FAST_TIMEOUT_MS;
        uint32_t target_ms = WIFI_TEST_FAST_TARGET_MS;
        uint32_t min_pass_pct = WIFI_TEST_MIN_PASS_PCT;

        if (argv[2] == NULL) {
            bk_printf("[xwifi] usage: xwifi fastbench <count> [timeout_ms] [target_ms] [min_pass_pct]\r\n");
            return;
        }

        if (wifi_test_parse_u32(argv[2], &count) != 0) {
            bk_printf("[xwifi] invalid count: %s\r\n", argv[2]);
            return;
        }
        if (count == 0U) {
            bk_printf("[xwifi] count must be > 0\r\n");
            return;
        }
        if ((argv[3] != NULL) && (wifi_test_parse_u32(argv[3], &timeout_ms) != 0)) {
            bk_printf("[xwifi] invalid timeout: %s\r\n", argv[3]);
            return;
        }
        if ((argv[4] != NULL) && (wifi_test_parse_u32(argv[4], &target_ms) != 0)) {
            bk_printf("[xwifi] invalid target: %s\r\n", argv[4]);
            return;
        }
        if ((argv[5] != NULL) && (wifi_test_parse_u32(argv[5], &min_pass_pct) != 0)) {
            bk_printf("[xwifi] invalid min_pass_pct: %s\r\n", argv[5]);
            return;
        }

        wifi_test_fast_bench(count, timeout_ms, target_ms, min_pass_pct);
        return;
    }

    if (strcmp(argv[1], "fastclear") == 0) {
        wifi_test_fast_clear();
        return;
    }

    if (strcmp(argv[1], "provinfo") == 0) {
        wifi_test_prov_info();
        return;
    }

    if (strcmp(argv[1], "provcfg") == 0) {
        if (argv[2] == NULL) {
            bk_printf("[xwifi] usage: xwifi provcfg <ssid> <passwd|-> [token|-]\r\n");
            return;
        }

        wifi_test_prov_cfg(argv[2], wifi_test_empty_if_dash(argv[3]), wifi_test_null_if_dash(argv[4]));
        return;
    }

    if (strcmp(argv[1], "provreinit") == 0) {
        wifi_test_prov_reinit();
        return;
    }

    if (strcmp(argv[1], "provswitch") == 0) {
        if (argv[2] == NULL) {
            bk_printf("[xwifi] usage: xwifi provswitch <ap|ez>\r\n");
            return;
        }

        wifi_test_prov_switch(argv[2]);
        return;
    }

    if (strcmp(argv[1], "apstart") == 0) {
        const char *passwd = NULL;
        uint32_t channel = WIFI_TEST_DEFAULT_AP_CHANNEL;

        if (argv[2] == NULL) {
            bk_printf("[xwifi] no ap ssid\r\n");
            return;
        }

        passwd = wifi_test_null_if_dash(argv[3]);
        if ((argv[4] != NULL) && (wifi_test_parse_u32(argv[4], &channel) != 0)) {
            bk_printf("[xwifi] invalid channel: %s\r\n", argv[4]);
            return;
        }

        wifi_test_ap_start(argv[2], passwd, channel);
        return;
    }

    if (strcmp(argv[1], "apstop") == 0) {
        OPERATE_RET ret = tkl_wifi_stop_ap();
        bk_printf("[xwifi] apstop ret=%d\r\n", ret);
        return;
    }

    if (strcmp(argv[1], "stress") == 0) {
        const char *passwd = NULL;
        uint32_t count = 0;
        uint32_t timeout_ms = WIFI_TEST_DEFAULT_WAIT_MS;

        if ((argv[2] == NULL) || (argv[4] == NULL)) {
            bk_printf("[xwifi] usage: xwifi stress <ssid> <passwd|-> <count> [timeout_ms]\r\n");
            return;
        }

        passwd = wifi_test_null_if_dash(argv[3]);

        if (wifi_test_parse_u32(argv[4], &count) != 0) {
            bk_printf("[xwifi] invalid count: %s\r\n", argv[4]);
            return;
        }
        if (count == 0) {
            bk_printf("[xwifi] count must be > 0\r\n");
            return;
        }
        if ((argv[5] != NULL) && (wifi_test_parse_u32(argv[5], &timeout_ms) != 0)) {
            bk_printf("[xwifi] invalid timeout: %s\r\n", argv[5]);
            return;
        }

        wifi_test_stress(argv[2], passwd, count, timeout_ms);
        return;
    }

    if (strcmp(argv[1], "events") == 0) {
        wifi_test_events_dump();
        return;
    }

    if (strcmp(argv[1], "eventclr") == 0) {
        wifi_test_events_clear();
        return;
    }

    if (strcmp(argv[1], "reboot") == 0) {
        uint32_t delay_ms = 0;

        if ((argv[2] != NULL) && (wifi_test_parse_u32(argv[2], &delay_ms) != 0)) {
            bk_printf("[xwifi] invalid delay: %s\r\n", argv[2]);
            return;
        }

        wifi_test_reboot(delay_ms);
        return;
    }

    if (wifi_test_parse_u32(argv[1], &value) == 0) {
        bk_printf("[xwifi] legacy numeric subcmd is not supported, use xwifi help\r\n");
        return;
    }

    bk_printf("[xwifi] unknown cmd: %s\r\n", argv[1]);
    wifi_test_usage();
}

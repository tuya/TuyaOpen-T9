/**
 * @file tkl_wifi.c
 * @brief T9 Wi-Fi adapter implementation for TuyaOS TKL.
 * @version 0.1
 * @date 2026-04-03
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tuya_error_code.h"
#include "tuya_os_adapter_error_code.h"
#include "tkl_system.h"
#include "tkl_wifi.h"

#include "components/event.h"
#include "components/netif.h"
#include "bk_netif.h"
#include "modules/wifi.h"
#include "os/mem.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define TKL_WIFI_SCAN_WAIT_MS         15000U
#define TKL_WIFI_SCAN_POLL_MS         50U
#define TKL_WIFI_COUNTRY_STR_LEN      3U
#define TKL_WIFI_MAC_STR_LEN          13U
#define TKL_WIFI_AP_ENTRY_SIZE        ((UINT_T)sizeof(AP_IF_S))

/* ---------------------------------------------------------------------------
 * Type definitions
 * --------------------------------------------------------------------------- */
typedef struct {
    BOOL_T inited;
    BOOL_T events_registered;
    BOOL_T scan_done;
    BOOL_T sta_started;
    BOOL_T ap_started;
    BOOL_T sniffer_enabled;
    BOOL_T sta_dhcp_enabled;
    WF_WK_MD_E work_mode;
    WF_STATION_STAT_E sta_status;
    WF_DISCONN_REASON_E last_disc_reason;
    WIFI_EVENT_CB event_cb;
    SNIFFER_CALLBACK sniffer_cb;
    WIFI_REV_MGNT_CB mgnt_cb;
    CHAR_T country_code[TKL_WIFI_COUNTRY_STR_LEN];
} TKL_WIFI_CTX_T;

/* ---------------------------------------------------------------------------
 * File scope variables
 * --------------------------------------------------------------------------- */
STATIC TKL_WIFI_CTX_T s_wifi = {
    .sta_dhcp_enabled = TRUE,
    .work_mode = WWM_POWERDOWN,
    .sta_status = WSS_IDLE,
    .last_disc_reason = TUYA_WLAN_REASON_NONE,
    .country_code = "CN",
};

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
// extern bk_err_t bk_wifi_init(VOID_T);
STATIC bk_err_t __wifi_event_cb(VOID_T *arg, event_module_t event_module, INT_T event_id, VOID_T *event_data);
STATIC bk_err_t __wifi_monitor_cb(CONST UINT8_T *frame, UINT32_T len, CONST wifi_frame_info_t *frame_info);
STATIC bk_err_t __wifi_filter_cb(CONST UINT8_T *frame, UINT32_T len, CONST wifi_frame_info_t *frame_info);
extern void bk_printf(const char *fmt, ...);

/* ---------------------------------------------------------------------------
 * Internal helper functions
 * --------------------------------------------------------------------------- */
STATIC CONST CHAR_T *__wifi_work_mode_name(WF_WK_MD_E mode)
{
    switch (mode) {
    case WWM_POWERDOWN:
        return "powerdown";
    case WWM_SNIFFER:
        return "sniffer";
    case WWM_STATION:
        return "station";
    case WWM_SOFTAP:
        return "softap";
    case WWM_STATIONAP:
        return "stationap";
    default:
        return "unknown";
    }
}

STATIC CONST CHAR_T *__wifi_tuya_event_name(WF_EVENT_E event)
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

/**
 * @brief Get the bounded length of a C string.
 * @param[in] str input string
 * @param[in] max_len maximum allowed length
 * @return bounded string length
 */
STATIC UINT_T __wifi_strnlen(CONST CHAR_T *str, UINT_T max_len)
{
    UINT_T len = 0;

    if (str == NULL) {
        return 0;
    }

    while ((len < max_len) && (str[len] != '\0')) {
        len++;
    }

    return len;
}

/**
 * @brief Copy a string into a fixed-size buffer.
 * @param[out] dst destination buffer
 * @param[in] dst_size destination buffer size
 * @param[in] src source string
 * @return none
 */
STATIC VOID_T __wifi_copy_str(CHAR_T *dst, UINT_T dst_size, CONST CHAR_T *src)
{
    UINT_T copy_len = 0;

    if ((dst == NULL) || (dst_size == 0)) {
        return;
    }

    dst[0] = '\0';
    if (src == NULL) {
        return;
    }

    copy_len = __wifi_strnlen(src, dst_size - 1);
    memcpy(dst, src, copy_len);
    dst[copy_len] = '\0';
}

STATIC VOID_T __fast_connect_ap_info_dump(CONST CHAR_T *tag, CONST wifi_sta_config_t *sta_cfg)
{
    if (sta_cfg == NULL) {
        return;
    }

    bk_printf("----------------------------------------------------------------------------------------\r\n");
    if (tag != NULL) {
        bk_printf("%s\r\n", tag);
    }
    bk_printf("ssid: %s\r\n", sta_cfg->ssid);
    bk_printf("bssid: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
              sta_cfg->bssid[0], sta_cfg->bssid[1], sta_cfg->bssid[2],
              sta_cfg->bssid[3], sta_cfg->bssid[4], sta_cfg->bssid[5]);
    bk_printf("security: %d, chan: %d\r\n", sta_cfg->security, sta_cfg->channel);
    bk_printf("psk: ********************************, pmf: %d\r\n", sta_cfg->pmf);
    bk_printf("pwd: ****************\r\n");
    bk_printf("ip: %d.%d.%d.%d\r\n", sta_cfg->ip_addr[0], sta_cfg->ip_addr[1], sta_cfg->ip_addr[2], sta_cfg->ip_addr[3]);
    bk_printf("netmask: %d.%d.%d.%d\r\n", sta_cfg->netmask[0], sta_cfg->netmask[1], sta_cfg->netmask[2], sta_cfg->netmask[3]);
    bk_printf("gw: %d.%d.%d.%d\r\n", sta_cfg->gw[0], sta_cfg->gw[1], sta_cfg->gw[2], sta_cfg->gw[3]);
    bk_printf("dns1: %d.%d.%d.%d\r\n", sta_cfg->dns1[0], sta_cfg->dns1[1], sta_cfg->dns1[2], sta_cfg->dns1[3]);
    bk_printf("----------------------------------------------------------------------------------------\r\n");
}

/**
 * @brief Convert a Beken return code into a generic adapter error.
 * @param[in] bk_ret Beken return code
 * @param[in] failed_ret fallback Tuya error code
 * @return mapped Tuya error code
 */
STATIC OPERATE_RET __wifi_map_ret(bk_err_t bk_ret, OPERATE_RET failed_ret)
{
    if (bk_ret == BK_OK) {
        return OPRT_OK;
    }

    if ((bk_ret == BK_ERR_NULL_PARAM) || (bk_ret == BK_ERR_PARAM)) {
        return OPRT_INVALID_PARM;
    }

    if (bk_ret == BK_ERR_NO_MEM) {
        return OPRT_MALLOC_FAILED;
    }

    if (bk_ret == BK_ERR_NOT_SUPPORT) {
        return OPRT_NOT_SUPPORTED;
    }

    return failed_ret;
}

/**
 * @brief Map Tuya AP auth mode to Beken security mode.
 * @param[in] auth Tuya auth mode
 * @return mapped Beken security mode
 */
STATIC wifi_security_t __wifi_auth_tuya_to_bk(WF_AP_AUTH_MODE_E auth)
{
    switch (auth) {
    case WAAM_OPEN:
        return WIFI_SECURITY_NONE;
    case WAAM_WEP:
        return WIFI_SECURITY_WEP;
    case WAAM_WPA_PSK:
        return WIFI_SECURITY_WPA_MIXED;
    case WAAM_WPA2_PSK:
        return WIFI_SECURITY_WPA2_MIXED;
    case WAAM_WPA_WPA2_PSK:
        return WIFI_SECURITY_WPA2_MIXED;
    case WAAM_WPA_WPA3_SAE:
        return WIFI_SECURITY_WPA3_WPA2_MIXED;
    case WAAM_UNKNOWN:
    default:
        return WIFI_SECURITY_AUTO;
    }
}

/**
 * @brief Map Beken security mode to Tuya AP auth mode.
 * @param[in] security Beken security mode
 * @return mapped Tuya auth mode
 */
STATIC WF_AP_AUTH_MODE_E __wifi_auth_bk_to_tuya(wifi_security_t security)
{
    switch (security) {
    case WIFI_SECURITY_NONE:
        return WAAM_OPEN;
    case WIFI_SECURITY_WEP:
        return WAAM_WEP;
    case WIFI_SECURITY_WPA_TKIP:
    case WIFI_SECURITY_WPA_AES:
    case WIFI_SECURITY_WPA_MIXED:
        return WAAM_WPA_PSK;
    case WIFI_SECURITY_WPA2_TKIP:
    case WIFI_SECURITY_WPA2_AES:
    case WIFI_SECURITY_WPA2_MIXED:
        return WAAM_WPA2_PSK;
    case WIFI_SECURITY_WPA3_SAE:
    case WIFI_SECURITY_WPA3_WPA2_MIXED:
        return WAAM_WPA_WPA3_SAE;
    default:
        return WAAM_UNKNOWN;
    }
}

/**
 * @brief Map Beken disconnect reason to Tuya disconnect reason.
 * @param[in] reason Beken disconnect reason
 * @return mapped Tuya disconnect reason
 */
STATIC WF_DISCONN_REASON_E __wifi_reason_bk_to_tuya(INT_T reason)
{
    switch (reason) {
    case WIFI_REASON_UNSPECIFIED:
        return TUYA_WLAN_REASON_UNSPECIFIED;
    case WIFI_REASON_NO_AP_FOUND:
        return TUYA_WLAN_REASON_SSID_NOT_FOUND;
    case WIFI_REASON_WRONG_PASSWORD:
        return TUYA_WLAN_REASON_WRONG_PASSWORD;
    case WIFI_REASON_PREV_AUTH_NOT_VALID:
        return TUYA_WLAN_REASON_PREV_AUTH_INVALID;
    case WIFI_REASON_INVALID_IE:
        return TUYA_WLAN_REASON_INVALID_RSN_IE;
    case WIFI_REASON_GROUP_CIPHER_NOT_VALID:
        return TUYA_WLAN_REASON_GROUP_CIPHER_INVALID;
    case WIFI_REASON_PAIRWISE_CIPHER_NOT_VALID:
        return TUYA_WLAN_REASON_PAIRWISE_CIPHER_INVALID;
    case WIFI_REASON_UNSUPPORTED_RSN_IE_VERSION:
        return TUYA_WLAN_REASON_UNSPPORTED_RSN_IE_VERSION;
    case WIFI_REASON_AKMP_NOT_VALID:
        return TUYA_WLAN_REASON_AKMP_INVALID;
    case WIFI_REASON_INVALID_RSN_IE_CAPAB:
        return TUYA_WLAN_REASON_INVALID_RSN_IE_CAP;
    case WIFI_REASON_CIPHER_SUITE_REJECTED:
        return TUYA_WLAN_REASON_CIPHER_REJECTED_PER_POLICY;
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        return TUYA_WLAN_REASON_4WAYS_HANDSHAKE_TIMEOUT;
    case WIFI_REASON_MICHAEL_MIC_FAILURE:
        return TUYA_WLAN_REASON_MIC_FAILURE;
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:
        return TUYA_WLAN_REASON_RSN_IE_IN_4WAYS_MISMATCH;
    case WIFI_REASON_DHCP_TIMEOUT:
        return TUYA_WLAN_REASON_UNSPECIFIED;
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY:
    case WIFI_REASON_BEACON_LOST:
        return TUYA_WLAN_REASON_INACTIVITY_DISCONNECT;
    case WIFI_REASON_DEAUTH_LEAVING:
    case WIFI_REASON_DISCONNECT_BY_APP:
        return TUYA_WLAN_REASON_DEAUTH_LEAVING;
    case WIFI_REASON_DISASSOC_AP_BUSY:
        return TUYA_WLAN_REASON_AP_UNABLE_TO_HANDLE_NEW_STA;
    default:
        return TUYA_WLAN_REASON_NONE;
    }
}

/**
 * @brief Convert Beken link state into Tuya station status.
 * @param[in] link_info Beken link state and reason
 * @return mapped Tuya station status
 */
STATIC WF_STATION_STAT_E __wifi_link_to_status(CONST wifi_linkstate_reason_t *link_info)
{
    if (link_info == NULL) {
        return WSS_IDLE;
    }

    switch (link_info->state) {
    case WIFI_LINKSTATE_STA_CONNECTING:
        return WSS_CONNECTING;
    case WIFI_LINKSTATE_STA_CONNECTED:
        return WSS_CONN_SUCCESS;
    case WIFI_LINKSTATE_STA_GOT_IP:
        return WSS_GOT_IP;
    case WIFI_LINKSTATE_STA_CONNECT_FAILED:
    case WIFI_LINKSTATE_STA_DISCONNECTED:
        if (link_info->reason_code == WIFI_REASON_WRONG_PASSWORD) {
            return WSS_PASSWD_WRONG;
        }
        if (link_info->reason_code == WIFI_REASON_NO_AP_FOUND) {
            return WSS_NO_AP_FOUND;
        }
        if (link_info->reason_code == WIFI_REASON_DHCP_TIMEOUT) {
            return WSS_DHCP_FAIL;
        }
        return WSS_CONN_FAIL;
    case WIFI_LINKSTATE_STA_IDLE:
    default:
        return WSS_IDLE;
    }
}

/**
 * @brief Convert Tuya Wi-Fi interface into a Beken netif ID.
 * @param[in] wf Tuya Wi-Fi interface
 * @param[out] ifx Beken netif ID
 * @return OPRT_OK on success, error code on failure
 */
STATIC OPERATE_RET __wifi_if_to_netif(WF_IF_E wf, netif_if_t *ifx)
{
    if (ifx == NULL) {
        return OPRT_INVALID_PARM;
    }

    switch (wf) {
    case WF_STATION:
        *ifx = NETIF_IF_STA;
        return OPRT_OK;
    case WF_AP:
        *ifx = NETIF_IF_AP;
        return OPRT_OK;
    default:
        return OPRT_INVALID_PARM;
    }
}

/**
 * @brief Parse an IPv4 string into 4 octets.
 * @param[in] ip_str IPv4 string
 * @param[out] ip IPv4 octet output buffer
 * @return TRUE on success, FALSE on failure
 */
STATIC BOOL_T __wifi_parse_ipv4(CONST CHAR_T *ip_str, UINT8_T ip[4])
{
    CHAR_T *end = NULL;
    unsigned long value = 0;
    UINT_T index = 0;
    CONST CHAR_T *cursor = ip_str;

    if ((ip_str == NULL) || (ip == NULL)) {
        return FALSE;
    }

    while (index < 4) {
        if ((*cursor == '\0') || (!isdigit((int)(unsigned char)*cursor))) {
            return FALSE;
        }

        value = strtoul(cursor, &end, 10);
        if ((end == cursor) || (value > 255)) {
            return FALSE;
        }

        ip[index++] = (UINT8_T)value;
        if (index == 4) {
            return (*end == '\0') ? TRUE : FALSE;
        }

        if (*end != '.') {
            return FALSE;
        }

        cursor = end + 1;
    }

    return FALSE;
}

/**
 * @brief Fill a Tuya IPv4 structure from a Beken IPv4 configuration.
 * @param[out] ip Tuya IP structure
 * @param[in] ip4 Beken IPv4 configuration
 * @param[in] dhcp_enabled DHCP state
 * @return none
 */
STATIC VOID_T __wifi_fill_tuya_ip(NW_IP_S *ip, CONST netif_ip4_config_t *ip4, BOOL_T dhcp_enabled)
{
    if ((ip == NULL) || (ip4 == NULL)) {
        return;
    }

    memset(ip, 0, sizeof(NW_IP_S));
    __wifi_copy_str(ip->ip, sizeof(ip->ip), ip4->ip);
    __wifi_copy_str(ip->mask, sizeof(ip->mask), ip4->mask);
    __wifi_copy_str(ip->gw, sizeof(ip->gw), ip4->gateway);
    __wifi_copy_str(ip->dns, sizeof(ip->dns), ip4->dns);
    ip->dhcpen = dhcp_enabled;
}

/**
 * @brief Fill a Beken IPv4 configuration from a Tuya IP structure.
 * @param[out] ip4 Beken IPv4 configuration
 * @param[in] ip Tuya IP structure
 * @return OPRT_OK on success, error code on failure
 */
STATIC OPERATE_RET __wifi_fill_bk_ip4(netif_ip4_config_t *ip4, CONST NW_IP_S *ip)
{
    if ((ip4 == NULL) || (ip == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if ((!ip->dhcpen) &&
        ((!__wifi_parse_ipv4(ip->ip, (UINT8_T[4]){0})) ||
         (!__wifi_parse_ipv4(ip->mask, (UINT8_T[4]){0})) ||
         (!__wifi_parse_ipv4(ip->gw, (UINT8_T[4]){0})) ||
         (!__wifi_parse_ipv4(ip->dns, (UINT8_T[4]){0})))) {
        return OPRT_INVALID_PARM;
    }

    memset(ip4, 0, sizeof(netif_ip4_config_t));
    __wifi_copy_str(ip4->ip, sizeof(ip4->ip), ip->ip);
    __wifi_copy_str(ip4->mask, sizeof(ip4->mask), ip->mask);
    __wifi_copy_str(ip4->gateway, sizeof(ip4->gateway), ip->gw);
    __wifi_copy_str(ip4->dns, sizeof(ip4->dns), ip->dns);
    return OPRT_OK;
}

/**
 * @brief Prepare a vendor country configuration from a Tuya enum.
 * @param[in] ccode Tuya country code
 * @param[out] country vendor country configuration
 * @return OPRT_OK on success, error code on failure
 */
STATIC OPERATE_RET __wifi_country_from_enum(COUNTRY_CODE_E ccode, wifi_country_t *country)
{
    if (country == NULL) {
        return OPRT_INVALID_PARM;
    }

    memset(country, 0, sizeof(wifi_country_t));
    country->schan = 1;
    country->max_tx_power = 20;
    country->policy = WIFI_COUNTRY_POLICY_MANUAL;

    switch (ccode) {
    case COUNTRY_CODE_CN:
        memcpy(country->cc, "CN", 2);
        country->nchan = 13;
        break;
    case COUNTRY_CODE_US:
        memcpy(country->cc, "US", 2);
        country->nchan = 11;
        break;
    case COUNTRY_CODE_JP:
        memcpy(country->cc, "JP", 2);
        country->nchan = 14;
        break;
    case COUNTRY_CODE_EU:
        memcpy(country->cc, "EU", 2);
        country->nchan = 13;
        break;
    case COUNTRY_CODE_TR:
        memcpy(country->cc, "TR", 2);
        country->nchan = 13;
        break;
    default:
        return OPRT_NOT_SUPPORTED;
    }

    country->cc[2] = '\0';
    return OPRT_OK;
}

/**
 * @brief Prepare a vendor country configuration from a country string.
 * @param[in] ccode country code string
 * @param[out] country vendor country configuration
 * @return OPRT_OK on success, error code on failure
 */
STATIC OPERATE_RET __wifi_country_from_str(CONST UCHAR_T *ccode, wifi_country_t *country)
{
    CHAR_T code[TKL_WIFI_COUNTRY_STR_LEN] = {0};

    if ((ccode == NULL) || (country == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (__wifi_strnlen((CONST CHAR_T *)ccode, 3) != 2) {
        return OPRT_INVALID_PARM;
    }

    code[0] = (CHAR_T)toupper(ccode[0]);
    code[1] = (CHAR_T)toupper(ccode[1]);
    code[2] = '\0';

    if (strncmp(code, "CN", 2) == 0) {
        return __wifi_country_from_enum(COUNTRY_CODE_CN, country);
    }
    if (strncmp(code, "US", 2) == 0) {
        return __wifi_country_from_enum(COUNTRY_CODE_US, country);
    }
    if (strncmp(code, "JP", 2) == 0) {
        return __wifi_country_from_enum(COUNTRY_CODE_JP, country);
    }
    if (strncmp(code, "EU", 2) == 0) {
        return __wifi_country_from_enum(COUNTRY_CODE_EU, country);
    }
    if (strncmp(code, "TR", 2) == 0) {
        return __wifi_country_from_enum(COUNTRY_CODE_TR, country);
    }

    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief Determine whether a disconnect reason represents connect failure.
 * @param[in] status current station status
 * @param[in] reason disconnect reason
 * @param[in] local_generated whether the event is locally generated
 * @return TRUE if it should be reported as connect failure
 */
STATIC BOOL_T __wifi_is_connect_fail(WF_STATION_STAT_E status, INT_T reason, BOOL_T local_generated)
{
    if (reason == WIFI_REASON_NO_AP_FOUND) {
        return TRUE;
    }

    if (reason == WIFI_REASON_WRONG_PASSWORD) {
        return TRUE;
    }

    if (reason == WIFI_REASON_DHCP_TIMEOUT) {
        return TRUE;
    }

    if ((status == WSS_CONNECTING) && (local_generated == FALSE)) {
        return TRUE;
    }

    return FALSE;
}

/**
 * @brief Notify the upper Wi-Fi event callback if registered.
 * @param[in] event Tuya Wi-Fi event
 * @param[in] arg event argument
 * @return none
 */
STATIC VOID_T __wifi_report_event(WF_EVENT_E event, VOID_T *arg)
{
    bk_printf("report event=%s(%d) reason=%d\r\n",
              __wifi_tuya_event_name(event), event,
              (arg != NULL) ? *(WF_DISCONN_REASON_E *)arg : TUYA_WLAN_REASON_NONE);

    if (s_wifi.event_cb != NULL) {
        s_wifi.event_cb(event, arg);
    }
}

/**
 * @brief Stop Wi-Fi resources that conflict with the target work mode.
 * @param[in] mode target work mode
 * @return OPRT_OK on success, error code on failure
 */
STATIC OPERATE_RET __wifi_apply_work_mode(WF_WK_MD_E mode)
{
    bk_err_t bk_ret = BK_OK;

    bk_printf("apply work mode current=%s(%d) target=%s(%d) sta_started=%d ap_started=%d sniffer=%d\r\n",
              __wifi_work_mode_name(s_wifi.work_mode), s_wifi.work_mode,
              __wifi_work_mode_name(mode), mode,
              s_wifi.sta_started, s_wifi.ap_started, s_wifi.sniffer_enabled);

    if ((mode != WWM_POWERDOWN) &&
        (mode != WWM_SNIFFER) &&
        (mode != WWM_STATION) &&
        (mode != WWM_SOFTAP) &&
        (mode != WWM_STATIONAP)) {
        bk_printf("apply work mode invalid target=%d\r\n", mode);
        return OPRT_INVALID_PARM;
    }

    if ((mode != WWM_SNIFFER) && (s_wifi.sniffer_enabled == TRUE)) {
        bk_ret = bk_wifi_monitor_stop();
        if (bk_ret != BK_OK) {
            bk_printf("monitor stop fail bk_ret=%d while switching mode\r\n", bk_ret);
            return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_WORKMODE_SET_FAILED);
        }

        (VOID_T)bk_wifi_monitor_register_cb(NULL);
        s_wifi.sniffer_enabled = FALSE;
        s_wifi.sniffer_cb = NULL;
    }

    if ((mode == WWM_POWERDOWN) || (mode == WWM_SNIFFER) || (mode == WWM_SOFTAP)) {
        if (s_wifi.sta_started == TRUE) {
            bk_ret = bk_wifi_sta_stop();
            if (bk_ret != BK_OK) {
                bk_printf("sta stop fail bk_ret=%d while switching mode\r\n", bk_ret);
                return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_WORKMODE_SET_FAILED);
            }

            s_wifi.sta_started = FALSE;
            s_wifi.sta_status = WSS_IDLE;
        }
    }

    if ((mode == WWM_POWERDOWN) || (mode == WWM_SNIFFER) || (mode == WWM_STATION)) {
        if (s_wifi.ap_started == TRUE) {
            bk_ret = bk_wifi_ap_stop();
            if (bk_ret != BK_OK) {
                bk_printf("ap stop fail bk_ret=%d while switching mode\r\n", bk_ret);
                return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_WORKMODE_SET_FAILED);
            }

            s_wifi.ap_started = FALSE;
        }
    }

    s_wifi.work_mode = mode;
    bk_printf("apply work mode done new_mode=%s(%d)\r\n",
              __wifi_work_mode_name(s_wifi.work_mode), s_wifi.work_mode);
    return OPRT_OK;
}

/**
 * @brief Register the event callbacks needed by the adapter.
 * @return OPRT_OK on success, error code on failure
 */
STATIC OPERATE_RET __wifi_register_events(VOID_T)
{
    bk_err_t bk_ret = BK_OK;

    if (s_wifi.events_registered == TRUE) {
        return OPRT_OK;
    }

    bk_ret = bk_event_register_cb(EVENT_MOD_WIFI, EVENT_WIFI_SCAN_DONE, __wifi_event_cb, NULL);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_COM_ERROR);
    }

    bk_ret = bk_event_register_cb(EVENT_MOD_WIFI, EVENT_WIFI_NETWORK_FOUND, __wifi_event_cb, NULL);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_COM_ERROR);
    }

    bk_ret = bk_event_register_cb(EVENT_MOD_WIFI, EVENT_WIFI_STA_CONNECTED, __wifi_event_cb, NULL);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_COM_ERROR);
    }

    bk_ret = bk_event_register_cb(EVENT_MOD_WIFI, EVENT_WIFI_STA_DISCONNECTED, __wifi_event_cb, NULL);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_COM_ERROR);
    }

    bk_ret = bk_event_register_cb(EVENT_MOD_NETIF, EVENT_NETIF_GOT_IP4, __wifi_event_cb, NULL);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_COM_ERROR);
    }

    bk_ret = bk_event_register_cb(EVENT_MOD_NETIF, EVENT_NETIF_DHCP_TIMEOUT, __wifi_event_cb, NULL);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_COM_ERROR);
    }

    s_wifi.events_registered = TRUE;
    return OPRT_OK;
}

/**
 * @brief Beken Wi-Fi/netif event callback.
 * @param[in] arg user argument
 * @param[in] event_module event module ID
 * @param[in] event_id event ID
 * @param[in] event_data event payload
 * @return BK_OK always
 */
STATIC bk_err_t __wifi_event_cb(VOID_T *arg, event_module_t event_module, INT_T event_id, VOID_T *event_data)
{
    WF_DISCONN_REASON_E tuya_reason = TUYA_WLAN_REASON_NONE;

    (void)arg;

    if ((event_module == EVENT_MOD_WIFI) && (event_id == EVENT_WIFI_SCAN_DONE)) {
        s_wifi.scan_done = TRUE;
        bk_printf("scan done\r\n");
        return BK_OK;
    }

    if ((event_module == EVENT_MOD_WIFI) && (event_id == EVENT_WIFI_NETWORK_FOUND)) {
        if (s_wifi.sta_status == WSS_CONNECTING) {
            s_wifi.sta_status = WSS_CONNECTING;
        }
        return BK_OK;
    }

    if ((event_module == EVENT_MOD_WIFI) && (event_id == EVENT_WIFI_STA_CONNECTED)) {
        wifi_event_sta_connected_t *conn = (wifi_event_sta_connected_t *)event_data;

        if (conn != NULL) {
            bk_printf("sta connected ssid=%s\r\n", conn->ssid);
        }
        if (s_wifi.sta_status == WSS_CONNECTING) {
            s_wifi.sta_status = WSS_CONN_SUCCESS;
        }
        return BK_OK;
    }

    if ((event_module == EVENT_MOD_NETIF) && (event_id == EVENT_NETIF_GOT_IP4)) {
        netif_event_got_ip4_t *got_ip = (netif_event_got_ip4_t *)event_data;

        if ((got_ip != NULL) && (got_ip->netif_if == NETIF_IF_STA)) {
            bk_printf("sta got ip ip=%s\r\n", got_ip->ip);
            s_wifi.sta_status = WSS_GOT_IP;
            __wifi_report_event(WFE_CONNECTED, NULL);
        }
        return BK_OK;
    }

    if ((event_module == EVENT_MOD_NETIF) && (event_id == EVENT_NETIF_DHCP_TIMEOUT)) {
        bk_printf("sta dhcp timeout\r\n");
        s_wifi.sta_status = WSS_DHCP_FAIL;
        s_wifi.last_disc_reason = TUYA_WLAN_REASON_UNSPECIFIED;
        __wifi_report_event(WFE_CONNECT_FAILED, &s_wifi.last_disc_reason);
        return BK_OK;
    }

    if ((event_module == EVENT_MOD_WIFI) && (event_id == EVENT_WIFI_STA_DISCONNECTED)) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        INT_T bk_reason = WIFI_REASON_RESERVED;
        BOOL_T local_generated = FALSE;

        if (disc != NULL) {
            bk_reason = disc->disconnect_reason;
            local_generated = disc->local_generated ? TRUE : FALSE;
        }

        tuya_reason = __wifi_reason_bk_to_tuya(bk_reason);
        s_wifi.last_disc_reason = tuya_reason;
        bk_printf("sta disconnected bk_reason=%d tuya_reason=%d local_generated=%d prev_status=%d\r\n",
                  bk_reason, tuya_reason, local_generated, s_wifi.sta_status);

        if (__wifi_is_connect_fail(s_wifi.sta_status, bk_reason, local_generated) == TRUE) {
            if (bk_reason == WIFI_REASON_NO_AP_FOUND) {
                s_wifi.sta_status = WSS_NO_AP_FOUND;
            } else if (bk_reason == WIFI_REASON_WRONG_PASSWORD) {
                s_wifi.sta_status = WSS_PASSWD_WRONG;
            } else if (bk_reason == WIFI_REASON_DHCP_TIMEOUT) {
                s_wifi.sta_status = WSS_DHCP_FAIL;
            } else {
                s_wifi.sta_status = WSS_CONN_FAIL;
            }
            __wifi_report_event(WFE_CONNECT_FAILED, &s_wifi.last_disc_reason);
        } else {
            s_wifi.sta_status = WSS_CONN_FAIL;
            __wifi_report_event(WFE_DISCONNECTED, &s_wifi.last_disc_reason);
        }

        return BK_OK;
    }

    return BK_OK;
}

/**
 * @brief Relay monitor frames to the Tuya sniffer callback.
 * @param[in] frame raw 802.11 frame
 * @param[in] len frame length
 * @param[in] frame_info frame metadata
 * @return BK_OK always
 */
STATIC bk_err_t __wifi_monitor_cb(CONST UINT8_T *frame, UINT32_T len, CONST wifi_frame_info_t *frame_info)
{
    if ((s_wifi.sniffer_cb != NULL) && (frame != NULL)) {
        s_wifi.sniffer_cb(frame, (UINT16_T)len, (frame_info != NULL) ? (INT8_T)frame_info->rssi : 0);
    }

    return BK_OK;
}

/**
 * @brief Relay received management frames to the Tuya callback.
 * @param[in] frame raw 802.11 frame
 * @param[in] len frame length
 * @param[in] frame_info frame metadata
 * @return BK_OK always
 */
STATIC bk_err_t __wifi_filter_cb(CONST UINT8_T *frame, UINT32_T len, CONST wifi_frame_info_t *frame_info)
{
    (void)frame_info;

    if ((s_wifi.mgnt_cb != NULL) && (frame != NULL)) {
        s_wifi.mgnt_cb((UCHAR_T *)frame, (UINT_T)len);
    }

    return BK_OK;
}

/* ---------------------------------------------------------------------------
 * Function implementations
 * --------------------------------------------------------------------------- */
/**
 * @brief Initialize the Wi-Fi adapter and register event callbacks.
 * @param[in] cb upper Wi-Fi event callback, nullable
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_init(WIFI_EVENT_CB cb)
{
    OPERATE_RET op_ret = OPRT_OK;

    bk_printf("init enter inited=%d events_registered=%d cb=%s\r\n",
              s_wifi.inited, s_wifi.events_registered, (cb != NULL) ? "set" : "null");

    s_wifi.event_cb = cb;
    if (s_wifi.inited == TRUE) {
        bk_printf("init reuse existing context\r\n");
        return __wifi_register_events();
    }

    // bk_ret = bk_event_init();
    // if (bk_ret != BK_OK) {
    //     return __wifi_map_ret(bk_ret, OPRT_COM_ERROR);
    // }

    // bk_ret = bk_netif_init();
    // if (bk_ret != BK_OK) {
    //     return __wifi_map_ret(bk_ret, OPRT_COM_ERROR);
    // }

    // bk_ret = bk_wifi_init();
    // if (bk_ret != BK_OK) {
    //     return __wifi_map_ret(bk_ret, OPRT_COM_ERROR);
    // }

    op_ret = __wifi_register_events();
    if (op_ret != OPRT_OK) {
        bk_printf("init register events fail ret=%d\r\n", op_ret);
        return op_ret;
    }

    s_wifi.inited = TRUE;
    bk_printf("init done\r\n");
    return OPRT_OK;
}

/**
 * @brief Test whether a Beken scan entry passes the optional SSID filter.
 * @param[in] ssid_filter optional exact-match SSID, NULL accepts everything
 * @param[in] bk_ap Beken scan entry
 * @return TRUE if the entry passes the filter
 */
STATIC BOOL_T __wifi_scan_ap_match(CONST SCHAR_T *ssid_filter, CONST wifi_scan_ap_info_t *bk_ap)
{
    UINT_T filter_len = 0;

    if (ssid_filter == NULL) {
        return TRUE;
    }

    filter_len = __wifi_strnlen(ssid_filter, WIFI_SSID_LEN + 1);
    if (filter_len != __wifi_strnlen((CONST CHAR_T *)bk_ap->ssid, WIFI_SSID_STR_LEN)) {
        return FALSE;
    }

    return (memcmp(ssid_filter, bk_ap->ssid, filter_len) == 0) ? TRUE : FALSE;
}

/**
 * @brief Trigger a blocking scan and copy the results into Tuya AP entries.
 * @param[in] scan_cfg Beken scan configuration, NULL scans all channels
 * @param[in] ssid_filter optional exact-match SSID applied to the results
 * @param[out] ap_ary scan result array, freed by tkl_wifi_release_ap()
 * @param[out] num result count
 * @return OPRT_OK on success, error code on failure
 */
STATIC OPERATE_RET __wifi_scan_blocking(CONST wifi_scan_config_t *scan_cfg, CONST SCHAR_T *ssid_filter, AP_IF_S **ap_ary, UINT_T *num)
{
    wifi_scan_result_t scan_result;
    AP_IF_S *ap_list = NULL;
    UINT_T wait_ms = 0;
    UINT_T match_num = 0;
    UINT_T copy_index = 0;
    bk_err_t bk_ret = BK_OK;
    INT_T index = 0;

    *ap_ary = NULL;
    *num = 0;
    memset(&scan_result, 0, sizeof(scan_result));

    s_wifi.scan_done = FALSE;
    bk_ret = bk_wifi_scan_start(scan_cfg);
    if (bk_ret != BK_OK) {
        bk_printf("scan start fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_AP_SCAN_FAILED);
    }

    while ((s_wifi.scan_done == FALSE) && (wait_ms < TKL_WIFI_SCAN_WAIT_MS)) {
        tkl_system_sleep(TKL_WIFI_SCAN_POLL_MS);
        wait_ms += TKL_WIFI_SCAN_POLL_MS;
    }

    if (s_wifi.scan_done == FALSE) {
        (VOID_T)bk_wifi_scan_stop();
        bk_printf("scan timeout after %u ms\r\n", (unsigned int)wait_ms);
        return OPRT_OS_ADAPTER_AP_SCAN_FAILED;
    }

    bk_ret = bk_wifi_scan_get_result(&scan_result);
    if (bk_ret != BK_OK) {
        bk_printf("scan get result fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_AP_SCAN_FAILED);
    }

    if ((scan_result.ap_num <= 0) || (scan_result.aps == NULL)) {
        bk_printf("scan result empty ap_num=%d\r\n", scan_result.ap_num);
        bk_wifi_scan_free_result(&scan_result);
        return OPRT_OS_ADAPTER_AP_NOT_FOUND;
    }

    for (index = 0; index < scan_result.ap_num; index++) {
        if (__wifi_scan_ap_match(ssid_filter, &scan_result.aps[index]) == TRUE) {
            match_num++;
        }
    }

    if (match_num == 0) {
        bk_printf("scan result filtered empty raw_num=%d\r\n", scan_result.ap_num);
        bk_wifi_scan_free_result(&scan_result);
        return OPRT_OS_ADAPTER_AP_NOT_FOUND;
    }

    ap_list = os_malloc((UINT_T)match_num * TKL_WIFI_AP_ENTRY_SIZE);
    if (ap_list == NULL) {
        bk_printf("scan malloc fail match_num=%u\r\n", (unsigned int)match_num);
        bk_wifi_scan_free_result(&scan_result);
        return OPRT_MALLOC_FAILED;
    }

    memset(ap_list, 0, (UINT_T)match_num * TKL_WIFI_AP_ENTRY_SIZE);
    for (index = 0; index < scan_result.ap_num; index++) {
        AP_IF_S *ap = NULL;
        wifi_scan_ap_info_t *bk_ap = &scan_result.aps[index];

        if (__wifi_scan_ap_match(ssid_filter, bk_ap) != TRUE) {
            continue;
        }

        ap = (AP_IF_S *)((UINT8_T *)ap_list + (copy_index * TKL_WIFI_AP_ENTRY_SIZE));
        ap->channel = bk_ap->channel;
        ap->rssi = (SCHAR_T)bk_ap->rssi;
        memcpy(ap->bssid, bk_ap->bssid, sizeof(ap->bssid));
        __wifi_copy_str((CHAR_T *)ap->ssid, sizeof(ap->ssid), bk_ap->ssid);
        ap->s_len = (UCHAR_T)__wifi_strnlen((CONST CHAR_T *)ap->ssid, WIFI_SSID_LEN);
        ap->security = (UCHAR_T)__wifi_auth_bk_to_tuya(bk_ap->security);
        ap->data_len = 0;
        copy_index++;
    }

    bk_wifi_scan_free_result(&scan_result);
    *ap_ary = ap_list;
    *num = copy_index;
    bk_printf("scan done copied_num=%u raw_num=%d\r\n", (unsigned int)*num, scan_result.ap_num);
    return OPRT_OK;
}

/**
 * @brief Trigger a blocking scan and copy the results into Tuya AP entries.
 * @param[in] ssid optional scan SSID
 * @param[out] ap_ary scan result array, freed by tkl_wifi_release_ap()
 * @param[out] num result count
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_scan_ap(CONST SCHAR_T *ssid, AP_IF_S **ap_ary, UINT_T *num)
{
    wifi_scan_config_t scan_cfg;

    if ((ap_ary == NULL) || (num == NULL)) {
        bk_printf("scan invalid param ap_ary=%d num=%d\r\n", ap_ary != NULL, num != NULL);
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        bk_printf("scan rejected: wifi not init\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    memset(&scan_cfg, 0, sizeof(scan_cfg));
    if (ssid != NULL) {
        if (__wifi_strnlen(ssid, WIFI_SSID_LEN + 1) > WIFI_SSID_LEN) {
            bk_printf("scan invalid ssid length ssid=%s\r\n", ssid);
            return OPRT_INVALID_PARM;
        }
        __wifi_copy_str(scan_cfg.ssid, sizeof(scan_cfg.ssid), ssid);
    }

    bk_printf("scan start ssid=%s\r\n", (ssid != NULL) ? ssid : "<all>");
    return __wifi_scan_blocking((ssid != NULL) ? &scan_cfg : NULL, ssid, ap_ary, num);
}

/**
 * @brief Scan APs by a unified scan parameter, supports band / channel / ssid scope.
 * @param[in] param scan parameter, see WF_SCAN_PARAM_T
 * @param[out] ap_ary scan result array, freed by tkl_wifi_release_ap()
 * @param[out] num result count
 * @return OPRT_OK on success, OPRT_NOT_SUPPORTED on WF_BAND_5G,
 *         error code on failure (an unsupported channel fails via the
 *         firmware: the scan never completes and the wait times out)
 */
OPERATE_RET tkl_wifi_scan_ap_ex(CONST WF_SCAN_PARAM_T *param, AP_IF_S **ap_ary, UINT_T *num)
{
    wifi_scan_config_t scan_cfg;
    CONST CHAR_T *band_name = "all";

    if ((param == NULL) || (ap_ary == NULL) || (num == NULL)) {
        bk_printf("scan_ex invalid param param=%d ap_ary=%d num=%d\r\n",
                  param != NULL, ap_ary != NULL, num != NULL);
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        bk_printf("scan_ex rejected: wifi not init\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (param->band == WF_BAND_5G) {
        bk_printf("scan_ex unsupported band=5g\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    if ((param->band != WF_BAND_ALL) && (param->band != WF_BAND_2_4G)) {
        bk_printf("scan_ex invalid band=%d\r\n", param->band);
        return OPRT_INVALID_PARM;
    }

    memset(&scan_cfg, 0, sizeof(scan_cfg));
    if (param->ssid != NULL) {
        if (__wifi_strnlen(param->ssid, WIFI_SSID_LEN + 1) > WIFI_SSID_LEN) {
            bk_printf("scan_ex invalid ssid length\r\n");
            return OPRT_INVALID_PARM;
        }
        __wifi_copy_str(scan_cfg.ssid, sizeof(scan_cfg.ssid), param->ssid);
    }

    if (param->channel != 0) {
        scan_cfg.chan_nb[0] = param->channel;
        scan_cfg.chan_cnt = 1;
        band_name = "chan";
    }

    bk_printf("scan_ex start ssid=%s band=%s chan=%u\r\n",
              (param->ssid != NULL) ? param->ssid : "<all>",
              band_name, param->channel);

    return __wifi_scan_blocking(&scan_cfg, param->ssid, ap_ary, num);
}

/**
 * @brief Release scan results allocated by tkl_wifi_scan_ap().
 * @param[in] ap scan result pointer
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_release_ap(AP_IF_S *ap)
{
    if (ap == NULL) {
        return OPRT_OK;
    }

    os_free(ap);
    return OPRT_OK;
}

/**
 * @brief Configure and start SoftAP mode.
 * @param[in] cfg SoftAP configuration
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_start_ap(CONST WF_AP_CFG_IF_S *cfg)
{
    wifi_ap_config_t ap_cfg;
    netif_ip4_config_t ip4_cfg;
    NW_IP_S ap_ip_cfg;
    CHAR_T ssid_buf[WIFI_SSID_LEN + 1] = {0};
    CHAR_T passwd_buf[WIFI_PASSWD_LEN + 1] = {0};
    bk_err_t bk_ret = BK_OK;
    OPERATE_RET op_ret = OPRT_OK;

    if (cfg == NULL) {
        bk_printf("start_ap invalid param: cfg is null\r\n");
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        bk_printf("start_ap rejected: wifi not init\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    memcpy(ssid_buf, cfg->ssid, (cfg->s_len <= WIFI_SSID_LEN) ? cfg->s_len : WIFI_SSID_LEN);
    memcpy(passwd_buf, cfg->passwd, (cfg->p_len <= WIFI_PASSWD_LEN) ? cfg->p_len : WIFI_PASSWD_LEN);

    bk_printf("start_ap enter ssid=%s key=%s chan=%u hidden=%u max_conn=%u dhcp=%d ip=%s mask=%s gw=%s dns=%s\r\n",
              ssid_buf, passwd_buf, cfg->chan, cfg->ssid_hidden, cfg->max_conn, cfg->ip.dhcpen,
              cfg->ip.ip, cfg->ip.mask, cfg->ip.gw, cfg->ip.dns);

    if ((cfg->s_len == 0) || (cfg->s_len > WIFI_SSID_LEN)) {
        bk_printf("start_ap invalid ssid len=%u\r\n", cfg->s_len);
        return OPRT_INVALID_PARM;
    }

    if (cfg->chan > WIFI_MAX_CHAN_NUM) {
        bk_printf("start_ap invalid channel=%u\r\n", cfg->chan);
        return OPRT_INVALID_PARM;
    }

    if ((cfg->md != WAAM_OPEN) && ((cfg->p_len < 8) || (cfg->p_len > WIFI_PASSWD_LEN))) {
        bk_printf("start_ap invalid passwd len=%u auth=%d\r\n", cfg->p_len, cfg->md);
        return OPRT_INVALID_PARM;
    }

    op_ret = __wifi_apply_work_mode((s_wifi.sta_started == TRUE) ? WWM_STATIONAP : WWM_SOFTAP);
    if (op_ret != OPRT_OK) {
        bk_printf("start_ap apply work mode fail ret=%d\r\n", op_ret);
        return op_ret;
    }

    memset(&ap_cfg, 0, sizeof(ap_cfg));
    memset(&ip4_cfg, 0, sizeof(ip4_cfg));
    memset(&ap_ip_cfg, 0, sizeof(ap_ip_cfg));
    memcpy(&ap_ip_cfg, &cfg->ip, sizeof(ap_ip_cfg));

    memcpy(ap_cfg.ssid, cfg->ssid, cfg->s_len);
    ap_cfg.ssid[cfg->s_len] = '\0';
    memcpy(ap_cfg.password, cfg->passwd, cfg->p_len);
    ap_cfg.password[cfg->p_len] = '\0';
    ap_cfg.channel = cfg->chan;
    ap_cfg.security = __wifi_auth_tuya_to_bk(cfg->md);
    ap_cfg.hidden = cfg->ssid_hidden ? 1 : 0;
    ap_cfg.max_con = cfg->max_conn;

    if ((ap_ip_cfg.dhcpen == FALSE) && (ap_ip_cfg.dns[0] == '\0')) {
        __wifi_copy_str(ap_ip_cfg.dns, sizeof(ap_ip_cfg.dns),
                        (ap_ip_cfg.gw[0] != '\0') ? ap_ip_cfg.gw : ap_ip_cfg.ip);
        bk_printf("start_ap fill empty dns=%s\r\n", ap_ip_cfg.dns);
    }

    op_ret = __wifi_fill_bk_ip4(&ip4_cfg, &ap_ip_cfg);
    if (op_ret != OPRT_OK) {
        bk_printf("start_ap invalid ip config ret=%d\r\n", op_ret);
        return op_ret;
    }

    bk_ret = bk_netif_set_ip4_config(NETIF_IF_AP, &ip4_cfg);
    if (bk_ret != BK_OK) {
        bk_printf("start_ap set ap ip fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_IP_GET_FAILED);
    }

    bk_ret = bk_wifi_ap_set_config(&ap_cfg);
    if (bk_ret != BK_OK) {
        bk_printf("start_ap set config fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_AP_START_FAILED);
    }

    bk_ret = bk_wifi_ap_start();
    if (bk_ret != BK_OK) {
        bk_printf("start_ap start fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_AP_START_FAILED);
    }

    s_wifi.ap_started = TRUE;
    if (s_wifi.work_mode != WWM_STATIONAP) {
        s_wifi.work_mode = WWM_SOFTAP;
    }
    bk_printf("start_ap done mode=%s(%d)\r\n",
              __wifi_work_mode_name(s_wifi.work_mode), s_wifi.work_mode);
    return OPRT_OK;
}

/**
 * @brief Stop SoftAP mode.
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_stop_ap(VOID_T)
{
    bk_err_t bk_ret = BK_OK;

    bk_printf("stop_ap enter ap_started=%d mode=%s(%d)\r\n",
              s_wifi.ap_started, __wifi_work_mode_name(s_wifi.work_mode), s_wifi.work_mode);

    if (s_wifi.inited == FALSE) {
        bk_printf("stop_ap rejected: wifi not init\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_wifi.ap_started == FALSE) {
        bk_printf("stop_ap skip: ap not started\r\n");
        return OPRT_OK;
    }

    bk_ret = bk_wifi_ap_stop();
    if (bk_ret != BK_OK) {
        bk_printf("stop_ap fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_AP_STOP_FAILED);
    }

    s_wifi.ap_started = FALSE;
    if (s_wifi.work_mode == WWM_SOFTAP) {
        s_wifi.work_mode = s_wifi.sta_started ? WWM_STATION : WWM_POWERDOWN;
    } else if (s_wifi.work_mode == WWM_STATIONAP) {
        s_wifi.work_mode = s_wifi.sta_started ? WWM_STATION : WWM_POWERDOWN;
    }
    bk_printf("stop_ap done new_mode=%s(%d)\r\n",
              __wifi_work_mode_name(s_wifi.work_mode), s_wifi.work_mode);
    return OPRT_OK;
}

/**
 * @brief Set the current monitor channel.
 * @param[in] chan target Wi-Fi channel
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_set_cur_channel(CONST UCHAR_T chan)
{
    wifi_channel_t channel_cfg;
    bk_err_t bk_ret = BK_OK;

    if ((chan < WIFI_MIN_CHAN_NUM) || (chan > WIFI_MAX_CHAN_NUM)) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    memset(&channel_cfg, 0, sizeof(channel_cfg));
    channel_cfg.primary = chan;
    channel_cfg.second = WIFI_SECOND_CHANNEL_NONE;

    bk_ret = bk_wifi_monitor_set_channel(&channel_cfg);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_CHAN_SET_FAILED);
    }

    return OPRT_OK;
}

/**
 * @brief Get the current Wi-Fi channel.
 * @param[out] chan current Wi-Fi channel
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_get_cur_channel(UCHAR_T *chan)
{
    bk_err_t bk_ret = BK_OK;

    if (chan == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    bk_ret = bk_wifi_get_channel();
    if ((bk_ret < WIFI_MIN_CHAN_NUM) || (bk_ret > WIFI_MAX_CHAN_NUM)) {
        return OPRT_OS_ADAPTER_CHAN_GET_FAILED;
    }

    *chan = (UCHAR_T)bk_ret;
    return OPRT_OK;
}

/**
 * @brief Enable or disable sniffer mode.
 * @param[in] en enable flag
 * @param[in] cb sniffer callback when enabling
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_set_sniffer(CONST BOOL_T en, CONST SNIFFER_CALLBACK cb)
{
    wifi_monitor_config_t monitor_cfg;
    bk_err_t bk_ret = BK_OK;
    OPERATE_RET op_ret = OPRT_OK;

    bk_printf("set_sniffer enter en=%d current=%d mode=%s(%d)\r\n",
              en, s_wifi.sniffer_enabled, __wifi_work_mode_name(s_wifi.work_mode), s_wifi.work_mode);

    if ((en == TRUE) && (cb == NULL)) {
        bk_printf("set_sniffer invalid param: enable with null cb\r\n");
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        bk_printf("set_sniffer rejected: wifi not init\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (en == FALSE) {
        if (s_wifi.sniffer_enabled == FALSE) {
            bk_printf("set_sniffer disable skip: already disabled\r\n");
            return OPRT_OK;
        }

        bk_ret = bk_wifi_monitor_stop();
        if (bk_ret != BK_OK) {
            bk_printf("set_sniffer disable fail bk_ret=%d\r\n", bk_ret);
            return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_SNIFFER_SET_FAILED);
        }

        (VOID_T)bk_wifi_monitor_register_cb(NULL);
        s_wifi.sniffer_enabled = FALSE;
        s_wifi.sniffer_cb = NULL;
        if (s_wifi.work_mode == WWM_SNIFFER) {
            s_wifi.work_mode = WWM_POWERDOWN;
        }
        bk_printf("set_sniffer disable done new_mode=%s(%d)\r\n",
                  __wifi_work_mode_name(s_wifi.work_mode), s_wifi.work_mode);
        return OPRT_OK;
    }

    op_ret = __wifi_apply_work_mode(WWM_SNIFFER);
    if (op_ret != OPRT_OK) {
        bk_printf("set_sniffer apply work mode fail ret=%d\r\n", op_ret);
        return op_ret;
    }

    memset(&monitor_cfg, 0, sizeof(monitor_cfg));
    monitor_cfg.rx_mcast_frame = 1;
    monitor_cfg.rx_dup_frame = 1;

    // bk_ret = bk_wifi_monitor_set_config(&monitor_cfg);
    // if (bk_ret != BK_OK) {
    //     return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_SNIFFER_SET_FAILED);
    // }

    bk_ret = bk_wifi_monitor_register_cb(__wifi_monitor_cb);
    if (bk_ret != BK_OK) {
        bk_printf("set_sniffer register cb fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_SNIFFER_SET_FAILED);
    }

    bk_ret = bk_wifi_monitor_start();
    if (bk_ret != BK_OK) {
        (VOID_T)bk_wifi_monitor_register_cb(NULL);
        bk_printf("set_sniffer start fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_SNIFFER_SET_FAILED);
    }

    s_wifi.sniffer_enabled = TRUE;
    s_wifi.sniffer_cb = cb;
    s_wifi.work_mode = WWM_SNIFFER;
    bk_printf("set_sniffer enable done\r\n");
    return OPRT_OK;
}

/**
 * @brief Get the IPv4 configuration of the specified Wi-Fi interface.
 * @param[in] wf Wi-Fi interface
 * @param[out] ip IP configuration
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_get_ip(CONST WF_IF_E wf, NW_IP_S *ip)
{
    netif_if_t ifx = NETIF_IF_INVALID;
    netif_ip4_config_t ip4_cfg;
    bk_err_t bk_ret = BK_OK;
    OPERATE_RET op_ret = OPRT_OK;

    if (ip == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    op_ret = __wifi_if_to_netif(wf, &ifx);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    memset(&ip4_cfg, 0, sizeof(ip4_cfg));
    bk_ret = bk_netif_get_ip4_config(ifx, &ip4_cfg);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_IP_GET_FAILED);
    }

    __wifi_fill_tuya_ip(ip, &ip4_cfg, (wf == WF_STATION) ? s_wifi.sta_dhcp_enabled : FALSE);
    return OPRT_OK;
}

/**
 * @brief Get the IPv6 configuration of the specified Wi-Fi interface.
 * @param[in] wf Wi-Fi interface
 * @param[in] type IPv6 address type
 * @param[out] ip IPv6 configuration
 * @return OPRT_NOT_SUPPORTED for T9
 */
OPERATE_RET tkl_wifi_get_ipv6(CONST WF_IF_E wf, NW_IP_TYPE type, NW_IP_S *ip)
{
    (void)wf;
    (void)type;
    (void)ip;
    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief Configure the IPv4 settings of the specified Wi-Fi interface.
 * @param[in] wf Wi-Fi interface
 * @param[in] ip IP configuration
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_set_ip(CONST WF_IF_E wf, NW_IP_S *ip)
{
    netif_if_t ifx = NETIF_IF_INVALID;
    netif_ip4_config_t ip4_cfg;
    bk_err_t bk_ret = BK_OK;
    OPERATE_RET op_ret = OPRT_OK;

    if (ip == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    op_ret = __wifi_if_to_netif(wf, &ifx);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    if ((wf == WF_AP) && (ip->dhcpen == TRUE)) {
        return OPRT_NOT_SUPPORTED;
    }

    if ((wf == WF_STATION) && (ip->dhcpen == TRUE)) {
        bk_ret = bk_netif_dhcpc_start(NETIF_IF_STA);
        if (bk_ret != BK_OK) {
            return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_IP_GET_FAILED);
        }
        s_wifi.sta_dhcp_enabled = TRUE;
        return OPRT_OK;
    }

    op_ret = __wifi_fill_bk_ip4(&ip4_cfg, ip);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    bk_ret = bk_netif_static_ip(ip4_cfg);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_IP_GET_FAILED);
    }

    if (wf == WF_STATION) {
        s_wifi.sta_dhcp_enabled = FALSE;
    }
    return OPRT_OK;
}

/**
 * @brief Configure the MAC address of the specified Wi-Fi interface.
 * @param[in] wf Wi-Fi interface
 * @param[in] mac target MAC address
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_set_mac(CONST WF_IF_E wf, CONST NW_MAC_S *mac)
{
    bk_err_t bk_ret = BK_OK;

    if (mac == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (wf == WF_AP) {
        return OPRT_NOT_SUPPORTED;
    }

    if ((mac->mac[0] == 0) && (mac->mac[1] == 0) && (mac->mac[2] == 0) &&
        (mac->mac[3] == 0) && (mac->mac[4] == 0) && (mac->mac[5] == 0)) {
        return OPRT_INVALID_PARM;
    }

    if ((mac->mac[0] == 0xFF) && (mac->mac[1] == 0xFF) && (mac->mac[2] == 0xFF) &&
        (mac->mac[3] == 0xFF) && (mac->mac[4] == 0xFF) && (mac->mac[5] == 0xFF)) {
        return OPRT_INVALID_PARM;
    }

    bk_ret = bk_wifi_set_mac_address((CHAR_T *)mac->mac);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_MAC_SET_FAILED);
    }

    return OPRT_OK;
}

/**
 * @brief Get the MAC address of the specified Wi-Fi interface.
 * @param[in] wf Wi-Fi interface
 * @param[out] mac MAC address
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_get_mac(CONST WF_IF_E wf, NW_MAC_S *mac)
{
    bk_err_t bk_ret = BK_OK;

    if (mac == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    memset(mac, 0, sizeof(NW_MAC_S));
    if (wf == WF_STATION) {
        bk_ret = bk_wifi_sta_get_mac(mac->mac);
    } else if (wf == WF_AP) {
        bk_ret = bk_wifi_ap_get_mac(mac->mac);
    } else {
        return OPRT_INVALID_PARM;
    }

    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_MAC_GET_FAILED);
    }

    return OPRT_OK;
}

/**
 * @brief Set the logical Wi-Fi work mode and stop conflicting resources.
 * @param[in] mode target work mode
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_set_work_mode(CONST WF_WK_MD_E mode)
{
    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    return __wifi_apply_work_mode(mode);
}

/**
 * @brief Get the current logical Wi-Fi work mode.
 * @param[out] mode current work mode
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_get_work_mode(WF_WK_MD_E *mode)
{
    if (mode == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *mode = s_wifi.work_mode;
    return OPRT_OK;
}

/**
 * @brief Build a fast-connect blob from the current STA connection.
 * @param[out] fast_ap_info output fast-connect blob
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_get_connected_ap_info(FAST_WF_CONNECTED_AP_INFO_T **fast_ap_info)
{
    FAST_WF_CONNECTED_AP_INFO_T *info = NULL;
    wifi_link_status_t link_status;
    wifi_sta_config_t sta_cfg;
    NW_IP_S ip;
    UINT8_T ip_addr[4] = {0};
    UINT8_T mask[4] = {0};
    UINT8_T gw[4] = {0};
    UINT8_T dns[4] = {0};
    bk_err_t bk_ret = BK_OK;

    if (fast_ap_info == NULL) {
        bk_printf("get_connected_ap_info invalid param: out is null\r\n");
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        bk_printf("get_connected_ap_info rejected: wifi not init\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    memset(&link_status, 0, sizeof(link_status));
    memset(&sta_cfg, 0, sizeof(sta_cfg));
    memset(&ip, 0, sizeof(ip));

    bk_ret = bk_wifi_sta_get_link_status(&link_status);
    if (bk_ret != BK_OK) {
        bk_printf("get_connected_ap_info get link status fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_APINFO_GET_FAILED);
    }

    if ((link_status.state != WIFI_LINKSTATE_STA_CONNECTED) &&
        (link_status.state != WIFI_LINKSTATE_STA_GOT_IP)) {
        bk_printf("get_connected_ap_info not connected state=%d\r\n", link_status.state);
        return OPRT_RESOURCE_NOT_READY;
    }

    bk_ret = bk_wifi_sta_get_config(&sta_cfg);
    if (bk_ret != BK_OK) {
        bk_printf("get_connected_ap_info get config fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_APINFO_GET_FAILED);
    }

    __wifi_copy_str(sta_cfg.ssid, sizeof(sta_cfg.ssid), link_status.ssid);
    memcpy(sta_cfg.bssid, link_status.bssid, sizeof(sta_cfg.bssid));
    sta_cfg.channel = link_status.channel;
    sta_cfg.security = link_status.security;
    sta_cfg.is_user_fast_connect = 1;

    if ((tkl_wifi_get_ip(WF_STATION, &ip) == OPRT_OK) &&
        (__wifi_parse_ipv4(ip.ip, ip_addr) == TRUE) &&
        (__wifi_parse_ipv4(ip.mask, mask) == TRUE) &&
        (__wifi_parse_ipv4(ip.gw, gw) == TRUE) &&
        (__wifi_parse_ipv4(ip.dns, dns) == TRUE)) {
        memcpy(sta_cfg.ip_addr, ip_addr, sizeof(sta_cfg.ip_addr));
        memcpy(sta_cfg.netmask, mask, sizeof(sta_cfg.netmask));
        memcpy(sta_cfg.gw, gw, sizeof(sta_cfg.gw));
        memcpy(sta_cfg.dns1, dns, sizeof(sta_cfg.dns1));
    }

    info = os_malloc(sizeof(FAST_WF_CONNECTED_AP_INFO_T) + sizeof(wifi_sta_config_t));
    if (info == NULL) {
        bk_printf("get_connected_ap_info malloc fail\r\n");
        return OPRT_MALLOC_FAILED;
    }

    info->len = sizeof(wifi_sta_config_t);
    memcpy(info->data, &sta_cfg, sizeof(wifi_sta_config_t));

    *fast_ap_info = info;
    __fast_connect_ap_info_dump("get connected ap info", &sta_cfg);
    return OPRT_OK;
}

/**
 * @brief Get the BSSID of the current STA uplink.
 * @param[out] mac BSSID output buffer
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_get_bssid(UCHAR_T *mac)
{
    wifi_link_status_t link_status;
    bk_err_t bk_ret = BK_OK;

    if (mac == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    memset(&link_status, 0, sizeof(link_status));
    bk_ret = bk_wifi_sta_get_link_status(&link_status);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_BSSID_GET_FAILED);
    }

    if ((link_status.state != WIFI_LINKSTATE_STA_CONNECTED) &&
        (link_status.state != WIFI_LINKSTATE_STA_GOT_IP)) {
        return OPRT_RESOURCE_NOT_READY;
    }

    memcpy(mac, link_status.bssid, WIFI_BSSID_LEN);
    return OPRT_OK;
}

/**
 * @brief Set the Wi-Fi country code from a Tuya enum.
 * @param[in] ccode Tuya country code enum
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_set_country_code(CONST COUNTRY_CODE_E ccode)
{
    wifi_country_t country;
    bk_err_t bk_ret = BK_OK;
    OPERATE_RET op_ret = OPRT_OK;

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    op_ret = __wifi_country_from_enum(ccode, &country);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    bk_ret = bk_wifi_set_country(&country);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_CCODE_SET_FAILED);
    }

    memcpy(s_wifi.country_code, country.cc, 2);
    s_wifi.country_code[2] = '\0';
    return OPRT_OK;
}

/**
 * @brief Trigger or confirm Wi-Fi RF calibration status.
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_set_rf_calibrated(VOID_T)
{
    int stat = bk_wifi_manual_cal_rfcali_status();

    bk_printf("tkl_wifi_set_rf_calibrated stat: %d\r\n", stat);

    if(stat == BK_OK) {
        return true;
    }

    return false;
}

/**
 * @brief Enable or disable station low-power mode.
 * @param[in] enable enable flag
 * @param[in] dtim listen interval / DTIM hint
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_set_lp_mode(CONST BOOL_T enable, CONST UCHAR_T dtim)
{
    bk_err_t bk_ret = BK_OK;

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (enable == TRUE) {
        if (dtim > 0) {
            bk_ret = bk_wifi_send_listen_interval_req(dtim);
            if (bk_ret != BK_OK) {
                return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_WF_LPMODE_SET_FAILED);
            }
        }

        bk_ret = bk_wifi_sta_pm_enable();
    } else {
        bk_ret = bk_wifi_sta_pm_disable();
    }

    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_WF_LPMODE_SET_FAILED);
    }

    return OPRT_OK;
}

/**
 * @brief Fast connect using a previously exported opaque blob.
 * @param[in] fast_ap_info opaque fast-connect blob
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_station_fast_connect(CONST FAST_WF_CONNECTED_AP_INFO_T *fast_ap_info)
{
    CONST wifi_sta_config_t *sta_cfg = NULL;
    bk_err_t bk_ret = BK_OK;
    OPERATE_RET op_ret = OPRT_OK;

    if (fast_ap_info == NULL) {
        bk_printf("fast_connect invalid param: info is null\r\n");
        return OPRT_INVALID_PARM;
    }

    if (fast_ap_info->len != sizeof(wifi_sta_config_t)) {
        bk_printf("fast_connect invalid blob len=%u expect=%u\r\n",
                  (unsigned int)fast_ap_info->len, (unsigned int)sizeof(wifi_sta_config_t));
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        bk_printf("fast_connect rejected: wifi not init\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    sta_cfg = (CONST wifi_sta_config_t *)fast_ap_info->data;
    bk_printf("fast_connect enter ssid=%s chan=%d security=%d bssid=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
              sta_cfg->ssid, sta_cfg->channel, sta_cfg->security,
              sta_cfg->bssid[0], sta_cfg->bssid[1], sta_cfg->bssid[2],
              sta_cfg->bssid[3], sta_cfg->bssid[4], sta_cfg->bssid[5]);
    __fast_connect_ap_info_dump("station fast connect", sta_cfg);

    if (sta_cfg->ssid[0] == '\0') {
        bk_printf("fast_connect invalid ssid empty\r\n");
        return OPRT_INVALID_PARM;
    }

    op_ret = __wifi_apply_work_mode(s_wifi.ap_started ? WWM_STATIONAP : WWM_STATION);
    if (op_ret != OPRT_OK) {
        bk_printf("fast_connect apply work mode fail ret=%d\r\n", op_ret);
        return op_ret;
    }

    bk_ret = bk_wifi_sta_set_config(sta_cfg);
    if (bk_ret != BK_OK) {
        bk_printf("fast_connect set config fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_FAST_CONN_FAILED);
    }

    bk_ret = bk_wifi_sta_start();
    if (bk_ret != BK_OK) {
        bk_printf("fast_connect sta start fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_FAST_CONN_FAILED);
    }

    s_wifi.sta_started = TRUE;
    s_wifi.sta_status = WSS_CONNECTING;
    if (s_wifi.work_mode != WWM_STATIONAP) {
        s_wifi.work_mode = WWM_STATION;
    }
    bk_printf("fast_connect started\r\n");
    return OPRT_OK;
}

/**
 * @brief Start a non-blocking STA connection attempt.
 * @param[in] ssid target SSID
 * @param[in] passwd target password, nullable for open AP
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_station_connect(CONST SCHAR_T *ssid, CONST SCHAR_T *passwd)
{
    wifi_sta_config_t sta_cfg;
    bk_err_t bk_ret = BK_OK;
    OPERATE_RET op_ret = OPRT_OK;

    if (ssid == NULL) {
        bk_printf("connect invalid param: ssid is null\r\n");
        return OPRT_INVALID_PARM;
    }

    if ((__wifi_strnlen(ssid, WIFI_SSID_LEN + 1) == 0) ||
        (__wifi_strnlen(ssid, WIFI_SSID_LEN + 1) > WIFI_SSID_LEN)) {
        bk_printf("connect invalid ssid len=%u ssid=%s\r\n",
                  (unsigned int)__wifi_strnlen(ssid, WIFI_SSID_LEN + 1), ssid);
        return OPRT_INVALID_PARM;
    }

    if ((passwd != NULL) && (__wifi_strnlen(passwd, WIFI_PASSWD_LEN + 1) > WIFI_PASSWD_LEN)) {
        bk_printf("connect invalid passwd len=%u\r\n",
                  (unsigned int)__wifi_strnlen(passwd, WIFI_PASSWD_LEN + 1));
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        bk_printf("connect rejected: wifi not init\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    bk_printf("connect enter ssid=%s passwd=%s current_mode=%s(%d)\r\n",
              ssid, (passwd != NULL) ? "<set>" : "<open>",
              __wifi_work_mode_name(s_wifi.work_mode), s_wifi.work_mode);

    op_ret = __wifi_apply_work_mode(s_wifi.ap_started ? WWM_STATIONAP : WWM_STATION);
    if (op_ret != OPRT_OK) {
        bk_printf("connect apply work mode fail ret=%d\r\n", op_ret);
        return op_ret;
    }

    memset(&sta_cfg, 0, sizeof(sta_cfg));
    __wifi_copy_str(sta_cfg.ssid, sizeof(sta_cfg.ssid), ssid);
    if (passwd != NULL) {
        __wifi_copy_str(sta_cfg.password, sizeof(sta_cfg.password), passwd);
    }
    sta_cfg.security = WIFI_SECURITY_AUTO;

    bk_ret = bk_wifi_sta_set_config(&sta_cfg);
    if (bk_ret != BK_OK) {
        bk_printf("connect set config fail bk_ret=%d ssid=%s\r\n", bk_ret, ssid);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_CONN_FAILED);
    }

    bk_ret = bk_wifi_sta_start();
    if (bk_ret != BK_OK) {
        bk_printf("connect sta start fail bk_ret=%d ssid=%s\r\n", bk_ret, ssid);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_CONN_FAILED);
    }

    s_wifi.sta_started = TRUE;
    s_wifi.sta_status = WSS_CONNECTING;
    s_wifi.sta_dhcp_enabled = TRUE;
    if (s_wifi.work_mode != WWM_STATIONAP) {
        s_wifi.work_mode = WWM_STATION;
    }
    bk_printf("connect started ssid=%s\r\n", ssid);
    return OPRT_OK;
}

/**
 * @brief Disconnect STA from the current AP.
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_station_disconnect(VOID_T)
{
    bk_err_t bk_ret = BK_OK;

    bk_printf("disconnect enter sta_started=%d status=%d\r\n", s_wifi.sta_started, s_wifi.sta_status);

    if (s_wifi.inited == FALSE) {
        bk_printf("disconnect rejected: wifi not init\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    bk_ret = bk_wifi_sta_stop();
    if (bk_ret != BK_OK) {
        bk_printf("disconnect fail bk_ret=%d\r\n", bk_ret);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_DISCONN_FAILED);
    }

    s_wifi.sta_started = FALSE;
    s_wifi.work_mode = s_wifi.ap_started ? WWM_SOFTAP : WWM_POWERDOWN;

    bk_printf("disconnect issued new_mode=%s(%d)\r\n",
              __wifi_work_mode_name(s_wifi.work_mode), s_wifi.work_mode);
    return OPRT_OK;
}

/**
 * @brief Get the RSSI of the connected STA uplink.
 * @param[out] rssi RSSI output
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_station_get_conn_ap_rssi(SCHAR_T *rssi)
{
    wifi_link_status_t link_status;
    bk_err_t bk_ret = BK_OK;

    if (rssi == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    memset(&link_status, 0, sizeof(link_status));
    bk_ret = bk_wifi_sta_get_link_status(&link_status);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_RSSI_GET_FAILED);
    }

    if ((link_status.state != WIFI_LINKSTATE_STA_CONNECTED) &&
        (link_status.state != WIFI_LINKSTATE_STA_GOT_IP)) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *rssi = (SCHAR_T)link_status.rssi;
    return OPRT_OK;
}

/**
 * @brief Get the SNR of the connected STA uplink.
 * @param[out] snr SNR output
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_station_get_conn_ap_snr(SCHAR_T *snr)
{
    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief Get the current station status.
 * @param[out] stat station status output
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_station_get_status(WF_STATION_STAT_E *stat)
{
    wifi_linkstate_reason_t link_info;
    bk_err_t bk_ret = BK_OK;

    if (stat == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    memset(&link_info, 0, sizeof(link_info));
    bk_ret = bk_wifi_sta_get_linkstate_with_reason(&link_info);
    if (bk_ret != BK_OK) {
        *stat = s_wifi.sta_status;
        bk_printf("get_status get linkstate fail bk_ret=%d cached_status=%d\r\n", bk_ret, s_wifi.sta_status);
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_STAT_GET_FAILED);
    }

    s_wifi.sta_status = __wifi_link_to_status(&link_info);
    s_wifi.last_disc_reason = __wifi_reason_bk_to_tuya(link_info.reason_code);
    *stat = s_wifi.sta_status;
    return OPRT_OK;
}

/**
 * @brief Send a raw management frame.
 * @param[in] buf frame buffer
 * @param[in] len frame length
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_send_mgnt(CONST UCHAR_T *buf, CONST UINT_T len)
{
    bk_err_t bk_ret = BK_OK;

    if ((buf == NULL) || (len == 0)) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    bk_ret = bk_wifi_send_raw((UINT8_T *)buf, (INT_T)len);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_MGNT_SEND_FAILED);
    }

    return OPRT_OK;
}

/**
 * @brief Enable or disable management frame receive callback.
 * @param[in] enable enable flag
 * @param[in] recv_cb callback when enabling
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_register_recv_mgnt_callback(CONST BOOL_T enable, CONST WIFI_REV_MGNT_CB recv_cb)
{
    wifi_filter_config_t filter_cfg;
    bk_err_t bk_ret = BK_OK;

    if ((enable == TRUE) && (recv_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    memset(&filter_cfg, 0, sizeof(filter_cfg));
    if (enable == TRUE) {
        filter_cfg.rx_all_default_mgmt = 1;
        filter_cfg.rx_probe_req = 1;
        filter_cfg.rx_probe_rsp = 1;
        filter_cfg.rx_all_beacon = 1;
        filter_cfg.rx_action = 1;

        // bk_ret = bk_wifi_filter_set_config(&filter_cfg);
        // if (bk_ret != BK_OK) {
        //     return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_MGNT_REG_FAILED);
        // }

        bk_ret = bk_wifi_filter_register_cb(__wifi_filter_cb);
        if (bk_ret != BK_OK) {
            return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_MGNT_REG_FAILED);
        }

        s_wifi.mgnt_cb = recv_cb;
        return OPRT_OK;
    }

    bk_ret = bk_wifi_filter_register_cb(NULL);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_MGNT_REG_FAILED);
    }

    // bk_ret = bk_wifi_filter_set_config(&filter_cfg);
    // if (bk_ret != BK_OK) {
    //     return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_MGNT_REG_FAILED);
    // }

    s_wifi.mgnt_cb = NULL;
    return OPRT_OK;
}

/**
 * @brief Execute Wi-Fi ioctl subcommands supported by this adapter.
 * @param[in] cmd ioctl command
 * @param[in] args command arguments
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_ioctl(WF_IOCTL_CMD_E cmd, VOID *args)
{
    switch (cmd) {
    case WFI_GET_LAST_DISCONN_REASON:
        if (args == NULL) {
            return OPRT_INVALID_PARM;
        }
        *(WF_DISCONN_REASON_E *)args = s_wifi.last_disc_reason;
        return OPRT_OK;
    case WFI_CONNECT_CMD:
        if (args == NULL) {
            return OPRT_INVALID_PARM;
        }
        return tkl_wifi_station_connect((CONST SCHAR_T *)((WF_IOCTL_CONN_T *)args)->ssid,
                                        (CONST SCHAR_T *)((WF_IOCTL_CONN_T *)args)->passwd);
    case WFI_AP_GET_STALIST_CMD:
    case WFI_BEACON_CMD:
    default:
        return OPRT_NOT_SUPPORTED;
    }
}

/**
 * @brief Get the currently configured country code string.
 * @param[out] ccode two-character country code buffer
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_get_country_code(UCHAR_T *ccode)
{
    wifi_country_t country;
    bk_err_t bk_ret = BK_OK;

    if (ccode == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    memset(&country, 0, sizeof(country));
    bk_ret = bk_wifi_get_country(&country);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_CCODE_SET_FAILED);
    }

    ccode[0] = (UCHAR_T)country.cc[0];
    ccode[1] = (UCHAR_T)country.cc[1];
    ccode[2] = '\0';
    memcpy(s_wifi.country_code, country.cc, 2);
    s_wifi.country_code[2] = '\0';
    return OPRT_OK;
}

/**
 * @brief Set the Wi-Fi country code from a string.
 * @param[in] ccode two-character country code string
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_wifi_set_country_code_v2(CONST UCHAR_T *ccode)
{
    wifi_country_t country;
    bk_err_t bk_ret = BK_OK;
    OPERATE_RET op_ret = OPRT_OK;

    if (s_wifi.inited == FALSE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    op_ret = __wifi_country_from_str(ccode, &country);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    bk_ret = bk_wifi_set_country(&country);
    if (bk_ret != BK_OK) {
        return __wifi_map_ret(bk_ret, OPRT_OS_ADAPTER_CCODE_SET_FAILED);
    }

    memcpy(s_wifi.country_code, country.cc, 2);
    s_wifi.country_code[2] = '\0';
    return OPRT_OK;
}

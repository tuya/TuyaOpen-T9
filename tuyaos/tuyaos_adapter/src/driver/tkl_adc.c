/**
 * @file tkl_adc.c
 * @brief Tuya TKL ADC adapter for T9 SARADC.
 * @version 0.1
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */

#include <string.h>

#include "sdkconfig.h"
#include "tkl_adc.h"
#include "tuya_error_code.h"
#include <common/bk_err.h>
#include <driver/adc.h>
#include "saradc_client.h"
#include "sys_driver.h"

#define ADC_DEV_NUM             1
#define ADC_DEV_CHANNEL_SUM     ADC_MAX
#define ADC_BUF_SIZE            32
#define ADC_REF_VALUE_MV        3300
#define ADC_OPERATE_TIMEOUT_MS  1000

static UINT16 adc_buf[ADC_BUF_SIZE];
static UINT8_T g_adc_init[ADC_DEV_CHANNEL_SUM] = {0};
static UINT8_T g_channel_id_div[ADC_DEV_CHANNEL_SUM] = {
    0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};
static UINT8_T g_adc_current_ch_num = 0;
static adc_config_t g_config[ADC_DEV_CHANNEL_SUM] = {0};
static TUYA_ADC_NUM_E adc[ADC_DEV_NUM] = {TUYA_ADC_NUM_MAX};

extern void bk_printf(const char *fmt, ...);

int platform_adc_init(void)
{
    return (bk_saradc_driver_init() == BK_OK) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET bk_to_oprt(bk_err_t ret)
{
    return (ret == BK_OK) ? OPRT_OK : OPRT_COM_ERROR;
}

static BOOL_T tkl_adc_port_is_valid(TUYA_ADC_NUM_E port_num)
{
    return (port_num < ADC_DEV_NUM);
}

static BOOL_T tkl_adc_port_is_open(TUYA_ADC_NUM_E port_num)
{
    return (tkl_adc_port_is_valid(port_num) && (adc[port_num] == port_num));
}

static adc_mode_t tuya_to_bk_adc_mode(TUYA_ADC_MODE_E adc_mode)
{
    switch (adc_mode) {
    case TUYA_ADC_SINGLE:
        return ADC_SINGLE_STEP_MODE;
    case TUYA_ADC_CONTINUOUS:
    case TUYA_ADC_SCAN:
        return ADC_CONTINUOUS_MODE;
    default:
        return ADC_CONTINUOUS_MODE;
    }
}

static void tkl_adc_reset_state(TUYA_ADC_NUM_E port_num)
{
    memset(g_adc_init, 0, sizeof(g_adc_init));
    memset(g_config, 0, sizeof(g_config));
    memset(g_channel_id_div, 0, sizeof(g_channel_id_div));

    g_channel_id_div[ADC_4] = 1;
    g_adc_current_ch_num = 0;

    if (tkl_adc_port_is_valid(port_num)) {
        adc[port_num] = TUYA_ADC_NUM_MAX;
    }
}

static void tkl_adc_clear_config_state(void)
{
    memset(g_adc_init, 0, sizeof(g_adc_init));
    memset(g_config, 0, sizeof(g_config));
    g_adc_current_ch_num = 0;
}

static void tkl_adc_fill_config(adc_config_t *config, UINT8_T ch_id,
                                TUYA_ADC_BASE_CFG_T *cfg)
{
    memset(config, 0, sizeof(*config));

    config->chan = (adc_chan_t)ch_id;
    config->adc_mode = tuya_to_bk_adc_mode(cfg->mode);
    config->src_clk = ADC_SCLK_XTAL;
    config->clk = 0x30A0C5;
    config->saturate_mode = ADC_SATURATE_MODE_3;
    config->steady_ctrl = 7;
    config->adc_filter = 0;
    config->sample_rate = (config->adc_mode == ADC_CONTINUOUS_MODE) ? 0 : 0x20;
    config->is_open = FALSE;
    config->output_buf = adc_buf;
    config->output_buf_len = cfg->conv_cnt;
    config->vol_div = g_channel_id_div[ch_id] ? ADC_VOL_DIV_1 : ADC_VOL_DIV_NONE;
}

static OPERATE_RET tkl_adc_start_channel(adc_config_t *config, BOOL_T *gpio_inited,
                                         BOOL_T *adc_inited, BOOL_T *started)
{
    bk_err_t ret;

    *gpio_inited = FALSE;
    *adc_inited = FALSE;
    *started = FALSE;

    ret = bk_adc_acquire();
    if (ret != BK_OK) {
        return bk_to_oprt(ret);
    }

    sys_drv_set_ana_pwd_gadc_buf(1);

    ret = bk_adc_chan_init_gpio(config->chan);
    if (ret != BK_OK) {
        goto err_exit;
    }
    *gpio_inited = TRUE;

    ret = bk_adc_init(config->chan);
    if (ret != BK_OK) {
        goto err_exit;
    }
    *adc_inited = TRUE;

    ret = bk_adc_set_config(config);
    if (ret != BK_OK) {
        goto err_exit;
    }

    ret = bk_adc_enable_bypass_clalibration();
    if (ret != BK_OK) {
        goto err_exit;
    }

    ret = bk_adc_start();
    if (ret != BK_OK) {
        goto err_exit;
    }
    *started = TRUE;

    ret = bk_adc_set_channel(config->chan);
    if (ret != BK_OK) {
        goto err_exit;
    }

    return OPRT_OK;

err_exit:
    if (*started) {
        bk_adc_stop();
        *started = FALSE;
    }

    if (*adc_inited) {
        bk_adc_deinit(config->chan);
        *adc_inited = FALSE;
    }

    if (*gpio_inited) {
        bk_adc_chan_deinit_gpio(config->chan);
        *gpio_inited = FALSE;
    }

    sys_drv_set_ana_pwd_gadc_buf(0);
    bk_adc_release();

    return bk_to_oprt(ret);
}

static void tkl_adc_stop_channel(adc_config_t *config, BOOL_T gpio_inited,
                                 BOOL_T adc_inited, BOOL_T started)
{
    if (started) {
        bk_adc_stop();
    }

    if (adc_inited) {
        bk_adc_deinit(config->chan);
    }

    if (gpio_inited) {
        bk_adc_chan_deinit_gpio(config->chan);
    }

    sys_drv_set_ana_pwd_gadc_buf(0);
    bk_adc_release();
}

static OPERATE_RET tkl_adc_read_raw_config(adc_config_t *config, INT32_T *data)
{
    BOOL_T gpio_inited;
    BOOL_T adc_inited;
    BOOL_T started;
    OPERATE_RET ret;

    ret = tkl_adc_start_channel(config, &gpio_inited, &adc_inited, &started);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = bk_to_oprt(bk_adc_read_raw(config->output_buf, config->output_buf_len,
                                     ADC_OPERATE_TIMEOUT_MS));
    if (ret == OPRT_OK) {
        for (INT32_T i = 0; i < config->output_buf_len; i++) {
            data[i] = (INT32_T)config->output_buf[i];
        }
    }

    tkl_adc_stop_channel(config, gpio_inited, adc_inited, started);

    return ret;
}

static OPERATE_RET tkl_adc_read_voltage_config(adc_config_t *config, INT32_T *data)
{
    BOOL_T gpio_inited;
    BOOL_T adc_inited;
    BOOL_T started;
    OPERATE_RET ret;
    UINT16 value = 0;

    ret = tkl_adc_start_channel(config, &gpio_inited, &adc_inited, &started);
    if (ret != OPRT_OK) {
        return ret;
    }

    for (INT32_T i = 0; i < config->output_buf_len; i++) {
        ret = bk_to_oprt(bk_adc_read(&value, ADC_OPERATE_TIMEOUT_MS));
        if (ret != OPRT_OK) {
            break;
        }

        data[i] = (INT32_T)bk_adc_data_calculate(value, config->chan);
    }

    tkl_adc_stop_channel(config, gpio_inited, adc_inited, started);

    return ret;
}

/**
 * @brief tuya kernel adc init
 *
 * @param[in] port_num: adc port number
 * @param[in] cfg: adc config
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_adc_init(TUYA_ADC_NUM_E port_num, TUYA_ADC_BASE_CFG_T *cfg)
{
    UINT8_T cnt = 0;

    if ((cfg == NULL) || !tkl_adc_port_is_valid(port_num) ||
        (cfg->ch_nums == 0) || (cfg->ch_nums > ADC_DEV_CHANNEL_SUM) ||
        (cfg->conv_cnt == 0) || (cfg->conv_cnt > ADC_BUF_SIZE)) {
        return OPRT_INVALID_PARM;
    }

    tkl_adc_clear_config_state();

    for (UINT8_T i = 0; i < ADC_DEV_CHANNEL_SUM; i++) {
        if (cfg->ch_list.data & (1U << i)) {
            if (cnt >= cfg->ch_nums) {
                tkl_adc_clear_config_state();
                return OPRT_INVALID_PARM;
            }

            tkl_adc_fill_config(&g_config[cnt], i, cfg);
            g_adc_init[cnt] = TRUE;
            cnt++;
        }
    }

    if (cnt != cfg->ch_nums) {
        tkl_adc_reset_state(port_num);
        return OPRT_INVALID_PARM;
    }

    if (bk_saradc_driver_init() != BK_OK) {
        tkl_adc_reset_state(port_num);
        return OPRT_COM_ERROR;
    }

    adc[port_num] = port_num;
    g_adc_current_ch_num = cnt;

    return OPRT_OK;
}

/**
 * @brief adc deinit
 *
 * @param[in] port_num: adc port number
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_adc_deinit(TUYA_ADC_NUM_E port_num)
{
    if (!tkl_adc_port_is_valid(port_num)) {
        return OPRT_INVALID_PARM;
    }

    tkl_adc_reset_state(port_num);

    return OPRT_OK;
}

/**
 * @brief get adc width
 *
 * @param[in] port_num: adc port number
 *
 * @return adc width
 */
UINT8_T tkl_adc_width_get(TUYA_ADC_NUM_E port_num)
{
    return tkl_adc_port_is_valid(port_num) ? 12 : 0;
}

/**
 * @brief get adc reference voltage
 *
 * @param[in] port_num: adc port number
 *
 * @return adc reference voltage(bat: mv)
 */
UINT32_T tkl_adc_ref_voltage_get(TUYA_ADC_NUM_E port_num)
{
    return tkl_adc_port_is_valid(port_num) ? ADC_REF_VALUE_MV : 0;
}

/**
 * @brief adc get temperature
 *
 * @return temperature(bat: 'C)
 */
INT32_T tkl_adc_temperature_get(VOID_T)
{
    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief adc read
 *
 * @param[in] port_num: adc port number
 * @param[out] buff: points to the list of data read from the ADC register
 * @param[in] len:  buff len
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_adc_read_data(TUYA_ADC_NUM_E port_num, INT32_T *buff, UINT16_T len)
{
    UINT16_T read_len = 0;
    OPERATE_RET ret = OPRT_OK;

    if ((buff == NULL) || !tkl_adc_port_is_open(port_num)) {
        return OPRT_INVALID_PARM;
    }

    for (UINT8_T i = 0; i < g_adc_current_ch_num; i++) {
        if (g_adc_init[i] == TRUE) {
            read_len += g_config[i].output_buf_len;
        }
    }

    if (read_len > len) {
        bk_printf("adc buff len err:%d need:%d\r\n", len, read_len);
        return OPRT_INVALID_PARM;
    }

    read_len = 0;
    for (UINT8_T i = 0; i < g_adc_current_ch_num; i++) {
        if (g_adc_init[i] == TRUE) {
            ret = tkl_adc_read_raw_config(&g_config[i], &buff[read_len]);
            if (ret != OPRT_OK) {
                return ret;
            }

            read_len += g_config[i].output_buf_len;
        }
    }

    return ret;
}

/**
 * @brief read single channel
 *
 * @param[in] port_num: adc port number
 * @param[in] ch_id: channel id in one adc unit
 * @param[out] data: convert result buffer
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_adc_read_single_channel(TUYA_ADC_NUM_E port_num, UINT8_T ch_id, INT32_T *data)
{
    if ((data == NULL) || !tkl_adc_port_is_open(port_num) ||
        (ch_id >= ADC_DEV_CHANNEL_SUM)) {
        return OPRT_INVALID_PARM;
    }

    for (UINT8_T i = 0; i < g_adc_current_ch_num; i++) {
        if ((g_adc_init[i] == TRUE) && (g_config[i].chan == ch_id)) {
            adc_config_t single = g_config[i];   /* local copy: read exactly one sample */
            single.output_buf_len = 1;
            return tkl_adc_read_raw_config(&single, data);
        }
    }

    return OPRT_INVALID_PARM;
}

/**
 * @brief read voltage
 *
 * @param[in] port_num: adc port number
 * @param[out] buff: points to the list of calculate voltage,bat : mv
 * @param[in] len:  buff len
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_adc_read_voltage(TUYA_ADC_NUM_E port_num, INT32_T *buff, UINT16_T len)
{
    UINT16_T read_len = 0;
    OPERATE_RET ret = OPRT_OK;

    if ((buff == NULL) || !tkl_adc_port_is_open(port_num)) {
        return OPRT_INVALID_PARM;
    }

    for (UINT8_T i = 0; i < g_adc_current_ch_num; i++) {
        if (g_adc_init[i] == TRUE) {
            read_len += g_config[i].output_buf_len;
        }
    }

    if (read_len > len) {
        bk_printf("adc voltage buff len err:%d need:%d\r\n", len, read_len);
        return OPRT_INVALID_PARM;
    }

    read_len = 0;
    for (UINT8_T i = 0; i < g_adc_current_ch_num; i++) {
        if (g_adc_init[i] == TRUE) {
            ret = tkl_adc_read_voltage_config(&g_config[i], &buff[read_len]);
            if (ret != OPRT_OK) {
                return ret;
            }

            read_len += g_config[i].output_buf_len;
        }
    }

    return ret;
}

OPERATE_RET tkl_adc_ioctl(ADC_IOCTL_CMD_E cmd, VOID *args)
{
    ADC_IOCTL_DIV_RESIS_T *div_cfg;
    UINT8_T ch_id;
    UINT8_T div;

    if (args == NULL) {
        return OPRT_INVALID_PARM;
    }

    div_cfg = (ADC_IOCTL_DIV_RESIS_T *)args;
    ch_id = div_cfg->channel;
    if (!tkl_adc_port_is_valid(div_cfg->port) ||
        (ch_id >= ADC_DEV_CHANNEL_SUM)) {
        return OPRT_INVALID_PARM;
    }

    switch (cmd) {
    case ADC_DIV_RESIS_CLOSE:
        div = 1;
        break;
    case ADC_DIV_RESIS_OPEN:
        div = 0;
        break;
    default:
        return OPRT_NOT_SUPPORTED;
    }

    g_channel_id_div[ch_id] = div;

    /* Apply to an already-configured channel so subsequent voltage reads take
     * effect (vol_div is otherwise only baked into g_config at init time). */
    for (UINT8_T i = 0; i < g_adc_current_ch_num; i++) {
        if ((g_adc_init[i] == TRUE) && (g_config[i].chan == ch_id)) {
            g_config[i].vol_div = div ? ADC_VOL_DIV_1 : ADC_VOL_DIV_NONE;
        }
    }

    return OPRT_OK;
}

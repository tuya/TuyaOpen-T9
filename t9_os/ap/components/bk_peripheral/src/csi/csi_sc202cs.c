#include <os/os.h>
#include <os/mem.h>
#include "csi_calib_mem.h"
#include "csi_sensor_devices.h"
#include <driver/mipi_csi.h>
#include <components/bk_camera_sensor.h>
#include <avdk_check.h>

#include <driver/mipi_csi.h>
#include <vsi_list.h>
#include <driver/isp_types.h>
#include <components/bk_isp_camera_types.h>
#include <common/avdk_pixel_types.h>

#include <driver/gpio.h>
#include <driver/gpio_types.h>
#include "gpio_driver.h"

#include "vsios_i2c.h"
#include "vsi_comm_video.h"
#include "vsi_comm_isp.h"
#include "vsi_comm_sns.h"
#include "mpi_isp_calib.h"

/*
 * SC202CS (SmartSens SC2356 die) 1600x1200@30fps MIPI RAW10 sensor.
 *
 * NOTE: no tuned SC202CS ISP calibration table exists yet; the values in
 * sc202cs_1600x1200_calib.h are copied from the SC3336 table (same vendor
 * family) as the bring-up default. Replace them with tuned SC202CS values
 * when available.
 */
#include "sc202cs_1600x1200_calib.h"

#define TAG "sc202cs"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)
#define LOGD(...) BK_LOGD(TAG, ##__VA_ARGS__)

#define sc202cs_WRITE_ADDRESS (0x6C)  /* 7-bit 0x36, SID low (default) */
#define sc202cs_READ_ADDRESS  (0x6D)

/* SC202CS SID strap picks one of two 7-bit slave addresses (datasheet):
 * SID low/floating -> 0x36 (write 0x6C), SID high -> 0x10 (write 0x20).
 * The module strap is unverified, so detect probes both and remembers
 * whichever one ACKs; the working address is reused by init/regs later.
 * 0xD8 = 7-bit 0x6C written as raw byte: guards against datasheets/tools
 * that quote the write byte "0x6C" as a 7-bit value instead of 8-bit. */
static const uint8_t sc202cs_addr_candidates[] = {sc202cs_WRITE_ADDRESS, 0x20, 0xD8};
static uint8_t sc202cs_work_addr = sc202cs_WRITE_ADDRESS;
#define CHIP_ID_ADDR_HB (0x3107)
#define CHIP_ID_ADDR_LB (0x3108)

/* SC202CS real PID, read out on the user's module (regs 0x3107/0x3108):
 * 0xEB52. Datasheet flag 0x36ff==0x00 (DemoSens) confirmed at the same time. */
#define CHIP_ID_VAL_HB (0xEB)
#define CHIP_ID_VAL_LB (0x52)

#define WIN_MAX_X 1600
#define WIN_MAX_Y 1200

#define UINT16_HB(x) (((x) >> 8) & 0xFF)
#define UINT16_LB(x) ((x) & 0xFF)

#define sc202cs_REG_BYTE_NUM  2   /* 16-bit register address */
#define sc202cs_DATA_BYTE_NUM 1   /* 8-bit register data */

#define sc202cs_REG_EXPOSURE_H 0x3e00
#define sc202cs_REG_EXPOSURE_M 0x3e01
#define sc202cs_REG_EXPOSURE_L 0x3e02
#define sc202cs_REG_VTS_H 0x320e
#define sc202cs_REG_VTS_L 0x320f
#define SC202CS_REG_MIRROR_FLIP 0x3221
#define SC202CS_MIRROR_BITS     ((1U << 1) | (1U << 2))
#define SC202CS_VFLIP_BITS      ((1U << 5) | (1U << 6))

/* Vendor FT INI: 24MHz in, 720Mbps 1-lane 10bit, 1600x1200@20fps base
 * (pixel rate 72Mpix/s, line time 26.67us): VTS 0x320e/0x320f = 0x0753 = 1875 */
#define SC202CS_FPS_BASE              20
#define SC202CS_FPS_MIN               5
#define SC202CS_VTS_1600x1200_20FPS   1875

#define SC202CS_1600x1200_LINEAR_MODE (0)

enum sc202cs_REG_INDEX {
    REG_VTS_H       = 0,
    REG_VTS_L       = 1,
    REG_EXPOSURE_H  = 2,
    REG_EXPOSURE_M  = 3,
    REG_EXPOSURE_L  = 4,

    REG_GAIN_3E06    = 5,
    REG_GAIN_3E07    = 6,
    REG_GAIN_3E09    = 7,
};

typedef struct vsisc202cs_DEVICE_S {
    vsi_u8_t i2cBus;
    vsios_i2c_attr_t i2cAttr;
    ISP_SNS_MODE_S snsMode;
    vsi_u8_t snsModeId;
    vsi_bool_t stream;
    AE_SNS_DEFAULT_S aeDefault;
    ISP_SNS_REGS_INFO_S snsRegsInfo;
} sc202cs_DEVICE_S;

typedef struct
{
    ISP_SNS_OBJ_S sensor_object;
    uint16_t base_win_x;
    uint16_t base_win_y;
    uint32_t init_exposure;
    bool init_exposure_valid;
} sc202cs_private_data_t;

static sc202cs_DEVICE_S *sc202csDev[ISP_DEV_CNT][ISP_PORT_CNT] = {0};

static ISP_CALIB_DATA_S *SC202CS_CalibParam_dynamic = NULL;

/*
 * Vendor init table (FT INI: 24MHz in, 720Mbps 1-lane MIPI RAW10 1600x1200@20fps,
 * 16-bit reg address / 8-bit data). Stream-on is issued separately via
 * sc202cs_SetStream(), followed by a 10ms delay as in the vendor sequence.
 */
static const uint16_t sensor_sc202cs_init_table[][2] = {
    {0x0103,0x01},
    {0x0100,0x00},
    {0x36e9,0x80},
    {0x36e9,0x24},
    {0x301f,0x01},
    {0x320e,0x07},
    {0x320f,0x53},
    {0x3301,0xff},
    {0x3304,0x68},
    {0x3306,0x40},
    {0x3308,0x08},
    {0x3309,0xa8},
    {0x330b,0xb0},
    {0x330c,0x18},
    {0x330d,0xff},
    {0x330e,0x20},
    {0x331e,0x59},
    {0x331f,0x99},
    {0x3333,0x10},
    {0x335e,0x06},
    {0x335f,0x08},
    {0x3364,0x1f},
    {0x337c,0x02},
    {0x337d,0x0a},
    {0x338f,0xa0},
    {0x3390,0x01},
    {0x3391,0x03},
    {0x3392,0x1f},
    {0x3393,0xff},
    {0x3394,0xff},
    {0x3395,0xff},
    {0x33a2,0x04},
    {0x33ad,0x0c},
    {0x33b1,0x20},
    {0x33b3,0x38},
    {0x33f9,0x40},
    {0x33fb,0x48},
    {0x33fc,0x0f},
    {0x33fd,0x1f},
    {0x349f,0x03},
    {0x34a6,0x03},
    {0x34a7,0x1f},
    {0x34a8,0x38},
    {0x34a9,0x30},
    {0x34ab,0xb0},
    {0x34ad,0xb0},
    {0x34f8,0x1f},
    {0x34f9,0x20},
    {0x3630,0xa0},
    {0x3631,0x92},
    {0x3632,0x64},
    {0x3633,0x43},
    {0x3637,0x49},
    {0x363a,0x85},
    {0x363c,0x0f},
    {0x3650,0x31},
    {0x3670,0x0d},
    {0x3674,0xc0},
    {0x3675,0xa0},
    {0x3676,0xa0},
    {0x3677,0x92},
    {0x3678,0x96},
    {0x3679,0x9a},
    {0x367c,0x03},
    {0x367d,0x0f},
    {0x367e,0x01},
    {0x367f,0x0f},
    {0x3698,0x83},
    {0x3699,0x86},
    {0x369a,0x8c},
    {0x369b,0x94},
    {0x36a2,0x01},
    {0x36a3,0x03},
    {0x36a4,0x07},
    {0x36ae,0x0f},
    {0x36af,0x1f},
    {0x36bd,0x22},
    {0x36be,0x22},
    {0x36bf,0x22},
    {0x36d0,0x01},
    {0x370f,0x02},
    {0x3721,0x6c},
    {0x3722,0x8d},
    {0x3725,0xc5},
    {0x3727,0x14},
    {0x3728,0x04},
    {0x37b7,0x04},
    {0x37b8,0x04},
    {0x37b9,0x06},
    {0x37bd,0x07},
    {0x37be,0x0f},
    {0x3901,0x02},
    {0x3903,0x40},
    {0x3905,0x8d},
    {0x3907,0x00},
    {0x3908,0x41},
    {0x391f,0x41},
    {0x3933,0x80},
    {0x3934,0x02},
    {0x3937,0x6f},
    {0x393a,0x01},
    {0x393d,0x01},
    {0x393e,0xc0},
    {0x39dd,0x41},
    {0x3e00,0x00},
    {0x3e01,0x4d},
    {0x3e02,0xc0},
    {0x3e03,0x0b},   /* datasheet 2.2.3: 0x3e03[3:0]=0xb for manual AEC/AGC control */
    {0x3e06,0x00},   /* DIG GAIN coarse: 0x00 = 1x */
    {0x3e07,0x80},   /* DIG FINE GAIN: 0x80 = 1.0x (reset value 0x40 = 0.5x, must be fixed) */
    {0x3e09,0x00},   /* ANA GAIN: 0x00 = 1x */
    {0x4509,0x28},
    {0x450d,0x61},
    // {0x0100,0x01},  /* stream on + 10ms delay, see sc202cs_SetStream() */
};

#define sc202cs_TABLE_SIZE(table) (sizeof(table) / sizeof(table[0]))

static int sc202cs_SetStream(ISP_PORT IspPort, vsi_bool_t stream);

static sc202cs_DEVICE_S *sc202cs_GetSensorDev(ISP_PORT IspPort)
{
    if (sc202csDev[IspPort.devId][IspPort.portId] == NULL)
    {
        sc202csDev[IspPort.devId][IspPort.portId] = os_malloc(sizeof(sc202cs_DEVICE_S));
        if (sc202csDev[IspPort.devId][IspPort.portId] == NULL)
        {
            LOGE("%s %d sc202csDev[%d][%d] malloc failed \r\n", __func__, __LINE__, IspPort.devId, IspPort.portId);
            return NULL;
        }
        os_memset(sc202csDev[IspPort.devId][IspPort.portId], 0 , sizeof(sc202cs_DEVICE_S));
    }

    return sc202csDev[IspPort.devId][IspPort.portId];
}

static int sc202cs_InitRegInfo(ISP_PORT IspPort)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }
    ISP_SNS_REGS_INFO_S *pSnsRegsInfo = &psc202csDev->snsRegsInfo;
    pSnsRegsInfo->snsDev = psc202csDev->i2cBus;

    pSnsRegsInfo->addrByteNum = psc202csDev->i2cAttr.reg_bytes;
    pSnsRegsInfo->dataByteNum = psc202csDev->i2cAttr.data_bytes;
    pSnsRegsInfo->slaveAddr   = psc202csDev->i2cAttr.slave_addr;
    pSnsRegsInfo->regCnt = 8;
    pSnsRegsInfo->delayMax = 2;

    pSnsRegsInfo->snsData[REG_VTS_H].delayFrameNum = 2;
    pSnsRegsInfo->snsData[REG_VTS_H].regAddr = sc202cs_REG_VTS_H;
    pSnsRegsInfo->snsData[REG_VTS_L].delayFrameNum = 2;
    pSnsRegsInfo->snsData[REG_VTS_L].regAddr = sc202cs_REG_VTS_L;

    pSnsRegsInfo->snsData[REG_EXPOSURE_H].delayFrameNum = 2;
    pSnsRegsInfo->snsData[REG_EXPOSURE_H].regAddr = sc202cs_REG_EXPOSURE_H;
    pSnsRegsInfo->snsData[REG_EXPOSURE_H].data = 0x00;   /* init table: 0x3e00=0x00 */
    pSnsRegsInfo->snsData[REG_EXPOSURE_M].delayFrameNum = 2;
    pSnsRegsInfo->snsData[REG_EXPOSURE_M].regAddr = sc202cs_REG_EXPOSURE_M;
    pSnsRegsInfo->snsData[REG_EXPOSURE_M].data = 0x4d;   /* init table: 0x3e01=0x4d */
    pSnsRegsInfo->snsData[REG_EXPOSURE_L].delayFrameNum = 2;
    pSnsRegsInfo->snsData[REG_EXPOSURE_L].regAddr = sc202cs_REG_EXPOSURE_L;
    pSnsRegsInfo->snsData[REG_EXPOSURE_L].data = 0xc0;   /* init table: 0x3e02=0xc0 */

    pSnsRegsInfo->snsData[REG_GAIN_3E06].delayFrameNum = 2;
    pSnsRegsInfo->snsData[REG_GAIN_3E06].regAddr = 0x3e06;
    pSnsRegsInfo->snsData[REG_GAIN_3E06].data = 0x00;   /* dig coarse 1x */
    pSnsRegsInfo->snsData[REG_GAIN_3E07].delayFrameNum = 2;
    pSnsRegsInfo->snsData[REG_GAIN_3E07].regAddr = 0x3e07;
    pSnsRegsInfo->snsData[REG_GAIN_3E07].data = 0x80;   /* dig fine 1.0x */
    pSnsRegsInfo->snsData[REG_GAIN_3E09].delayFrameNum = 2;
    pSnsRegsInfo->snsData[REG_GAIN_3E09].regAddr = 0x3e09;
    pSnsRegsInfo->snsData[REG_GAIN_3E09].data = 0x00;   /* ana gain 1x */

    return BK_OK;
}

static int sc202cs_SensorInit(ISP_PORT IspPort, vsi_u8_t snsDev)
{
    if (SC202CS_CalibParam_dynamic == NULL)
    {
        SC202CS_CalibParam_dynamic = CSI_CALIB_MALLOC(sizeof(SC202CS_1600x1200_CalibParam));
        if (SC202CS_CalibParam_dynamic == NULL)
        {
            LOGE("Failed to malloc SC202CS_CalibParam_dynamic\n");
            return BK_FAIL;
        }
        os_memcpy(SC202CS_CalibParam_dynamic, &SC202CS_1600x1200_CalibParam, sizeof(SC202CS_1600x1200_CalibParam));
    }

    sc202cs_DEVICE_S *psc202csDev = sc202cs_GetSensorDev(IspPort);
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }
    int ret;

    os_memset(psc202csDev, 0, sizeof(*psc202csDev));
    psc202csDev->i2cBus              = snsDev;
    psc202csDev->i2cAttr.slave_addr  = sc202cs_work_addr;
    psc202csDev->i2cAttr.reg_bytes   = sc202cs_REG_BYTE_NUM;
    psc202csDev->i2cAttr.data_bytes  = sc202cs_DATA_BYTE_NUM;
    sc202cs_InitRegInfo(IspPort);

    ret = vsios_i2c_sys_init(snsDev);
    if (ret) {
        LOGE("Failed to i2c init %d\n", snsDev);
        return BK_FAIL;
    }

    sc202cs_SetStream(IspPort, 0);

    return  BK_OK;
}

static int sc202cs_SensorExit(ISP_PORT IspPort)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }

    vsios_i2c_sys_exit(psc202csDev->i2cBus);

    if (SC202CS_CalibParam_dynamic != NULL)
    {
        CSI_CALIB_FREE(SC202CS_CalibParam_dynamic);
        SC202CS_CalibParam_dynamic = NULL;
    }
    return  BK_OK;
}

static int sc202cs_WriteReg(ISP_PORT IspPort, vsi_u32_t addr, vsi_u32_t data)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }
    vsi_u8_t i2cBus = psc202csDev->i2cBus;
    vsios_i2c_attr_t *pI2cAttr = &psc202csDev->i2cAttr;

    vsios_i2c_write(i2cBus, pI2cAttr, addr, data);

    return  BK_OK;
}

static int sc202cs_ReadReg(ISP_PORT IspPort, vsi_u32_t addr, vsi_u32_t *pData)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }
    vsi_u8_t i2cBus = psc202csDev->i2cBus;
    vsios_i2c_attr_t *pI2cAttr = &psc202csDev->i2cAttr;

    *pData = vsios_i2c_read(i2cBus, pI2cAttr, addr);

    return  BK_OK;
}

static int sc202cs_InitAeDefault(ISP_PORT IspPort)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }
    AE_SNS_DEFAULT_S *pAeSnsDft = &psc202csDev->aeDefault;
    ISP_SNS_REGS_INFO_S *pSnsRegsInfo = &psc202csDev->snsRegsInfo;
    vsi_u32_t fps = psc202csDev->snsMode.fps;
    vsi_u32_t vts;

    if (psc202csDev->snsModeId != SC202CS_1600x1200_LINEAR_MODE)
    {
        return BK_FAIL;
    }

    /* fps carried in ACCU units; single mode, VTS scales linearly with fps */
    if (fps == 0
        || fps < SC202CS_FPS_MIN * ISP_SNS_FPS_ACCU
        || fps > SC202CS_FPS_BASE * ISP_SNS_FPS_ACCU)
    {
        fps = SC202CS_FPS_BASE * ISP_SNS_FPS_ACCU;
    }

    vts = (vsi_u32_t)SC202CS_VTS_1600x1200_20FPS * SC202CS_FPS_BASE * ISP_SNS_FPS_ACCU / fps;

    pAeSnsDft->fullLinesStd = vts;
    pAeSnsDft->fullLines = pAeSnsDft->fullLinesStd;
    pAeSnsDft->fps = fps;
    pAeSnsDft->fullLinesMax = SC202CS_VTS_1600x1200_20FPS * SC202CS_FPS_BASE;
    pAeSnsDft->linesPer500ms =
        pAeSnsDft->fullLines * pAeSnsDft->fps / (2 * ISP_SNS_FPS_ACCU);

    pAeSnsDft->maxIntLine  = pAeSnsDft->fullLines - 8;
    pAeSnsDft->minIntLine  = 8;
    pAeSnsDft->intLineStep = 1;

    pAeSnsDft->maxAgain = 4096;    /* datasheet: 16x analog (0x3e09) * 4x digital (0x3e06*0x3e07) */
    pAeSnsDft->minAgain  = 64;
    pAeSnsDft->againStep = 1;

    pAeSnsDft->maxDgain  = 1024;
    pAeSnsDft->minDgain  = 1024;
    pAeSnsDft->dgainStep = 1;

    pAeSnsDft->aeTarget = 62;
    pAeSnsDft->dampOver = 0x40;
    pAeSnsDft->dampUnder = 0x40;
    pAeSnsDft->tolerance = 2;
    pAeSnsDft->initExposure = 0x08 * pAeSnsDft->minAgain;

    pSnsRegsInfo->snsData[REG_VTS_H].data = ((vts & 0xFF00) >> 8);
    pSnsRegsInfo->snsData[REG_VTS_L].data = (vts & 0xFF);

    sc202cs_private_data_t *private_data =
        (sc202cs_private_data_t *)VSI_ISP_SnsGetUserData(IspPort);
    if (private_data && private_data->init_exposure_valid) {
        uint64_t min_exposure =
            (uint64_t)pAeSnsDft->minIntLine *
            pAeSnsDft->minAgain *
            pAeSnsDft->minDgain / ISP_SNS_GAIN_ACCU;
        uint64_t max_exposure =
            (uint64_t)pAeSnsDft->maxIntLine *
            pAeSnsDft->maxAgain *
            pAeSnsDft->maxDgain / ISP_SNS_GAIN_ACCU;
        uint32_t init_exposure = private_data->init_exposure;

        if (init_exposure < min_exposure) {
            init_exposure = (uint32_t)min_exposure;
        } else if (init_exposure > max_exposure) {
            init_exposure = (uint32_t)max_exposure;
        }

        pAeSnsDft->initExposure = init_exposure;
        private_data->init_exposure_valid = false;
        LOGI("restore initExposure=%u range=[%u,%u]\n",
             (unsigned)init_exposure,
             (unsigned)min_exposure,
             (unsigned)max_exposure);
    }

    return BK_OK;
}

static int sc202cs_SetMode(ISP_PORT IspPort, ISP_SNS_MODE_S *pSnsMode)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }
    if ((pSnsMode->width == psc202csDev->snsMode.width) &&
        (pSnsMode->height == psc202csDev->snsMode.height) &&
        (pSnsMode->hdrMode == psc202csDev->snsMode.hdrMode) &&
        (pSnsMode->stichMode == psc202csDev->snsMode.stichMode) &&
        (pSnsMode->fps == psc202csDev->snsMode.fps)) {
        return BK_OK;
    }
    LOGI("sc202cs_SetMode: %d, %d, %d, %d, %d\n", pSnsMode->width, pSnsMode->height, pSnsMode->hdrMode, pSnsMode->stichMode, pSnsMode->fps/ISP_SNS_FPS_ACCU);

    if ((pSnsMode->width  == WIN_MAX_X) &&
        (pSnsMode->height == WIN_MAX_Y) &&
        (pSnsMode->hdrMode == HDR_MODE_LINEAR) &&
        (pSnsMode->fps >= SC202CS_FPS_MIN * ISP_SNS_FPS_ACCU) &&
        (pSnsMode->fps <= SC202CS_FPS_BASE * ISP_SNS_FPS_ACCU)) {
        os_memcpy(&psc202csDev->snsMode, pSnsMode, sizeof(*pSnsMode));
        psc202csDev->snsModeId = SC202CS_1600x1200_LINEAR_MODE;
        sc202cs_InitAeDefault(IspPort);
    } else {
        LOGE("sc202cs_SetMode: not supported width: %d, height: %d, fps: %d\n", pSnsMode->width, pSnsMode->height, pSnsMode->fps);
        return BK_FAIL;
    }

    return BK_OK;
}

static int sc202cs_SetStream(ISP_PORT IspPort, vsi_bool_t stream)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }

    if (stream) {
        int ret = sc202cs_WriteReg(IspPort, 0x0100, 0x01);
        LOGI("sc202cs_SetStream ON (0x0100=01) write ret=%d\n", ret);
        /* vendor sequence: delay 10ms after stream-on */
        rtos_delay_milliseconds(10);
        vsi_u32_t rb = 0xFFFF;
        int rb_ret = sc202cs_ReadReg(IspPort, 0x0100, &rb);
        LOGI("sc202cs_SetStream readback 0x0100=0x%02x (ret=%d)\n",
             (unsigned)rb, rb_ret);
    } else {
        int ret = sc202cs_WriteReg(IspPort, 0x0100, 0x00);
        LOGI("sc202cs_SetStream OFF (0x0100=00) write ret=%d\n", ret);
    }

    psc202csDev->stream = stream;

    return BK_OK;
}

static int sc202cs_SetIspDefault(ISP_PORT IspPort)
{
    VSI_MPI_ISP_SetCalib(IspPort, SC202CS_CalibParam_dynamic);

    return BK_OK;
}

static int sc202cs_GetAeDefault(ISP_PORT IspPort, AE_SNS_DEFAULT_S *pAeSnsDft)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }

    os_memcpy(pAeSnsDft, &psc202csDev->aeDefault, sizeof(*pAeSnsDft));

    return BK_OK;
}

static int sc202cs_SetFps(ISP_PORT IspPort, vsi_u32_t fps)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    AE_SNS_DEFAULT_S *pAeSnsDft = &psc202csDev->aeDefault;
    ISP_SNS_REGS_INFO_S *pSnsRegsInfo = &psc202csDev->snsRegsInfo;
    vsi_u32_t vts;

    if (psc202csDev->snsModeId != SC202CS_1600x1200_LINEAR_MODE)
    {
        return BK_FAIL;
    }

    if ((fps <= SC202CS_FPS_BASE * ISP_SNS_FPS_ACCU) &&
        (fps >= SC202CS_FPS_MIN * ISP_SNS_FPS_ACCU)) {
        vts = (vsi_u32_t)SC202CS_VTS_1600x1200_20FPS * SC202CS_FPS_BASE * ISP_SNS_FPS_ACCU / fps;
    } else {
        return BK_FAIL;
    }

    vts = (vts >= pAeSnsDft->fullLinesMax) ? pAeSnsDft->fullLinesMax : vts;
    pSnsRegsInfo->snsData[REG_VTS_H].data = ((vts & 0xFF00) >> 8);
    pSnsRegsInfo->snsData[REG_VTS_L].data = (vts & 0xFF);
    pAeSnsDft->fullLines = vts;
    pAeSnsDft->fps = fps;
    pAeSnsDft->maxIntLine = pAeSnsDft->fullLines - 8;
    LOGI("sc202cs_SetFps: fullLines: %d, fps: %d, maxIntLine: %d\n", pAeSnsDft->fullLines, pAeSnsDft->fps/ISP_SNS_FPS_ACCU, pAeSnsDft->maxIntLine);

    return BK_OK;
}

static int sc202cs_SlowFrameRate(ISP_PORT IspPort, vsi_u32_t fullLines)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    const vsi_u32_t fps_min = SC202CS_FPS_MIN * ISP_SNS_FPS_ACCU;
    const vsi_u32_t fps_max = SC202CS_FPS_BASE * ISP_SNS_FPS_ACCU;
    vsi_u32_t tempfps;

    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }

    AE_SNS_DEFAULT_S *pAeSnsDft = &psc202csDev->aeDefault;
    ISP_SNS_REGS_INFO_S *pSnsRegsInfo = &psc202csDev->snsRegsInfo;

    if (psc202csDev->snsModeId != SC202CS_1600x1200_LINEAR_MODE)
    {
        return BK_FAIL;
    }

    fullLines = (fullLines > pAeSnsDft->fullLinesMax) ? pAeSnsDft->fullLinesMax : fullLines;
    if (fullLines == 0)
    {
        LOGE("sc202cs_SlowFrameRate: fullLines is 0\n");
        return BK_FAIL;
    }

    /* fps = VTS_BASE * (20 * ACCU) / fullLines, keep fps in [5, 20] @ ACCU */
    tempfps = (vsi_u32_t)(((vsi_u64_t)fps_max * SC202CS_VTS_1600x1200_20FPS) / fullLines);
    tempfps = (tempfps/100) * 100;

    if (tempfps < fps_min)
    {
        tempfps = fps_min;
    }
    else if (tempfps > fps_max)
    {
        tempfps = fps_max;
    }

    fullLines = (vsi_u32_t)(((vsi_u64_t)SC202CS_VTS_1600x1200_20FPS * SC202CS_FPS_BASE * ISP_SNS_FPS_ACCU) / tempfps);

    pAeSnsDft->fullLines = fullLines;

    pSnsRegsInfo->snsData[REG_VTS_H].data = ((fullLines & 0xFF00) >> 8);
    pSnsRegsInfo->snsData[REG_VTS_L].data = (fullLines & 0xFF);

    pAeSnsDft->maxIntLine = fullLines - 8;
    pAeSnsDft->fps = tempfps;

    return VSI_SUCCESS;
}

static int sc202cs_GetSnsRegInfo(ISP_PORT IspPort, ISP_SNS_REGS_INFO_S *pSnsRegsInfo)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }

    os_memcpy(pSnsRegsInfo, &psc202csDev->snsRegsInfo, sizeof(*pSnsRegsInfo));

    return BK_OK;
}

static int sc202cs_IntTimeUpdate(ISP_PORT IspPort, vsi_u32_t *pIntLine)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }

    ISP_SNS_REGS_INFO_S *pSnsRegsInfo = &psc202csDev->snsRegsInfo;

    if (psc202csDev->snsModeId == SC202CS_1600x1200_LINEAR_MODE) {
        /* 0x3e00[3:0]:H, 0x3e01:M, 0x3e02[7:4]:L (same encoding as SC3336) */
        pSnsRegsInfo->snsData[REG_EXPOSURE_H].data = ((*pIntLine >> 12) & 0x0F);
        pSnsRegsInfo->snsData[REG_EXPOSURE_M].data = ((*pIntLine >> 4) & 0xFF);
        pSnsRegsInfo->snsData[REG_EXPOSURE_L].data = (*pIntLine & 0xF) << 4;
    }

    return BK_OK;
}

static void sc202cs_CalcGain(vsi_u32_t *pAgain, vsi_u32_t *gain_3e06, vsi_u32_t *gain_3e07, vsi_u32_t *gain_3e09)
{
    uint32_t total_gain = *pAgain;  /* in 1/64x units: 64 = 1x */
    uint32_t a_gain;                /* analog coarse step, 1/64x units */
    uint32_t d_gain;                /* digital base before fine, 1/64x units */
    uint32_t fine;                   /* digital fine gain, 256 = 1.0x */

    total_gain = total_gain < 0x40 ? 0x40 : total_gain;
    total_gain = total_gain > 0x1000 ? 0x1000 : total_gain;   /* datasheet: 16x ana * 4x dig max */

    /* ANA GAIN (0x3e09) is coarse only, table 2-4:
       1/2/4/8/16x -> 0x00/0x01/0x03/0x07/0x0f */
    if (total_gain < 0x80) {
        *gain_3e09 = 0x00;
        a_gain = 0x40;
    } else if (total_gain < 0x100) {
        *gain_3e09 = 0x01;
        a_gain = 0x80;
    } else if (total_gain < 0x200) {
        *gain_3e09 = 0x03;
        a_gain = 0x100;
    } else if (total_gain < 0x400) {
        *gain_3e09 = 0x07;
        a_gain = 0x200;
    } else {
        *gain_3e09 = 0x0f;
        a_gain = 0x400;
    }

    /* DIG GAIN (0x3e06) coarse 2x, engaged only when 16x analog + fine gain
       at its ~2x limit is still not enough */
    *gain_3e06 = 0x00;
    d_gain = a_gain;
    if (total_gain >= 0x7f8)
    {
        *gain_3e06 = 0x01;
        d_gain = a_gain * 2;
    }

    /* DIG FINE GAIN (0x3e07): 1.0~2.0x with 1/32x steps, table 2-5:
       reg = 0x80 + 128 * (fine - 1), i.e. 0x80~0xff */
    fine = (total_gain << 8) / d_gain;
    fine = fine < 0x100 ? 0x100 : fine;
    fine = fine > 0x1fe ? 0x1fe : fine;
    *gain_3e07 = 0x80 + ((fine - 0x100) >> 1);
}

static int sc202cs_GainUpdate(ISP_PORT IspPort, vsi_u32_t *pAgain, vsi_u32_t *pDgain)
{
    sc202cs_DEVICE_S *psc202csDev = sc202csDev[IspPort.devId][IspPort.portId];
    if (psc202csDev == NULL)
    {
        LOGE("%s %d failed\n", __func__, __LINE__);
        return BK_FAIL;
    }

    if (psc202csDev->snsModeId == SC202CS_1600x1200_LINEAR_MODE) {
        *pDgain = 1024;
        vsi_u32_t gain_3e06 = 0, gain_3e07 = 0, gain_3e09 = 0;
        sc202cs_CalcGain(pAgain, &gain_3e06, &gain_3e07, &gain_3e09);
        psc202csDev->snsRegsInfo.snsData[REG_GAIN_3E09].data = gain_3e09;
        psc202csDev->snsRegsInfo.snsData[REG_GAIN_3E06].data = gain_3e06;
        psc202csDev->snsRegsInfo.snsData[REG_GAIN_3E07].data = gain_3e07;
    }

    return BK_OK;
}

static int sc202cs_InitIspSnsFunc(ISP_SNS_FUNC_S *pIspSnsFunc)
{
    pIspSnsFunc->pfnSensorInit    = sc202cs_SensorInit;
    pIspSnsFunc->pfnSensorExit    = sc202cs_SensorExit;
    pIspSnsFunc->pfnWriteReg      = sc202cs_WriteReg;
    pIspSnsFunc->pfnReadReg       = sc202cs_ReadReg;
    pIspSnsFunc->pfnSetMode       = sc202cs_SetMode;
    pIspSnsFunc->pfnSetStream     = sc202cs_SetStream;
    pIspSnsFunc->pfnSetIspDefault = sc202cs_SetIspDefault;

    return AVDK_ERR_OK;
}

static int sc202cs_InitAeSnsFunc(AE_SNS_FUNC_S *pAeSnsFunc)
{
    pAeSnsFunc->pfnGetAeDefault  = sc202cs_GetAeDefault;
    pAeSnsFunc->pfnSetFps        = sc202cs_SetFps;
    pAeSnsFunc->pfnSlowFrameRate = sc202cs_SlowFrameRate;
    pAeSnsFunc->pfnIntTimeUpdate = sc202cs_IntTimeUpdate;
    pAeSnsFunc->pfnGainUpdate    = sc202cs_GainUpdate;
    pAeSnsFunc->pfnSetExpRatio   = NULL;
    pAeSnsFunc->pfnGetSnsRegInfo = sc202cs_GetSnsRegInfo;

    return AVDK_ERR_OK;
}

const ISP_SNS_OBJ_S snssc202csObj = {
    .pfnInitIspSnsFunc = sc202cs_InitIspSnsFunc,
    .pfnInitAeSnsFunc  = sc202cs_InitAeSnsFunc,
};

static const ISP_PUB_ATTR_S sc202cs_mipi_linear_attr = {
    .pSnsObj      = (void*)&snssc202csObj,
    .ispInputType = INPUT_TYPE_SENSOR,
    .ispMode      = ISP_MODE_RAW,
    .hdrMode      = HDR_MODE_LINEAR,
    .pixelFormat  = PIXEL_FORMAT_BGGR10,
    .snsFps       = 20 * ISP_SNS_FPS_ACCU,
};

static const bk_camera_sensor_format_t sc202cs_format_array[] = {
    {
        .width = 1600,
        .height = 1200,
        .fps = 20,
        .output_pixel_fmt = BK_PIXEL_FORMAT_BGGR10,
    },

    {
        .width = 1600,
        .height = 1200,
        .fps = 15,
        .output_pixel_fmt = BK_PIXEL_FORMAT_BGGR10,
    },

    {
        .width = 1600,
        .height = 1200,
        .fps = 10,
        .output_pixel_fmt = BK_PIXEL_FORMAT_BGGR10,
    },
};

static avdk_err_t sc202cs_init(bk_camera_sensor_ctlr_t *controller)
{
    bk_mipi_csi_ext_set_enable(0);

    bk_mipi_csi_phy_term_set(0x303, 0x808);

    return 0;
}

static sc202cs_private_data_t *sc202cs_get_private(
    bk_camera_csi_sensor_t *sensor)
{
    return (sc202cs_private_data_t *)sensor->data.private_data;
}

static void sc202cs_capture_window_base(bk_camera_csi_sensor_t *sensor)
{
    uint8_t hb = 0;
    uint8_t lb = 0;
    bk_camera_bus_t *bus = sensor->config.bus;
    sc202cs_private_data_t *private_data = sc202cs_get_private(sensor);

    bus->read16(bus, 0x3210, &hb);
    bus->read16(bus, 0x3211, &lb);
    private_data->base_win_x = (uint16_t)((hb << 8) | lb);
    bus->read16(bus, 0x3212, &hb);
    bus->read16(bus, 0x3213, &lb);
    private_data->base_win_y = (uint16_t)((hb << 8) | lb);
}

static void sc202cs_write_window_regs(bk_camera_csi_sensor_t *sensor)
{
    bk_camera_bus_t *bus = sensor->config.bus;
    sc202cs_private_data_t *private_data = sc202cs_get_private(sensor);
    uint16_t win_x = private_data->base_win_x;
    uint16_t win_y = private_data->base_win_y;

    /* Toggle window start by 1 pixel to preserve BGGR Bayer phase. */
    if (sensor->data.hmirror)
    {
        win_x ^= 1;
    }
    if (sensor->data.vflip)
    {
        win_y ^= 1;
    }

    bus->write16(bus, 0x3210, UINT16_HB(win_x));
    bus->write16(bus, 0x3211, UINT16_LB(win_x));
    bus->write16(bus, 0x3212, UINT16_HB(win_y));
    bus->write16(bus, 0x3213, UINT16_LB(win_y));
}

static void sc202cs_apply_mirror_reg(bk_camera_csi_sensor_t *sensor)
{
    uint8_t tmp;
    bk_camera_bus_t *bus = sensor->config.bus;

    if (bus == NULL || sensor->data.private_data == NULL)
    {
        return;
    }

    tmp = 0;
    if (sensor->data.hmirror)
    {
        tmp |= SC202CS_MIRROR_BITS;
    }
    if (sensor->data.vflip)
    {
        tmp |= SC202CS_VFLIP_BITS;
    }
    bus->write16(bus, SC202CS_REG_MIRROR_FLIP, tmp);

    sc202cs_write_window_regs(sensor);
}

static avdk_err_t sc202cs_set_ppi(bk_camera_sensor_ctlr_t *controller, uint16_t width, uint16_t height)
{
    bk_camera_csi_sensor_t *csi_sensor = __containerof(controller, bk_camera_csi_sensor_t, ops);
    AVDK_RETURN_ON_FALSE(csi_sensor, AVDK_ERR_INVAL, TAG, "csi sensor is NULL");
    bk_camera_bus_t *bus = csi_sensor->config.bus;

    if (width > WIN_MAX_X || width <= 0 || height > WIN_MAX_Y || height <= 0)
    {
        LOGE("%s, %d width: %d, height: %d, fail...\n", __func__, __LINE__, width, height);
        return -1;
    }

    if (width == WIN_MAX_X && height == WIN_MAX_Y) {
        uint32_t size = sc202cs_TABLE_SIZE(sensor_sc202cs_init_table);
        for (int i = 0; i < size; i++) {
            int ret = bus->write16(bus, sensor_sc202cs_init_table[i][0],
                                   sensor_sc202cs_init_table[i][1]);
            if (ret) {
                LOGE("init_table[%d/%u] reg 0x%04x write FAILED ret=%d\n",
                     i, (unsigned)size, sensor_sc202cs_init_table[i][0], ret);
                return -1;
            }
            /* Vendor INI carries settle delays that a plain reg table cannot
             * express; approximate the critical ones:
             *  - 0x0103 write asserts PLL soft reset -> let it settle
             *  - 0x36e9=0x80 asserts the mipi/analog block reset -> hold it
             *    asserted for a moment before the 0x24 release write below. */
            if (sensor_sc202cs_init_table[i][0] == 0x0103) {
                rtos_delay_milliseconds(5);
            } else if (sensor_sc202cs_init_table[i][0] == 0x36e9 &&
                       sensor_sc202cs_init_table[i][1] == 0x80) {
                rtos_delay_milliseconds(5);
            }
        }
        LOGI("init_table downloaded: %u entries, all ACK\n", (unsigned)size);
    } else {
        LOGE("not supported width: %d, height: %d\n", width, height);
        return -1;
    }

    /* SC202CS module is 1-lane MIPI: N_LANES reg (CSI host +0x04) needs manual config */
    bk_mipi_csi_controller_init(width, height, 0x2b);

    csi_sensor->data.width = width;
    csi_sensor->data.height = height;

    sc202cs_capture_window_base(csi_sensor);

    return 0;
}

static avdk_err_t sc202cs_set_fps(bk_camera_sensor_ctlr_t *controller, uint16_t fps)
{
    bk_camera_csi_sensor_t *csi_sensor = __containerof(controller, bk_camera_csi_sensor_t, ops);
    AVDK_RETURN_ON_FALSE(csi_sensor, AVDK_ERR_INVAL, TAG, "csi sensor is NULL");
    bk_camera_bus_t *bus = csi_sensor->config.bus;

    /* VTS scales linearly: vts = 1875 * 30 / fps (16-bit register) */
    if (fps > SC202CS_FPS_BASE
        || fps < 2
        || (SC202CS_VTS_1600x1200_20FPS * SC202CS_FPS_BASE / fps) > 0xFFFF)
    {
        LOGE("%s, unsupported fps: %d\n", __func__, fps);
        return -1;
    }

    uint32_t vts = (uint32_t)SC202CS_VTS_1600x1200_20FPS * SC202CS_FPS_BASE / fps;
    bus->write16(bus, sc202cs_REG_VTS_L, (vts & 0xff));
    bus->write16(bus, sc202cs_REG_VTS_H, (vts >> 8));

    csi_sensor->data.fps = fps;
    return BK_OK;
}

static avdk_err_t sc202cs_set_hmirror(bk_camera_sensor_ctlr_t *controller, bool enable)
{
    bk_camera_csi_sensor_t *csi_sensor = __containerof(controller, bk_camera_csi_sensor_t, ops);
    AVDK_RETURN_ON_FALSE(csi_sensor, AVDK_ERR_INVAL, TAG, "csi sensor is NULL");

    csi_sensor->data.hmirror = enable;
    sc202cs_apply_mirror_reg(csi_sensor);
    return AVDK_ERR_OK;
}

static avdk_err_t sc202cs_set_vflip(bk_camera_sensor_ctlr_t *controller, bool enable)
{
    bk_camera_csi_sensor_t *csi_sensor = __containerof(controller, bk_camera_csi_sensor_t, ops);
    AVDK_RETURN_ON_FALSE(csi_sensor, AVDK_ERR_INVAL, TAG, "csi sensor is NULL");

    csi_sensor->data.vflip = enable;
    sc202cs_apply_mirror_reg(csi_sensor);
    return AVDK_ERR_OK;
}

static avdk_err_t sc202cs_set_format(bk_camera_sensor_ctlr_t *controller, bk_camera_sensor_format_t *format)
{
    bk_camera_csi_sensor_t *csi_sensor = __containerof(controller, bk_camera_csi_sensor_t, ops);
    AVDK_RETURN_ON_FALSE(csi_sensor, AVDK_ERR_INVAL, TAG, "csi sensor is NULL");
    AVDK_RETURN_ON_FALSE(format, AVDK_ERR_INVAL, TAG, "format is NULL");
    sc202cs_apply_mirror_reg(csi_sensor);
    sc202cs_set_ppi(controller, format->width, format->height);
    sc202cs_set_fps(controller, format->fps);
    bk_mipi_csi_controller_reset();
    sc202cs_apply_mirror_reg(csi_sensor);
    return AVDK_ERR_OK;
}

static avdk_err_t sc202cs_ctrl(bk_camera_sensor_ctlr_t *controller, uint8_t cmd, uint16_t addr, uint8_t val)
{
    bk_camera_csi_sensor_t *csi_sensor = __containerof(controller, bk_camera_csi_sensor_t, ops);
    AVDK_RETURN_ON_FALSE(csi_sensor, AVDK_ERR_INVAL, TAG, "csi sensor is NULL");
    bk_camera_bus_t *bus = csi_sensor->config.bus;

    if (cmd == 0) // sensor reg read
    {
        uint8_t dump_val;
        bus->read16(bus, addr, &dump_val);
        LOGI("sc202cs {%04x, %02x}\n", addr, dump_val);
    }

    if (cmd == 1) // sensor reg write
    {
        uint8_t dump_val;
        bus->read16(bus, addr, &dump_val);
        LOGI("sc202cs {%04x, %02x} -> {%04x, %02x}\n", addr, dump_val, addr, val);
        bus->write16(bus, addr, val);
    }

    if (cmd == 2) // standy, stream stop
    {
        LOGI("sc202cs stream stop\n");
        bus->write16(bus, 0x0100, 0x00);
    }

    if (cmd == 3) // resume from standy, stream start
    {
        LOGI("sc202cs stream start\n");
        bus->write16(bus, 0x0100, 0x01);
    }

    return 0;
}

static avdk_err_t sc202cs_ioctl(bk_camera_sensor_ctlr_t *controller,
                               uint32_t cmd,
                               void *arg)
{
    bk_camera_csi_sensor_t *csi_sensor =
        __containerof(controller, bk_camera_csi_sensor_t, ops);
    AVDK_RETURN_ON_FALSE(csi_sensor, AVDK_ERR_INVAL, TAG, "csi sensor is NULL");

    sc202cs_private_data_t *private_data = sc202cs_get_private(csi_sensor);
    AVDK_RETURN_ON_FALSE(private_data, AVDK_ERR_INVAL, TAG, "private data is NULL");

    switch (cmd) {
        case BK_CAMERA_SENSOR_IOCTL_SET_INIT_EXPOSURE:
        {
            bk_camera_sensor_init_exposure_t *config =
                (bk_camera_sensor_init_exposure_t *)arg;
            AVDK_RETURN_ON_FALSE(config && config->exposure,
                                 AVDK_ERR_INVAL, TAG,
                                 "init exposure is invalid");
            private_data->init_exposure = config->exposure;
            private_data->init_exposure_valid = true;
            return AVDK_ERR_OK;
        }
        case BK_CAMERA_SENSOR_IOCTL_GET_DEFAULT_CPROC:
        {
            bk_isp_cproc_attr_t *out = (bk_isp_cproc_attr_t *)arg;
            const ISP_CPROC_ATTR_S *src;

            AVDK_RETURN_ON_FALSE(out, AVDK_ERR_INVAL, TAG,
                                 "default cproc arg is NULL");
            src = &SC202CS_1600x1200_CalibParam.modules.cproc;
            out->enable = src->enable ? 1 : 0;
            out->op_type = src->opType;
            out->manual.brightness = src->manualAttr.brightness;
            out->manual.contrast = src->manualAttr.contrast;
            out->manual.saturation = src->manualAttr.saturation;
            out->manual.hue = src->manualAttr.hue;
            os_memcpy(out->auto_attr.brightness, src->autoAttr.brightness,
                      sizeof(out->auto_attr.brightness));
            os_memcpy(out->auto_attr.contrast, src->autoAttr.contrast,
                      sizeof(out->auto_attr.contrast));
            os_memcpy(out->auto_attr.saturation, src->autoAttr.saturation,
                      sizeof(out->auto_attr.saturation));
            os_memcpy(out->auto_attr.hue, src->autoAttr.hue,
                      sizeof(out->auto_attr.hue));
            return AVDK_ERR_OK;
        }
        default:
            return AVDK_ERR_UNSUPPORTED;
    }
}

static void *sc202cs_get_sensor_object(bk_camera_sensor_ctlr_t *controller)
{
    bk_camera_csi_sensor_t *csi_sensor = __containerof(controller, bk_camera_csi_sensor_t, ops);
    AVDK_RETURN_ON_FALSE(csi_sensor, NULL, TAG, "csi sensor is NULL");
    sc202cs_private_data_t *private_data = sc202cs_get_private(csi_sensor);
    AVDK_RETURN_ON_FALSE(private_data, NULL, TAG, "private data is NULL");
    return (void *)&private_data->sensor_object;
}

static void *sc202cs_get_sensor_cfg(bk_camera_sensor_ctlr_t *controller)
{
    bk_camera_csi_sensor_t *csi_sensor = __containerof(controller, bk_camera_csi_sensor_t, ops);
    AVDK_RETURN_ON_FALSE(csi_sensor, NULL, TAG, "csi sensor is NULL");
    return (void*)csi_sensor->sensor_config;
}

static avdk_err_t sc202cs_query_support_formats(bk_camera_sensor_ctlr_t *controller, bk_camera_sensor_format_array_t *format_array)
{
    AVDK_RETURN_ON_FALSE(format_array, AVDK_ERR_INVAL, TAG, "format array is NULL");
    format_array->format_array = &sc202cs_format_array[0];
    format_array->size = ARRAY_SIZE(sc202cs_format_array);
    return AVDK_ERR_OK;
}

avdk_err_t sc202cs_detect(bk_camera_sensor_handle_t *handle, bk_camera_sensor_config_t *config)
{
    uint8_t hb_id = 0, lb_id = 0, flag = 0xFF;
    uint8_t cand = 0;
    uint32_t cand_idx;
    int hb_ret = BK_FAIL, lb_ret = BK_FAIL, flag_ret = BK_FAIL;
    int retry;

    AVDK_RETURN_ON_FALSE(config->bus, AVDK_ERR_GENERIC, TAG, "bus is NULL");

    LOGI("%s, rest_pin: %d, pwdn_pin: %d\n", __func__, config->pin_reset, config->pin_pwdn);
    /* enable camera power */
    if (config->pin_pwdn != 0xFF)
    {
        BK_LOG_ON_ERR(bk_gpio_enable_output(config->pin_pwdn));
        bk_gpio_set_capacity(config->pin_pwdn, GPIO_DRIVER_CAPACITY_3);
        bk_gpio_set_output_high(config->pin_pwdn);
        rtos_delay_milliseconds(10);
    }

    /* full reset pulse: hold low, then release high; the sensor's I2C needs
     * MCLK plus >=10ms after reset release before it answers */
    if (config->pin_reset != 0xFF)
    {
        BK_LOG_ON_ERR(bk_gpio_enable_output(config->pin_reset));
        bk_gpio_set_capacity(config->pin_reset, GPIO_DRIVER_CAPACITY_3);
        bk_gpio_set_output_low(config->pin_reset);
        rtos_delay_milliseconds(5);
        bk_gpio_set_output_high(config->pin_reset);
        rtos_delay_milliseconds(20);
    }

    /* NOTE: sw_i2c leaves *value untouched when the slave does not ACK,
     * so the return code must be checked before using the value.
     * Probe both documented slave addresses: SID low -> 0x6C, SID high -> 0x20. */
    for (retry = 0; retry < 3 && (hb_ret != BK_OK || lb_ret != BK_OK); retry++)
    {
        for (cand_idx = 0; cand_idx < ARRAY_SIZE(sc202cs_addr_candidates); cand_idx++)
        {
            cand = sc202cs_addr_candidates[cand_idx];
            config->bus->write_address = cand;
            hb_ret = config->bus->read16(config->bus, CHIP_ID_ADDR_HB, &hb_id);
            lb_ret = config->bus->read16(config->bus, CHIP_ID_ADDR_LB, &lb_id);
            if (hb_ret == BK_OK && lb_ret == BK_OK)
            {
                sc202cs_work_addr = cand;
                break;
            }
            LOGW("%s no-ack at 0x%02X (7-bit 0x%02X), retry %d\n",
                 __func__, cand, cand >> 1, retry);
        }
        if (hb_ret != BK_OK || lb_ret != BK_OK)
        {
            rtos_delay_milliseconds(10);
        }
    }

    if (hb_ret != BK_OK || lb_ret != BK_OK)
    {
        config->bus->write_address = sc202cs_WRITE_ADDRESS;
        LOGE("%s chip id read failed: no ack at 0x6C/0x20/0xD8, "
             "check wiring/pull-up/VDD/OE/MCLK\n", __func__);
        return AVDK_ERR_GENERIC;
    }
    LOGI("%s ACK at write addr 0x%02X (7-bit 0x%02X)\n",
         __func__, sc202cs_work_addr, sc202cs_work_addr >> 1);

    /* vendor DemoSens presence flag for this module: 0x36ff == 0x00 */
    flag_ret = config->bus->read16(config->bus, 0x36ff, &flag);
    LOGI("%s chip id: 0x%02X%02X, flag(0x36ff): ret=%d val=0x%02X\n",
         __func__, hb_id, lb_id, flag_ret, flag);

    if (hb_id != CHIP_ID_VAL_HB
        || lb_id != CHIP_ID_VAL_LB)
    {
        if (flag_ret == BK_OK && flag == 0x00)
        {
            /* true SC202CS PID is not publicly documented; the vendor
             * DemoSens flag matched, accept for bring-up and log the
             * actual id so CHIP_ID_VAL_* can be fixed with the real value */
            LOGW("%s chip id 0x%02X%02X unexpected but DemoSens flag ok, accept\n",
                 __func__, hb_id, lb_id);
        }
        else
        {
            LOGE("%s chip id mismatch: 0x%02X%02X (expect 0x%02X%02X)\n",
                 __func__, hb_id, lb_id, CHIP_ID_VAL_HB, CHIP_ID_VAL_LB);
            return AVDK_ERR_GENERIC;
        }
    }

    LOGI("%s success id: 0x%02X%02X\n", __func__, hb_id, lb_id);

    size_t alloc_size = sizeof(bk_camera_csi_sensor_t) +
                        sizeof(sc202cs_private_data_t);
    bk_camera_csi_sensor_t *csi_sensor = os_malloc(alloc_size);
    AVDK_RETURN_ON_FALSE(csi_sensor, AVDK_ERR_NOMEM, TAG, AVDK_ERR_NOMEM_TEXT);
    os_memset(csi_sensor, 0, alloc_size);
    csi_sensor->data.private_data = (void *)(csi_sensor + 1);
    sc202cs_private_data_t *private_data = sc202cs_get_private(csi_sensor);
    private_data->sensor_object = snssc202csObj;
    private_data->sensor_object.userData = private_data;
    config->bus->write_address = sc202cs_work_addr;
    os_memcpy(&csi_sensor->config, config, sizeof(bk_camera_sensor_config_t));

    csi_sensor->ops.init = sc202cs_init;
    csi_sensor->ops.set_format = sc202cs_set_format;
    csi_sensor->ops.reg_ctrl = sc202cs_ctrl;
    csi_sensor->ops.ioctl = sc202cs_ioctl;
    csi_sensor->ops.set_hmirror = sc202cs_set_hmirror;
    csi_sensor->ops.set_vflip = sc202cs_set_vflip;
    csi_sensor->ops.get_sensor_object = sc202cs_get_sensor_object;
    csi_sensor->ops.get_sensor_cfg = sc202cs_get_sensor_cfg;
    csi_sensor->ops.query_support_formats = sc202cs_query_support_formats;

    csi_sensor->isp_pub_attr = &sc202cs_mipi_linear_attr;
    csi_sensor->sensor_config = NULL;
    *handle = (bk_camera_sensor_handle_t)&csi_sensor->ops;

    return AVDK_ERR_OK;
}

BK_CAMERA_SENSOR_DETECT_SECTION(sc202cs_detect, CSI_CAMERA_PORT);

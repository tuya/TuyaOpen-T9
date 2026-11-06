/**
 * @if copyright_display
 *      Copyright (C), 2018-2021, Arm Technology (China) Co., Ltd.
 *      All rights reserved
 *
 *      The content of this file or document is CONFIDENTIAL and PROPRIETARY
 *      to Arm Technology (China) Co., Ltd. It is subject to the terms of a
 *      License Agreement between Licensee and Arm Technology (China) Co., Ltd
 *      restricting among other things, the use, reproduction, distribution
 *      and transfer.  Each of the embodiments, including this information and
 *      any derivative work shall retain this copyright notice.
 * @endif
 */

#include "dubhe_hash.h"
#include "dubhe_regs.h"
#include "dubhe_driver.h"
#include "dubhe_event.h"
#include "pal_string.h"
#include "pal_time.h"
#include "pal_log.h"
#include "reg_base.h"

#define PROC_TIME_FOR_ONE_BLOCK  ( 50 )

#define DBH_HASH_DMA_ADDR(p) ((uint32_t)SOC_SRAM_PERI_ADDR((uintptr_t)(p)))

/*
 * The TE200 DMA master can only reach SRAM (CPU alias 0x2Cxxxxxx, peripheral
 * alias 0x28/0x38xxxxxx). A buffer in PSRAM (0x60/0x64xxxxxx) or flash
 * (0x02/0x04xxxxxx) is silently ignored: the command still completes with
 * ret == 0 but nothing is written back, which looks exactly like
 * "arm_ce_hash_finish() returned 0 and the digest buffer was untouched".
 *
 * libtuyaos.a hands us contexts and output buffers that live in PSRAM, so
 * every buffer that is given to the engine is staged through SRAM first.
 * The staging buffers are static (.bss -> SRAM) and are only touched while
 * DBH_HASH_MUTEX is held, see arm_ce_hash_update()/arm_ce_hash_finish().
 */
#define DUBHE_HASH_IO_STAGE_LEN    64
#define DUBHE_HASH_EXTRA_STAGE_LEN 64
#define DUBHE_HASH_IN_STAGE_LEN    512

static uint8_t s_hash_io_stage[DUBHE_HASH_IO_STAGE_LEN];
static uint8_t s_hash_extra_stage[DUBHE_HASH_EXTRA_STAGE_LEN];
static uint8_t s_hash_in_stage[DUBHE_HASH_IN_STAGE_LEN];

static int dubhe_hash_dma_is_sram( const void *p )
{
    switch ( ( (uint32_t)(uintptr_t)p ) >> 24 ) {
    case 0x28:
    case 0x2C:
    case 0x2D:
    case 0x38:
    case 0x3C:
        return 1;
    default:
        return 0;
    }
}

/* DMA address of ctx->extra_data, staged through SRAM when the context
 * itself lives in PSRAM. The engine only reads this buffer. */
static uint32_t dubhe_hash_extra_dma_addr( arm_ce_hash_context_t *ctx )
{
    if ( dubhe_hash_dma_is_sram( ctx->extra_data ) )
        return DBH_HASH_DMA_ADDR( ctx->extra_data );

    pal_memcpy( s_hash_extra_stage, ctx->extra_data,
                sizeof( ctx->extra_data ) );
    return DBH_HASH_DMA_ADDR( s_hash_extra_stage );
}

static uint32_t dubhe_hash_digest_len( arm_ce_hash_mode_t mode )
{
    switch ( mode ) {
    case ARM_HASH_MODE_SHA1:
        return 20;
    case ARM_HASH_MODE_SHA224:
        return 28;
    default:
        return 32;
    }
}

#define GET_HASH_LEN( ctx )                                                    \
    ( ( ctx->mode == ARM_HASH_MODE_SHA1 )                                      \
          ? DBH_HASH_SHA1_TEMP_DATA_LEN                                        \
          : ( ctx->mode == ARM_HASH_MODE_SHA224 )                              \
                ? DBH_HASH_SHA224_TEMP_DATA_LEN                                \
                : ( ctx->mode == ARM_HASH_MODE_SHA256 )                        \
                      ? DBH_HASH_SHA256_TEMP_DATA_LEN                          \
                      : ( ctx->mode == ARM_HASH_MODE_SM3 )                     \
                            ? DBH_HASH_SM3_TEMP_DATA_LEN                       \
                            : DBH_HASH_SHA1_TEMP_DATA_LEN )

#define BIT_MASK( V, M ) ( ( V ) & ( M ) )

#define CHECK_RET( )                                                           \
    do {                                                                       \
        switch ( ret ) {                                                       \
        case 0:                                                                \
            break;                                                             \
        case DBH_HASH_BUS_ERR:                                                 \
            arm_ce_hash_driver_init( );                                        \
            return ret;                                                        \
        default:                                                               \
            return ret;                                                        \
        }                                                                      \
    } while ( 0 )

#define CHECK_CONDITION(_cond, _prompt, _jump)                                 \
    do {                                                                       \
        if( !( _cond ) ){                                                      \
            PAL_LOG_ERR( "%s\n", ( _prompt ) );                                \
            goto _jump;                                                        \
        }                                                                      \
    } while ( 0 )


static void dubhe_zeroize( void *v, size_t n )
{
    volatile unsigned char *p = (unsigned char *) v;
    while ( n-- )
        *p++ = 0;
}

static arm_ce_hash_context_t *_g_ce_hash_ctx = NULL;

#if defined( DUBHE_FOR_RUNTIME )
static void arm_ce_hash_process_mask_intr( uint32_t mask )
{
    volatile uint32_t value = 0;

    value = DBH_READ_REGISTER( HASH, HASH_INTR_MASK );
    DBH_REG_FLD_SET( HASH_INTR_MASK, CMD_INTR, value, (mask & 0x01) );
    DBH_REG_FLD_SET( HASH_INTR_MASK, INVALID_CMD, value, (mask & 0x01) );
    DBH_REG_FLD_SET( HASH_INTR_MASK, PADDING_ERR, value, (mask & 0x01) );
    DBH_REG_FLD_SET( HASH_INTR_MASK, BUS_ERR, value, (mask & 0x01) );
    /* Watermark interrupt should be masked */
    DBH_REG_FLD_SET( HASH_INTR_MASK, WM_INTR, value, 0x1 );
    DBH_WRITE_REGISTER( HASH, HASH_INTR_MASK, value );
}

#define ARM_CE_HASH_DISABLE_INTERUPT()                                          \
    do{                                                                         \
            arm_ce_hash_process_mask_intr(0x01);                                \
    }while(0)

#define ARM_CE_HASH_ENABLE_INTERUPT()                                           \
    do{                                                                         \
            arm_ce_hash_process_mask_intr(0x00);                                \
    }while(0)

#endif

void arm_ce_hash_driver_init( void )
{
    volatile uint32_t value = 0;
#if defined( DUBHE_SECURE )
    dubhe_clk_enable( DBH_MODULE_HASH );
    /* Reset HASH module */
    value = DBH_READ_REGISTER( TOP_CTRL, RESET_CTRL );
    DBH_REG_FLD_SET( RESET_CTRL, HASH, value, 0x1 );
    DBH_WRITE_REGISTER( TOP_CTRL, RESET_CTRL, value );

    value = DBH_READ_REGISTER( TOP_CTRL, RESET_CTRL );
    DBH_REG_FLD_SET( RESET_CTRL, HASH, value, 0x0 );
    DBH_WRITE_REGISTER( TOP_CTRL, RESET_CTRL, value );
#endif

#if defined( DUBHE_FOR_RUNTIME )
    /* Set command queue watermark, set to 0, there's no watermark interrupt
     * generated */
    value = ( 0x0 << 1 );

    /* Unmask command interrupt */
    ARM_CE_HASH_DISABLE_INTERUPT();

#endif

    /* Run */
    value |= 0x1;
    DBH_WRITE_REGISTER( HASH, HASH_CTRL, value );
}

/* Get engine status, busy return -1, free return 0 */
static int arm_ce_hash_read_hash_stat( void )
{
    volatile uint32_t stat       = 0;
    volatile uint8_t avail_slots = 0, engine_stat = 0;

    stat = DBH_READ_REGISTER( HASH, HASH_STAT );
    avail_slots =
        ( stat >> DBH_HASH_ENGINE_QUEUE_AVAIL_SLOTS_BIT_SHIFT ) & 0xFF;
    engine_stat = stat & 0x3;

    if ( ( avail_slots == DBH_HASH_QUEUE_DEPTH )
         && ( engine_stat <= ARM_CE_HASH_ENG_WAIT_FOR_CMD ) )
        return ( DBH_HASH_ENGINE_IDLE );
    else
        return ( DBH_HASH_ENGINE_BUSY );
}

static int _arm_ce_hash_intr_handler( void )
{
    volatile uint32_t value = 0;
    int ret        = 0;

    value = DBH_READ_REGISTER( HASH, HASH_INTR_STAT );

    if ( BIT_MASK( value, DBH_HASH_CMD_INTR ) ) {
        ret = 0;
    }

    if ( BIT_MASK( value, DBH_HASH_BUS_ERR_INTR ) ) {
        ret = DBH_HASH_BUS_ERR;
        PAL_LOG_ERR( "value(%X) HASH process error \
                    unexpect interupt DBH_HASH_BUS_ERR\n",
                     value );
        goto exit;
    }

    if ( BIT_MASK( value, DBH_HASH_INVALID_CMD_INTR ) ) {
        ret = DBH_HASH_INVALID_CMD_ERR;
        PAL_LOG_ERR( "value(%X) HASH process error \
                    unexpect interupt DBH_HASH_INVALID_CMD_ERR\n",
                     value );
        goto exit;
    }

    if ( BIT_MASK( value, DBH_HASH_PADDING_ERR_INTR ) ) {
        ret = DBH_HASH_PADDING_ERR;
        PAL_LOG_ERR( "value(%X) HASH process error \
                    unexpect interupt DBH_HASH_PADDING_ERR_INTR\n",
                     value );
        goto exit;
    }

    if ( BIT_MASK( value, DBH_HASH_WM_INTR ) ) {
        PAL_LOG_ERR( "value(%X) HASH process error \
                    unexpect interupt DBH_HASH_WM_INTR\n",
                     value );
        goto exit;
    }

exit:
    /*write clear the interrupt*/
    DBH_WRITE_REGISTER( HASH, HASH_INTR_STAT, value );

    return ( ret );
}

static int arm_ce_hash_wait_engine_idle( void )
{
    uint32_t value = 0;
    int ret        = 0;
#if defined( DBH_HASH_DEBUG )
    PAL_LOG_DEBUG( "### START WAIT HASH_STAT_REG: 0x%08x ###\n",
                   DBH_BASE_SECURE_HASH + DBH_HASH_STAT_REG_OFFSET );
#endif
    for ( ;; ) {
        ret = _arm_ce_hash_intr_handler( );
        if ( ret != 0 ) {
            goto exit;
        }

        value = arm_ce_hash_read_hash_stat( );
        if ( value != 0 ) {
#if defined( DUBHE_FOR_RUNTIME )
            pal_udelay( PROC_TIME_FOR_ONE_BLOCK );
#endif
            continue;
        } else
            break;
    }

#if defined( DBH_HASH_DEBUG )
    PAL_LOG_DEBUG( "### EXIT WAIT HASH_STAT_REG: 0x%08x, AVAIL_SLOTS: 0x1, "
                   "ENGINE_STATE: 0x1 ###\n",
                   DBH_BASE_SECURE_HASH + DBH_HASH_STAT_REG_OFFSET );
#endif
exit:
    return ret;
}

static int arm_ce_hash_execute_hash_init( arm_ce_hash_context_t *ctx,
                                          arm_ce_hash_vect_source_t vector,
                                          int vect_is_addr,
                                          unsigned char *external_vector,
                                          size_t iv_len,
                                          arm_ce_hash_mode_t hash_mode )
{
    int i           = 0;
    size_t iv_words = 0;
    uint32_t value = 0, hash_iv_addr = 0;
    uint32_t *temp_vect = NULL;
    int ret             = 0;

    value = DBH_HASH_INIT_OPCODE << DBH_HASH_OPCODE_BIT_SHIFT;
    value |= ( hash_mode << DBH_HASH_MODE_BIT_SHIFT );

    if ( vector == ARM_HASH_VECT_DEFAULT ) {
        /* Set initial vector source default value,
         * and this command will not trigger an interrupt
         */
        value |= DBH_HASH_VECT_SRC_INTERNAL;
        value &= 0xFFFFFFFE;

        /* Function part */
        DBH_WRITE_REGISTER( HASH, HASH_QUEUE, value );
    } else if ( vector == ARM_HASH_VECT_EXTERNAL ) {
        if ( external_vector == NULL ) {
            return ( DBH_HASH_VECT_SRC_ERR );
        }

        /* hash_temp lives inside the (possibly PSRAM) context. */
        if ( ( vect_is_addr == DBH_HASH_IV_IS_ADDR )
             && !dubhe_hash_dma_is_sram( external_vector ) ) {
            if ( iv_len > sizeof( s_hash_io_stage ) )
                return ( DBH_HASH_VECT_SRC_ERR );
            pal_memcpy( s_hash_io_stage, external_vector, iv_len );
            external_vector = s_hash_io_stage;
        }

        /* Set initial vector source external value */
        value |= DBH_HASH_VECT_SRC_EXTERNAL;
        /* Set initial length */
        if ( ctx->total_bits_num != 0 )
            value |= 0x4;
        /* Set this command will not trigger an interrupt */
        value &= 0xFFFFFFFE;
        if ( vect_is_addr == DBH_HASH_IV_IS_ADDR ) {
            value |= 0x8;
            hash_iv_addr = DBH_HASH_DMA_ADDR( external_vector );
            /* Function part */
            DBH_WRITE_REGISTER( HASH, HASH_QUEUE, value );
            /* Parameter part */
            /* Set HASH initial vector */
            DBH_WRITE_REGISTER( HASH, HASH_QUEUE, hash_iv_addr );
        } else {
            /* Function part */
            DBH_WRITE_REGISTER( HASH, HASH_QUEUE, value );
            /* Parameter part */
            /* Set HASH initial vector */
            temp_vect = (uint32_t *) external_vector;
            iv_words  = iv_len / 4;
            for ( i = 0; i < iv_words; i++ )
                DBH_WRITE_REGISTER( HASH, HASH_QUEUE, *temp_vect++ );
        }
        /* Set HASH total bits length, split 64 bits into two parts, 32 bits for
         * each */
        if ( ctx->total_bits_num != 0 ) {
            value = ( ctx->total_bits_num & 0xFFFFFFFF );
            DBH_WRITE_REGISTER( HASH, HASH_QUEUE, value );
            value = ( ctx->total_bits_num >> 0x20 );
            DBH_WRITE_REGISTER( HASH, HASH_QUEUE, value );
        }
    } else {
        return ( DBH_HASH_VECT_SRC_ERR );
    }

    /* Set the flag to indicate if the context need to finish. */
    ctx->finish_flag = 1;
    /* Wait for the command execution to complete */
    ret = arm_ce_hash_wait_engine_idle( );
    CHECK_RET();

    return ret;
}

static int arm_ce_hash_execute_hash_process( arm_ce_hash_context_t *ctx,
                                             const unsigned char *input,
                                             size_t ilen )
{
    size_t input_data_len = 0;
    uint32_t value = 0, addr = 0;
    uint32_t old_extra_len = ctx->count;
    int ret                = 0;

    value = DBH_HASH_PROCESS_OPCODE << DBH_HASH_OPCODE_BIT_SHIFT;

    /* Extra data bytes number */
    ctx->count = ( ilen + old_extra_len ) % HASH_BUF_LEN;
    if ( ilen + old_extra_len >= HASH_BUF_LEN ) {

        /* First handle old extra data, then the new input data */
        if ( old_extra_len != 0 ) {
            /* HASH_process function part */
            DBH_WRITE_REGISTER( HASH, HASH_QUEUE, value );
            /* HASH process parameter part */
            addr = dubhe_hash_extra_dma_addr( ctx );
            DBH_WRITE_REGISTER( HASH, HASH_QUEUE, addr );

            DBH_WRITE_REGISTER( HASH, HASH_QUEUE, old_extra_len );
            ret = arm_ce_hash_wait_engine_idle( );
            CHECK_RET( );
            ctx->total_bits_num += old_extra_len * 8;
        }

        value          = DBH_HASH_PROCESS_OPCODE << DBH_HASH_OPCODE_BIT_SHIFT;
        input_data_len = ilen - ctx->count;

#if defined( DUBHE_FOR_RUNTIME )
        /* If input data input_data_len > INTR_THRESHOLD, use interrupt mode,
         * else polling mode */

        if ( input_data_len > INTR_THRESHOLD ){
            value |= 0x1;
            ARM_CE_HASH_ENABLE_INTERUPT();
        }
#endif
        /* HASH_process function part */
        DBH_WRITE_REGISTER( HASH, HASH_QUEUE, value );
        /* HASH process parameter part */
        addr = DBH_HASH_DMA_ADDR( input );
        DBH_WRITE_REGISTER( HASH, HASH_QUEUE, addr );
        /* HASH process parameter part */
        DBH_WRITE_REGISTER( HASH, HASH_QUEUE, input_data_len );

        /* Total data bits number */
        ctx->total_bits_num += input_data_len * 8;
        /* Save new extra data */
        pal_memset( ctx->extra_data, 0, sizeof( ctx->extra_data ) );
        pal_memcpy(
            ctx->extra_data, ( input + ilen - ctx->count ), ctx->count );
    } else {
        /* If ilen + old_extra_len < HASH_BUF_LEN */
        /* Save input data and return. */
        pal_memcpy( ctx->extra_data + old_extra_len, input, ilen );
        return ( 0 );
    }

#if defined( DUBHE_FOR_RUNTIME )
    if ( input_data_len <= INTR_THRESHOLD ) {
        ret = arm_ce_hash_wait_engine_idle( );
        CHECK_RET( );
    } else {
        ret = dubhe_wait_event( DBH_EVENT_HASH_CMD_EXCUTED );
        ARM_CE_HASH_DISABLE_INTERUPT();
        CHECK_RET( );
    }
#else
    ret = arm_ce_hash_wait_engine_idle( );
    CHECK_RET( );
#endif

    return ( 0 );
}

static int arm_ce_hash_execute_hash_finish( arm_ce_hash_context_t *ctx,
                                            uint8_t padding,
                                            unsigned char *buf )
{
    uint32_t process_value = 0, finish_value = 0;
    uint32_t extra_data    = 0;
    uint32_t data          = 0;
    uint8_t *stage_buf     = NULL;
    int ret                = 0;

    /* The engine writes the intermediate value / digest straight to buf; if
     * buf is not SRAM the write is dropped, so redirect it to the staging
     * buffer and copy the result back once the command is done. */
    if ( dubhe_hash_dma_is_sram( buf ) ) {
        data = DBH_HASH_DMA_ADDR( buf );
    } else {
        stage_buf = s_hash_io_stage;
        pal_memset( stage_buf, 0, sizeof( s_hash_io_stage ) );
        data = DBH_HASH_DMA_ADDR( stage_buf );
    }
    extra_data = dubhe_hash_extra_dma_addr( ctx );

    /* filter uninitialized finish request */
    if ( !ctx->finish_flag ) {
        PAL_LOG_ERR( "%s:%d not initialised yet!!\n", __func__, __LINE__ );
        return ret;
    }

    finish_value = DBH_HASH_FINISH_OPCODE << DBH_HASH_OPCODE_BIT_SHIFT;

    /* Intermediate value will be saved in the buf */
    if ( padding == 0 ) {
        /* Function part: no padding, no interrupt */
        DBH_WRITE_REGISTER( HASH, HASH_QUEUE, finish_value );
        /* Parameter part: intermediate data address*/
        DBH_WRITE_REGISTER( HASH, HASH_QUEUE, data );
        ret = arm_ce_hash_wait_engine_idle( );
        CHECK_RET( );
        if ( stage_buf != NULL )
            pal_memcpy( buf, stage_buf, GET_HASH_LEN( ctx ) );
        ctx->hash_temp_valid = 1;
        ctx->finish_flag     = 0;
        return ret;
    }
    /* If extra data count is not zero, execute HASH process command first */
    if ( ctx->count != 0 ) {
        process_value = DBH_HASH_PROCESS_OPCODE << DBH_HASH_OPCODE_BIT_SHIFT;

        /* HASH_process function part */
        DBH_WRITE_REGISTER( HASH, HASH_QUEUE, process_value );
        /* HASH process parameter part */
        DBH_WRITE_REGISTER( HASH, HASH_QUEUE, extra_data );
        DBH_WRITE_REGISTER( HASH, HASH_QUEUE, ctx->count );
        ret = arm_ce_hash_wait_engine_idle( );
        CHECK_RET( );
    }

    /* HASH finish command function part: padding mode, no interrupt */
    finish_value |= ( 0x1 << 0x7 );
    DBH_WRITE_REGISTER( HASH, HASH_QUEUE, finish_value );

    /* HASH finish parameter part: output address */
    DBH_WRITE_REGISTER( HASH, HASH_QUEUE, data );
    /* Wait for the command execution to complete */
    ret = arm_ce_hash_wait_engine_idle( );
    CHECK_RET( );
    if ( stage_buf != NULL )
        pal_memcpy( buf, stage_buf, dubhe_hash_digest_len( ctx->mode ) );
    dubhe_zeroize( ctx, sizeof( arm_ce_hash_context_t ) );
    return ret;
}

int arm_ce_hash_init( arm_ce_hash_context_t *ctx )
{
    unsigned char output[HASH_BUF_LEN / 2];

    if ( ctx != NULL ) {
        // check finish flag here, if needed then finish
        if ( ctx->finish_flag ) {
            arm_ce_hash_finish( ctx, output );
        }

        dubhe_zeroize( ctx, sizeof( arm_ce_hash_context_t ) );
        return ( 0 );
    } else
        return ( DBH_HASH_PARAM_INVAL_ERR );
}

int arm_ce_hash_clone( arm_ce_hash_context_t *dst, arm_ce_hash_context_t *src )
{
    int ret                      = 0;
    uint8_t padding              = 0;
    arm_ce_hash_mode_t hash_mode = src->mode;
    uint32_t hash_temp_len       = 0;
    arm_ce_hash_vect_source_t vect_src;

    if ( ( NULL == dst ) || ( NULL == src ) ) {
        PAL_LOG_ERR(
            "Check parameters context might probably not initialised!!\n" );
        ret = DBH_HASH_PARAM_INVAL_ERR;
        goto exit;
    }

#if defined( DUBHE_FOR_RUNTIME )
    dubhe_mutex_lock( DBH_HASH_MUTEX );
#endif
    /* Finish old session */
    if ( dst == _g_ce_hash_ctx ) {
        ret = arm_ce_hash_execute_hash_finish( dst, padding, dst->hash_temp );
        if ( ret != 0 ) {
            goto cleanup;
        }
        _g_ce_hash_ctx = NULL;
    }

    if ( ( src != _g_ce_hash_ctx ) && ( src->hash_temp_valid == 1 ) ) {
        pal_memcpy( dst, src, sizeof( arm_ce_hash_context_t ) );
        goto cleanup;
    }

    ret = arm_ce_hash_execute_hash_finish( src, padding, src->hash_temp );
    if ( ret != 0 ) {
        goto cleanup;
    }
    /* Re-initialize Dubhe */
    /* Execute Dubhe HASH_init command, load hash intermediate data */
    hash_temp_len = GET_HASH_LEN( src );

    if ( src->hash_temp_valid == 1 )
        vect_src = ARM_HASH_VECT_EXTERNAL;
    else
        vect_src = ARM_HASH_VECT_DEFAULT;

    ret = arm_ce_hash_execute_hash_init( src,
                                         vect_src,
                                         DBH_HASH_IV_IS_ADDR,
                                         src->hash_temp,
                                         hash_temp_len,
                                         hash_mode );
    if ( ret != 0 ) {
        goto cleanup;
    }

    pal_memcpy( dst, src, sizeof( arm_ce_hash_context_t ) );
    _g_ce_hash_ctx = src;

cleanup:
#if defined( DUBHE_FOR_RUNTIME )
    dubhe_mutex_unlock( DBH_HASH_MUTEX );
#endif
exit:
    return ( ret );
}

int arm_ce_hash_start( arm_ce_hash_context_t *ctx,
                       arm_ce_hash_vect_source_t vector,
                       int vect_is_addr,
                       unsigned char *external_vector,
                       size_t iv_len,
                       arm_ce_hash_mode_t hash_mode )
{
    int ret = 0;
    uint8_t padding;
    unsigned char garbage[64] = {0};

    if ( ( ctx == NULL ) ) {
        PAL_LOG_ERR( "Check parameters context might \
                    probably not initialised!! iv_len(%d)\n",
                     iv_len );
        ret = DBH_HASH_PARAM_INVAL_ERR;
        goto exit;
    }

#if defined( DUBHE_FOR_RUNTIME )
    dubhe_mutex_lock( DBH_HASH_MUTEX );
#endif
    if ( _g_ce_hash_ctx != ctx ) {
        /* First finish old session (_g_ce_hash_ctx), then load new session(ctx)
         */
        if ( _g_ce_hash_ctx != NULL ) {
            ret = arm_ce_hash_execute_hash_finish(
                _g_ce_hash_ctx, 0, _g_ce_hash_ctx->hash_temp );
            _g_ce_hash_ctx = NULL;
            CHECK_CONDITION((0==ret),
                            "swap out previously context failed", cleanup);
        }
    } else {
        /*
         * This session re-start, flush garbage data. before execute
         * finish command must check if it's finish flag is set,
         * if not no need to excecute finish command
         */
        if ( _g_ce_hash_ctx != NULL ) {
            padding = 1;
            ret = arm_ce_hash_execute_hash_finish( _g_ce_hash_ctx,
                                                    padding, garbage );
            _g_ce_hash_ctx = NULL;
            CHECK_CONDITION((0==ret),
                            "hash finish error during switching context!",
                             cleanup);
        }
    }

    ctx->mode = hash_mode;
    ret       = arm_ce_hash_execute_hash_init( ctx,
                                         ARM_HASH_VECT_DEFAULT,
                                         vect_is_addr,
                                         external_vector,
                                         iv_len,
                                         hash_mode );
    if ( ret < 0 ) {
        PAL_LOG_ERR( " execute hash init failed when hash start\n" );
        goto cleanup;
    }

    _g_ce_hash_ctx = ctx;

cleanup:
#if defined( DUBHE_FOR_RUNTIME )
    dubhe_mutex_unlock( DBH_HASH_MUTEX );
#endif

exit:
    return ret;
}

int arm_ce_hash_update( arm_ce_hash_context_t *ctx,
                        const unsigned char *input,
                        size_t ilen )
{
    int ret                = 0;
    uint32_t hash_temp_len = 0;
    arm_ce_hash_vect_source_t vect_src;

    if ( ( ctx == NULL ) ) {
        ret = DBH_HASH_PARAM_INVAL_ERR;
        goto exit;
    }

    arm_ce_hash_mode_t hash_mode = ctx->mode;
    if ( ( hash_mode < ARM_HASH_MODE_SHA1 )
         || ( hash_mode > ARM_HASH_MODE_SM3 ) ) {
        ret = DBH_HASH_MODE_ERR;
        goto exit;
    }

#if defined( DUBHE_FOR_RUNTIME )
    /* Get command queue lock */
    dubhe_mutex_lock( DBH_HASH_MUTEX );
#endif
    /* Multi-session */
    if ( _g_ce_hash_ctx != ctx ) {
        /* First finish old session(_g_ce_hash_ctx), then load new session(ctx)
         */
        if ( _g_ce_hash_ctx != NULL ) {
            ret = arm_ce_hash_execute_hash_finish(
                _g_ce_hash_ctx, 0, _g_ce_hash_ctx->hash_temp );
            _g_ce_hash_ctx = NULL;
            CHECK_CONDITION(ret == 0,
                            "hash finish error during switching context!",
                            cleanup);
        }

        /* Re-initialize Dubhe */
        /* Execute Dubhe HASH_init command, load hash intermediate data */
        hash_temp_len = GET_HASH_LEN( ctx );

        if ( ctx->hash_temp_valid == 1 )
            vect_src = ARM_HASH_VECT_EXTERNAL;
        else
            vect_src = ARM_HASH_VECT_DEFAULT;

        ret = arm_ce_hash_execute_hash_init( ctx,
                                             vect_src,
                                             DBH_HASH_IV_IS_ADDR,
                                             ctx->hash_temp,
                                             hash_temp_len,
                                             hash_mode );
        if ( ret != 0 ) {
            PAL_LOG_ERR( " execute hash init failed when update, reason: %x\n",
                         ret );
            goto cleanup;
        }
        _g_ce_hash_ctx = ctx;
    }

    /* Execute Dubhe HASH_process command. When the input does not live in
     * SRAM (TLS hashes records that sit in PSRAM or flash) feed it to the
     * engine in SRAM-sized slices; the hash state carries across calls. */
    if ( dubhe_hash_dma_is_sram( input ) ) {
        ret = arm_ce_hash_execute_hash_process( ctx, input, ilen );
    } else {
        const unsigned char *src = input;
        size_t left              = ilen;

        ret = 0;
        while ( left != 0 ) {
            size_t n = left > sizeof( s_hash_in_stage )
                           ? sizeof( s_hash_in_stage )
                           : left;

            pal_memcpy( s_hash_in_stage, src, n );
            ret = arm_ce_hash_execute_hash_process( ctx, s_hash_in_stage, n );
            if ( ret != 0 )
                break;
            src += n;
            left -= n;
        }
    }
    if ( ret != 0 ) {
        PAL_LOG_ERR( " execute hash process failed when update, reason: %x\n",
                     ret );
        goto cleanup;
    }

cleanup:
#if defined( DUBHE_FOR_RUNTIME )
    dubhe_mutex_unlock( DBH_HASH_MUTEX );
#endif

exit:
    return ret;
}

int arm_ce_hash_finish( arm_ce_hash_context_t *ctx, unsigned char *output )
{
    int ret = -1;
    uint8_t padding;
    uint32_t hash_temp_len = 0;
    arm_ce_hash_vect_source_t vect_src;

    if ( ( ctx == NULL ) || ( NULL == output ) ) {
        PAL_LOG_ERR( "context might probably not initialised!!\n" );
        ret = DBH_HASH_PARAM_INVAL_ERR;
        goto exit;
    }

#if defined( DUBHE_FOR_RUNTIME )
    /* Get global lock */
    dubhe_mutex_lock( DBH_HASH_MUTEX );
#endif
    if ( ctx->hash_temp_valid == 1 )
        vect_src = ARM_HASH_VECT_EXTERNAL;
    else
        vect_src = ARM_HASH_VECT_DEFAULT;

    hash_temp_len = GET_HASH_LEN( ctx );

    if ( _g_ce_hash_ctx == ctx ) {
        /* even though invoke arm_ce_hash_finish right after arm_ce_hash_init
           should get a default hash ouput*/

        if ( !ctx->finish_flag ) {
            ret = arm_ce_hash_execute_hash_init( ctx,
                                                 vect_src,
                                                 DBH_HASH_IV_IS_ADDR,
                                                 ctx->hash_temp,
                                                 hash_temp_len,
                                                 ctx->mode );

            if ( ret < 0 ) {
                PAL_LOG_ERR( " execute hash init failed when finish\n" );
                goto cleanup;
            }
        }

        padding = 1;
        ret = arm_ce_hash_execute_hash_finish( ctx, padding, output );
    } else {
        /*  when finished the session must check it's finish flag first, if not
         * set don't need to finish it  */
        if ( _g_ce_hash_ctx != NULL ) {
            /* Save current session, then load new session */
            padding = 0;
            ret = arm_ce_hash_execute_hash_finish(
                _g_ce_hash_ctx, padding, _g_ce_hash_ctx->hash_temp );
            _g_ce_hash_ctx = NULL;
            CHECK_CONDITION(ret == 0,
                            "hash finish error during switching context!",
                             cleanup);
        }

        ret = arm_ce_hash_execute_hash_init( ctx,
                                             vect_src,
                                             DBH_HASH_IV_IS_ADDR,
                                             ctx->hash_temp,
                                             hash_temp_len,
                                             ctx->mode );
        if ( ret < 0 ) {
            PAL_LOG_ERR( " execute hash init failed when finish\n" );
            goto cleanup;
        }

        _g_ce_hash_ctx = ctx;
        padding        = 1;
        ret = arm_ce_hash_execute_hash_finish( ctx, padding, output );
    }

    _g_ce_hash_ctx = NULL;

cleanup:
#if defined( DUBHE_FOR_RUNTIME )
    dubhe_mutex_unlock( DBH_HASH_MUTEX );
#endif
exit:
    return ret;
}

void arm_ce_hash_free( arm_ce_hash_context_t *ctx )
{
    uint8_t padding;
    unsigned char garbge[64] = {0};

#if defined( DUBHE_FOR_RUNTIME )
    /* Get global lock */
    dubhe_mutex_lock( DBH_HASH_MUTEX );
#endif

    if ( ( _g_ce_hash_ctx == ctx ) && ( _g_ce_hash_ctx != NULL ) ) {
        padding = 1;
        arm_ce_hash_execute_hash_finish( ctx, padding, garbge );
        _g_ce_hash_ctx = NULL;
    }

#if defined( DUBHE_FOR_RUNTIME )
    dubhe_mutex_unlock( DBH_HASH_MUTEX );
#endif
}

/*************************** The End Of File*****************************/

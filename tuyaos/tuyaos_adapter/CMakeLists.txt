set(incs
    include
    src/test
    src/misc
    src/misc/include
    src/security
    src/misc/mf_test
    tuyaopen
    src/test/test_lvgl
    src/test/test_ringbuffer
    # src/test/test_face_recognize
    # src/test/test_face_recog
    $ENV{ARMINO_PATH}/../../../../apps/$ENV{APP_NAME}/include
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/adc
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/atomic
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/bluetooth
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/cellular
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/display
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/flash
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/gpio
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/hci
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/i2c
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/i2s
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/init/include
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/media
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/mipi
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/network
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/pinmux
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/pwm
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/rtc/include
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/security
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/spi/include
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/qspi
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/system
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/timer
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/uart
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/utilities/include
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/wakeup
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/watchdog
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/wifi
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/tal_compatible
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/lvgl
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/wired
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/ipc
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/rgb
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/mcu8080
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/dvp
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/dma2d
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/uvc
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/audio
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/tal_mtd/include
    # $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/asr
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/vad
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/kws
)

# TuyaOpen: OPEN_HEADER_DIR from build_param.json. TuyaOS: scan project include/.
if(DEFINED ENV{TUYA_HEADER_DIR} AND NOT "$ENV{TUYA_HEADER_DIR}" STREQUAL "")
    string(REPLACE " " ";" TUYA_HEADER_DIR_LIST "$ENV{TUYA_HEADER_DIR}")
    foreach(head ${TUYA_HEADER_DIR_LIST})
        list(APPEND incs ${head})
    endforeach()
else()
    list(APPEND incs "$ENV{TUYA_PROJECT_DIR}/include/base/include")
endif()

set(srcs
    src/system/tkl_memory.c
    src/system/tkl_mutex.c
    src/system/tkl_network.c
    src/system/tkl_output.c
    src/system/tkl_queue.c
    src/system/tkl_semaphore.c
    src/system/tkl_system.c
    src/system/tkl_thread.c
    # src/system/tkl_atomic.c
    # src/system/tkl_task_notify.c
    src/driver/tkl_watchdog.c
    src/driver/tkl_hci.c
    # src/driver/tkl_ipc.c
    # src/misc/tuya_tkl_ipc.c
    # src/misc/tkl_mftest.c
    include/init/src/tkl_init_system.c
)

if ("${CONFIG_CPU_INDEX}" STREQUAL "0")
    # tkl_init_system.c always references tkl_ota_*; tkl_ota.c provides CPU0 stubs under #elif CONFIG_CPU_INDEX == 0
    list(APPEND srcs src/system/tkl_ota.c)
    # list(APPEND srcs src/driver/tkl_hci_0.c)
    if (CONFIG_TUYA_TEST_CLI)
        list(APPEND srcs src/test/cli_tuya_test_0.c)
        list(APPEND srcs src/test/test_memory.c)
    endif() # CONFIG_TUYA_TEST_CLI
else()
    list(APPEND srcs
        src/driver/tkl_wifi.c
        src/driver/tkl_wired.c
        # src/driver/tkl_hci_1.c
        # src/driver/tkl_video_enc.c
        # src/driver/tkl_video_in.c
        # src/driver/tkl_mic.c
        # src/driver/tkl_speaker.c
        # src/driver/tkl_cellular.c
        src/driver/tkl_pinmux.c
        # src/driver/tkl_rgb.c
        # src/driver/tkl_8080.c
        src/driver/tkl_adc.c
        src/driver/tkl_pwm.c
        src/driver/tkl_uart.c
        src/driver/tkl_flash.c
        tuyaopen/tuyaopen_license.c
        # src/driver/tkl_rtc.c
        src/driver/tkl_spi.c
        src/driver/tkl_qspi.c
        src/driver/tkl_gpio.c
        src/driver/tkl_i2c.c
        src/driver/tkl_timer.c
        src/driver/tkl_mipi_dsi.c
        src/driver/tkl_mipi_csi.c
        src/driver/tkl_image_codec.c
        src/driver/tkl_uvc.c
        # src/driver/tkl_i2s_v2.c
        src/driver/tkl_aud_dmic.c
        src/driver/tkl_aud_dac.c
        src/driver/tkl_aud_adc.c
        src/driver/tkl_audio.c
        src/driver/tkl_audio_frontend.c
        src/driver/tkl_vad.c
        src/driver/tkl_kws.c
        # src/driver/tkl_wired.c
        src/driver/tkl_i2s.c
        # src/driver/tkl_dma2d.c
        src/driver/tkl_dvp.c
        # src/driver/tkl_jpeg_codec.c
        # src/driver/tkl_sdio.c

        src/misc/encrypt.c
        src/misc/fs_init.c
        src/misc/usr_sdio.c
        # src/misc/dhcpserver.c
        # src/misc/qflash.c
        # src/misc/sflash.c
        # src/misc/bk_adapter.c
        src/misc/tuya_dma.c
        src/misc/tkl_media_buf.c
        src/misc/tkl_osd_engine.c
        src/misc/tkl_pp.c
        src/misc/tkl_gpu.c
        src/misc/tkl_venc.c
        src/misc/tkl_vdec.c
        src/misc/tkl_codec_gate.c
        src/misc/tkl_aud_capture.c
        src/misc/tkl_mftest.c
        src/misc/mf_test/mt_mipi.c
        src/misc/mf_test/mt_adc.c
        # src/misc/mftest/mt_audio.c

        src/system/tkl_fs.c
        src/system/tkl_ota.c
        src/system/tkl_sleep.c
        src/driver/tkl_wakeup.c
        # src/driver/tkl_asr.c
        # src/misc/ty_vad_app.c

        include/init/src/tkl_init_common.c
        include/init/src/tkl_init_network.c
        include/init/src/tkl_init_wifi.c
        include/init/src/tkl_init_wired.c
	include/init/src/tkl_init_cellular.c
        include/utilities/src/tuya_hashmap.c
        include/utilities/src/tuya_ringbuf.c
        include/utilities/src/tuya_smartpointer.c
        include/utilities/src/tuya_queue.c
        include/utilities/src/tuya_mem_heap.c
        include/utilities/src/tuya_list.c
        include/utilities/src/tuya_tools.c
    )

    # ------------------------------------------------------------------
    # tal_audio_subsys: prebuilt AEC+NS / VAD / KWS algorithm library.
    #
    # lib/libtal_audio_subsys.a is built for Cortex-M55 hard-float with the TAL
    # OS backend (tal_malloc / tal_log / tkl_system_get_millisecond), which is
    # what libtuyaos.a already provides on this core. It is a static C++ archive
    # with .dtcm_noinit arrays, so it needs no special linker script handling --
    # the AP linker script already places that section.
    #
    # The include path and the TKL_AF_WITH_* switches below are added together:
    # without the archive the sources compile their passthrough / stub paths.
    # ------------------------------------------------------------------
    set(TAL_AUDIO_SUBSYS_DIR ${CMAKE_CURRENT_LIST_DIR}/prebuilt/tal_audio_subsys)
    list(APPEND incs ${TAL_AUDIO_SUBSYS_DIR}/include)

    # TKL security backend implemented on mbedTLS. AP core only: libtuyaos.a
    # (which carries the mbedTLS implementation) is linked for CONFIG_CPU_INDEX
    # == 1, see the block at the end of this file. Each file is an empty
    # translation unit unless the matching ENABLE_PLATFORM_* option is set in
    # tuya_iot_config.h.
    list(APPEND srcs
        src/security/tkl_hash.c
        src/security/tkl_symmetry.c
        src/security/tkl_asymmetrical.c
    )

    # ------------------------------------------------------------------
    # Beken DUBHE (TrustEngine) hardware crypto backend.
    #
    # Required whenever CONFIG_TRUSTENGINE is set: driver.c calls
    # dubhe_driver_init() at boot and otp_ll.h reads dubhe_inited, both defined in
    # bk_trustengine/dubhe_driver/src/common/dubhe_driver.c. bk_trustengine is not a
    # component of its own, and psa_mbedtls only compiles it behind
    # CONFIG_PSA_MBEDTLS -- which must stay OFF here, because it would also build a
    # second mbedtls (3.6.6) that collides with the 3.6.5 inside libtuyaos.a. So the
    # backend is built as part of this component instead.
    #
    # The mbedTLS ALT route is deliberately NOT used. dubhe_alt/src/*.c and the
    # former forked copies under src/security/alt/ override the mbedTLS entry
    # points inside libtuyaos.a, but they treat an mbedtls_sha256_context as an
    # arm_ce_hash_context_t. lib_tls 3.6.5 ships no ALT headers, so that context
    # is 108 bytes while the engine's is 136: mode/finish_flag/prv land past the
    # end of the allocation, arm_ce_hash_finish() then reads finish_flag == 0
    # and returns 0 without writing the digest, and dubhe_zeroize() corrupts the
    # neighbouring heap block. Hardware hashing is done in
    # src/security/tkl_hash.c instead, calling arm_ce_hash_* with a context
    # allocated at the engine's own size.
    #
    # dubhe_lib/** is deliberately NOT built: it replaces mbedtls sources (bignum.c,
    # rsa.c, ecp.c, cipher.c, ctr_drbg.c, ...) and would redefine symbols already
    # provided by libtuyaos.a.
    # ------------------------------------------------------------------
    if (CONFIG_TRUSTENGINE)
        set(TE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../t9_os/ap/components/bk_trustengine)
        set(AP_DIR ${CMAKE_CURRENT_LIST_DIR}/../../t9_os/ap)

        list(APPEND incs
            ${TE_DIR}
            ${TE_DIR}/dubhe_alt/inc
            ${TE_DIR}/dubhe_driver/inc
            ${TE_DIR}/dubhe_driver/inc/crypto
            ${TE_DIR}/dubhe_driver/inc/common
            ${TE_DIR}/spe/inc
            ${TE_DIR}/spe/inc/pal
            ${TE_DIR}/spe/inc/hal
            # spe/src/misc.c includes tfm_core_utils.h (AP-side shim) and
            # dubhe_alt/src/{gcm,ccm}.c include mbedtls' internal common.h;
            # both live in armino's psa_mbedtls.
            ${AP_DIR}/components/psa_mbedtls/mbedtls_port/inc
            ${AP_DIR}/components/psa_mbedtls/mbedtls/library
        )

        list(APPEND srcs
            ${TE_DIR}/spe/src/misc.c

            ${TE_DIR}/dubhe_driver/src/common/dubhe_driver.c
            ${TE_DIR}/dubhe_driver/src/common/dubhe_event.c
            ${TE_DIR}/dubhe_driver/src/common/dubhe_intr_handler.c
            ${TE_DIR}/dubhe_driver/src/common/dubhe_sram_alloc.c

            ${TE_DIR}/dubhe_driver/src/crypto/sca/dubhe_sca.c
            ${TE_DIR}/dubhe_driver/src/crypto/hash/dubhe_hash.c
            ${TE_DIR}/dubhe_driver/src/crypto/aead/dubhe_gcm.c
            ${TE_DIR}/dubhe_driver/src/crypto/aead/dubhe_ccm.c
            ${TE_DIR}/dubhe_driver/src/crypto/mac/dubhe_cmac.c
            ${TE_DIR}/dubhe_driver/src/crypto/mac/dubhe_cbcmac.c
            ${TE_DIR}/dubhe_driver/src/crypto/mac/dubhe_ghash.c
            ${TE_DIR}/dubhe_driver/src/crypto/trng/dubhe_trng.c
            ${TE_DIR}/dubhe_driver/src/crypto/otp/dubhe_otp.c
            ${TE_DIR}/dubhe_driver/src/crypto/aca/dubhe_aca.c
            ${TE_DIR}/dubhe_driver/src/crypto/aca/dubhe_aca_hw.c

            ${TE_DIR}/dubhe_driver/src/hal_pal/pal_heap.c
            ${TE_DIR}/dubhe_driver/src/hal_pal/pal_log.c
            ${TE_DIR}/dubhe_driver/src/hal_pal/pal_mutex.c
            ${TE_DIR}/dubhe_driver/src/hal_pal/pal_preempt.c
            ${TE_DIR}/dubhe_driver/src/hal_pal/pal_semaphore.c
            ${TE_DIR}/dubhe_driver/src/hal_pal/pal_signal.c
            ${TE_DIR}/dubhe_driver/src/hal_pal/pal_string.c
            ${TE_DIR}/dubhe_driver/src/hal_pal/pal_time.c

            # mbedTLS ALT implementations. They provide the mbedtls_* symbols
            # that the TLS library no longer exports once MBEDTLS_*_ALT is set
            # in its own build (see the target_compile_definitions block below).
            # For this to be ABI-safe the TLS library MUST be compiled against
            # the same alt headers in ${TE_DIR}/dubhe_alt/inc: lib_tls 3.6.5
            # ships no alt headers, so a prebuilt library bakes in a 108-byte
            # mbedtls_sha256_context while arm_ce_hash_context_t is 136 bytes,
            # and the engine state lands past the end of the allocation.
            ${CMAKE_CURRENT_LIST_DIR}/src/security/alt/aes.c
            ${CMAKE_CURRENT_LIST_DIR}/src/security/alt/sha1.c
            ${CMAKE_CURRENT_LIST_DIR}/src/security/alt/sha256.c
            ${CMAKE_CURRENT_LIST_DIR}/src/security/alt/gcm.c
            ${CMAKE_CURRENT_LIST_DIR}/src/security/alt/ccm.c
        )
    endif()

    if (CONFIG_TUYA_TEST_CLI)
        if (CONFIG_LVGL AND CONFIG_LVGL_V9)
            list(APPEND srcs
                src/test/test_lvgl/tuya_lvgl_widgets_demo.c
                src/test/test_lvgl/lvgl_stress_demo.c
                src/test/test_lvgl/lv_demo_widgets.c
                src/test/test_lvgl/assets/img_clothes.c
                src/test/test_lvgl/assets/img_demo_widgets_avatar.c
                src/test/test_lvgl/assets/img_demo_widgets_needle.c
                src/test/test_lvgl/assets/img_lvgl_logo.c
            )
        endif()

        list(APPEND incs
            src/test/test_media
            src/test/test_media/osd_assets
            # src/test/coremark
            # src/test/coremark/posix
        )
        list(APPEND srcs
            src/test/test_adc.c
            # src/test/test_audio.c
            # src/test/test_lcd.c
            src/test/test_pwm.c
            # src/test/test_qspi.c
            # src/test/test_mp3.c
            src/test/test_timer.c
            src/test/test_sleep.c
            # src/test/test_usb.c
            # src/test/test_spi.c
            src/test/test_ringbuffer/rb.c
            src/test/test_media/test_mipi_dsi.c
            src/test/test_media/test_mipi.c
            src/test/test_media/osd_assets/bk_font.c
            src/test/test_media/osd_assets/bk_img.c
            src/test/test_media/test_osd.c
            src/test/test_media/test_overlay.c
            src/test/test_media/test_record.c
            src/test/test_media/test_image.c
            src/test/test_media/test_camera_mipi.c
            src/test/test_media/test_uvc.c
            src/test/test_media/test_adc.c
            src/test/test_media/test_dac.c
            src/test/test_media/test_dmic.c
            # src/test/test_media/test_pwm.c
            # src/test/test_media.c
            # src/test/test_mic.c
            # src/test/test_speaker.c
            src/test/test_i2s.c
            # src/test/test_mtd.c
            src/test/test_i2c.c
            # src/test/test_mtd/qspi_flash_GD5F1G.c
            # src/test/test_media/test_media_audio.c
            # src/test/test_media/test_media_camera.c
            # src/test/test_media/test_media_h264.c
            # src/test/test_media/test_media_lcd.c
            # src/test/test_eth.c
            # src/test/test_iperf.c
            # src/test/test_mem.c
            # src/test/test_dwt.c
            src/test/test_wifi.c
            # src/test/test_dvp/dvp_gc2145.c
            # src/test/test_dvp/test_dvp.c
            # src/test/coremark/core_list_join.c
            # src/test/coremark/core_main.c
            # src/test/coremark/core_matrix.c
            # src/test/coremark/core_state.c
            # src/test/coremark/core_util.c
            # src/test/coremark/posix/core_portme.c
            src/test/test_common.c
            src/test/test_cpu.c
            # src/test/test_lp.c
            src/test/test_gpio.c
            src/test/test_sdcard.c
            src/test/test_memory.c
            src/test/test_ota.c
            # src/test/test_tkl_security.c
            # src/test/test_mbedtls.c
            # src/test/test_plat.c
            # src/test/test_face_recognize/AvdkDetectionModel.cc
            # src/test/test_face_recognize/box.c
            # src/test/test_face_recognize/YolofaceDetectionModel.cc
            # src/test/test_face_recognize/yoloface_detect_model_data.cc
            # src/test/test_face_recognize/test_face_recognize.cc
            # src/test/test_face_recog/AvdkMultiModel.cc
            # src/test/test_face_recog/FaceDetectionRuntime.cc
            # src/test/test_face_recog/FaceVerifyRuntime.cc
            # src/test/test_face_recog/FaceRecognitionModel.cc
            # src/test/test_face_recog/face_gallery.cc
            # src/test/test_face_recog/test_face_recog.cc
            # src/test/test_littlefs.c
            src/test/cli_tuya_test_1.c
            src/test/test_i2c.c
        )
    endif() # CONFIG_TUYA_TEST_CLI

endif()



function(FIND_INCLUDE_DIR_WITH_IGNORE result curdir)
    # 初始化输出目录列表
    set(dirs "")

    # 递归查找所有子目录
    file(GLOB_RECURSE allFiles "${curdir}/*")

    # 通过循环所有文件来构建目录列表
    foreach(file ${allFiles})
        get_filename_component(dirPath ${file} DIRECTORY)
        # 如果这个目录还没有在列表中，就加入它
        if(NOT "${dirPath}" IN_LIST dirs)
            list(APPEND dirs ${dirPath})
        endif()
    endforeach()

    # 移除重复的目录，以防万一
    list(REMOVE_DUPLICATES dirs)

    # 处理ignoreDirs参数
    set(ignoreDirs ${ARGN})
    set(filteredDirs "")

    # 遍历所有目录并过滤掉忽略的目录及其子目录
    foreach(dir ${dirs})
        set(shouldIgnore FALSE)
        foreach(ignoreDir ${ignoreDirs})
            # 如果目录是忽略目录或其子目录，则标记为忽略
            if("${dir}" MATCHES "^${ignoreDir}")
                set(shouldIgnore TRUE)
            endif()
        endforeach()
        # 如果目录没有被忽略，则加入到最终的目录列表
        if(NOT shouldIgnore)
            list(APPEND filteredDirs ${dir})
        endif()
    endforeach()

    # 设置函数的返回值
    set(${result} ${filteredDirs} PARENT_SCOPE)
endfunction()

if(NOT (DEFINED ENV{TUYA_HEADER_DIR} AND NOT "$ENV{TUYA_HEADER_DIR}" STREQUAL ""))
    # TuyaOS: 查找 include 目录下的所有 *.h 头文件
    FIND_INCLUDE_DIR_WITH_IGNORE("INCLUDE_DIR_LIST_SDK" "$ENV{TUYA_PROJECT_DIR}/include")
    list(APPEND incs "${INCLUDE_DIR_LIST_SDK}")

    set(ext_include_path $ENV{TUYA_PROJECT_DIR}/include)
    execute_process(
        COMMAND sh -c "find ${ext_include_path} -name lib_tls -type d"
        OUTPUT_VARIABLE OUTPUT_VAR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    list(APPEND incs "${OUTPUT_VAR}/include")
endif()

set(lwip_select $ENV{TUYA_LWIP_STACK_USED})
if (lwip_select STREQUAL "lwip_bk")
    message(STATUS "--------------- bk lwip ---------------")
else ()
    message(STATUS "--------------- tuya lwip ---------------")
endif()

if ("$ENV{ARMINO_SOC}" STREQUAL "bk7259")
    set(priv_requires_list bk_common bk_log bk_cli lwip_intf_v2_1 os_source)
elseif ("$ENV{ARMINO_SOC}" STREQUAL "bk7259_ap")
    set(priv_requires_list bk_common bk_modem bk_log bk_cli lwip_intf_v2_1 os_source lvgl bk_vfs media_service bk_usb audio_record audio_play bk_tflite_micro avdk_utils bk_peripheral bk_netif bk_encoder bk_gpu bk_draw_osd)
else()
    message(FATAL_ERROR soc:"$ENV{ARMINO_SOC}" is not valid!)
endif() # CONFIG_CPU_INDEX

armino_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${incs}
    PRIV_REQUIRES ${priv_requires_list}
)

# Stub .c files (e.g. tkl_memory.c) return OPRT_NOT_SUPPORTED from VOID_T* APIs; GCC 14 -Werror treats
# that as -Wint-conversion. Downgrade to warning for this component only (no source edits).
if(TARGET "${COMPONENT_LIB}")
    target_compile_options("${COMPONENT_LIB}" PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wno-error=int-conversion>
        $<$<COMPILE_LANGUAGE:C>:-Wno-error=incompatible-pointer-types>
        $<$<COMPILE_LANGUAGE:C>:-Wno-error=implicit-function-declaration>
    )

    # Board audio geometry from the T9 Kconfig. The adapter sources do not
    # include tuya_kconfig.h, so these have to arrive as -D; the #ifndef
    # defaults in tkl_audio_frontend.c only cover a missing setting.
    foreach(_af_var MIC0_LANE MIC1_LANE REF_LANE BF_AZIMUTH_DEG BF_MIC_SPACING_MM)
        if(DEFINED CONFIG_TKL_AF_${_af_var} AND NOT "${CONFIG_TKL_AF_${_af_var}}" STREQUAL "")
            target_compile_definitions("${COMPONENT_LIB}" PRIVATE
                TKL_AF_${_af_var}=${CONFIG_TKL_AF_${_af_var}})
        endif()
    endforeach()

    # tal_audio_subsys is linked on the AP core only (see the srcs block above).
    # The feature switches mirror it so the sources and the archive never drift:
    # without the library the front end is a mic0 passthrough and VAD/KWS are
    # OPRT_NOT_SUPPORTED stubs.
    if ("${CONFIG_CPU_INDEX}" STREQUAL "1")
        if(EXISTS "${TAL_AUDIO_SUBSYS_DIR}/lib/libtal_audio_subsys.a")
            add_prebuilt_library(tal_audio_subsys "${TAL_AUDIO_SUBSYS_DIR}/lib/libtal_audio_subsys.a")
            target_link_libraries("${COMPONENT_LIB}" INTERFACE tal_audio_subsys)
            target_compile_definitions("${COMPONENT_LIB}" PRIVATE
                TKL_AF_WITH_TAL_LIB=1
                TKL_AF_WITH_VAD=1
                TKL_AF_WITH_KWS=1
            )
            message(STATUS "--------------- tal_audio_subsys: linked ---------------")
        else()
            message(STATUS "--------------- tal_audio_subsys: not available, AEC/VAD/KWS off ---------------")
        endif()
    endif()

    if (CONFIG_TRUSTENGINE)
        # DUBHE runtime window. Mirrors hw_defs in psa_mbedtls/hw.cmake, minus
        # MBEDTLS_LOG_LEVEL_ID (not referenced by any bk_trustengine source).
        # DUBHE_SECURE is appended separately below, gated on CONFIG_SPE.
        target_compile_definitions("${COMPONENT_LIB}" PRIVATE
            CRYPTO_HW_ACCELERATOR
            CRYPTO_HW_ACCELERATOR_OTP_PROVISIONING
            ARM_CE_DUBHE
            ARM_CE_DUBHE_SCA
            ARM_CE_DUBHE_TRNG
            ARM_CE_DUBHE_OTP
            ARM_CE_DUBHE_HASH
            ARM_CE_DUBHE_ACA
            DUBHE_FOR_RUNTIME
        )

        # mbedTLS ALT switch. It has to be flipped on BOTH sides at the same
        # time: the TLS library must be rebuilt with the same five macros, and
        # this component provides the matching mbedtls_* entry points from
        # src/security/alt/. Turning it on here against the stock, ALT-off
        # libtuyaos.a is an ABI mismatch -- that archive keeps allocating its
        # own software context (108-byte mbedtls_sha256_context, 276-byte
        # mbedtls_aes_context) while the alt code casts it to the engine's
        # 136-byte arm_ce_hash_context_t. Off until the rebuilt library lands.
        #
        # Only modules DUBHE actually accelerates are listed: no MD5 (no alt
        # header, no engine), no SHA-512, no CMAC, no RSA/ECP.
        #
        # 2026-10-22: libs/libtuyaos.a in this tree was rebuilt by Tuya with
        # these five macros (nm shows no T mbedtls_aes_init/sha1/sha256/gcm/ccm,
        # only U) and its DWARF has arm_ce_hash_context=136,
        # arm_ce_sca_context=128, i.e. it was compiled against the alt headers
        # under dubhe_alt/inc. The ABI therefore matches and the switch is ON.
        # Set it back to OFF if this tree is pointed at a stock software lib.
        #
        # TuyaOpen: this tree is exactly that case. TLS comes from the component
        # layered on top (src/libtls, mbedTLS 3.1.0, software only) and the
        # vendor libtuyaos.a here is the stock ALT-off build, so the downstream
        # default is OFF. CONFIG_TRUSTENGINE is off as well, which already keeps
        # everything below out of the build; this default is the second guard.
        option(TUYA_TLS_LIB_ALT "TLS library was built with the DUBHE MBEDTLS_*_ALT macros" OFF)
        if (TUYA_TLS_LIB_ALT)
            target_compile_definitions("${COMPONENT_LIB}" PRIVATE
                MBEDTLS_AES_ALT
                # Beken's aes_alt implementation only emits dubhe_aes_setkey_*
                # when the fine-grained switch is set (they come from
                # mbedtls_accelerator_config.h in Beken's own build). Tuya's
                # 3.6.5 aes.h ignores both, so they are local to this component.
                MBEDTLS_AES_SETKEY_ENC_ALT
                MBEDTLS_AES_SETKEY_DEC_ALT
                MBEDTLS_GCM_ALT
                MBEDTLS_CCM_ALT
                MBEDTLS_SHA1_ALT
                MBEDTLS_SHA256_ALT
            )
        endif()

        # Mirrors hw.cmake ("SPE=1: Secure TE window + local PM. SPE=0: Normal
        # TE window, SPE owns power."): with CONFIG_SPE set there is no separate
        # TF-M secure world, so the core that drives the engine owns the secure
        # TE window and does its own PM. The AP is that owner here: because
        # PSA_MBEDTLS stays disabled on the AP its own hw.cmake never runs, so
        # the macro has to be injected explicitly. The engine is single-instance
        # (one register window, one IRQ source, one descriptor region), so only
        # one core may own it -- which is why the CP keeps TRUSTENGINE off.
        if (CONFIG_SPE)
            target_compile_definitions("${COMPONENT_LIB}" PRIVATE DUBHE_SECURE)
        endif()

        # Route mbedtls' software big-number exponentiation -- what RSA uses
        # inside libtuyaos.a -- to the __wrap_* shims in
        # src/security/tkl_asymmetrical.c.
        #
        # libtuyaos.a is linked *before* this component (see the library order
        # in build.ninja: ... libtuyaos.a libtuyaos_adapter.a ...), so a plain
        # duplicate definition of mbedtls_mpi_exp_mod() would either lose to the
        # archive copy or raise a multiple-definition error. --wrap instead
        # redirects every reference -- including the ones made from inside
        # libtuyaos.a -- and leaves the original definition in place for
        # __real_* to call. It is the same mechanism the build already uses for
        # malloc/free/printf.
        #
        # Both entry points are needed: rsa.c.o calls
        # mbedtls_mpi_exp_mod_unsafe from mbedtls_rsa_public() and
        # mbedtls_mpi_exp_mod from mbedtls_rsa_private().
        #
        # The EC triple covers ECDHE and ECDSA. mbedtls_ecp_mul_restartable is
        # called directly by ecdh.c.o; mbedtls_ecp_muladd_restartable is the
        # only wrap-able entry for ECDSA, because mbedtls_ecp_muladd_restartable
        # reaches the point multiplication through the static
        # mbedtls_ecp_mul_shortcuts() -> ecp_mul_restartable_internal(), a local
        # symbol the linker cannot redirect. mbedtls_ecp_mul needs its own
        # entry: it calls mbedtls_ecp_mul_restartable from inside the same
        # ecp.c.o, and --wrap does not redirect intra-object references.
        target_link_libraries("${COMPONENT_LIB}" INTERFACE
            -Wl,--wrap=mbedtls_mpi_exp_mod
            -Wl,--wrap=mbedtls_mpi_exp_mod_unsafe
            -Wl,--wrap=mbedtls_ecp_mul
            -Wl,--wrap=mbedtls_ecp_mul_restartable
            -Wl,--wrap=mbedtls_ecp_muladd_restartable
        )
    endif()
endif()

if ("${CONFIG_CPU_INDEX}" STREQUAL "1")
    if(DEFINED ENV{TUYA_LIBS} AND NOT "$ENV{TUYA_LIBS}" STREQUAL "")
        set(tuya_libs "$ENV{TUYA_LIBS}")
        string(REPLACE " " ";" tuya_libs "${tuya_libs}")
        foreach(lib ${tuya_libs})
            set(lib_name "lib${lib}.a")
            add_prebuilt_library(${lib_name} "$ENV{TUYA_LIBS_DIR}/${lib_name}")
            target_link_libraries(${COMPONENT_LIB} INTERFACE ${lib_name})
        endforeach()
    else()
        file(GLOB LIB_FILES "$ENV{TUYA_PROJECT_DIR}/libs/*.a")
        target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIB_FILES})
    endif()
endif()

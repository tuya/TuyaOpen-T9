set(incs
    include
    src/test
    src/misc
    src/test/test_lvgl
    src/test/test_ringbuffer
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
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/gpu
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/vcodec
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/jpeg_codec
    $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/tal_mtd/include
    $ENV{TUYA_PROJECT_DIR}/include/base/include
    # $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/asr
    # $ENV{ARMINO_PATH}/../../tuyaos/tuyaos_adapter/include/vad
)

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
    # src/driver/tkl_hci.c
    src/driver/tkl_ipc.c
    src/misc/tuya_tkl_ipc.c
    # src/misc/tkl_mftest.c
    include/init/src/tkl_init_system.c
)

if ("${CONFIG_CPU_INDEX}" STREQUAL "0")
    # tkl_init_system.c always references tkl_ota_*; tkl_ota.c provides CPU0 stubs under #elif CONFIG_CPU_INDEX == 0
    list(APPEND srcs src/system/tkl_ota.c)
    list(APPEND srcs src/driver/tkl_hci_0.c)
    if (CONFIG_TUYA_TEST_CLI)
        list(APPEND srcs src/test/cli_tuya_test_0.c)
    endif() # CONFIG_TUYA_TEST_CLI
else()
    list(APPEND srcs
        src/driver/tkl_wifi.c
        src/driver/tkl_wired.c
        src/driver/tkl_hci_1.c
        # src/driver/tkl_video_enc.c
        # src/driver/tkl_video_in.c
        # src/driver/tkl_mic.c
        # src/driver/tkl_speaker.c
        # src/driver/tkl_audio.c
        # src/driver/tkl_cellular.c
        # src/driver/tkl_pinmux.c
        # src/driver/tkl_rgb.c
        # src/driver/tkl_8080.c
        src/driver/tkl_adc.c
        src/driver/tkl_pwm.c
        src/driver/tkl_uart.c
        src/driver/tkl_flash.c
        # src/driver/tkl_rtc.c
        src/driver/tkl_spi.c
        src/driver/tkl_qspi.c
        src/driver/tkl_gpio.c
        src/driver/tkl_i2c.c
        src/driver/tkl_timer.c
        src/driver/tkl_mipi_dsi.c
        src/driver/tkl_camera_mipi.c
        src/driver/tkl_gpu.c
        src/driver/tkl_venc.c
        src/driver/tkl_vdec.c
        # src/driver/tkl_wired.c
        # src/driver/tkl_i2s.c
        # src/driver/tkl_dma2d.c
        # src/driver/tkl_dvp.c
        # src/driver/tkl_jpeg_codec.c
        # src/driver/tkl_sdio.c

        src/misc/encrypt.c
        # src/misc/fs_init.c
        # src/misc/dhcpserver.c
        # src/misc/qflash.c
        # src/misc/sflash.c
        # src/misc/bk_adapter.c
        # src/misc/tuya_dma.c
        src/misc/tkl_mftest.c
        # src/misc/mftest/mt_audio.c

        # src/system/tkl_fs.c
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

    if (CONFIG_TUYA_TEST_CLI)
        list(APPEND incs
            # src/test/test_media
            # src/test/coremark
            # src/test/coremark/posix
        )
        list(APPEND srcs
            # src/test/test_adc.c
            # src/test/test_audio.c
            # src/test/test_lcd.c
            # src/test/test_pwm.c
            # src/test/test_qspi.c
            # src/test/test_mp3.c
            # src/test/test_timer.c
            # src/test/test_usb.c
            # src/test/test_spi.c
            src/test/test_ringbuffer/rb.c
            src/test/test_media/test_mipi.c
            src/test/test_media/test_gpu.c
            src/test/test_media/test_venc.c
            # src/test/test_media.c
            # src/test/test_mic.c
            # src/test/test_speaker.c
            # src/test/test_i2s.c
            # src/test/test_mtd.c
            # src/test/test_i2s.c
            # src/test/test_mtd/qspi_flash_GD5F1G.c
            # src/test/test_media/test_media_audio.c
            # src/test/test_media/test_media_camera.c
            # src/test/test_media/test_media_h264.c
            # src/test/test_media/test_media_lcd.c
            # src/test/test_eth.c
            # src/test/test_iperf.c
            # src/test/test_mem.c
            # src/test/test_dwt.c
            # src/test/test_wifi.c
            # src/test/test_dvp/dvp_gc2145.c
            # src/test/test_dvp/test_dvp.c
            # src/test/coremark/core_list_join.c
            # src/test/coremark/core_main.c
            # src/test/coremark/core_matrix.c
            # src/test/coremark/core_state.c
            # src/test/coremark/core_util.c
            # src/test/coremark/posix/core_portme.c
            # src/test/test_common.c
            # src/test/test_lp.c
            # src/test/test_gpio.c
            # src/test/test_sdcard.c
            # src/test/test_littlefs.c
            src/test/cli_tuya_test_1.c
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

#查找include目录下的所有*.h头文件
FIND_INCLUDE_DIR_WITH_IGNORE("INCLUDE_DIR_LIST_SDK" "$ENV{TUYA_PROJECT_DIR}/include")
list(APPEND incs "${INCLUDE_DIR_LIST_SDK}")

# add mbedtls dir
set(ext_include_path $ENV{TUYA_PROJECT_DIR}/include)
execute_process(
    COMMAND sh -c "find ${ext_include_path} -name lib_tls -type d"
    OUTPUT_VARIABLE OUTPUT_VAR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
#message("Output from shell command: ${OUTPUT_VAR}")
list(APPEND incs "${OUTPUT_VAR}/include")

set(lwip_select $ENV{TUYA_LWIP_STACK_USED})
if (lwip_select STREQUAL "lwip_bk")
    message(STATUS "--------------- bk lwip ---------------")
else ()
    message(STATUS "--------------- tuya lwip ---------------")
endif()

if ("$ENV{ARMINO_SOC}" STREQUAL "bk7259")
    set(priv_requires_list bk_common bk_log bk_cli lwip_intf_v2_1 os_source)
elseif ("$ENV{ARMINO_SOC}" STREQUAL "bk7259_ap")
    set(priv_requires_list bk_common bk_modem bk_log bk_cli lwip_intf_v2_1 os_source lvgl bk_vfs multimedia media_service bk_draw_osd bk_usb audio_record audio_play)
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
endif()

if ("${CONFIG_CPU_INDEX}" STREQUAL "1")
# 搜索指定目录下所有.a文件
file(GLOB LIB_FILES "$ENV{TUYA_PROJECT_DIR}/libs/*.a")
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIB_FILES})

endif()

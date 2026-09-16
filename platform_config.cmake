list_subdirectories(PLATFORM_PUBINC_1 ${PLATFORM_PATH}/tuyaos/tuyaos_adapter)

set(PLATFORM_PUBINC_2
    ${PLATFORM_PATH}/t9_os/ap/include
    ${PLATFORM_PATH}/t9_os/ap/include/soc/bk7259
    ${PLATFORM_PATH}/t9_os/build/bk7259/tuya_app/bk7259/config
    ${PLATFORM_PATH}/t9_os/build/bk7259/tuya_app/bk7259_ap/config
    ${PLATFORM_PATH}/t9_os/ap/components/bk_libs/bk7259_ap/config
    ${PLATFORM_PATH}/t9_os/ap/components/lwip_intf_v2_1/lwip-2.1.2
)

set(PLATFORM_PUBINC
    ${PLATFORM_PUBINC_1}
    ${PLATFORM_PUBINC_2})

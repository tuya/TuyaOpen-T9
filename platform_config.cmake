list_subdirectories(PLATFORM_PUBINC_1 ${PLATFORM_PATH}/tuyaos/tuyaos_adapter)

# Core/P2P compile view: self-contained lwipopts.h and arch/cc.h for root CMake
# translation units (tuyaopen-core / P2P). Placing it ahead of PLATFORM_PUBINC_2
# avoids clean-build failures from generated sdkconfig.h / bk_uart.h while
# keeping the t9_os lwIP build on its own native configuration.
set(PLATFORM_CORE_LWIP_VIEW "${CMAKE_CURRENT_LIST_DIR}/tuyaos/tuyaos_adapter/include/lwip_view")
list(INSERT PLATFORM_PUBINC_1 0 "${PLATFORM_CORE_LWIP_VIEW}")

# POSIX-name shims over newlib/lwIP used by the P2P/library sources:
# <netinet/in.h>, <sys/un.h>. lwIP's own compat/posix supplies <arpa/inet.h>,
# <netdb.h>, <net/if.h>, and <sys/socket.h>.
list(INSERT PLATFORM_PUBINC_1 0
    "${CMAKE_CURRENT_LIST_DIR}/tuyaos/tuyaos_adapter/include/posix_compat")

set(PLATFORM_PUBINC_2
    ${PLATFORM_PATH}/t9_os/ap/include
    ${PLATFORM_PATH}/t9_os/ap/include/soc/bk7259
    ${PLATFORM_PATH}/t9_os/ap/middleware/driver/include/bk_private
    ${PLATFORM_PATH}/t9_os/build/bk7259/tuya_app/bk7259/config
    ${PLATFORM_PATH}/t9_os/build/bk7259/tuya_app/bk7259_ap/config
    ${PLATFORM_PATH}/t9_os/ap/components/bk_libs/bk7259_ap/config
    ${PLATFORM_PATH}/t9_os/ap/components/lwip_intf_v2_1/lwip-2.1.2/port
    ${PLATFORM_PATH}/t9_os/ap/components/lwip_intf_v2_1/lwip-2.1.2/src/include
    ${PLATFORM_PATH}/t9_os/ap/components/lwip_intf_v2_1/lwip-2.1.2/src/include/compat/posix
)

# This platform implements the TKL file system API used by core's FS ops card.
add_definitions(-DTUYAOPEN_HAS_TKL_FS=1)

set(PLATFORM_PUBINC
    ${PLATFORM_PUBINC_1}
    ${PLATFORM_PUBINC_2})

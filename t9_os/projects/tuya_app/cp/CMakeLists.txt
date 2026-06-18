set(incs .)
set(srcs cp_main.c customer_msg.c)

if (CONFIG_VND_CAL)
if (CONFIG_OVERRIDE_VND_CAL)
    list(APPEND srcs
        vnd_cal.c
    )
endif()
endif()

list(APPEND incs
    $ENV{ARMINO_PATH}/../../../../adapter/utilities/include
    $ENV{ARMINO_PATH}/../../../../include/base/include
)

set(prev_req controller_if lwip_intf_v2_1 bt)

armino_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "${incs}"
    PRIV_REQUIRES "${prev_req}"
    REQUIRES tuyaos_adapter
)

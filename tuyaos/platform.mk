TUYA_PLATFORM_DIR := $(dir $(lastword $(MAKEFILE_LIST)))/..


# kernel include dir
TUYA_PLATFORM_CFLAGS += -I$(TUYA_PLATFORM_DIR)/tuyaos/tuyaos_adapter/include
TUYA_PLATFORM_CFLAGS += -I$(TUYA_PLATFORM_DIR)/tuyaos/tuyaos_adapter/src/camera
TUYA_PLATFORM_CFLAGS += -I$(TUYA_PLATFORM_DIR)/t9_os/ap/include

# lwip include dir
ifneq (${CONFIG_ENABLE_LWIP},y)
$(info ------ use bk lwip ------)
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/build/bk7259/tuya_app/bk7259/config
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/build/bk7259/tuya_app/bk7259_ap/config
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/middleware/driver/include/bk_private
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/include
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/include/common
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/components/bk_rtos/include
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/middleware/driver/include/bk_private
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/components/lwip_intf_v2_1/lwip-2.1.2/port
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/components/lwip_intf_v2_1/lwip-2.1.2/src/include
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/components/lwip_intf_v2_1/lwip-2.1.2/src/include/compat
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/components/lwip_intf_v2_1/lwip-2.1.2/src/include/lwip
TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/ap/components/lwip_intf_v2_1/lwip-2.1.2/src/include/netif
else
$(info ------ use tuya lwip ------)
endif

# # TODO: lvgl include dir
# CONFIG_LVGL_V8_VALUE := $(shell cat ${TUYA_PLATFORM_DIR}/t9_os/projects/tuya_app/config/bk7259_cp1/config | sed -n 's/^CONFIG_LVGL_V8=\(.*\)/\1/p')
# CONFIG_LVGL_V9_VALUE := $(shell cat ${TUYA_PLATFORM_DIR}/t9_os/projects/tuya_app/config/bk7259_cp1/config | sed -n 's/^CONFIG_LVGL_V9=\(.*\)/\1/p')

# $(info ------ use bk lvgl ------)

# ifeq (${CONFIG_LVGL_V8_VALUE},y)
# $(info ------ use CONFIG_LVGL_V8 ------)
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/lvgl/lvgl_v8
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/lvgl/lvgl_v8/src
# else ifeq (${CONFIG_LVGL_V9_VALUE},y)
# $(info ------ use CONFIG_LVGL_V9 ------)
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/lvgl/lvgl_v9
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/lvgl/lvgl_v9/src
# else
# $(info ------ no spec lvgl version, default use lvgl v8 ------)
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/lvgl/lvgl_v8
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/lvgl/lvgl_v8/src
# endif

# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/bk_idk/components/bk_vfs
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/multimedia/include
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/multimedia/mailbox
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/bk_idk/components/bk_common/include
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/media_service/include
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/multimedia/pipeline
#eez include
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/eez-framework/src
# TUYA_PLATFORM_CFLAGS += -I $(TUYA_PLATFORM_DIR)/t9_os/components/eez-framework/src/eez/libs/agg

# compile parameters
TUYA_PLATFORM_CFLAGS += -g -Os -std=c99 -nostdlib -Wall -Wno-format -Wno-unknown-pragmas -Wno-address-of-packed-member -ffunction-sections -fsigned-char -fdata-sections -fno-strict-aliasing -ggdb -std=gnu99 -Wno-old-style-declaration -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse -end-group


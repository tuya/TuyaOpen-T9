COMPILER_TOOLCHAIN_PATH := $(CURDIR)/../../../../tools/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi/bin
ifneq ($(TUYA_TOOLCHAIN_PATH),)
	COMPILER_TOOLCHAIN_PATH := $(TUYA_TOOLCHAIN_PATH)/bin
endif

PRE_BUILD_TARGET :=

ifeq ($(SUPPORT_BOOTLOADER),true)
	PRE_BUILD_TARGET += bootloader
endif


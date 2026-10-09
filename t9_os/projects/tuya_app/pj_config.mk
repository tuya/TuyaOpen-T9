COMPILER_TOOLCHAIN_PATH := $(CURDIR)/../../toolchain/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi/bin

PRE_BUILD_TARGET :=

ifeq ($(SUPPORT_BOOTLOADER),true)
	PRE_BUILD_TARGET += bootloader
endif


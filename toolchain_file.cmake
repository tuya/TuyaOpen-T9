set(TOOLCHAIN_DIR "${PLATFORM_PATH}/../tools/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi")
IF(APPLE)
    IF(OPEN_SYSTEM_PROCESSOR MATCHES "arm64")
        set(TOOLCHAIN_DIR "${PLATFORM_PATH}/../tools/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi")
        # solve gcc error: unrecognized command-line option '-arch'
        set(CMAKE_SYSTEM_NAME Linux)
        set(CMAKE_SYSTEM_PROCESSOR ARM)
    ELSE()  # x86_64 — ARM has no 14.3 package for Intel Mac
        set(TOOLCHAIN_DIR "${PLATFORM_PATH}/../tools/arm-gnu-toolchain-14.2.rel1-darwin-x86_64-arm-none-eabi")
    ENDIF()
ELSEIF(WIN32)
    set(TOOLCHAIN_DIR "${PLATFORM_PATH}/../tools/arm-gnu-toolchain-14.3.rel1-mingw-w64-i686-arm-none-eabi")
ELSEIF(OPEN_SYSTEM_PROCESSOR MATCHES "arm64")
    # Linux aarch64: tools/cmake/get_system_processor.py maps aarch64 -> arm64
    set(TOOLCHAIN_DIR "${PLATFORM_PATH}/../tools/arm-gnu-toolchain-14.3.rel1-aarch64-arm-none-eabi")
ENDIF()
set(TOOLCHAIN_PRE "arm-none-eabi-")

IF (WIN32)
    set(CMAKE_AR "${TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PRE}ar.exe")
    set(CMAKE_C_COMPILER "${TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PRE}gcc.exe")
    set(CMAKE_CXX_COMPILER "${TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PRE}g++.exe")
ELSE ()
    set(CMAKE_AR "${TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PRE}ar")
    set(CMAKE_C_COMPILER "${TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PRE}gcc")
    set(CMAKE_CXX_COMPILER "${TOOLCHAIN_DIR}/bin/${TOOLCHAIN_PRE}g++")
ENDIF ()

SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_DIR}/bin)

# Flags from tuyaos/platform.mk (GCC 14 / Cortex-M33). Extra -Wno-error=*
# because GCC 14 treats several TuyaOpen sources as errors that 10.3 did not.
set(CMAKE_C_FLAGS "-g -Os -std=c99 -nostdlib -mthumb -Wall -Wno-format -Wno-unknown-pragmas -Wno-address-of-packed-member -ffunction-sections -fsigned-char -fdata-sections -fno-strict-aliasing -ggdb -std=gnu99 -Wno-old-style-declaration -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse -Wno-error=incompatible-pointer-types -Wno-error=implicit-function-declaration -Wno-error=int-conversion -Wno-error=implicit-int -Wno-error=return-mismatch -end-group")
set(CMAKE_CXX_FLAGS "-g -Os -std=gnu++17 -nostdlib -mthumb -Wno-format -Wno-unknown-pragmas -Wno-address-of-packed-member -Wno-deprecated-declarations -mno-unaligned-access -fno-threadsafe-statics -Wno-unused-function -ffunction-sections -fdata-sections -fno-strict-aliasing -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse -mthumb -fno-exceptions -Wno-error=incompatible-pointer-types -Wno-error=implicit-function-declaration -Wno-error=int-conversion -Wno-error=return-mismatch")

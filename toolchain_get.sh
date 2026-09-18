#!/usr/bin/env bash

set -e

if [ -z "$1" ]; then
    echo "Please input toolchain path"
    exit 1
fi

TOOLCHAIN_PATH=$1

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "$TOP_DIR"

echo "Start board prepare ..."

if [ ! -d "$TOOLCHAIN_PATH" ]; then
    mkdir -p "$TOOLCHAIN_PATH"
    echo "Toolchain $TOOLCHAIN_PATH not exist"
fi

echo "Start download toolchain"

MIRROR=0

function get_country_code()
{
    echo "get_country_code ..."
    if [ -n "$OPEN_COUNTRY_CODE" ]; then
        if [ "$OPEN_COUNTRY_CODE" = "China" ]; then
            MIRROR=1
            echo "enable cn mirror (OPEN_COUNTRY_CODE)"
            return
        fi
        MIRROR=2
        return
    fi

    if command -v python3 &>/dev/null; then
        PYTHON_CMD=python3
    elif command -v python &>/dev/null && python --version | grep -q '^Python 3'; then
        PYTHON_CMD=python
    else
        echo "Python 3 is not installed."
        exit 1
    fi

    MIRROR=$(${PYTHON_CMD} ${TOP_DIR}/tools/get_country.py)
    if [ x"$MIRROR" = x"1" ]; then
        echo "enable cn mirror"
    fi
}

get_country_code

cd "$TOOLCHAIN_PATH"
HOST_MACHINE=$(uname -m)
TUYA_CDN="https://images.tuyacn.com/smart/embed/package/tuyaopen"
ARM_BLOB_143="https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/14.3.rel1/binrel"
ARM_BLOB_142="https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/14.2.rel1/binrel"

case "$(uname -s)" in
Linux*)
    SYSTEM_NAME="Linux"
    if [ "$HOST_MACHINE" = "x86_64" ]; then
        TOOLCHAIN_FILE=arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz
        TOOLCHAIN_NAME=$TOOLCHAIN_PATH/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi
        TOOLCHAIN_SIZE=149789432
        TOOLCHAIN_SHA256=8f6903f8ceb084d9227b9ef991490413014d991874a1e34074443c2a72b14dbd
        ARM_URL=$ARM_BLOB_143/$TOOLCHAIN_FILE
    elif [ "$HOST_MACHINE" = "aarch64" ]; then
        TOOLCHAIN_FILE=arm-gnu-toolchain-14.3.rel1-aarch64-arm-none-eabi.tar.xz
        TOOLCHAIN_NAME=$TOOLCHAIN_PATH/arm-gnu-toolchain-14.3.rel1-aarch64-arm-none-eabi
        TOOLCHAIN_SIZE=148423580
        TOOLCHAIN_SHA256=2d465847eb1d05f876270494f51034de9ace9abe87a4222d079f3360240184d3
        ARM_URL=$ARM_BLOB_143/$TOOLCHAIN_FILE
    else
        echo "Toolchain not support, Please download toolchain from https://developer.arm.com/downloads/-/gnu-a"
        exit 1
    fi
    ;;
Darwin*)
    SYSTEM_NAME="Apple"
    if [ "$HOST_MACHINE" = "x86_64" ]; then
        TOOLCHAIN_FILE=arm-gnu-toolchain-14.2.rel1-darwin-x86_64-arm-none-eabi.tar.xz
        TOOLCHAIN_NAME=$TOOLCHAIN_PATH/arm-gnu-toolchain-14.2.rel1-darwin-x86_64-arm-none-eabi
        TOOLCHAIN_SIZE=145118692
        TOOLCHAIN_SHA256=2d9e717dd4f7751d18936ae1365d25916534105ebcb7583039eff1092b824505
        ARM_URL=$ARM_BLOB_142/$TOOLCHAIN_FILE
    elif [ "$HOST_MACHINE" = "arm64" ]; then
        TOOLCHAIN_FILE=arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi.tar.xz
        TOOLCHAIN_NAME=$TOOLCHAIN_PATH/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi
        TOOLCHAIN_SIZE=137816224
        TOOLCHAIN_SHA256=30f4d08b219190a37cded6aa796f4549504902c53cfc3c7e044a8490b6eba1f7
        ARM_URL=$ARM_BLOB_143/$TOOLCHAIN_FILE
    else
        echo "Toolchain not support, Please download toolchain from https://developer.arm.com/downloads/-/gnu-a"
        exit 1
    fi
    ;;
MINGW* | CYGWIN* | MSYS*)
    SYSTEM_NAME="Windows"
    TOOLCHAIN_FILE=arm-gnu-toolchain-14.3.rel1-mingw-w64-i686-arm-none-eabi.zip
    TOOLCHAIN_NAME=$TOOLCHAIN_PATH/arm-gnu-toolchain-14.3.rel1-mingw-w64-i686-arm-none-eabi
    TOOLCHAIN_SIZE=291670457
    TOOLCHAIN_SHA256=836ebe51fd71b6542dd7884c8fb2011192464b16c28e4b38fddc9350daba5ee8
    ARM_URL=$ARM_BLOB_143/$TOOLCHAIN_FILE
    ;;
*)
    SYSTEM_NAME="Unknown"
    exit 1
    ;;
esac

if [ x"$MIRROR" = x"1" ]; then
    TOOLCHAIN_URL=$TUYA_CDN/$TOOLCHAIN_FILE
    TOOLCHAIN_FALLBACK_URL=$ARM_URL
else
    TOOLCHAIN_URL=$ARM_URL
    TOOLCHAIN_FALLBACK_URL=""
fi

echo "Running on [$SYSTEM_NAME]"

if [ -d "$TOOLCHAIN_NAME" ] && [ -n "$(ls -A "$TOOLCHAIN_NAME" 2>/dev/null)" ]; then
    echo "Toolchain $TOOLCHAIN_NAME check Successful"
    echo "Run board prepare success ..."
    exit 0
fi

MAX_DOWNLOAD_ATTEMPTS=2

cleanup_toolchain_artifacts() {
    rm -rf "$TOOLCHAIN_FILE"
    rm -rf "$TOOLCHAIN_NAME"
}

file_sha256() {
    if command -v sha256sum &>/dev/null; then
        sha256sum "$1" | awk '{print $1}'
    else
        shasum -a 256 "$1" | awk '{print $1}'
    fi
}

file_size() {
    if stat -c %s "$1" &>/dev/null; then
        stat -c %s "$1"
    else
        stat -f %z "$1"
    fi
}

download_toolchain_package() {
    local url=$1
    echo "Downloading $url"
    if command -v wget &>/dev/null; then
        wget "$url" -O "$TOOLCHAIN_FILE" || {
            echo "Download toolchain failed"
            cleanup_toolchain_artifacts
            return 1
        }
    else
        curl -L "$url" -o "$TOOLCHAIN_FILE" || {
            echo "Download toolchain failed"
            cleanup_toolchain_artifacts
            return 1
        }
    fi
    return 0
}

verify_toolchain_package() {
    if [ "$TOOLCHAIN_SHA256" != "$(file_sha256 "$TOOLCHAIN_FILE")" ]; then
        echo "Toolchain file checksum error"
        cleanup_toolchain_artifacts
        return 1
    fi

    if [ "$TOOLCHAIN_SIZE" != "$(file_size "$TOOLCHAIN_FILE")" ]; then
        echo "Toolchain file size error"
        cleanup_toolchain_artifacts
        return 1
    fi

    return 0
}

extract_toolchain_package() {
    FILE_EXTENSION=${TOOLCHAIN_FILE##*.}

    echo "start decompression"
    echo "FILE_EXTENSION: ${FILE_EXTENSION}"

    rm -rf "$TOOLCHAIN_NAME"

    if [ "$FILE_EXTENSION" = "xz" ] || [ "$FILE_EXTENSION" = "bz2" ]; then
        tar -xvf "$TOOLCHAIN_FILE" -C "$TOOLCHAIN_PATH" || {
            echo "Toolchain extract failed"
            cleanup_toolchain_artifacts
            return 1
        }
    elif [ "$FILE_EXTENSION" = "zip" ]; then
        mkdir -p "$TOOLCHAIN_NAME"
        unzip -o "$TOOLCHAIN_FILE" -d "$TOOLCHAIN_NAME" || {
            echo "Toolchain extract failed"
            cleanup_toolchain_artifacts
            return 1
        }
    else
        echo "File not support"
        cleanup_toolchain_artifacts
        return 1
    fi

    return 0
}

try_download() {
    if download_toolchain_package "$TOOLCHAIN_URL"; then
        return 0
    fi
    if [ -n "$TOOLCHAIN_FALLBACK_URL" ]; then
        echo "Retry ARM official mirror"
        download_toolchain_package "$TOOLCHAIN_FALLBACK_URL"
        return $?
    fi
    return 1
}

retry_toolchain_prepare() {
    local attempt=1

    while [ "$attempt" -le "$MAX_DOWNLOAD_ATTEMPTS" ]; do
        if [ -f "$TOOLCHAIN_FILE" ]; then
            if ! verify_toolchain_package; then
                if [ "$attempt" -ge "$MAX_DOWNLOAD_ATTEMPTS" ]; then
                    return 1
                fi
                attempt=$((attempt + 1))
                echo "Retrying toolchain download: ${attempt}/${MAX_DOWNLOAD_ATTEMPTS}"
                continue
            fi
        else
            if ! try_download || ! verify_toolchain_package; then
                if [ "$attempt" -ge "$MAX_DOWNLOAD_ATTEMPTS" ]; then
                    return 1
                fi
                attempt=$((attempt + 1))
                echo "Retrying toolchain download: ${attempt}/${MAX_DOWNLOAD_ATTEMPTS}"
                continue
            fi
        fi

        if extract_toolchain_package; then
            return 0
        fi

        if [ "$attempt" -ge "$MAX_DOWNLOAD_ATTEMPTS" ]; then
            return 1
        fi

        attempt=$((attempt + 1))
        echo "Retrying toolchain download: ${attempt}/${MAX_DOWNLOAD_ATTEMPTS}"
    done

    return 1
}

if ! retry_toolchain_prepare; then
    exit 1
fi

echo "Run board prepare success ..."

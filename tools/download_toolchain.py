#!/usr/bin/env python3
# coding=utf-8

import os
import platform
import urllib.request
import ssl
import zipfile

from tools.util import (
    get_country_code, rm_rf, calc_sha256,
    extract_archive,
)


ARM_BLOB = "https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/{rel}/binrel"

LAST_PROGRESS = 0
MAX_DOWNLOAD_ATTEMPTS = 2


def _host_key():
    system = platform.system().lower()
    machine = platform.machine().lower()
    if system.startswith("win") or system.startswith("cygwin") or system.startswith("msys"):
        return "windows"
    return f"{system}_{machine}"


def _arm_url(rel, name):
    return f"{ARM_BLOB.format(rel=rel)}/{name}"


def get_toolchain_package_info():
    country_code = get_country_code()
    host_key = _host_key()
    print(f"get package from [{country_code} {host_key}]")

    packages = {
        "linux_x86_64": {
            "rel": "14.3.rel1",
            "name": "arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz",
            "url_cn": "https://images.tuyacn.com/rms-static/a0799090-c2fa-11f1-9a8d-736398ab592b-1791451711513.tar.xz?tyName=arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz",
            "size": 149789432,
            "sha256": "8f6903f8ceb084d9227b9ef991490413014d991874a1e34074443c2a72b14dbd",
            "folder": "arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi",
        },
        "linux_aarch64": {
            "rel": "14.3.rel1",
            "name": "arm-gnu-toolchain-14.3.rel1-aarch64-arm-none-eabi.tar.xz",
            "url_cn": "https://images.tuyacn.com/rms-static/a07bb370-c2fa-11f1-82af-d1f3191773d6-1791451711527.tar.xz?tyName=arm-gnu-toolchain-14.3.rel1-aarch64-arm-none-eabi.tar.xz",
            "size": 148423580,
            "sha256": "2d465847eb1d05f876270494f51034de9ace9abe87a4222d079f3360240184d3",
            "folder": "arm-gnu-toolchain-14.3.rel1-aarch64-arm-none-eabi",
        },
        # 14.3 dropped darwin-x86_64; keep 14.2 for both darwin hosts.
        "darwin_x86_64": {
            "rel": "14.2.rel1",
            "name": "arm-gnu-toolchain-14.2.rel1-darwin-x86_64-arm-none-eabi.tar.xz",
            "url_cn": "https://images.tuyacn.com/smart/embed/package/tuyaopen/arm-gnu-toolchain-14.2.rel1-darwin-x86_64-arm-none-eabi.tar.xz",
            "size": 145118692,
            "sha256": "2d9e717dd4f7751d18936ae1365d25916534105ebcb7583039eff1092b824505",
            "folder": "arm-gnu-toolchain-14.2.rel1-darwin-x86_64-arm-none-eabi",
        },
        "darwin_arm64": {
            "rel": "14.2.rel1",
            "name": "arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.tar.xz",
            "url_cn": "https://images.tuyacn.com/smart/embed/package/tuyaopen/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.tar.xz",
            "size": 134812148,
            "sha256": "c7c78ffab9bebfce91d99d3c24da6bf4b81c01e16cf551eb2ff9f25b9e0a3818",
            "folder": "arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi",
        },
        "windows": {
            "rel": "14.3.rel1",
            "name": "arm-gnu-toolchain-14.3.rel1-mingw-w64-i686-arm-none-eabi.zip",
            "url_cn": "https://images.tuyacn.com/rms-static/a07b3e40-c2fa-11f1-9a8d-736398ab592b-1791451711524.zip?tyName=arm-gnu-toolchain-14.3.rel1-mingw-w64-i686-arm-none-eabi.zip",
            "size": 291670457,
            "sha256": "836ebe51fd71b6542dd7884c8fb2011192464b16c28e4b38fddc9350daba5ee8",
            "folder": "arm-gnu-toolchain-14.3.rel1-mingw-w64-i686-arm-none-eabi",
        },
    }

    package_info = packages.get(host_key)
    if not package_info:
        print("##############################")
        print(f"Warning: Toolchain not support [{host_key}]")
        print("Please download toolchain from ->")
        print("https://developer.arm.com/downloads/-/gnu-a")
        print("##############################")
        return {}

    name = package_info["name"]
    arm_url = _arm_url(package_info["rel"], name)
    if country_code == "China":
        package_info["url"] = package_info["url_cn"]
        package_info["fallback_url"] = arm_url
    else:
        package_info["url"] = arm_url
        package_info["fallback_url"] = ""
    return package_info


def _gcc_path(folder_path):
    gcc = os.path.join(folder_path, "bin", "arm-none-eabi-gcc")
    gcc_exe = gcc + ".exe"
    if os.path.isfile(gcc):
        return gcc
    if os.path.isfile(gcc_exe):
        return gcc_exe
    return ""


def _show_progress(block_num, block_size, total_size):
    global LAST_PROGRESS
    downloaded = block_num * block_size
    progress = 0
    if total_size > 0:
        progress = min(100, (downloaded / total_size) * 100)

    progress = int(progress)
    if LAST_PROGRESS != progress:
        print(f"\rprogress: {progress}%", end="")
        LAST_PROGRESS = progress if progress < 100 else 0


def _download_from_url(url, download_file):
    try:
        context = ssl._create_unverified_context()
        with urllib.request.urlopen(url, context=context) as response:
            total_size = int(response.headers.get('Content-Length', 0))
            block_size = 8192
            bytes_downloaded = 0

            with open(download_file, 'wb') as f:
                while True:
                    buffer = response.read(block_size)
                    if not buffer:
                        break

                    f.write(buffer)
                    bytes_downloaded += len(buffer)
                    block_num = bytes_downloaded // block_size

                    _show_progress(block_num, block_size, total_size)
            print()

    except Exception as e:
        print(f"Error: download failed: {str(e)}")
        rm_rf(download_file)
        return False

    return True


def wget_toolchain_package(toolchain_root, package_info) -> bool:
    name = package_info["name"]
    download_file = os.path.join(toolchain_root, name)

    if os.path.exists(download_file):
        print(f"[Toolchain package exists]: {download_file}")
        return True

    urls = [package_info["url"]]
    fallback = package_info.get("fallback_url") or ""
    if fallback and fallback not in urls:
        urls.append(fallback)

    for url in urls:
        print(f"[Downloading package]: {url}")
        if _download_from_url(url, download_file):
            return True

    return False


def check_toolchain_package(toolchain_root, package_info) -> bool:
    name = package_info["name"]
    package = os.path.join(toolchain_root, name)

    real_size = os.stat(package).st_size
    exp_size = package_info["size"]
    if real_size != exp_size:
        print(f"Error size: real[{real_size}], expectation[{exp_size}]")
        return False

    real_sha256 = calc_sha256(package)
    exp_sha256 = package_info["sha256"]
    if real_sha256 != exp_sha256:
        print(f"Error sha256: real[{real_sha256}], expectation[{exp_sha256}]")
        return False

    print("Toolchain package is OK.")
    return True


def unzip_toolchain_package(toolchain_root, package_info) -> bool:
    name = package_info["name"]
    package = os.path.join(toolchain_root, name)
    dest = toolchain_root
    if name.endswith(".zip"):
        dest = os.path.join(toolchain_root, package_info["folder"])
    print(f"[Extracting toolchain package]: {package}")
    return extract_archive(package, dest)


def cleanup_toolchain_cache(toolchain_root, package_info, remove_folder=False) -> bool:
    name = package_info["name"]
    package = os.path.join(toolchain_root, name)
    if os.path.exists(package):
        print(f"[Removing invalid toolchain package]: {package}")
        rm_rf(package)

    if remove_folder:
        folder = package_info["folder"]
        folder_path = os.path.join(toolchain_root, folder)
        if os.path.exists(folder_path):
            print(f"[Removing incomplete toolchain folder]: {folder_path}")
            rm_rf(folder_path)

    return True


def prepare_toolchain_package(toolchain_root, package_info) -> bool:
    name = package_info["name"]
    package = os.path.join(toolchain_root, name)

    if os.path.exists(package):
        if check_toolchain_package(toolchain_root, package_info):
            return True
        cleanup_toolchain_cache(toolchain_root, package_info)

    if not wget_toolchain_package(toolchain_root, package_info):
        cleanup_toolchain_cache(toolchain_root, package_info)
        return False

    if not check_toolchain_package(toolchain_root, package_info):
        cleanup_toolchain_cache(toolchain_root, package_info)
        return False

    return True


def download_toolchain(toolchain_root) -> bool:
    package_info = get_toolchain_package_info()
    if not package_info:
        return False

    folder = package_info["folder"]
    folder_path = os.path.join(toolchain_root, folder)

    if _gcc_path(folder_path):
        print(f"[Toolchain folder is exists]: {folder_path}")
        return True

    if os.path.lexists(folder_path):
        cleanup_toolchain_cache(toolchain_root, package_info,
                                remove_folder=True)

    for idx in range(MAX_DOWNLOAD_ATTEMPTS):
        if idx > 0:
            print(f"[Retrying toolchain download]: {idx + 1}/{MAX_DOWNLOAD_ATTEMPTS}")

        if not prepare_toolchain_package(toolchain_root, package_info):
            continue

        rm_rf(folder_path)
        if unzip_toolchain_package(toolchain_root, package_info) \
                and _gcc_path(folder_path):
            return True

        cleanup_toolchain_cache(toolchain_root, package_info,
                                remove_folder=True)

    return False


def _bashtools_path(tools_root):
    for rel in (("bash", "bin", "bash.exe"), ("bash", "bin", "bash")):
        path = os.path.join(tools_root, *rel)
        if os.path.isfile(path):
            return path
    return ""


def _zip_is_valid(package) -> bool:
    try:
        with zipfile.ZipFile(package, "r") as zip_ref:
            return zip_ref.testzip() is None
    except Exception as e:
        print(f"Error: invalid package {package}: {e}")
        return False


def download_bashtools(tools_root) -> bool:
    name = "bash.zip"
    url = "https://images.tuyacn.com/smart/embed/package/vscode/data/bash.zip"
    download_file = os.path.join(tools_root, name)

    bash = _bashtools_path(tools_root)
    if bash:
        print(f"[Bashtools is exists]: {bash}")
        return True

    if os.path.exists(download_file) and not _zip_is_valid(download_file):
        print(f"[Removing invalid bashtools package]: {download_file}")
        rm_rf(download_file)

    if not os.path.exists(download_file):
        print(f"[Downloading bashtools package]: {url}")
        if not _download_from_url(url, download_file):
            return False
        if not _zip_is_valid(download_file):
            print(f"[Removing invalid bashtools package]: {download_file}")
            rm_rf(download_file)
            return False

    print(f"[Extracting bashtools package]: {download_file}")
    if not extract_archive(download_file, tools_root):
        rm_rf(os.path.join(tools_root, "bash"))
        return False

    bash = _bashtools_path(tools_root)
    if not bash:
        print("Error: bash not found after extracting bashtools package.")
        rm_rf(os.path.join(tools_root, "bash"))
        return False

    print(f"[Bashtools installed]: {bash}")
    return True

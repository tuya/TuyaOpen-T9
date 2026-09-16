#!/usr/bin/env python3
# coding=utf-8

import os
import sys

from tools.util import extract_archive, get_system_name


TOOLCHAIN_FOLDER = "arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi"
TOOLCHAIN_TAR = f"{TOOLCHAIN_FOLDER}.tar"


def main():
    root = os.path.dirname(os.path.abspath(__file__))
    toolchain_root = os.path.join(root, "toolchain")
    gcc = os.path.join(toolchain_root, TOOLCHAIN_FOLDER, "bin",
                       "arm-none-eabi-gcc")

    if os.path.isfile(gcc):
        print(f"Toolchain already extracted: {gcc}")
        sys.exit(0)

    sys_name = get_system_name()
    if sys_name != "linux":
        print(f"Error: T9 first-pass toolchain is linux x86_64 only "
              f"(host={sys_name}).")
        sys.exit(1)

    tar_path = os.path.join(toolchain_root, TOOLCHAIN_TAR)
    if not os.path.isfile(tar_path):
        print(f"Error: Not found {tar_path}")
        sys.exit(1)

    print(f"Extracting {tar_path} ...")
    if not extract_archive(tar_path, toolchain_root):
        print("Error: extract toolchain failed.")
        sys.exit(1)

    if not os.path.isfile(gcc):
        print(f"Error: gcc missing after extract: {gcc}")
        sys.exit(1)

    sys.exit(0)


if __name__ == "__main__":
    main()

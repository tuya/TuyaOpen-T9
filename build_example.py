#!/usr/bin/env python3
# coding=utf-8
# $1 - params path: $1/build_param.[cmake/config/json]
# $2 - user cmd: build/clean/...

import os
import sys
import json
import shutil

from tools.util import (
    need_settarget, record_target, do_subprocess, get_system_name
)
from tools.download_toolchain import get_toolchain_package_info


TARGET = "bk7259"


def parser_para_file(json_file):
    if not os.path.isfile(json_file):
        print(f"Error: Not found [{json_file}].")
        return {}
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception as e:
        print(f"Parser json error:  [{str(e)}].")
        return {}


def set_environment(root, build_param_path, param_data):
    os.environ["TUYA_PROJECT_DIR"] = root
    os.environ["BUILD_PARAM_DIR"] = build_param_path
    os.environ["TUYA_HEADER_DIR"] = param_data["OPEN_HEADER_DIR"]
    os.environ["TUYA_LIBS_DIR"] = param_data["OPEN_LIBS_DIR"]
    os.environ["TUYA_LIBS"] = param_data["PLATFORM_NEED_LIBS"]
    os.environ["TUYA_APP_NAME"] = param_data["CONFIG_PROJECT_NAME"]
    os.environ["TUYA_APP_VERSION"] = param_data["CONFIG_PROJECT_VERSION"]
    os.environ["TUYA_BIN_OUTPUT_DIR"] = param_data["BIN_OUTPUT_DIR"]
    open_root = param_data["OPEN_ROOT"]
    os.environ["TUYAOS_BOARD_PATH"] = os.path.join(open_root, "boards", "T9")


def _board_log_uart_intent(param_data):
    p = param_data.get("CONFIG_BOARD_LOG_UART_PORT")
    return (str(p) if p is not None else "default"), p


def set_board_log_uart_env(build_param_path, param_data):
    _, port = _board_log_uart_intent(param_data)
    if port is not None:
        frag = os.path.join(build_param_path, "tuya_logport.config")
        with open(frag, "w", encoding="utf-8") as f:
            f.write(f"CONFIG_UART_PRINT_PORT={port}\n")
        os.environ["TUYA_APP_SDKCONFIG_APPEND"] = frag
    else:
        os.environ.pop("TUYA_APP_SDKCONFIG_APPEND", None)


def sync_board_log_uart(build_root, param_data):
    intent, port = _board_log_uart_intent(param_data)
    gen_dir = os.path.join(build_root, "build", TARGET)
    stamp = os.path.join(build_root, "build", ".tuya_logport_intent")

    prev = None
    if os.path.isfile(stamp):
        with open(stamp, "r", encoding="utf-8") as f:
            prev = f.read().strip()

    need_wipe = (port is not None) if prev is None else (prev != intent)

    if need_wipe and os.path.isdir(gen_dir):
        print(f"[tuya] log-uart intent changed ({prev} -> {intent}), "
              f"wiping generated config: {gen_dir}")
        shutil.rmtree(gen_dir, ignore_errors=True)

    os.makedirs(os.path.dirname(stamp), exist_ok=True)
    with open(stamp, "w", encoding="utf-8") as f:
        f.write(intent)


def clean(build_root, toolchain_folder_path, bash_path):
    build_root = build_root.replace("\\", "/")
    toolchain_folder_path = toolchain_folder_path.replace("\\", "/")
    cmd = f"export TUYA_TOOLCHAIN_PATH={toolchain_folder_path}; "
    cmd += f"cd {build_root}; make clean"
    if os.path.exists(bash_path):
        cmd = f"{bash_path} -c '{cmd}'"
    do_subprocess(cmd)


def build(build_root, toolchain_folder_path, bash_path,
          target, app_name, app_ver) -> bool:
    build_root = build_root.replace("\\", "/")
    toolchain_folder_path = toolchain_folder_path.replace("\\", "/")
    cmd = f"export TUYA_TOOLCHAIN_PATH={toolchain_folder_path}; "
    cmd += f"cd {build_root}; make {target} PROJECT=tuya_app"
    cmd += f" APP_NAME={app_name}"
    cmd += f" APP_VERSION={app_ver}"
    cmd += " -j"
    if os.path.exists(bash_path):
        cmd = f"{bash_path} -c '{cmd}'"

    ret = do_subprocess(cmd)
    if ret != 0:
        print("Error: Build error.")
        return False
    return True


def do_with_compile(root, build_root, user_cmd,
                    target, build_param_path, param_data,
                    toolchain_folder_path, bash_path):
    set_environment(root, build_param_path, param_data)
    set_board_log_uart_env(build_param_path, param_data)

    app_target_file = os.path.join(build_root, ".app")
    app_name = param_data["CONFIG_PROJECT_NAME"]
    if "clean" == user_cmd or need_settarget(app_target_file, app_name):
        clean(build_root, toolchain_folder_path, bash_path)
        if "clean" == user_cmd:
            sys.exit(0)

    sync_board_log_uart(build_root, param_data)

    record_target(app_target_file, app_name)
    app_ver = param_data["CONFIG_PROJECT_VERSION"]
    if not build(build_root, toolchain_folder_path, bash_path,
                 target, app_name, app_ver):
        sys.exit(1)


def main():
    if len(sys.argv) < 3:
        print(f"Error: At least 2 parameters are needed {sys.argv}.")
        sys.exit(1)
    build_param_path = sys.argv[1]
    user_cmd = sys.argv[2]

    root = os.path.dirname(os.path.abspath(__file__))
    build_root = os.path.join(root, "t9_os")
    build_param_file = os.path.join(build_param_path, "build_param.json")
    param_data = parser_para_file(build_param_file)
    if not len(param_data):
        sys.exit(1)

    platform_root = os.path.abspath(os.path.join(root, ".."))
    toolchain_root = os.path.join(platform_root, "tools")
    toolchain_package_info = get_toolchain_package_info()
    if not toolchain_package_info:
        sys.exit(1)
    toolchain_folder = toolchain_package_info["folder"]
    toolchain_folder_path = os.path.join(toolchain_root, toolchain_folder)

    bash_path = ""
    if "windows" == get_system_name():
        bash_path = os.path.join(root, "tools", "bash", "bin", "bash.exe")

    do_with_compile(root, build_root, user_cmd, TARGET,
                    build_param_path, param_data,
                    toolchain_folder_path, bash_path)
    sys.exit(0)


if __name__ == "__main__":
    main()

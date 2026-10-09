#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPIO配置生成工具

此脚本根据默认GPIO配置生成最终的usr_gpio_cfg.h头文件（已移除 app 配置覆盖逻辑）。

使用方法:
    python3 user_gpio_config.py <default_config.json> --output <output_file.h>

示例:
    python3 vendor/T9/t9_os/projects/tuya_app/tuya_scripts/user_gpio_config.py \\
        vendor/T9/t9_os/projects/tuya_app/tuya_scripts/tuya_default_gpio_config.json \\
        --output vendor/T9/t9_os/projects/tuya_app/ap/config/bk7259_ap/usr_gpio_cfg.h

Copyright © 2025 Tuya Inc.
"""

import json
import os
import sys
import argparse
from typing import Dict
# from datetime import datetime

# ANSI转义码定义
RED = '\033[91m'
GREEN = '\033[92m'
CYAN = '\033[96m'
YELLOW = '\033[93m'
RESET = '\033[0m'


def print_error(message: str):
    """打印错误信息"""
    sys.stderr.write(f"{RED}错误: {message}{RESET}\n")
    sys.stderr.flush()


def print_warning(message: str):
    """打印警告信息"""
    print(f"{YELLOW}警告: {message}{RESET}")


def print_success(message: str):
    """打印成功信息"""
    print(f"{GREEN}{message}{RESET}")


def print_info(message: str):
    """打印普通信息"""
    print(f"{CYAN}{message}{RESET}")


def load_json_file(file_path: str, optional: bool = False) -> Dict:
    """
    加载JSON配置文件

    Args:
        file_path: JSON文件路径
        optional: 是否为可选文件，True时文件不存在返回空字典

    Returns:
        解析后的JSON对象，如果optional=True且文件不存在则返回空字典

    Raises:
        ValueError: 文件不存在或解析失败（仅当optional=False时）
    """
    if not os.path.exists(file_path):
        if optional:
            print_warning(f"  配置文件不存在: {file_path} (将使用默认配置)")
            return {}
        raise ValueError(f"文件不存在: {file_path}")

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            print_info(f"  已加载配置文件: {file_path}")
            return data
    except json.JSONDecodeError as e:
        raise ValueError(f"JSON解析失败 {file_path}: {str(e)}")
    except Exception as e:
        raise ValueError(f"读取文件失败 {file_path}: {str(e)}")


def build_gpio_configs(default_config: Dict) -> Dict[str, Dict]:
    """
    从默认配置构建 GPIO 配置字典（已移除 app 配置覆盖逻辑）

    Args:
        default_config: 默认GPIO配置

    Returns:
        GPIO配置字典，key为gpio_id
    """
    merged_gpio = {}

    for item in default_config.get('gpio_map', []):
        gpio_id = item.get('gpio_id')
        if gpio_id:
            merged_gpio[gpio_id] = item.copy()

    print_info(f"已加载 {len(merged_gpio)} 个GPIO配置")

    return merged_gpio


def generate_header_file(gpio_configs: Dict[str, Dict], output_path: str):
    """
    生成GPIO配置头文件

    Args:
        gpio_configs: GPIO配置字典
        output_path: 输出文件路径
    """
    # GPIO配置字段顺序（必须与 gpio_default_map_t 结构体定义一致，bk7259 新版 10 字段）
    field_order = [
        'gpio_id',
        'second_func_en',
        'second_func_dev',
        'time_sharing_func_dev',
        'pull_mode',
        'int_en',
        'int_type',
        'low_power_io_ctrl',
        'driver_capacity',
        'gpio_skip'
    ]

    def format_row(item):
        """将单个GPIO配置项格式化为 C 结构体初始化片段 {GPIO_x, ...}"""
        try:
            values = [item[field] for field in field_order]
        except KeyError as e:
            raise KeyError(f"字段 {e} 在GPIO配置项 {item['gpio_id']} 中缺失")
        return "{" + ", ".join(f"{v: <2}" if i == 0 else v
                               for i, v in enumerate(values)) + "}"

    # 按GPIO编号排序
    sorted_items = sorted(
        gpio_configs.values(),
        key=lambda x: int(x['gpio_id'].replace('GPIO_', ''))
    )

    # 带条件编译(condition)的GPIO通过 GPIO_x_DEFAULT_MAP 宏间接展开，
    # 这样在该宏条件不成立时该 GPIO 不被配置（保持上电默认态）。
    cond_groups = {}  # condition(str) -> [item, ...]，保持顺序
    for item in sorted_items:
        cond = item.get('condition')
        if cond:
            cond_groups.setdefault(cond, []).append(item)

    cond_lines = []
    for cond, items in cond_groups.items():
        cond_lines.append(f"#if {cond}")
        for it in items:
            cond_lines.append(f"#define {it['gpio_id']}_DEFAULT_MAP {format_row(it)},")
        cond_lines.append("#else")
        for it in items:
            cond_lines.append(f"#define {it['gpio_id']}_DEFAULT_MAP")
        cond_lines.append("#endif")
    conditional_section = ("\n".join(cond_lines) + "\n\n") if cond_lines else ""

    # 生成每个GPIO配置行（条件项引用其宏，普通项直接写出结构体）
    config_lines = []
    for item in sorted_items:
        if item.get('condition'):
            config_lines.append(f"\t{item['gpio_id']}_DEFAULT_MAP \\")
        else:
            config_lines.append(f"\t{format_row(item)},\\")

    # 生成头文件内容
    header_content = f"""// Copyright 2020-2021 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 此文件由 user_gpio_config.py 自动生成
// 请勿手动编辑

#pragma once

#ifdef __cplusplus
extern "C" {{
#endif

{conditional_section}/*	 gpio_id  second_func_en  second_func_dev  time_sharing_func_dev  pull_mode  int_en  int_type  low_power_io_ctrl  driver_capacity  gpio_skip */
#define GPIO_DEFAULT_DEV_CONFIG  \\
{{\\
{chr(10).join(config_lines)}
}}

#ifdef __cplusplus
}}
#endif
"""

    # 确保输出目录存在
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print_info(f"已创建输出目录: {output_dir}")

    # 写入文件
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(header_content)

    print_success(f"生成GPIO配置头文件: {output_path}")

def gpio_process():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='GPIO配置生成工具 - 根据默认配置生成头文件',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 user_gpio_config.py default_config.json -o output.h
  python3 user_gpio_config.py \\
      vendor/T9/t9_os/projects/tuya_app/tuya_scripts/tuya_default_gpio_config.json \\
      --output vendor/T9/t9_os/projects/tuya_app/ap/config/bk7259_ap/usr_gpio_cfg.h
        """
    )

    parser.add_argument(
        'default_config',
        help='基础默认配置文件路径 (JSON格式，必需)'
    )

    parser.add_argument(
        '-o', '--output',
        required=True,
        help='输出头文件路径 (usr_gpio_cfg.h)'
    )

    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='显示详细信息'
    )

    args = parser.parse_args()

    try:
        print_info("=" * 60)
        print_info("GPIO配置合并")
        print_info("=" * 60)

        # 加载配置文件
        print_info("\n[1/3] 加载配置文件...")
        default_config = load_json_file(args.default_config, optional=False)

        # 解析配置（仅使用默认配置，已移除 app 配置覆盖逻辑）
        print_info("\n[2/3] 解析配置...")
        merged_gpio = build_gpio_configs(default_config)

        # 生成头文件
        print_info("\n[3/3] 生成头文件...")
        generate_header_file(merged_gpio, args.output)

        print_info("\n" + "=" * 60)
        print_success("GPIO配置文件生成完成")
        print_info("=" * 60)

    except ValueError as e:
        print_error(str(e))
        sys.exit(1)
    except KeyError as e:
        print_error(f"配置字段缺失: {str(e)}")
        sys.exit(1)
    except Exception as e:
        print_error(f"未预期的错误: {str(e)}")
        if args.verbose if 'args' in locals() else False:
            import traceback
            traceback.print_exc()
        sys.exit(1)

def main():
    gpio_process()

if __name__ == "__main__":
    main()

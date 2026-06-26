#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tuya固件打包工具

此脚本用于自动化固件打包流程，包括：
- 创建输出目录
- 生成OTA升级文件
- 复制固件和调试文件
- 支持compress，partition和compatible三种OTA模式

使用方法:
    python3 tuya_package.py --app-name <name> --version <ver> --platform <platform> \\
        --project-dir <dir> --app-config <config.json> [--ci-path <path>]

Copyright © 2025 Tuya Inc.
"""

import os
import sys
import argparse
import subprocess
import shutil
import glob
from pathlib import Path

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


def run_command(cmd: list, shell: bool = False, capture_output: bool = False,
                check: bool = True, silent: bool = False) -> subprocess.CompletedProcess:
    """
    执行命令

    Args:
        cmd: 命令列表或字符串
        shell: 是否使用shell执行
        capture_output: 是否捕获输出
        check: 是否检查返回码
        silent: 是否静默执行（不输出到终端）

    Returns:
        subprocess.CompletedProcess对象
    """
    try:
        if silent and not capture_output:
            result = subprocess.run(
                cmd,
                shell=shell,
                check=check,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        else:
            result = subprocess.run(
                cmd,
                shell=shell,
                check=check,
                capture_output=capture_output,
                text=True
            )
        return result
    except subprocess.CalledProcessError as e:
        print_error(f"命令执行失败: {' '.join(cmd) if isinstance(cmd, list) else cmd}")
        if capture_output and e.stderr:
            print_error(e.stderr)
        raise


def ensure_dir(path: str):
    """确保目录存在，不存在则创建"""
    Path(path).mkdir(parents=True, exist_ok=True)


def copy_files(src_pattern: str, dst_dir: str):
    """
    复制文件（支持通配符）

    Args:
        src_pattern: 源文件路径（可包含通配符）
        dst_dir: 目标目录
    """
    files = glob.glob(src_pattern)
    if not files:
        print_warning(f"没有找到匹配的文件: {src_pattern}")
        return

    ensure_dir(dst_dir)
    for file in files:
        if os.path.isfile(file):
            shutil.copy2(file, dst_dir)
            # print_info(f"  复制: {file} -> {dst_dir}")


class TuyaPackager:
    """Tuya固件打包器"""

    def __init__(self, args):
        self.app_name    = os.environ.get('TUYA_APP_NAME', '').strip()
        self.version     = os.environ.get('TUYA_APP_VERSION', '').strip()
        self.platform    = os.environ.get('TUYA_APP_PLATFORM', '').strip() or 'bk7259'
        self.project_dir = os.environ.get('TUYA_PROJECT_DIR', '').strip()
        self.verbose     = args.verbose

        # 检查必需的环境变量
        if not self.app_name:
            raise RuntimeError("环境变量 TUYA_APP_NAME 未设置")
        if not self.version:
            raise RuntimeError("环境变量 TUYA_APP_VERSION 未设置")
        if not self.project_dir:
            raise RuntimeError("环境变量 TUYA_PROJECT_DIR 未设置")

        if self.verbose:
            print_info(f"应用名称: {self.app_name}")
            print_info(f"版本号: {self.version}")
            print_info(f"平台: {self.platform}")
            print_info(f"项目目录: {self.project_dir}")

        # Armino 根目录：.../t9_os（本文件在 t9_os/projects/tuya_app/tuya_scripts/ 下，需向上 4 级）
        _armino_root = Path(__file__).resolve().parent.parent.parent.parent
        self.work_dir = str(_armino_root)
        os.chdir(self.work_dir)

        # 设置路径
        self.tuya_app_demo_path = "projects/tuya_app"
        self.scripts_dir = f"{self.tuya_app_demo_path}/tuya_scripts"

        # OTA 模式配置：apps/*/app_resource_config.json 已废弃，统一改用脚本目录下的
        # default_ota_mode.json 决定 SMP 固件的 OTA 升级模式（compress/partition/compatible）。
        self.ota_mode_config = f"{self.scripts_dir}/default_ota_mode.json"

        _bl2_under_build = (
            _armino_root / "build" / self.platform / "tuya_app" / self.platform / "bl2.bin"
        )
        _bl2_under_files = _armino_root / self.scripts_dir / "files" / "bl2.bin"
        if _bl2_under_build.is_file():
            _bl2_rel = f"build/{self.platform}/tuya_app/{self.platform}/bl2.bin"
        elif _bl2_under_files.is_file():
            _bl2_rel = "projects/tuya_app/tuya_scripts/files/bl2.bin"
        else:
            _bl2_rel = f"build/{self.platform}/tuya_app/{self.platform}/bl2.bin"

        # 设置输出路径 - 从环境变量读取CI_PACKAGE_PATH
        ci_path = os.environ.get('CI_PACKAGE_PATH', '').strip()
        if ci_path:
            self.output_path = ci_path
            print_info(f"使用CI打包路径: {ci_path}")
        else:
            self.output_path = os.path.join(
                self.project_dir, f"apps/{self.app_name}/output/{self.version}"
            )

        self.debug_path = os.path.join(self.output_path, "debug")

        # 工具路径
        self.tools = {
            'diff2ya': f"{self.scripts_dir}/diff2ya",
            'format_bin': f"{self.scripts_dir}/format_up_bin.py",
            'create_ua': f"{self.scripts_dir}/create_ua_file.py",
            'parse_flash': f"{self.scripts_dir}/parse_flash_table.py",
            'get_section': f"{self.scripts_dir}/get_map_section.py",
            'smp_ota': f"{self.scripts_dir}/create_ug_file.py",
            'ota_select': f"{self.scripts_dir}/user_ota_select.py",
        }

        # 文件路径
        self.files = {
            'bl2_bin': _bl2_rel,
            'cp_bin': f"build/{self.platform}/tuya_app/{self.platform}/app.bin",
            'ap_bin': f"build/{self.platform}/tuya_app/{self.platform}_ap/app.bin",
            'all_bin': f"build/{self.platform}/tuya_app/package/all-app.bin",
            'ua_bin': f"build/{self.platform}/tuya_app/package/ua_file.bin",
            'ug_bin': f"build/{self.platform}/tuya_app/package/ug_file.bin",
            'ty_ota': f"build/{self.platform}/tuya_app/package/ty_ug_file.bin",
            'smp_ty_ota': f"build/{self.platform}/tuya_app/package/ty_smp_ug_file.bin",
            'ap_map': f"build/{self.platform}/tuya_app/{self.platform}_ap/app.map",
            'partition': f"{self.tuya_app_demo_path}/partitions/{self.platform}/auto_partitions.csv",
        }

    def create_directories(self):
        """创建必要的目录"""
        if self.verbose:
            print_info(f"创建输出目录...{self.output_path}")
        ensure_dir(self.output_path)
        ensure_dir(self.debug_path)
        ensure_dir(os.path.join(self.debug_path, self.platform))
        ensure_dir(os.path.join(self.debug_path, f"{self.platform}_ap"))

    def get_section_offset(self, map_file: str, section: str) -> int:
        """获取section偏移地址"""
        result = run_command(
            ['python3', self.tools['get_section'], map_file, section],
            capture_output=True
        )
        return int(result.stdout.strip())

    def get_section_offset_optional(self, map_file: str, section: str):
        """
        同 get_section_offset，但 map 中无该符号时返回 None。
        SMP 关闭时 AP 链接脚本导出 __vector_table 而非 __vector_core0_table。
        """
        result = subprocess.run(
            ['python3', self.tools['get_section'], map_file, section],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            return None
        try:
            return int(result.stdout.strip().split()[0])
        except (ValueError, IndexError):
            return None

    def get_ota_mode(self) -> str:
        """获取OTA模式（从 default_ota_mode.json 解析，缺失时回退 compress）"""
        if not os.path.isfile(self.ota_mode_config):
            print_warning(
                f"OTA 配置文件不存在: {self.ota_mode_config}，回退使用 compress 模式"
            )
            return "compress"
        result = run_command(
            ['python3', self.tools['ota_select'], self.ota_mode_config],
            capture_output=True
        )
        return result.stdout.strip()

    def get_partition_size(self, partition_name: str) -> int:
        """
        获取分区大小

        Args:
            partition_name: 分区名称（如 'ota'）

        Returns:
            分区大小（字节）
        """
        # 保存当前目录
        current_dir = os.getcwd()
        try:
            # 获取绝对路径
            parse_flash_abs = os.path.abspath(self.tools['parse_flash'])

            # 切换到分区表所在目录
            partition_dir = os.path.dirname(self.files['partition'])
            os.chdir(partition_dir)

            # 执行命令获取分区大小（使用绝对路径）
            result = run_command(
                ['python3', parse_flash_abs, partition_name, 'size'],
                capture_output=True
            )
            return int(result.stdout.strip())
        finally:
            # 恢复原目录
            os.chdir(current_dir)

    def get_partition_size_optional(self, partition_name: str):
        """
        查询分区大小；分区不存在时返回 None（T9 分区表可能无 primary_tuyaboot）。
        """
        current_dir = os.getcwd()
        try:
            parse_flash_abs = os.path.abspath(self.tools['parse_flash'])
            partition_dir = os.path.dirname(self.files['partition'])
            os.chdir(partition_dir)
            result = subprocess.run(
                ['python3', parse_flash_abs, partition_name, 'size'],
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode != 0:
                return None
            return int(result.stdout.strip())
        except (ValueError, OSError):
            return None
        finally:
            os.chdir(current_dir)

    def calculate_split_point(self) -> int:
        """获取涂鸦段信息"""
        if self.verbose:
            print_info("获取涂鸦段信息...")
        ap_start_section_addr = self.get_section_offset_optional(
            self.files['ap_map'], '__vector_core0_table'
        )
        if ap_start_section_addr is None:
            ap_start_section_addr = self.get_section_offset_optional(
                self.files['ap_map'], '__vector_table'
            )
        if ap_start_section_addr is None:
            raise RuntimeError(
                "AP app.map 中未找到 __vector_core0_table 或 __vector_table（请检查 SMP 与链接脚本）"
            )

        # _ty_section_start 仅在 ld 含 .ty_text_section 时存在；否则用 __etext 或退回与向量同址
        ap_ty_section_addr = self.get_section_offset_optional(
            self.files['ap_map'], '_ty_section_start'
        )
        if ap_ty_section_addr is None:
            ap_ty_section_addr = self.get_section_offset_optional(
                self.files['ap_map'], '__etext'
            )
        if ap_ty_section_addr is None:
            if self.verbose:
                print_warning(
                    "未找到 _ty_section_start / __etext，split 使用 AP 向量基址（无独立涂鸦 Flash 段）"
                )
            ap_ty_section_addr = ap_start_section_addr

        """bl2 + cp + a-section (no tuyaboot row => bl2_raw_size = 0)"""

        bl2_sz = self.get_partition_size_optional('primary_tuyaboot')
        if bl2_sz is None:
            bl2_sz = self.get_partition_size_optional('primary_bootloader')
        if bl2_sz is None:
            bl2_raw_size = 0
        else:
            bl2_raw_size = int((bl2_sz * 32) / 34)

        cp_physical_size = self.get_partition_size('primary_cp_app')
        cp_raw_size = int((cp_physical_size * 32) / 34)

        split_point =  bl2_raw_size + cp_raw_size + ap_ty_section_addr - ap_start_section_addr

        print_info(f"ap_start_section_addr: {ap_start_section_addr}")
        print_info(f"ap_ty_section_addr: {ap_ty_section_addr}")
        print_info(f"split_point: {split_point}")

        return split_point

    def create_ua_file(self):
        """创建UA文件"""
        if self.verbose:
            print_info("创建UA文件...")
        run_command([
            'python3', self.tools['create_ua'],
            self.files['partition'],
            self.files['bl2_bin'],
            self.files['cp_bin'],
            self.files['ap_bin'],
            f"--ua_file={self.files['ua_bin']}"
        ])

    def generate_ota_files(self, split_point: int, ota_size: int, ota_mode: str):
        """
        生成OTA升级文件

        Args:
            split_point: CP+AP分割点
            ota_size: OTA分区大小（从分区表获取）
            ota_mode: OTA模式（compress/partition/compatible）
        """
        print_info(f"生成OTA文件 (模式: {ota_mode}, OTA分区大小: {ota_size})...")

        if ota_mode == "compress":
            # 生成SMP OTA文件（使用仓库内 Python 脚本，避免预编译 create_ug_file 与宿主环境不兼容）
            cmd = [
                'python3', self.tools['smp_ota'],
                self.files['ua_bin'],
                '-o', self.files['smp_ty_ota'],
                '--total-size', '8024064',
                '--ota-size', str(ota_size),
                '--a-size', str(split_point),
                '--gap-size', '524288',
                '-m', '1'
            ]
            if self.verbose:
                cmd.append('-v')
            run_command(cmd)

        elif ota_mode == "partition":
            # 生成SMP OTA文件
            cmd = [
                'python3', self.tools['smp_ota'],
                self.files['ua_bin'],
                '-o', self.files['smp_ty_ota'],
                '--total-size', '8024064',
                '--ota-size', str(ota_size),
                '--a-size', str(split_point),
                '--gap-size', '524288',
                '-m', '2'
            ]
            if self.verbose:
                cmd.append('-v')
            run_command(cmd)

        else:
            raise ValueError(f"不支持的OTA模式: {ota_mode}")

    def copy_output_files(self):

        # 复制主固件文件
        output_files = [
            (self.files['all_bin'], f"{self.app_name}_QIO_{self.version}.bin"),
            (self.files['ua_bin'], f"{self.app_name}_UA_{self.version}.bin"),
            # (self.files['ty_ota'], f"{self.app_name}_UG_OLD_{self.version}.bin"),
            (self.files['smp_ty_ota'], f"{self.app_name}_UG_{self.version}.bin"),
        ]

        for src, dst_name in output_files:
            dst = os.path.join(self.output_path, dst_name)
            shutil.copy2(src, dst)
            if self.verbose:
                print_info(f"  {src} -> {dst}")

    def copy_debug_files(self):

        cp_debug = os.path.join(self.debug_path, self.platform)
        ap_debug = os.path.join(self.debug_path, f"{self.platform}_ap")

        # CP调试文件
        copy_files(f"build/{self.platform}/tuya_app/{self.platform}/app*", cp_debug)
        copy_files(f"build/{self.platform}/tuya_app/{self.platform}/size_map*", cp_debug)
        copy_files(f"build/{self.platform}/tuya_app/{self.platform}/sdkconfig", cp_debug)
        copy_files(f"projects/tuya_app/cp/config/{self.platform}/*", cp_debug)

        # AP调试文件
        copy_files(f"build/{self.platform}/tuya_app/{self.platform}_ap/app*", ap_debug)
        copy_files(f"build/{self.platform}/tuya_app/{self.platform}_ap/size_map*", ap_debug)
        copy_files(f"build/{self.platform}/tuya_app/{self.platform}_ap/sdkconfig", ap_debug)
        copy_files(f"projects/tuya_app/ap/config/{self.platform}_ap/*", ap_debug)

        # 分区表
        copy_files(f"projects/tuya_app/partitions/{self.platform}/*", self.debug_path)

    def package(self):
        """执行打包流程"""
        try:
            if self.verbose:
                print_info("开始固件打包")

            # 检查all-app.bin是否存在
            if not os.path.exists(self.files['all_bin']):
                print_error(f"编译产物不存在: {self.files['all_bin']}")
                return 1

            # 创建目录
            self.create_directories()

            # 计算分割点
            split_point = self.calculate_split_point()

            # 创建UA文件
            self.create_ua_file()

            # 获取OTA分区大小
            ota_size = self.get_partition_size('ota')
            if self.verbose:
                print_info(f"OTA分区大小: {ota_size} 字节")

            # 获取OTA模式
            ota_mode = self.get_ota_mode()

            # 生成OTA文件
            self.generate_ota_files(split_point, ota_size, ota_mode)

            if self.verbose:
                print_info("输出文件...")
            # 复制输出文件
            self.copy_output_files()

            # 复制调试文件
            self.copy_debug_files()

            # 打印成功信息
            if self.verbose:
                print_info("固件打包完成")
                print_success(f"\n输出目录: {self.output_path}")

            return 0

        except Exception as e:
            print_error(f"打包失败: {str(e)}")
            if self.verbose:
                import traceback
                traceback.print_exc()
            return 1


def tuya_package_process():
    """主函数 - 可被其他脚本调用（从环境变量读取参数）"""
    parser = argparse.ArgumentParser(
        description='Tuya固件打包工具',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='显示详细信息'
    )

    # 使用 parse_known_args() 而不是 parse_args()，忽略未知参数
    # 这样从 Make 调用时不会因为额外参数而报错
    args, unknown = parser.parse_known_args()

    if unknown and args.verbose:
        print_warning(f"忽略未知参数: {' '.join(unknown)}")

    packager = TuyaPackager(args)
    return packager.package()

def firmware_info():
    """
    获取固件信息（用于 bk_build_package.py）

    Returns:
        固件信息字符串，格式：app_name_version
    """
    # 直接从环境变量读取，避免创建完整的 TuyaPackager 实例
    app_name = os.environ.get('TUYA_APP_NAME', '').strip()
    version = os.environ.get('TUYA_APP_VERSION', '').strip()

    if not app_name or not version:
        # 如果环境变量未设置，返回空字符串
        return ""

    return f"{app_name}_{version}\n"

def main():
    tuya_package_process()

if __name__ == "__main__":
    sys.exit(main())

#! /usr/bin/env python3
# vim:fenc=utf-8
#
# Copyright © 2025 cc <cc@tuya>
#
# Distributed under terms of the TUYA license.

"""
Generate cloud OTA package (ty_smp_ug_file.bin) for Beken RBL direct-write path.

Without ENABLE_MULTI_SECTION_UPGRADE, tkl_ota writes the downloaded file as-is
into BK_PARTITION_OTA. The payload must be a pure Beken RBL image (no Tuya
0x4D4D4D / 0x4C5A4350 header). Bootloader decompresses RBL on reboot.
"""

import os
import struct
import subprocess
import sys

import click
import lzma
import zlib


RBL_PACKAGER_REL = "tools/env_tools/rtt_ota/ota-rbl/ota_packager_python.py"
RBL_HEADER_DIR_REL = "tools/env_tools/rtt_ota/ota-rbl"


def add_package_header(original_data, compress_data, ota_type, mode, verbose=False):
    """
    Legacy Tuya LZCP header (64 bytes) + compressed payload.
    Used only with ENABLE_MULTI_SECTION_UPGRADE + tal_diff_ota.
    """
    compress_crc32 = zlib.crc32(compress_data) & 0xffffffff
    original_crc32 = zlib.crc32(original_data) & 0xffffffff

    attr1 = 0
    if ota_type == 0x4C5A4351:
        attr1 = len(original_data)

    if verbose:
        click.echo(f"ota_type:                      {ota_type:#x}")
        click.echo(f"original_crc32:                {original_crc32:#x}")
        click.echo(f"len(original_data):            {len(original_data):#x}")
        click.echo(f"compress_crc32:                {compress_crc32:#x}")
        click.echo(f"len(compress_data):            {len(compress_data):#x}")
        click.echo(f"attr1:                         {attr1:#x}")

    patch_header = struct.pack(
        "<16I",
        0x004D4D4D,
        ota_type,
        original_crc32,
        original_crc32,
        len(original_data),
        len(original_data),
        64,
        0,
        len(compress_data),
        compress_crc32,
        0,
        0x1000,
        0x20000,
        attr1,
        0,
        0,
    )
    return patch_header + compress_data


def lz_compress(src):
    filterz = [{
        "id": lzma.FILTER_LZMA2,
        "dict_size": 1024 * 4,
        "depth": 0,
        "mode": lzma.MODE_NORMAL,
        "nice_len": 273,
        "lc": 3,
        "lp": 0,
        "pb": 2,
        "mf": lzma.MF_BT4,
    }]
    return lzma.compress(data=src, format=lzma.FORMAT_XZ, check=lzma.CHECK_CRC32, filters=filterz)


def pack_beken_rbl(input_file, output_file, armino_path, project_dir, soc, verbose=False):
    """
    Wrap Beken ota_packager_python.py packager sub-command.
    Output is pure .rbl without Tuya ug_head / diff2ya header.
    """
    packager = os.path.join(armino_path, RBL_PACKAGER_REL)
    header_dir = os.path.join(armino_path, RBL_HEADER_DIR_REL)

    if not os.path.isfile(packager):
        raise click.ClickException(f"Beken RBL packager not found: {packager}")
    if not os.path.isdir(header_dir):
        raise click.ClickException(f"Beken RBL header dir not found: {header_dir}")

    cmd = [
        sys.executable,
        packager,
        "packager",
        "-i", os.path.abspath(input_file),
        "-o", os.path.abspath(output_file),
        "-g", header_dir,
        "-ap", os.path.abspath(armino_path),
        "-pjd", project_dir,
        "-soc", soc,
    ]

    if verbose:
        click.echo("Beken RBL packager command:")
        click.echo(" ".join(cmd))

    result = subprocess.run(cmd, capture_output=not verbose, text=True)
    if result.returncode != 0:
        err = result.stderr or result.stdout or "unknown error"
        raise click.ClickException(f"Beken RBL packager failed: {err}")

    if not os.path.isfile(output_file):
        raise click.ClickException(f"Beken RBL output missing: {output_file}")

    if verbose and result.stdout:
        click.echo(result.stdout)


@click.command()
@click.argument("input_file", type=click.Path(exists=True))
@click.option("-o", "--output", "output_file", required=True, type=click.Path(), help="输出 OTA 包路径 (ty_smp_ug_file.bin)")
@click.option("--ota-size", "ota_size", required=True, type=click.INT, help="OTA 分区大小 (bytes)")
@click.option("--armino-path", "armino_path", required=True, type=click.Path(exists=True), help="t9_os 根目录")
@click.option("--project-dir", "project_dir", default="projects/tuya_app", show_default=True, help="Armino 工程相对路径")
@click.option("--soc", default="bk7259", show_default=True, help="SoC 名称")
@click.option("--total-size", "total_size", default=0, type=click.INT, help="[mode=2] 固件区总大小")
@click.option("--a-size", "a_size", default=0, type=click.INT, help="[mode=2] A 段大小")
@click.option("--gap-size", "gap_size", default=0, type=click.INT, help="[mode=2] 预留 gap 大小")
@click.option("-m", "mode", required=True, type=click.INT, help="1=Beken RBL  2=涂鸦分段压缩(需 MULTI_SECTION)")
@click.option("-v", "--verbose", is_flag=True, help="详细输出")
@click.help_option("-h", "--help", help="显示帮助")
def main(input_file, output_file, ota_size, armino_path, project_dir, soc,
         total_size, a_size, gap_size, mode, verbose):
    if verbose:
        click.echo("========================================")
        click.echo(f"输入文件:      {input_file}")
        click.echo(f"输出文件:      {output_file}")
        click.echo(f"OTA 分区大小:  {ota_size:#x}")
        click.echo(f"Armino 路径:   {armino_path}")
        click.echo(f"工程路径:      {project_dir}")
        click.echo(f"SoC:           {soc}")
        click.echo(f"打包模式:      {mode}")
        click.echo("========================================")

    if mode == 1:
        pack_beken_rbl(input_file, output_file, armino_path, project_dir, soc, verbose)
        rbl_size = os.path.getsize(output_file)
        if rbl_size >= ota_size:
            raise click.ClickException(
                f"RBL 过大: {rbl_size} bytes, OTA 分区可用 {ota_size} bytes"
            )
        click.echo(f"使用 Beken RBL 压缩升级 (纯 RBL, 无涂鸦包头), size={rbl_size}")
        return

    if mode == 2:
        if total_size <= 0 or a_size <= 0:
            raise click.ClickException("mode=2 需要 --total-size 与 --a-size")

        with open(input_file, "rb") as in_file:
            all_app = in_file.read()

        first_segment = all_app[:a_size]
        first_segment_compress = lz_compress(first_segment)
        compress_ug = add_package_header(
            first_segment, first_segment_compress, 0x4C5A4351, mode, verbose
        )

        actually_a_size = int((a_size / 32) * 34)
        if len(compress_ug) < (total_size - (actually_a_size + gap_size)):
            click.echo("使用可覆盖分段压缩升级 (0x4C5A4351, 需 ENABLE_MULTI_SECTION_UPGRADE)")
            second_segment = all_app[a_size:]
            with open(output_file, "wb") as out_file:
                out_file.write(compress_ug)
                out_file.write(second_segment)
        else:
            raise click.ClickException(
                f"压缩包过大. 压缩后: {len(first_segment_compress)}, "
                f"可用: {total_size - (a_size + gap_size)}"
            )
        return

    raise click.ClickException(f"不支持的模式: {mode}, 1=Beken RBL  2=涂鸦分段压缩")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# coding=utf-8

import datetime
import os
import platform
import shutil
import hashlib
import tarfile
import zipfile


# Timezone fallback cache only. OPEN_COUNTRY_CODE is read on every get.
COUNTRY_CODE = ""

SYSTEM_NAME = ""


def set_country_code():
    global COUNTRY_CODE
    if COUNTRY_CODE:
        return COUNTRY_CODE

    try:
        offset = datetime.datetime.now().astimezone().utcoffset()
        if offset is not None and int(offset.total_seconds()) == 8 * 3600:
            COUNTRY_CODE = "China"
        else:
            COUNTRY_CODE = "Other"
    except Exception as e:
        print(f"country code error: {e}")
        COUNTRY_CODE = "Other"

    return COUNTRY_CODE


def get_country_code():
    env = os.environ.get("OPEN_COUNTRY_CODE", "").strip()
    if env:
        return env
    return set_country_code()


def set_system_name():
    global SYSTEM_NAME
    _env = platform.system().lower()
    if "linux" in _env:
        SYSTEM_NAME = "linux"
    elif "darwin" in _env:
        machine = "x86" if "x86" in platform.machine().lower() else "arm64"
        SYSTEM_NAME = f"darwin_{machine}"
    else:
        SYSTEM_NAME = "windows"
    return SYSTEM_NAME


def get_system_name():
    global SYSTEM_NAME
    if len(SYSTEM_NAME):
        return SYSTEM_NAME
    return set_system_name()


def rm_rf(file_path):
    if os.path.isfile(file_path):
        os.remove(file_path)
    elif os.path.isdir(file_path):
        shutil.rmtree(file_path)
    return True


def calc_sha256(file_path: str) -> str:
    sha256_hash = hashlib.sha256()

    try:
        with open(file_path, "rb") as f:
            for byte_block in iter(lambda: f.read(4096), b""):
                sha256_hash.update(byte_block)
        return sha256_hash.hexdigest()
    except FileNotFoundError:
        print(f"Error: Not found {file_path}")
        return ""
    except PermissionError:
        print(f"Error: Permission not support {file_path}.")
        return ""
    except Exception as e:
        print(f"Exception: {str(e)}")
        return ""


def copy_file(source, target, force=True) -> bool:
    if not os.path.exists(source):
        print(f"Not found [{source}].")
        return False
    if not force and os.path.exists(target):
        return True

    target_dir = os.path.dirname(target)
    if target_dir:
        os.makedirs(target_dir, exist_ok=True)
    shutil.copy(source, target)
    return True


def need_settarget(target_file, target):
    if not os.path.exists(target_file):
        return True
    with open(target_file, "r", encoding='utf-8') as f:
        old_target = f.read().strip()
    print(f"old_target: {old_target}")
    if target != old_target:
        return True
    return False


def record_target(target_file, target):
    with open(target_file, "w", encoding='utf-8') as f:
        f.write(target)
    return True


def calc_md5sum(file_path: str) -> str:
    try:
        with open(file_path, "rb") as f:
            md5obj = hashlib.md5()
            md5obj.update(f.read())
            return md5obj.hexdigest()
    except FileNotFoundError:
        print(f"Error: Not found {file_path}")
        return ""
    except Exception as e:
        print(f"Exception: {str(e)}")
        return ""


def extract_archive(file_path: str, dest_dir: str) -> bool:
    if not os.path.exists(file_path):
        print(f"Error: Not found {file_path}.")
        return False

    os.makedirs(dest_dir, exist_ok=True)

    try:
        if file_path.endswith('.zip'):
            with zipfile.ZipFile(file_path, 'r') as zip_ref:
                zip_ref.extractall(path=dest_dir)
        elif file_path.endswith('.tar.gz'):
            with tarfile.open(file_path, 'r:gz') as tar:
                tar.extractall(path=dest_dir)
        elif file_path.endswith('.tar.bz2'):
            with tarfile.open(file_path, 'r:bz2') as tar:
                tar.extractall(path=dest_dir)
        elif file_path.endswith('.tar.xz'):
            with tarfile.open(file_path, 'r:xz') as tar:
                tar.extractall(path=dest_dir)
        elif file_path.endswith('.tar'):
            with tarfile.open(file_path, 'r') as tar:
                tar.extractall(path=dest_dir)
        else:
            print(f"Error: extract not support {file_path}.")
            return False
    except (zipfile.BadZipFile, tarfile.TarError) as e:
        print(f"Error: Invalid archive {file_path}: {e}")
        return False
    except Exception as e:
        print(f"Exception: {str(e)}")
        return False

    return True


def do_subprocess(cmd: str) -> int:
    if not cmd:
        print("Subprocess cmd is empty.")
        return 0

    print(f"[do subprocess]: {cmd}")

    ret = 1
    try:
        ret = os.system(cmd)
    except Exception as e:
        print(f"Do subprocess error: {str(e)}")
        print(f"do subprocess: {cmd}")
        return 1
    return ret

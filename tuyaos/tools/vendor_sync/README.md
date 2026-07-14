# vendor_sync：T9 原厂 `t9_os` 同步辅助工具

完整流程见：[`VENDOR_UPSTREAM_SYNC.md`](VENDOR_UPSTREAM_SYNC.md)

## 依赖

- bash
- git
- rsync
- ripgrep（`rg`）

## 工作方式

在 `vendor/T9` 仓库根目录执行，或任意目录传入 `--repo-root /path/to/vendor/T9`。

脚本通过同时存在 `t9_os/` 与 `.git` 识别仓库根。

## 工具列表

| 脚本 | 说明 |
|------|------|
| `lib/common.sh` | 公共辅助（定位根目录、依赖检查、路径排序） |
| `list_tuya_marked.sh` | 扫描 `Modified by TUYA` / `Modified by Tuya` / `Added by TUYA` |
| `list_business_delta.sh` | 业务相对 upstream 的 `git diff --name-status` |
| `import_pristine.sh` | rsync 干净原厂树到 `t9_os/`（排除 `.git`；不 push/tag） |
| `report_upgrade.sh` | 升级影响报告：原厂 diff、业务 delta、业务∩原厂主队列 |
| `merge_assist.sh` | 冲突 ∪ 交集 ∪ 标注触碰；有冲突时 exit 2 |
| `tests/run_tests.sh` | fixture 回归测试 |

## 用法示例

```bash
cd /path/to/vendor/T9

# 扫描 TUYA 标注
./tuyaos/tools/vendor_sync/list_tuya_marked.sh

# 业务相对某基线的完整 delta（含新增文件）
./tuyaos/tools/vendor_sync/list_business_delta.sh --upstream upstream/bk-4.0.1.4

# 在 upstream/bk 分支上导入干净原厂（先手动 checkout upstream 分支）
./tuyaos/tools/vendor_sync/import_pristine.sh --src /path/to/oem_checkout --commit \
  --message "Import pristine BK/Armino t9_os 4.0.1.5"

# 升级影响报告
./tuyaos/tools/vendor_sync/report_upgrade.sh \
  --old upstream/bk-4.0.1.4 \
  --new upstream/bk-4.0.1.5 \
  --out /tmp/t9_upgrade_report.md

# 合并后人工队列
./tuyaos/tools/vendor_sync/merge_assist.sh \
  --old upstream/bk-4.0.1.4 \
  --new upstream/bk-4.0.1.5

# 回归测试
./tuyaos/tools/vendor_sync/tests/run_tests.sh
```

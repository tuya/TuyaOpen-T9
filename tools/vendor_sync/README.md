# vendor_sync：T9 原厂 `t9_os` 同步辅助工具

配合流程文档：`../../docs/VENDOR_UPSTREAM_SYNC.md`（Task 7 后可用）。

## 依赖

- bash
- git
- rsync
- ripgrep（`rg`）

## 工作方式

在 `vendor/T9` 仓库根目录执行，或任意目录传入 `--repo-root /path/to/vendor/T9`。

脚本通过存在 `t9_os/` 且存在 `.git` 来识别仓库根。

## 工具列表

| 脚本 | 说明 | 状态 |
|------|------|------|
| `lib/common.sh` | 公共辅助函数 | 已提供 |
| `list_tuya_marked.sh` | 扫描 TUYA 标注文件 | 待补 |
| `list_business_delta.sh` | 业务相对 upstream 的 delta | 待补 |
| `report_upgrade.sh` | 升级影响报告（业务∩原厂） | 待补 |
| `import_pristine.sh` | 导入干净原厂树到 `t9_os/` | 待补 |
| `merge_assist.sh` | 合并后人工复核队列 | 待补 |
| `tests/run_tests.sh` | fixture 回归测试 | 待补 |

## 快速检查

```bash
# 从 vendor/T9 根目录
bash -n tools/vendor_sync/lib/common.sh
```

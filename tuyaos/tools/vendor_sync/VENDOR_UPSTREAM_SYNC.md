# T9 原厂 `t9_os` Upstream 同步操作手册

配套设计：仓库外 `tuyaos-iot` 文档 `docs/superpowers/specs/2026-07-14-t9-vendor-upstream-sync-design.md`  
配套工具：本目录脚本（路径：`tuyaos/tools/vendor_sync/`）  
工具详解：[`VENDOR_SYNC_GUIDE.md`](VENDOR_SYNC_GUIDE.md)

本手册假设工作目录为 **vendor/T9 仓库根**（该目录自身为 git root）。

---

## 0. 前置条件

1. 已取得原厂干净 checkout（git tag），版本记为 `N`，并记下原厂 commit SHA。
2. 业务分支工作区干净（无未提交改动）。
3. 工具可用：`bash`、`git`、`rsync`、`rg`。
4. （建议）打备份 tag：`backup/before-upstream-bootstrap`。

---

## 1. 首次建立干净基线（一次性）

目标：`upstream/bk` 历史只含原厂导入；`t9_os/` 与原厂 `N` 一致；**无嵌套 `.git`**；**不含 `tuyaos/`**。

```bash
# 1) orphan 分支
git checkout --orphan upstream/bk
git rm -rf .   # 谨慎：确认已备份

# 2) 只导入干净 t9_os（排除原厂 .git）
mkdir -p t9_os
./tuyaos/tools/vendor_sync/import_pristine.sh --src /path/to/oem_clean_N --commit \
  --message "Import pristine BK/Armino t9_os ${N}"

# 如 orphan 后工具脚本已被 git rm，先从业务分支取出工具再执行 import：
# git checkout feature/vendor_sync-tools -- tuyaos/tools/vendor_sync

# 3) tag 并推送（人工确认后）
git tag -a "upstream/bk-${N}" -m "Pristine OEM t9_os ${N}; oem_sha=<SHA>"
git push -u origin upstream/bk
git push origin "upstream/bk-${N}"
```

验收：

- `t9_os/.git` 不存在
- 与原厂 `N` 的 `diff` 无未解释差异
- `git log` 无业务/TUYA 提交

---

## 2. 业务分支挂接公共祖先（强烈建议，一次性）

推荐 **2A**。可在业务主分支（如 `feature/t9_smp`）上做，也可先在特性分支（如 `feature/vendor_sync-tools`）上挂接，验证后再合回主业务分支。

### 2.1 为何会报 `refusing to merge unrelated histories`

`upstream/bk` 由 **orphan** 创建，与业务分支**没有共同祖先**（`git merge-base` 为空）。首次挂接时 Git 会拒绝合并，这是预期现象，不是 tag 做错了。

### 2.2 首次挂接命令（必须加 `--allow-unrelated-histories`）

```bash
git checkout feature/t9_smp   # 或 feature/vendor_sync-tools 等业务/特性分支
git merge "upstream/bk-${N}" \
  --allow-unrelated-histories \
  -m "Merge pristine upstream/bk-${N} as vendor baseline"
```

冲突原则：

- 纯原厂文件 → 取 upstream
- 已有 TUYA 定制 → 保留业务侧，确认标注仍在
- `tuyaos/` 仅业务侧存在 → 保留业务侧

### 2.3 挂接之后

- 业务历史已与 `upstream/bk` 相连；**以后**合并 `upstream/bk-${N+1}` 时**一般不再需要** `--allow-unrelated-histories`。
- 该参数**仅用于**「orphan 干净基线 ↔ 业务」的这一次首次挂接，不要用于其它无关分支乱合。
- 可选核对：`./tuyaos/tools/vendor_sync/list_business_delta.sh --upstream upstream/bk-${N} | head`

---

## 3. 常态升级 `N` → `N+1`

### 3.1 在 upstream 线上更新

```bash
git checkout upstream/bk
./tuyaos/tools/vendor_sync/import_pristine.sh --src /path/to/oem_clean_N+1 --commit \
  --message "Import pristine BK/Armino t9_os ${N+1}"
git tag -a "upstream/bk-${N+1}" -m "Pristine OEM t9_os ${N+1}; oem_sha=<SHA>"
git push origin upstream/bk "upstream/bk-${N+1}"
```

### 3.2 合入业务升级分支

```bash
git checkout feature/t9_smp
git checkout -b "feature/t9_smp_update_bk_v${N+1}"
git merge "upstream/bk-${N+1}"
```

### 3.3 报告与人工队列

```bash
./tuyaos/tools/vendor_sync/report_upgrade.sh \
  --old "upstream/bk-${N}" \
  --new "upstream/bk-${N+1}" \
  --out "t9_os/TUYA_BK_SDK_UPDATE_${N+1}_NOTES.md"

./tuyaos/tools/vendor_sync/merge_assist.sh \
  --old "upstream/bk-${N}" \
  --new "upstream/bk-${N+1}"
```

人工优先处理：

1. 冲突文件（merge_assist exit 2）
2. **业务∩原厂**（PRIMARY REVIEW QUEUE）
3. TUYA 标注触碰子集
4. 业务新增文件与原厂新增路径碰撞

可将 report 输出作为 `TUYA_BK_SDK_UPDATE_*_NOTES.md` 初稿，再补模块风险与测试矩阵。

### 3.4 验证

编译 + 红/黄项冒烟 → Review → 合回业务分支。旧 tag **保留**。

---

## 4. 禁止做法

- 在业务分支整树覆盖 `t9_os` 后再切「upstream」分支冒充干净历史
- 用单一 upstream 分支 force 覆盖而不打版本 tag
- 把原厂嵌套 `.git` 提交进本仓库
- 把 `tuyaos/` 导入 upstream 快照

---

## 5. 回归测试

```bash
./tuyaos/tools/vendor_sync/tests/run_tests.sh
```

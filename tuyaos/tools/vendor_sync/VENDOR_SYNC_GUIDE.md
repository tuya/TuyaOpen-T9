# vendor_sync 工具详解与使用流程

> 文档：`tuyaos/tools/vendor_sync/VENDOR_SYNC_GUIDE.md`  
> 仓库根：`vendor/T9`（该目录自身为 git root）  
> 操作手册（步骤 checklist）：[`VENDOR_UPSTREAM_SYNC.md`](VENDOR_UPSTREAM_SYNC.md)  
> 简要用法：[`README.md`](README.md)

本文说明每个工具的作用、适用时机、参数含义，以及端到端使用流程（含合并冲突时的脚本执行顺序）。

---

## 1. 设计背景（一句话）

用 **干净原厂 upstream tag + git 三方合并** 替代整树 Beyond Compare；用脚本自动算出 **「业务动过 ∧ 原厂也动过」** 的人工主队列。  
`Modified by TUYA` 标注只是子集；新建 CSI/屏幕驱动等**未标注**改动，靠业务相对 upstream 的完整 delta 覆盖。

---

## 2. 目录与依赖

| 文件 | 类型 |
|------|------|
| `lib/common.sh` | 公共库（被其它脚本 source，一般不单独执行） |
| `import_pristine.sh` | 导入干净原厂到 `t9_os/` |
| `list_tuya_marked.sh` | 列出带 TUYA 标注的文件 |
| `list_business_delta.sh` | 列出业务相对 upstream 的完整变更 |
| `report_upgrade.sh` | 生成升级影响 Markdown 报告 |
| `merge_assist.sh` | 合并后/冲突中的人工处理队列 |
| `tests/run_tests.sh` | fixture 回归测试 |
| `VENDOR_UPSTREAM_SYNC.md` | 首次基线 / 常态升级操作步骤 |
| `VENDOR_SYNC_GUIDE.md` | 工具详解与场景流程（本文） |
| `README.md` | 短用法 |

**依赖：** `bash`、`git`、`rsync`；`rg`（ripgrep）可选，未安装时标注扫描回退到 `grep`

**工作方式：**

```bash
cd /path/to/vendor/T9
# 或任意目录，加：
#   --repo-root /path/to/vendor/T9
```

脚本通过「存在 `t9_os/` + `.git`」识别仓库根。

---

## 3. 各工具详解

### 3.1 `lib/common.sh`（库）

**作用：** 不单独作为命令使用。提供：

- `vs_die` / `vs_need_cmd`：统一报错与依赖检查  
- `vs_resolve_repo_root`：根据 `--repo-root` 或向上查找定位 `vendor/T9`  
- `vs_lines_sort`：路径列表按 `LC_ALL=C` 排序、去空行、去 CR  

其它 `*.sh` 通过 `source "$SCRIPT_DIR/lib/common.sh"` 引用。

---

### 3.2 `import_pristine.sh` — 导入干净原厂树

**作用：** 用 `rsync` 把原厂干净源码同步进当前仓库的 `t9_os/`。

- 排除：`.git/`、`.github/`、`.vscode/`、`.idea/`  
- 导入后若仍存在 `t9_os/.git` 会报错退出（防止嵌套仓库）  
- 可选 `--commit`（只提交 `t9_os`）  
- **不会**自动打 tag、**不会** push  

**何时用：**

1. 首次在 `upstream/bk`（orphan）上建立干净基线  
2. 常态升级时，在 `upstream/bk` 上导入原厂 `N+1`  

**注意：** `upstream/bk` 上通常没有 `tuyaos/`，脚本可能不在当前树。可先：

```bash
git checkout <业务分支> -- tuyaos/tools/vendor_sync
```

或从另一 worktree / 绝对路径调用脚本，并加 `--repo-root`。

**用法：**

```bash
# 先确保当前就在要写入的分支上（一般是 upstream/bk）
./tuyaos/tools/vendor_sync/import_pristine.sh \
  --src /path/to/oem_clean_checkout \
  --commit \
  --message "Import pristine BK/Armino t9_os 4.0.1.5"
```

| 参数 | 含义 |
|------|------|
| `--src DIR` | 原厂干净树根目录（必填）；其内容会同步到 `t9_os/` |
| `--commit` | 导入后 `git add -A t9_os` 并 commit |
| `--message MSG` | 提交说明（配合 `--commit`） |
| `--repo-root DIR` | 指定 vendor/T9 根 |

导入成功后需**人工**：

```bash
git tag -a upstream/bk-<版本> -m "Pristine OEM ...; oem_sha=<SHA>"
git push origin upstream/bk "upstream/bk-<版本>"
```

---

### 3.3 `list_tuya_marked.sh` — TUYA 标注文件清单

**作用：** 在 `t9_os` 下搜索并列出包含以下字符串的文件（路径相对仓库根，排序去重）：

- `Modified by TUYA`  
- `Modified by Tuya`  
- `Added by TUYA`  

排除 `bk_tflite_micro` 等大噪声路径。

**含义边界：** 这只是业务改动的**子集**。没写标注的修改、纯新建驱动文件，不会出现在这里。

**何时用：**

- 清点「已知标注定制点」  
- 配合 `report_upgrade` 看「标注 ∩ 原厂变更」  
- 日常抽查标注是否还在  

**用法：**

```bash
./tuyaos/tools/vendor_sync/list_tuya_marked.sh
./tuyaos/tools/vendor_sync/list_tuya_marked.sh --repo-root /path/to/vendor/T9
```

---

### 3.4 `list_business_delta.sh` — 业务相对 upstream 的完整 delta

**作用：** 执行：

```text
git diff --name-status <upstream>...HEAD -- t9_os
```

输出状态码 + 路径，例如：

| 状态 | 含义 |
|------|------|
| `M` | 业务改过原厂已有文件（有无标注都算） |
| `A` | 业务新建文件（如 CSI/屏幕驱动） |
| `D` | 业务删过的文件 |

**何时用：**

- 升级前看清「我们相对某基线动过什么」  
- 找未标注改动、补 `Added by TUYA` / 文件清单  
- 与原厂 `old..new` 手工对照（完整自动化见 `report_upgrade`）  

**用法：**

```bash
./tuyaos/tools/vendor_sync/list_business_delta.sh \
  --upstream upstream/bk-4.0.1.4
```

| 参数 | 含义 |
|------|------|
| `--upstream REF` | 干净基线 tag/commit（必填），一般为当前业务所基于的 upstream |
| `--repo-root DIR` | 可选 |

---

### 3.5 `report_upgrade.sh` — 升级影响报告（核心）

**作用：** 对比「旧 upstream / 新 upstream / 当前 HEAD」，生成 Markdown 报告。

内部逻辑概要：

1. **OEM changed** = `git diff --name-only old..new -- t9_os`  
2. **Business delta** = `git diff --name-only old...HEAD -- t9_os`（含新增）  
3. **Intersection（PRIMARY REVIEW QUEUE）** = 业务 ∩ 原厂 → **人工主队列**  
4. **TUYA-marked ∩ OEM** = 标注列表 ∩ 原厂变更  
5. **Business-only added** = 业务新增文件  
6. **Path collisions** = 业务新增路径 ∩ 原厂新增路径  
7. **Diffstat** = 原厂 old..new 的 `--stat`  

**与是否正在 merge 冲突无关：** 比的是 tag 与 `HEAD`（冲突未提交完成时，`HEAD` 仍是业务侧 tip），**merge 前 / 冲突中 / merge 完成后都可以跑。**

**何时用：**

- 升版前预览风险面  
- 冲突中辅助排优先级  
- merge 完成后作为 `TUYA_BK_SDK_UPDATE_*_NOTES.md` **初稿**（再人工补风险分级、TKL、测试矩阵）  

**用法：**

```bash
./tuyaos/tools/vendor_sync/report_upgrade.sh \
  --old upstream/bk-4.0.1.4 \
  --new upstream/bk-4.0.1.5 \
  --out t9_os/TUYA_BK_SDK_UPDATE_4.0.1.5_NOTES.md
```

省略 `--out` 则打印到 stdout。

| 参数 | 含义 |
|------|------|
| `--old REF` | 旧干净 upstream tag（必填） |
| `--new REF` | 新干净 upstream tag（必填） |
| `--out FILE` | 写入文件（可选） |
| `--repo-root DIR` | 可选 |

**读报告时优先看：**「Intersection / PRIMARY REVIEW QUEUE」，不是只看标注章节。

---

### 3.6 `merge_assist.sh` — 合并人工队列

**作用：** 输出三节，便于按清单打开文件处理：

1. **CONFLICTS** — 当前未解决冲突文件（`git diff --diff-filter=U`）  
2. **INTERSECT** — 业务 ∩ 原厂（同 report 主队列）  
3. **MARKED_OEM** — 标注 ∩ 原厂  

**退出码：**

| 码 | 含义 |
|----|------|
| `0` | 当前无冲突 |
| `2` | 仍有冲突 |

**关键时机：** `CONFLICTS` 只有在 **merge 已开始但尚未解决完并 commit** 时才有内容。若你已经解完冲突并提交，再跑则 CONFLICTS 为空，错过最佳时机。

**用法：**

```bash
./tuyaos/tools/vendor_sync/merge_assist.sh \
  --old upstream/bk-4.0.1.4 \
  --new upstream/bk-4.0.1.5
```

**建议处理顺序：** CONFLICTS → INTERSECT → MARKED_OEM。

---

### 3.7 `tests/run_tests.sh` — 回归测试

**作用：** 在临时 fixture 仓库中验证：

- 标注扫描排序  
- 业务 delta  
- `import_pristine` 排除嵌套 `.git`  
- report 交集非空（两边都改的文件）  
- merge_assist exit 0 + 交集  

**何时用：** 改脚本逻辑后、合入前。

```bash
./tuyaos/tools/vendor_sync/tests/run_tests.sh
```

---

## 4. 工具在流程中的位置（总图）

```text
原厂干净包
    │
    ▼
import_pristine.sh  ──►  upstream/bk 上 commit
    │
    ▼ 人工 tag / push
upstream/bk-N 、 upstream/bk-N+1
    │
    ▼ git merge（业务分支）
业务树（含 TUYA / 新驱动 / tuyaos）
    │
    ├─ list_business_delta.sh   业务相对旧基线完整变更
    ├─ list_tuya_marked.sh      标注子集
    ├─ report_upgrade.sh        影响报告（含业务∩原厂）
    └─ merge_assist.sh          冲突队列 + 交集队列
```

---

## 5. 场景流程

### 5.1 场景 A：首次建立 upstream 基线（一次性）

目标：冻结一份干净原厂 `N`，并让业务分支与之有正确公共祖先。

1. 备份业务分支；准备原厂干净 checkout（记下 SHA）。  
2. `git checkout --orphan upstream/bk`（注意备份；必要时取回工具目录）。  
3. **`import_pristine.sh --src <原厂N> --commit`**  
4. `git tag upstream/bk-N` 并 push。  
5. 回到业务分支（或特性分支如 `feature/vendor_sync-tools`）挂接基线。  
   orphan 与业务无共同祖先，首次合并会报 `refusing to merge unrelated histories`，需：

   ```bash
   git merge upstream/bk-N \
     --allow-unrelated-histories \
     -m "Merge pristine upstream/bk-N as vendor baseline"
   ```

   之后升级再 `git merge upstream/bk-N+1` 时，一般**不再**需要该参数。  
6. 可选：`list_business_delta.sh --upstream upstream/bk-N` 核对业务相对基线的改动。  

此阶段一般还用不上「两版原厂」的 `report_upgrade`（还没有 N+1）。

详细命令见 `VENDOR_UPSTREAM_SYNC.md` §1–§2。

---

### 5.2 场景 B：原厂 N → N+1（常态升级）**【最常用】**

#### 步骤总览

```text
① checkout upstream/bk
② import_pristine（N+1）→ commit → tag upstream/bk-N+1 → push
③ checkout 业务分支 → 拉升级分支 → git merge upstream/bk-N+1
④ （若有冲突）先跑 merge_assist + report_upgrade，再解冲突
⑤ 完成 merge commit 后，用 report 写 NOTES，复查 INTERSECT（含自动合并项）
⑥ 编译 / 冒烟 → Review → 合回业务分支
```

#### ③～⑤：冲突时先脚本还是先解冲突？

**先跑脚本，再解冲突。**

| 时机 | 建议动作 |
|------|----------|
| merge 前（可选） | 先跑一遍 `report_upgrade --old N --new N+1`，心里有 INTERSECT 名单 |
| `git merge` 后若有冲突 | **立刻**跑 `merge_assist`（此时 CONFLICTS 有内容）+ `report_upgrade` |
| 然后 | 按 CONFLICTS → INTERSECT → MARKED 解冲突并 `git add` / 完成 merge |
| merge 完成后 | 再落盘/润色 report 作 NOTES；复查**无冲突但在 INTERSECT 里**的文件（自动合并仍可能行为错）|

原因简述：

- `merge_assist` 的 CONFLICTS 依赖「仍在 merging」状态；解完再跑会变成空。  
- `report_upgrade` 不依赖冲突状态，前/中/后都可用。  

#### 示例命令（在 vendor/T9 根）

```bash
# --- ①② upstream 导入 ---
git checkout upstream/bk
# 若缺工具：git checkout feature/xxx -- tuyaos/tools/vendor_sync
./tuyaos/tools/vendor_sync/import_pristine.sh \
  --src /path/to/oem_N+1 --commit \
  --message "Import pristine BK/Armino t9_os ${N+1}"
git tag -a "upstream/bk-${N+1}" -m "Pristine OEM ...; oem_sha=<SHA>"
git push origin upstream/bk "upstream/bk-${N+1}"

# --- ③ merge ---
git checkout feature/t9_smp   # 你的业务分支
git checkout -b "feature/t9_smp_update_bk_v${N+1}"
git merge "upstream/bk-${N+1}"

# --- ④ 有冲突：先报告再解 ---
./tuyaos/tools/vendor_sync/merge_assist.sh \
  --old "upstream/bk-${N}" --new "upstream/bk-${N+1}"
./tuyaos/tools/vendor_sync/report_upgrade.sh \
  --old "upstream/bk-${N}" --new "upstream/bk-${N+1}" \
  --out "/tmp/t9_upgrade_${N+1}.md"
# ... 按队列解冲突，完成 merge ...

# --- ⑤ NOTES ---
./tuyaos/tools/vendor_sync/report_upgrade.sh \
  --old "upstream/bk-${N}" --new "upstream/bk-${N+1}" \
  --out "t9_os/TUYA_BK_SDK_UPDATE_${N+1}_NOTES.md"
```

---

### 5.3 场景 C：平时抽查（不升版）

```bash
./tuyaos/tools/vendor_sync/list_tuya_marked.sh

./tuyaos/tools/vendor_sync/list_business_delta.sh \
  --upstream upstream/bk-<当前基线版本>
```

用于补标注、梳理新建驱动落点、评估「哪些改动适合迁出 `t9_os`」。

---

## 6. 工具 vs 人工边界

| 工具负责 | 你负责 |
|----------|--------|
| 导入干净树、排除嵌套 `.git` | 选择正确分支；orphan/tag/push |
| 计算业务 delta / 交集 / 标注触碰 | 解冲突时保留正确的 TUYA 逻辑与行为 |
| Markdown 报告骨架与文件名单 | 风险分级、TKL 影响、测试矩阵与冒烟结论 |
| exit 码提示是否有冲突 | 编译与板上验证 |

---

## 7. 禁止做法（再强调）

1. 在**业务分支**上整树覆盖 `t9_os` 后再伪造成 `upstream`（历史含 TUYA，三方合并变差）。  
2. 用单一 upstream 分支 force 覆盖、不打版本 tag（丢掉可回溯祖先）。  
3. 把原厂嵌套 `.git` 提交进本仓库。  
4. 把 `tuyaos/`（含本工具）导入 upstream 快照。  
5. 只看标注列表、忽略「业务∩原厂」和新建文件。  

---

## 8. 快速对照表

| 我现在想… | 用哪个 |
|-----------|--------|
| 把干净原厂灌进 `t9_os` | `import_pristine.sh` |
| 看有哪些 TUYA 标注文件 | `list_tuya_marked.sh` |
| 看业务相对某 upstream 改了啥（含新文件） | `list_business_delta.sh` |
| 出升级影响报告 / NOTES 初稿 | `report_upgrade.sh` |
| merge 冲突中拿处理队列 | `merge_assist.sh`（冲突未解时） |
| 改完脚本自测 | `tests/run_tests.sh` |
| 跟手册逐步点操作 | `VENDOR_UPSTREAM_SYNC.md` |

---

配套设计见上层仓库：`docs/superpowers/specs/2026-07-14-t9-vendor-upstream-sync-design.md`（若有）。

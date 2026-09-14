# Tuya Face Recognition（Detect + Gallery）Demo 移植说明

本文档记录 `test_face_recog` 人脸检测 + 特征比对 demo 在 Tuya 代码框架里的适配方式、运行命令、相关文件改动和已遇到的问题。

> 说明：本 demo 与 `test_face_recognize`（`xface`，YoloFace 56×56 仅检测+画框）不同。本目录实现的是原厂 SCRFD 检测 + 对齐 + embedding 校验 + SD 人脸库，CLI 入口为 **`xrecog`**。

## 目标

原始 demo 位于 `vendor/T9/t9_os/projects/tflite_micro/face_recognition`（及配套 AVDK face recog 链路），启动后自动打开 MIPI 摄像头、MIPI 屏、GPU，并通过 SP 通道取 **320×320** RGB 图像，跑人脸检测模型，再做对齐与特征提取/比对。

Tuya 侧移植目标是：

- 不直接依赖原始 `projects/common_components/avdk_nn_module` 的实现。
- 在 `tuyaos_adapter` 里本地保留一份多模型加载、检测/校验 runtime 与 gallery 实现。
- 复用已有 `xsd`、`xmipi` 测试命令挂载 SD、打开摄像头和屏幕。
- 通过 **`xrecog`** 命令读取 MIPI **SP** 通道图像，执行检测 / 注册 / 1:N 校验。
- 检测与比对结果先串口打印；**当前不画人脸框**（不依赖 `box.c` / GPU overlay）。

## 本目录文件

`vendor/T9/tuyaos/tuyaos_adapter/src/test/test_face_recog` 中的主要文件如下：

- `test_face_recog.cc`：Tuya CLI demo 入口，实现 `xrecog` 命令、模型常驻、SP 取帧和 detect/enroll/verify 循环。
- `AvdkMultiModel.cc/.h`：从原 demo 的多模型/TFLM 加载逻辑拷贝并适配，支持从 SD 加载 Vela tflite、interpreter、tensor arena。
- `FaceDetectionRuntime.cc/.h`：SCRFD 风格检测 runtime（输入 320×320 int8），阈值 / NMS / decode。
- `FaceVerifyRuntime.cc/.h`：对齐后人脸 embedding 推理与 L2 归一化（512 维）。
- `FaceRecognitionModel.cc/.h`：检测 → CPU similarity warp（112×112）→ 可选特征提取的整条 pipeline。
- `face_gallery.cc/.h`：基于 `tkl_fs` 的 SD 人脸库（注册样本、1:N cosine、list/clear）。
- `avdk_nn_types.h`：本地化所需类型头文件。

> 启用方式：在 `vendor/T9/tuyaos/tuyaos_adapter/CMakeLists.txt` 中取消 `test_face_recog` 相关源文件注释，并在 `cli_tuya_test_1.c` 中取消 `{"xrecog", ...}` 注册注释后重新编译。

## 运行命令

模型与人脸库都在 SD 上，因此必须先 `xsd mount`。

**只跑识别，不要预览**（最短路径，不需要屏幕）：

```text
ap_cmd xsd mount
ap_cmd xmipi cam open frame 640 1024 rgb888 320 320
ap_cmd xrecog init
ap_cmd xrecog detect once
ap_cmd xrecog dump
ap_cmd xrecog enroll once
ap_cmd xrecog verify once
ap_cmd xrecog list
ap_cmd xrecog clear all
ap_cmd xrecog release
ap_cmd xmipi cam close
```

**要同时上屏预览**：先把面板点起来，摄像头用 `display` 打开 MP：

```text
ap_cmd xsd mount
ap_cmd xmipi dsi open_gpu jd9165t_mipi_1024x600 1
ap_cmd xmipi cam open display 640 1024 rgb888 320 320
ap_cmd xrecog init
...
```

要点：

- **SP 必须正好 320×320 且是 `rgb888`**。不匹配时 `recog_read_sp_frame()` 直接拒绝并打印
  `... (need 320x320 BGRX)`；用 `nv12` 打开 SP 同样不行。
- `open` 的参数含义：`<mp_kinds> [mp_w] [mp_h] [sp_fmt] [sp_w] [sp_h]`。
  MP 是唯一能挂 flexa 的通道，SP 恒为 frame 模式，只供取帧。
- MP 用 `frame` 还是 `display` 只影响有没有预览，**对识别没有影响** —— 模型只吃 SP。
- **不再需要 `xgpu`**：GPU 已经收进摄像头 facade 内部，`display` 一开就自动建好。
- 人脸需相对正对、距离合适；侧脸/过远时检测分容易低于阈值。
- 若检测长期偏低，先用 `xrecog dump` 看 SP 图是否倒置/旋转。

### SD 资源准备

```text
/sdcard/tflite/face_detection_int8_vela.tflite
/sdcard/tflite/face_verify_int8_vela.tflite
/sdcard/faces/                          # xrecog init / enroll 自动创建
```

人脸库目录约定：

```text
/sdcard/faces/face_XXXX/
  face_N.ppm        # 112x112 RGB 对齐图
  feature_N.bin     # 512-d L2 embedding
```

每个 ID 默认 **3** 个样本（`FACE_GALLERY_SAMPLES_PER_ID`）。

## xrecog 命令说明

`test_face_recog.cc` 提供以下命令：

- `xrecog help`：打印帮助。
- `xrecog init`：准备 SD/gallery，加载检测+校验模型，分配 320×320×4 帧缓存。
- `xrecog detect once|start [delay_ms]|stop`：仅检测；`start` 循环，`delay_ms` 默认 200ms。
- `xrecog enroll once|start [delay_ms]|stop`：检测 + 对齐 + 提特征，写入 gallery。
- `xrecog verify once|start [delay_ms]|stop`：检测 + 对齐 + 提特征，对 gallery 做 1:N。
- `xrecog thr <0.0-1.0>`：覆盖检测 score 阈值，**默认 0.5**（与原厂一致）。
- `xrecog layout auto|bgrx|rgbx`：4bpp 通道顺序；本板默认/推荐 **bgrx**（`rgbx` 更差）。
- `xrecog dump [path]`：把当前 SP 320×320 存成 RGB PPM（默认 `/sdcard/sp_dump.ppm`），用于核对朝向/内容。
- `xrecog list`：列出 gallery profile 与样本数。
- `xrecog clear [id|all]`：清除指定 ID 或全部。
- `xrecog release`：释放常驻模型和帧缓存；必须先停任务。
- `xrecog status`：打印 running/stop/mode/rounds/delay/model_ready。

模型是常驻的。`xrecog init` 成功后，后续 `detect/enroll/verify` 复用同一模型，直到 `xrecog release`。

### 阈值约定

| 环节 | 默认值 | 说明 |
|------|--------|------|
| 检测 score | `0.5` | `FaceDetectionRuntime` / `xrecog thr` |
| 最小框 | `32×32` | 小于此尺寸的框丢弃，避免噪声框 |
| 校验 best cosine | `0.70` | `kFaceVerifySameThreshold` |
| 校验 avg cosine | `0.62` | `FACE_GALLERY_AVG_THRESHOLD` |

PASS 条件：`best >= 0.70` **且** `avg >= 0.62`。

## 数据流

当前数据链路如下：

1. `xsd mount` 挂载 SD，保证 tflite 与 `/sdcard/faces` 可访问。
2. `xmipi cam open` 一步带起 sensor、ISP、两个通道，以及（选了 `display` 时）GPU 与面板：
   - **MP** 通道：`display` / `h264` 走 flexa，直接进 GPU 或编码器。
   - **SP** 通道：模型输入，**320×320** RGB888（实际 4bpp），frame mode。
3. `xrecog` 通过 `tkl_mipi_csi_get_frame(&frame, TUYA_MIPI_CSI_CHANNEL_SECONDARY, timeout)`
   取帧。**帧是借来的**：用完必须 `frame.free_cb(frame.data)` 还回去，见下面「帧归还契约」。
4. `FaceRecognitionModel::run()`：
   - 4bpp BGRX→RGB，像素值减 128，得到检测 int8 输入；
   - 检测 decode + NMS + 尺寸过滤；
   - enroll/verify 时对最佳脸做 CPU similarity warp → 112×112；
   - `FaceVerifyRuntime` 提 512-d embedding 并 L2 归一化。
5. enroll：`face_gallery_enroll_save()` 写 `face_N.ppm` + `feature_N.bin`。
6. verify：`face_gallery_verify()` 对完整 profile（3 样本）做 cosine，取 best/avg 判 PASS/FAIL。

SP 帧大小按 `320 * 320 * 4` 处理。虽然格式常标记为 `BK_PIXEL_FORMAT_RGB888`，但底层与模型预处理按 **4 bytes/pixel** 使用，并按 **BGRX** 拆通道（与原厂 FaceRecognition / `xface` 侧约定一致）。

## 相关 TKL 和测试文件适配

为跑通该 demo，除本目录外还依赖以下已有适配（多数在 `xface` 移植时已完成，本 demo 复用）。

### tkl_mipi_csi.h/c

路径：

- `vendor/T9/tuyaos/tuyaos_adapter/include/mipi/tkl_mipi_csi.h`
- `vendor/T9/tuyaos/tuyaos_adapter/src/driver/tkl_mipi_csi.c`

主要能力：

- `TKL_MIPI_CSI_CHANNEL_PRIMARY` / `TKL_MIPI_CSI_CHANNEL_SECONDARY`。
- SP 配置字段集中在 `cfg.secondary`：`output_kinds`、`width`、`height`、`format`。
  SP 只接受 `TKL_MIPI_CSI_OUTPUT_FRAME`，设为 `TKL_MIPI_CSI_OUTPUT_NONE` 即关闭。缓冲数量、
  flexa 标志和 work mode 由适配层根据 path 配置推导。
- `tkl_mipi_csi_get_frame()` 支持按 channel 读 MP 或 SP。
- SP 为 RGB888 时返回大小按 `width * height * 4`（这颗 PP 的 RGB888 是 4 字节/像素）。
- MP 挂了 display 或 h264 就在跑 flexa，没有帧队列可读，取帧返回 `OPRT_NOT_SUPPORTED`；
  SP frame mode 可直接读。

#### 帧归还契约（重要）

`tkl_mipi_csi_get_frame()` **借出**缓冲，不转移所有权：

```c
TUYA_MEDIA_BUF_T frame;
if (tkl_mipi_csi_get_frame(&frame, TUYA_MIPI_CSI_CHANNEL_SECONDARY, timeout) == OPRT_OK) {
    /* ... 用 frame.data ... */
    frame.free_cb(frame.data);   /* 每条退出路径都要走到 */
}
```

释放函数随帧走（`frame.free_cb`），调用方不需要知道它来自哪个池，也**不要**自己
`free()`。SP 池只有 2 级深，**漏还一次就少一格**：本 demo 每轮 400 KB，
历史上漏还要跑到约 26 轮才把池子卡死，日志里表现为 `alloc failed`。
`recog_read_sp_frame()` 的每个失败分支都已经补上归还。

### xmipi 测试命令

路径：

- `src/test/test_media/test_mipi.c`：`xmipi` 前门，只负责分发到 `cam` / `dsi`。
- `src/test/test_media/test_camera_mipi.c`：`xmipi cam ...`
- `src/test/test_media/test_mipi_dsi.c`：`xmipi dsi ...`

主要作用：

- `xmipi cam open <mp_kinds> [w] [h] [sp_fmt] [spw] [sph]`：一步带起整条管线。
  本 demo 用 SP=`rgb888 320 320`（不是 `xface` 的 `56 56`）。
- `xmipi dsi open_gpu [panel] [compress]`：面板 + DPU decompress，只有要预览时才需要。
- `xmipi cam close`：整条管线落下。

> **已删除的命令**：`xgpu`（`test_gpu.c`）和 `xmipi camera <sub>` 都不存在了。
> GPU 已经收进摄像头 facade 内部，业务和测试都不再直接持有 GPU 句柄；
> `xmipi camera` 整套被 `xmipi cam` 取代。

### 与 xface 的差异（对照）

| 项 | `xface`（`test_face_recognize`） | `xrecog`（`test_face_recog`） |
|----|----------------------------------|-------------------------------|
| 模型 | YoloFace 56×56（内嵌数组） | SCRFD 320×320 + verify（SD Vela） |
| 输入 SP | 56×56 | 320×320 |
| 能力 | 检测 + GPU 画框 | 检测 + 对齐 + 注册/1:N |
| 画框 | `box.c` + flex overlay | 无（仅串口） |
| 依赖 SD 模型 | 否 | 是 |

## 已遇到的问题

### 检测长期 faces=0 / score 偏低

现象：`max_score` 很低，或 `pass>0` 但 `faces=0`。

原因与处理：

- SP 图相对预览旋转/侧视：正对后 score 可稳定到 **0.5+**；阈值恢复原厂 **0.5**。
- 大量小框被 **32×32** 最小框过滤：日志里可能出现 `max_score` 较高但 `faces=0`。
- `2304×1296 → 320×320` 是非方形 FOV 压缩，细节不如原厂方形 sensor 路径；尽量让人脸占画面足够大。
- 通道顺序：保持 `layout bgrx`；`rgbx` 实测更差。

### mkdir /sdcard/faces 返回 8

现象：`tkl_fs_mkdir failed, path:/sdcard/faces ret=8 errno=0`，一轮刷 20 多次。

原因：FatFS 把 `FR_EXIST(8)` 放在**返回值**里、`errno` 保持 0，而
`tkl_fs_mkdir()` 原来判的是 `errno != EEXIST`，那个豁免永远不生效。

处理：**已修**（`src/system/tkl_fs.c`）——mkdir 失败后用 `opendir()` 验证路径是否
已是可用目录，是则静默返回 0。功能上本来就无害，`face_gallery_prepare()` 一直按
可复用目录处理；修掉的是刷屏。

### verify FAIL / enroll 回调失败

现象：同人偶发 FAIL，或 `enroll callback failed`。

原因：

- gallery 为空或不完整（未凑满 3 样本也可写入，但 1:N 只对完整 profile 比较）。
- 姿态差、框被裁到边界（如 `y≈0`）导致对齐质量差、cosine 偏低。
- detect 有 score 但框小于 32 被滤掉，enroll/verify 路径会走失败回调。

处理：先 `xrecog detect` 确认稳定出框，再 `enroll` 采满 3 张，最后 `verify`；必要时临时 `xrecog thr` 调试（默认仍建议 0.5）。

### 与 xface “更好检中”的误解

`xface` 命中率更高不代表 `xrecog` 坏了：二者模型、输入尺寸、阈值语义不同（YoloFace 对 sigmoid 后分数用更高阈值；SCRFD 为 dequant score + 0.5）。应按本链路正对、距离、dump 图来验收。

### 编译开关默认关闭

现象：串口无 `xrecog` 命令。

处理：打开 `CMakeLists.txt` 与 `cli_tuya_test_1.c` 中相关注释并重编。

## 典型正常日志

> **板级记录（2026-09-01）**：`init` 从 SD 载入两个模型（700064 + 2378560 字节）、NPU 起来；
> `detect once` 跑了 11 轮，每轮 `faces=1 ok=1`，score 落在 0.594–0.762；
> `enroll` 三次走满 1/3 → 2/3 → 3/3；`verify` 两次都 PASS（0.858/0.852、0.794/0.725，
> 阈值 0.70/0.62）；`list` 报 `profiles=2`，`clear all` 后归零。
> 全程 17 次 SP 取帧，**零 `alloc failed`、零帧未归还** —— 借用契约干净。

### 检测成功

```text
[xrecog] init ok
[xrecog] model ready
[xrecog] task created
[xrecog] task start mode=0 rounds=1 delay=0ms
[xrecog] round 1 start
[xrecog] box count=1 best=score=0.720 x=... y=... w=... h=...
[xrecog] round 1 done faces=1 mode=0 ok=1 fmt=... max_score=0.720 pass=...
[xrecog] task exit stop=0 rounds_done=1
```

`faces=1` 表示尺寸过滤后仍有有效人脸；`faces=0` 且 `max_score` 较高时，优先怀疑最小框过滤或框几何异常。

### 注册成功（同一 ID 采满 3 张）

```text
[xrecog] enroll ok profile=1 sample=1/3
[xrecog] enroll ok profile=1 sample=2/3
[xrecog] enroll ok profile=1 sample=3/3
```

### 校验 PASS

```text
[xrecog] verify profile=1 sample=1 score=0.812 avg=0.745 thr=0.70/0.62 => PASS
```

### 校验 FAIL（他人或姿态差）

```text
[xrecog] verify profile=1 sample=2 score=0.410 avg=0.380 thr=0.70/0.62 => FAIL
```

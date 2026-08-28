# NOKOV–EGO Hand Validation

在一台 Linux 工作站上运行 XINGYING、采集 NOKOV 头环刚体与左右手 Hand(24)，并对 DAS-Ego TF 卡 MCAP 完成时间同步、VIO 空间手眼标定及后续 WiLoR 21 点真值评价准备。

当前已经用真实设备和数据打通：

- XINGYING/NOKOV 90 Hz SDK 数据采集；
- `head_rigidbody` 四 Marker、`Body1_Left` 和 `Body1_Right` 各 24 点同步记录；
- EGO IMU 与 NOKOV 头环刚体角速度的时间偏移估计；
- `/robot0/vio/eef_pose` 与头环刚体的 `AX=XB` 空间标定；
- 原始 MCAP、NOKOV CSV、同步结果和空间矩阵全部保存在同一 session。

EGO 头环仍在设备端录制到 TF 卡，但不再需要 Windows NOKOV 上位机，也不再需要 Windows→Linux 的数据中转。

```text
Linux 工作站
├── XINGYING：八相机标定、CAP 原始录制、资产管理
├── nokovpy：逐帧写 NOKOV CSV
├── EGO TF 卡 MCAP：复制到同一 session
└── Python 后处理
    ├── IMU ↔ head_rigidbody 时间同步
    ├── VIO ↔ NOKOV 空间手眼标定
    └── Hand(24) ↔ WiLoR(21) 评价准备
```

## 1. GitHub 仓库和外部文件

仓库包含跨平台 Python 采集器、Linux 启动脚本、同步/标定代码、测试、文档和空 session 模板。

以下内容受体积、现场配置或许可证限制，由 `.gitignore` 排除：

- EGO MCAP、XINGYING CAP、TRC、C3D 和实验 CSV；
- NOKOV `nokovpy` wheel 和 XINGYING 安装包；
- 当前八相机现场标定和刚体/Hand(24) 资产；
- WiLoR、MANO、MediaPipe 模型。

需要手动提供的文件和下载方式见：

- [外部文件、下载地址与 U 盘清单](docs/required_assets_and_downloads_zh.md)
- [机器可读资产清单](docs/external_assets_manifest.json)

特别注意：仓库中归档分析过的 `CalWand.vc0～vc5` 是旧六相机资料，不能用于当前八相机系统。当前 Linux XINGYING 必须加载同一组八台相机生成的现场标定，或重新对八台相机标定。

## 2. Linux 首次安装

### 2.1 克隆

```bash
cd /home/zdh
git clone https://github.com/zdh1213112/nokov_ego_hand_validation.git
cd /home/zdh/nokov_ego_hand_validation
```

当前机器已经存在项目时直接进入目录即可。

### 2.2 补充 NOKOV Linux SDK wheel

将厂家交付的 wheel 复制到：

```text
vendor/nokov_python_sdk/nokovpy-3.0.1-py3-none-any.whl
```

示例：

```bash
mkdir -p vendor/nokov_python_sdk
cp /你的SDK目录/nokovpy-3.0.1-py3-none-any.whl \
  vendor/nokov_python_sdk/

python3 tools/verify_external_assets.py --profile linux-capture
```

这个 wheel 内应包含 Linux x86-64 的 `libnokov_sdk.so`。它来自 NOKOV 厂商交付包，没有公开下载直链，也不上传 GitHub。

### 2.3 创建统一环境

```bash
./tools/setup_nokov_linux.sh
```

脚本读取根目录 [`environment.yml`](environment.yml)，创建或更新 Conda 环境
`nokov-ego-validation`（Python 3.10），然后安装 NOKOV SDK、MCAP、NumPy、
Matplotlib、SciPy，并运行采集、时间同步和空间标定测试。

如果 wheel 没有复制到默认位置，也可以显式指定：

```bash
./tools/setup_nokov_linux.sh /绝对路径/nokovpy-3.0.1-py3-none-any.whl
```

激活环境：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate nokov-ego-validation
```

如果不激活也没有关系，项目 Linux 脚本会通过 `conda run` 自动使用该环境。
使用其他环境名时，在安装和运行时都设置，例如：

```bash
export NOKOV_CONDA_ENV=my-nokov-env
./tools/setup_nokov_linux.sh
```

### 2.4 为什么 Windows 不创建环境也能录制

`capture_nokov_hand24.py` 的录制部分只需要 Python 标准库和 NOKOV SDK，
不需要 NumPy、SciPy、MCAP 或 Matplotlib。Windows 系统 Python 如果已经安装
`nokovpy`，或者通过 `--sdk-wheel` 指定厂商 wheel，就可以直接录制。

Linux 也支持同样的无 Conda 采集方式：

```bash
python3 tools/capture_nokov_hand24.py \
  --sdk-wheel /绝对路径/nokovpy-3.0.1-py3-none-any.whl \
  --server 10.1.1.198 \
  --output sessions/session_test/nokov \
  --rigid-only \
  --head-rigidbody head_rigidbody \
  --duration 30
```

但时间同步和空间标定需要第三方科学计算库，因此正式 Linux 项目统一使用 Conda。

## 3. 启动并检查 Linux XINGYING

推荐以当前桌面用户启动：

```bash
cd /home/zdh/nokov_ego_hand_validation
./tools/launch_xingying_linux.sh
```

脚本会正确使用 `/run/user/<uid>` 作为 Qt 运行目录，并阻止重复启动第二个 SDK Server。若厂商安装路径不同：

```bash
XINGYING_BIN=/实际路径/XINGYING ./tools/launch_xingying_linux.sh
```

在 XINGYING 中必须确认：

1. 连接的是本次现场的全部 8 台相机；
2. 加载的是八相机标定，3D 视图中的相机位于实际空间位置，而不是全部叠在世界原点；
3. 加载 `head_rigidbody`，4 个 Marker 跟踪稳定；
4. 加载左右手 Hand(24)，每只手恰好 24 个命名 Marker；
5. 开启 SDK/Data Adapter 广播；
6. SDK 服务网卡地址为 `10.1.1.198`，或在后续命令中替换成实际地址。

列出实时资产：

```bash
./tools/list_nokov_assets_linux.sh 10.1.1.198
cat sessions/_discovery/nokov/asset_descriptions.json
```

对当前实际名称做严格检查：

```bash
./tools/run_nokov_python.sh tools/check_nokov_linux_environment.py \
  --server 10.1.1.198 \
  --head-rigidbody head_rigidbody \
  --hand-markerset Body1_Left \
  --hand-markerset Body1_Right
```

必须以 `NOKOV Linux environment: OK` 结束。若输出仍是 `Tracker0/Tracker1`，说明打开的不是正式头环/左右手工程，不要开始采集。

## 4. Linux 正式采集

每次实验使用唯一 session 名称，只能包含英文字母、数字、点、下划线和连字符。例如：

```text
session_bimanual_20260827_001
```

采集脚本会自动创建：

```text
sessions/SESSION_NAME/
├── ego/
├── nokov/
│   └── raw_capture/
├── synchronization/
├── calibration/
├── config/
└── evaluation/
```

### 4.1 仅录制头环刚体

用于时间同步和空间标定的冒烟测试：

```bash
./tools/capture_nokov_linux.sh \
  --session session_head_sync_005 \
  --mode rigid \
  --server 10.1.1.198 \
  --head-rigidbody head_rigidbody \
  --duration 0 \
  --start-delay 5
```

`--duration 0` 表示持续录制，按 `Ctrl+C` 停止。

### 4.2 同时录制左右手 Hand(24) 和头环刚体

当前真实资产名称为 `Body1_Left` 和 `Body1_Right` 时：

```bash
./tools/capture_nokov_linux.sh \
  --session session_bimanual_20260827_001 \
  --mode bimanual \
  --server 10.1.1.198 \
  --head-rigidbody head_rigidbody \
  --left-hand Body1_Left \
  --right-hand Body1_Right \
  --duration 0 \
  --start-delay 5 \
  --queue-size 1024
```

现场名称不同时，以 `asset_descriptions.json` 为准，不能猜测名称。

### 4.3 推荐录制顺序

1. 在 XINGYING 确认 8 台相机空间位置和三维重建正常；
2. 确认 `head_rigidbody`、左右手 24 点可见；
3. 在 XINGYING 开始 CAP 原始录制；
4. 执行 Linux SDK 采集命令；
5. 在 5 秒倒计时期间启动 EGO TF 卡录制；
6. 静止 3 秒；
7. 做左右转头、抬头、低头和侧倾，用于时间同步；
8. 左右手完成张手、握拳、逐指屈曲、拇指对掌、分指、并指和手腕旋转；
9. 再做一组明显头部动作；
10. 静止 3 秒；
11. 停止 EGO；
12. 终端按 `Ctrl+C` 停止 SDK CSV；
13. 最后停止 XINGYING CAP。

SDK CSV 和 XINGYING CAP 是两份互补数据。正式实验必须保留 CAP，用于回放、重新解算和重新导出。

### 4.4 从 EGO TF 卡复制 MCAP

```bash
cp -a /media/zdh/TF_CARD/本次录制.mcap \
  sessions/SESSION_NAME/ego/recording.mcap

sha256sum sessions/SESSION_NAME/ego/recording.mcap
```

如果存在多个 MCAP，必须人工确认本次录制对应的文件，不能把多次实验混入同一个 session。

将 XINGYING CAP 复制或导出到：

```text
sessions/SESSION_NAME/nokov/raw_capture/
```

## 5. 采集输出和质量检查

双手模式至少产生：

```text
nokov/
├── nokov_frames.csv
├── nokov_markers.csv
├── nokov_rigid_bodies.csv
├── nokov_rigid_body_markers.csv
├── nokov_skeleton_segments.csv
├── asset_descriptions.json
├── capture_metadata.json
├── marker_names.txt
└── events.csv
```

预检：

```bash
./tools/check_capture.sh sessions/SESSION_NAME
cat sessions/SESSION_NAME/nokov/capture_metadata.json
```

重点确认：

- `queue_dropped_frames = 0`；
- `callback_errors = 0`；
- `Body1_Left` 和 `Body1_Right` 每帧描述点数为 24；
- `head_rigidbody` 有大量有效位姿；
- 帧号和 `device_timestamp_raw` 单调递增；
- EGO MCAP 存在且来自同一次录制。

NOKOV SDK 在点丢失时可能输出 `9999999.0`，同时仍把 SDK 有效位保持为 1。本项目会同时检查有效位、有限数和坐标绝对值，新的 CSV 会把该情况写为 `valid=0`。旧 CSV 的检查、时间同步和空间标定也会过滤这个哨兵值。

不要把 `nokov_markers.csv` 前 21 项直接当作 WiLoR 21 点；必须建立 24→21 语义映射，并保留逐点可见性掩码。

## 6. 时间同步

安装脚本已经准备好后处理依赖。执行：

```bash
./tools/run_linux_sync.sh SESSION_NAME head_rigidbody
```

输出：

```text
synchronization/
├── imu_nokov_sync.json
├── imu_nokov_aligned_signals.csv
├── nokov_pose_at_ego_imu_timestamps.csv
├── ego_nokov_interpolation_validation.json
└── imu_nokov_sync.png
```

查看：

```bash
cat sessions/SESSION_NAME/synchronization/imu_nokov_sync.json
xdg-open sessions/SESSION_NAME/synchronization/imu_nokov_sync.png
```

建议相关系数至少 `0.8`，正式数据最好 `0.9` 以上。时间映射定义为：

```text
nokov_relative_s = ego_relative_s + b
```

算法使用角速度模长，因此估计第一阶段时间偏移时不要求先知道 IMU 与刚体之间的固定旋转。

同步脚本会自动处理 EGO IMU 约 `200 Hz` 与 NOKOV 约 `90 Hz` 的采样率差异，不按行号或帧号硬配对。对每条真实 EGO IMU 时间戳，程序先计算：

```text
nokov_relative_s = a * ego_relative_s + b
```

再查找该 NOKOV 时刻前后的两个有效刚体样本。位置使用线性插值，姿态使用最短路径四元数 SLERP。默认不允许跨越超过 `50 ms` 的刚体数据空洞，避免用插值掩盖丢帧。

`nokov_pose_at_ego_imu_timestamps.csv` 每一行对应一条原始 EGO IMU 记录，主要字段包括：

- `ego_timestamp_ns`：EGO 原始 IMU 时间戳；
- `nokov_target_timestamp_raw`：映射后的 NOKOV 时间戳；
- `nokov_left_frame`、`nokov_right_frame`：参与插值的两帧；
- `interpolation_alpha`：两帧之间的插值比例；
- `x_mm...qw`：插值后的 NOKOV 刚体位姿；
- `valid_interpolation`：该行是否可以安全使用。

查看频率、覆盖率、插值间隔和精确时间戳上的角速度相关性：

```bash
cat sessions/SESSION_NAME/synchronization/ego_nokov_interpolation_validation.json
```

后续匹配 EGO 图像或 WiLoR 结果时也必须采用同一原则：用图像自身的时间戳经过 `a,b` 映射，再把 NOKOV 数据插值到该时刻；不能用“第 N 帧对应第 N 帧”。插值后的输出虽然可以是 200 Hz 或图像帧率，但其真实动态带宽仍由 NOKOV 原始 90 Hz 数据决定。

## 7. VIO 空间手眼标定

先使用 DAS-Ego 官方 VIO 后处理生成包含：

```text
/robot0/vio/eef_pose
```

的 `*_ego_vio.mcap`，放入同一 session 的 `ego/`。然后运行统一入口：

```bash
./tools/run_nokov_python.sh tools/run_ego_nokov_alignment.py \
  --session-dir sessions/SESSION_NAME \
  --rigid-body head_rigidbody \
  --max-offset-s 30
```

如果 VIO 文件名不符合自动发现规则：

```bash
./tools/run_nokov_python.sh tools/run_ego_nokov_alignment.py \
  --session-dir sessions/SESSION_NAME \
  --ego-vio-mcap ego/你的VIO结果.mcap \
  --rigid-body head_rigidbody \
  --max-offset-s 30
```

统一入口依次执行：

```text
EGO 原始 MCAP + NOKOV rigid CSV
  -> IMU/刚体时间同步
  -> 导出 VIO pose.txt
  -> AX=XB 空间手眼标定
  -> T_B_E、T_Wm_We、T_We_Wm
  -> ego_nokov_alignment_summary.json
```

最终统一结果：

```text
sessions/SESSION_NAME/calibration/ego_nokov_alignment_summary.json
```

变换约定为 `T_A_B` 将 B 坐标变换到 A 坐标。把 NOKOV 世界点转换到 Ego/VIO 世界系时，先把 NOKOV mm 乘 `0.001` 变成 m，再应用 `T_We_Wm`。

复用规则：

- `T_B_E` 仅在反光球刚体相对头环完全未移动时可复用；
- `T_Wm_We/T_We_Wm` 与本次 VIO 世界原点绑定，每次 VIO 重新初始化都要重新求解；
- 八相机重新标定、刚体原点/轴向改变或 Marker 重贴后，全部空间变换重新计算；
- `session_head_sync_001` 的 `+0.01 s` 细化只属于该 session，不能套到新数据。

## 8. NOKOV 24点与 WiLoR 21点相机叠加

已经提供 [`render_nokov_wilor_camera_alignment.py`](tools/render_nokov_wilor_camera_alignment.py)，可将时间同步后的 NOKOV 左右手24个 Marker 和六目 WiLoR 左右手21个关节画到同一原始 Double-Sphere 相机画面。

```bash
python3 tools/render_nokov_wilor_camera_alignment.py \
  --session-dir sessions/SESSION_NAME \
  --dataset sessions/SESSION_NAME/visualization/normalized_multiview \
  --fusion sessions/SESSION_NAME/visualization/fusion_multiview \
  --hand-eye-json sessions/SESSION_NAME/calibration/T_nokov_ego_vio_provisional.json \
  --camera camera2 \
  --output-dir sessions/SESSION_NAME/visualization/camera2_alignment
```

完整的图像解码、WiLoR 推理、六目融合、叠加命令和 session004 实测结论见 [NOKOV/WiLoR 相机投影可视化手册](docs/nokov_wilor_camera_overlay_zh.md)。

对 session004 的逐层复核进一步定位到：VIO body/IMU frame 与 `camera_info.T_b_c`
使用的 GEN base/rig frame 之间缺少固定桥接变换。完整的官方坐标定义核对、候选链数值
对比和四宫格验证见 [坐标链逐层诊断](docs/coordinate_chain_diagnosis_zh.md)。

注意：NOKOV 物理 Marker 与 WiLoR 解剖关节不逐点对应。该视频用于检查整体时空对齐，不直接计算24→21逐关节误差。

## 9. 坐标系约定

```text
官方 Ego/VIO：X 前、Y 左、Z 上
原始 MCAP IMU：X 左、Y 下、Z 后
IMU -> Ego：[-imu_z, +imu_x, -imu_y]
```

时间同步使用角速度模长，对固定换轴不敏感；按轴比较、姿态外参和轨迹变换必须应用上述映射。详见 [DAS-Ego IMU 坐标系说明](docs/das_ego_imu_coordinate_system_zh.md)。

## 10. 项目结构

```text
.
├── README.md
├── environment.yml            # Linux Conda Python 3.10 基础环境
├── assets/                    # 当前八相机现场标定；被 Git 忽略
├── docs/
├── ego_wilor/
├── sessions/                  # 真实实验数据；被 Git 忽略
├── tools/
│   ├── launch_xingying_linux.sh
│   ├── setup_nokov_linux.sh
│   ├── run_nokov_python.sh
│   ├── list_nokov_assets_linux.sh
│   ├── capture_nokov_linux.sh
│   ├── capture_nokov_hand24.py
│   ├── synchronize_ego_imu_nokov.py
│   ├── run_ego_nokov_alignment.py
│   └── render_nokov_wilor_camera_alignment.py
└── vendor/                    # 厂商 SDK；被 Git 忽略
```

每个 session 只能对应一次 NOKOV 录制、一次 EGO 录制、一次佩戴和一套现场标定。

## 11. GitHub 发布

```bash
git status --short
git ls-files | grep -E '\.(mcap|cap|c3d|trc|ckpt|pt|pkl|whl|so|exe)$' \
  && echo '发现不应上传的二进制文件'
```

不要使用 `git add -f` 绕过 `.gitignore`。被试数据、模型、NOKOV SDK 和现场标定通过受控存储/U 盘备份，不通过 GitHub 分发。

旧 Windows 脚本仍保留用于兼容，但不再是主流程：[Windows 部署兼容文档](docs/github_and_nokov_windows_deployment_zh.md)。

## 12. 详细文档

- [头环刚体—IMU 时间同步](docs/head_rigidbody_imu_time_sync_zh.md)
- [2026-08-27 四组真实数据结果](docs/head_sync_dataset_results_20260827.md)
- [NOKOV 24 点 → EGO/WiLoR 21 点方案](docs/nokov24_to_ego21_validation_plan_zh.md)
- [NOKOV/WiLoR 相机投影可视化](docs/nokov_wilor_camera_overlay_zh.md)
- [DAS-Ego IMU 坐标系](docs/das_ego_imu_coordinate_system_zh.md)
- [外部资产清单](docs/required_assets_and_downloads_zh.md)

## 13. 当前边界

- Linux NOKOV 采集、EGO IMU 时间同步和 VIO 空间变换已经打通；
- 24 个反光 Marker 与 WiLoR 21 个解剖关节不要求物理重合；
- NOKOV 原始位置单位为 mm，EGO/WiLoR 通常为 m；
- 每个新 session 都需要自己的时间映射和 VIO 世界变换；
- 已能生成 NOKOV 24点与 WiLoR 21点共同相机叠加视频；正式逐关节评价仍需定义非重合点位的评价指标。

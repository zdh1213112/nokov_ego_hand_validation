# NOKOV–EGO Hand Validation

独立项目目录，用于采集 NOKOV Hand(24) 光学动捕真值，并与 EGO/WiLoR 的
21 关节手部结果进行时空对齐、点位映射和精度评价。

项目位置：`/home/zdh/nokov_ego_hand_validation`

## Repository scope

本地完整工作目录目前包含采集、分析代码以及已安装的第三方资产。GitHub 仓库只提交可审查的代码、文档和配置模板，默认不提交：

- XINGYING 安装包和未确认再分发许可的 NOKOV SDK；
- WiLoR、MANO、MediaPipe 模型权重；
- Orbbec 等第三方二进制；
- 现场 CalWand/CalFrame 标定资产；
- EGO/NOKOV 原始录制、参考输出和被试数据。

克隆 GitHub 仓库后，NOKOV Windows 上位机只需补充厂家提供的 `nokovpy` wheel，即可运行刚体/Marker采集和 IMU 时间同步；完整 WiLoR 离线评价还需按各自许可证补充模型和第三方依赖。

项目运行路径不依赖原来的中文目录。现场硬件、XINGYING 服务、相机标定和 EGO 录制仍属于运行时外部条件。

最终 24→21 评价需要先取得真实的 24 点名称、头部刚体数据、EGO 同步录制和
`T_head_ego_base` 外参。

## Coordinate convention

DAS-Ego 的官方局部坐标系和 MCAP 原始 IMU 传感器轴并不相同。本项目统一采用：

```text
Ego/VIO：X 前、Y 左、Z 上
原始 IMU：X 左、Y 下、Z 后
IMU -> Ego：[-imu_z, +imu_x, -imu_y]
```

详细依据、精确拟合矩阵、camera2 安装方向和同步代码注意事项见
[`docs/das_ego_imu_coordinate_system_zh.md`](docs/das_ego_imu_coordinate_system_zh.md)。

头环四点刚体与 EGO IMU 的最小时间同步闭环见
[`docs/head_rigidbody_imu_time_sync_zh.md`](docs/head_rigidbody_imu_time_sync_zh.md)。

2026-08-27四组首轮真实同步数据结果见
[`docs/head_sync_dataset_results_20260827.md`](docs/head_sync_dataset_results_20260827.md)。

GitHub 发布和 NOKOV Windows 上位机部署见
[`docs/github_and_nokov_windows_deployment_zh.md`](docs/github_and_nokov_windows_deployment_zh.md)。

## Directory layout

```text
nokov_ego_hand_validation/
├── README.md
├── assets/
│   └── nokov_calibration/
├── docs/
├── ego_wilor/
├── reference_data/
│   └── ego_multiview/
├── sessions/
│   ├── README.md
│   └── session_001/
├── tools/
└── vendor/
    ├── nokov_python_sdk/
    └── xingying/
```

目录职责：

- `assets/`：本地现场 NOKOV 相机和坐标系标定资产，不进入 Git；
- `docs/`：方案、数据格式和标定说明；
- `ego_wilor/`：EGO 采集、WiLoR 推理、六目融合和 MANO 代码及模型；
- `reference_data/`：本地 EGO 六目参考结果，不进入 Git；
- `sessions/`：只提交空模板，真实 session 不进入 Git；
- `tools/`：NOKOV 采集、数据检查和验证入口；
- `vendor/`：本地 XINGYING 安装程序和 NOKOV SDK，不进入 Git。

## Quick start

### 1. Full local workspace self-check

```bash
cd /home/zdh/nokov_ego_hand_validation
./tools/check_project.sh
```

模板 session 显示 `MISSING` 是正常的，表示现场数据尚未采集。

这个完整检查要求本地已经放回模型、SDK、标定和参考资产。仅用于 NOKOV Windows 上位机采集时，先按照 [`docs/github_and_nokov_windows_deployment_zh.md`](docs/github_and_nokov_windows_deployment_zh.md) 执行 `tools\setup_nokov_windows.cmd`。

### 2. Discover live NOKOV assets

先启动 XINGYING，加载 `Left Hand(24)` 或 `Right Hand(24)`，确认
`head_rigidbody`，然后启动 Data Adapter/SDK 服务：

```bash
python3 tools/capture_nokov_hand24.py \
  --server 10.1.1.198 \
  --list-only
```

准确的 MarkerSet 和刚体名称会写入：

```text
sessions/session_001/nokov/asset_descriptions.json
```

### 3. Record NOKOV Hand(24)

将下面的名称替换为 `asset_descriptions.json` 中的准确名称：

```bash
python3 tools/capture_nokov_hand24.py \
  --server 10.1.1.198 \
  --hand-markerset "Right Hand(24)" \
  --head-rigidbody "head_rigidbody" \
  --duration 30 \
  --start-delay 5
```

两只手同时采集时重复 `--hand-markerset`：

```bash
python3 tools/capture_nokov_hand24.py \
  --server 10.1.1.198 \
  --hand-markerset "Left Hand(24)" \
  --hand-markerset "Right Hand(24)" \
  --head-rigidbody "head_rigidbody" \
  --duration 30
```

Windows 可以依次运行：

```text
tools/list_nokov_assets.cmd
tools/capture_nokov_30s.cmd
```

采集器自动加载 `vendor/nokov_python_sdk/` 中的 wheel，不要求提前执行
`pip install`。默认输出包括：

```text
sessions/session_001/nokov/
├── nokov_frames.csv
├── nokov_markers.csv
├── nokov_rigid_bodies.csv
├── nokov_rigid_body_markers.csv
├── nokov_skeleton_segments.csv
├── asset_descriptions.json
├── capture_metadata.json
├── events.csv
└── marker_names.txt
```

其中 `nokov_markers.csv` 是 Hand(24) 三维坐标，单位 mm；
`nokov_rigid_bodies.csv` 是头部刚体逐帧位姿。

SDK CSV 不代替 XINGYING CAP 原始工程。正式实验应同时在 XINGYING 中录制，
保存 CAP，并导出 TRC 和 C3D 作为可追溯备份。

### 4. Validate a capture

```bash
./tools/check_capture.sh ./sessions/session_001
```

完成点位映射、同步和头部到 EGO 外参后运行：

```bash
./tools/check_calibrated_session.sh ./sessions/session_001
```

## Session data rules

每次实验复制 `sessions/session_001` 并使用新名称，例如：

```text
sessions/session_20260827_subject01_right_hand
```

不要混合不同被试、重新佩戴、不同相机标定或不同时间段的数据。模板中的
`TODO`、`null` 和 `PLACEHOLDER_DO_NOT_USE` 必须替换成真实采集或标定值。

## Recommended capture sequence

1. 静态张手 5–10 秒；
2. 五根手指依次单独屈曲；
3. 握拳、张手、分指、并指和拇指对掌；
4. 手静止、头部转动；
5. 头和手同时运动；
6. 开头、中间、结尾各做一次明显的“张手—握拳—张手”同步动作。

## Important limitations

- 24 个反光点不能按数组前 21 项直接当作 WiLoR 21 关节；
- marker 表面位置不一定等于解剖关节中心；
- NOKOV 为 mm，EGO/WiLoR 输出通常为 m；
- MCAP 原始 IMU 三轴不等于官方 Ego/VIO 局部三轴，三轴运算前必须按文档换轴；
- 头戴 EGO 会移动，必须使用逐帧头部刚体和 `T_head_ego_base`；
- 参考输出与新 session 不同步，不能直接拿来计算真值误差；
- 第三方模型和 SDK 仍受各自许可证约束，不应未经确认上传到公开仓库。

更详细的采集命令见 `tools/README.md`，完整验证方案见
`docs/nokov24_to_ego21_validation_plan_zh.md`，Ego/IMU 坐标约定见
`docs/das_ego_imu_coordinate_system_zh.md`。

# Tools

本目录包含 Linux XINGYING/NOKOV 实时采集、NOKOV 导出检查、EGO 输出检查和 session 完整性检查。
所有默认路径均相对于项目根目录计算，移动整个项目后仍可运行。

## Commands

项目自检：

```bash
cd /home/zdh/nokov_ego_hand_validation
./tools/check_project.sh
```

Linux 首次安装：

```bash
./tools/setup_nokov_linux.sh
conda activate nokov-ego-validation
```

启动 XINGYING：

```bash
./tools/launch_xingying_linux.sh
```

列出现场 MarkerSet 和刚体：

```bash
./tools/list_nokov_assets_linux.sh 10.1.1.198
```

只采集头环刚体，不采 Hand(24)：

```bash
./tools/capture_nokov_linux.sh \
  --session session_head_sync_001 \
  --mode rigid \
  --duration 0
```

使用 EGO IMU 和 NOKOV 头环刚体角速度估计时间偏移：

```bash
./tools/run_nokov_python.sh tools/synchronize_ego_imu_nokov.py \
  --ego-mcap sessions/session_head_sync_001/ego/recording.mcap \
  --nokov-csv sessions/session_head_sync_001/nokov/nokov_rigid_bodies.csv \
  --rigid-body head_rigidbody \
  --output-dir sessions/session_head_sync_001/synchronization \
  --nokov-time-field device_timestamp_raw \
  --nokov-time-scale 0.001
```

完整现场步骤见 `docs/head_rigidbody_imu_time_sync_zh.md`。

时间同步和 VIO 空间手眼标定统一入口：

```bash
./tools/run_nokov_python.sh tools/run_ego_nokov_alignment.py \
  --session-dir sessions/session_head_sync_001 \
  --rigid-body head_rigidbody \
  --max-offset-s 30 \
  --fine-time-correction-s 0.01
```

新 session 不应直接复制示例的 `0.01 s`，省略该参数或使用该 session 自己确认的细化量。
统一结果写入：

```text
sessions/SESSION_NAME/calibration/ego_nokov_alignment_summary.json
```

采集双手 30 秒：

```bash
./tools/capture_nokov_linux.sh \
  --session session_bimanual_001 \
  --mode bimanual \
  --left-hand Body1_Left \
  --right-hand Body1_Right \
  --duration 30
```

持续采集双手直到按 `Ctrl+C`：

```bash
./tools/capture_nokov_linux.sh \
  --session session_bimanual_002 \
  --mode bimanual \
  --duration 0
```

检查新采集：

```bash
./tools/check_capture.sh ./sessions/session_001
```

检查已完成映射与外参的 session：

```bash
./tools/check_calibrated_session.sh ./sessions/session_001
```

单独检查 NOKOV TRC：

```bash
./tools/run_nokov_python.sh tools/inspect_nokov_hand_export.py \
  sessions/session_001/nokov/hand24.trc \
  --expected-markers 24
```

单独检查 SDK CSV：

```bash
./tools/run_nokov_python.sh tools/inspect_nokov_hand_export.py \
  sessions/session_001/nokov/nokov_markers.csv \
  --markerset "Right Hand(24)" \
  --expected-markers 24
```

C3D 检查需要可选依赖：

```bash
python3 -m pip install -r tools/requirements-validation.txt
```

Windows 兼容入口（不再是主流程）：

```text
list_nokov_assets.cmd
capture_nokov_30s.cmd
```

## Files

```text
launch_xingying_linux.sh
    以当前 Linux 桌面用户启动 XINGYING，设置 Qt runtime 并防止重复启动。

setup_nokov_linux.sh
    使用 environment.yml 创建统一的 Conda 采集和后处理环境。

run_nokov_python.sh
    优先使用 nokov-ego-validation Conda 环境；采集时允许退回系统 Python。

list_nokov_assets_linux.sh
    连接现场 SDK Server 并保存实时资产描述。

capture_nokov_linux.sh
    创建 session 并包装刚体/双手两种 Linux 采集模式。

check_nokov_linux_environment.py
    检查 Linux、XINGYING、NOKOV SDK、实时刚体和 Hand(24) 资产。

capture_nokov_hand24.py
    通过 NOKOV SDK 记录 MarkerSet、刚体、刚体 marker、骨骼和时间戳。

synchronize_ego_imu_nokov.py
    通过 EGO IMU 与 NOKOV 头环刚体角速度模长估计第一阶段时间偏移，
    并将90 Hz NOKOV位置/姿态插值到每条真实200 Hz EGO IMU时间戳。

export_ego_vio_pose.py
    从 DAS-Ego VIO MCAP 的 /robot0/vio/eef_pose 导出 TUM 位姿文本。

calibrate_ego_vio_nokov.py
    使用同步后的 NOKOV 刚体与 Ego/VIO 位姿求解 AX=XB、T_B_E 和世界变换。

run_ego_nokov_alignment.py
    一次执行时间同步、VIO 位姿导出、空间标定并生成统一 JSON 报告。

render_nokov_wilor_camera_alignment.py
    把时间同步后的NOKOV左右手24点和EGO/WiLoR左右手21点投到同一GEN原始相机画面，
    输出视频、预览图、逐帧CSV和空间投影诊断报告。

diagnose_nokov_coordinate_chain.py
    逐层检查NOKOV世界、头环刚体、VIO body、GEN base、相机光学系和Double-Sphere投影，
    对比漏变换/矩阵正逆/轴重标定候选并输出四宫格、曲线、逐帧CSV与诊断JSON；
    使用--render-videos时还会输出轴修正版、拟合修正版和三联前后对比视频。

refine_nokov_marker_camera_alignment.py
    使用 RGB 画面中手套反光球的亮斑，对诊断链做相机局部 SE(3) 修正；分别渲染
    Hand(24) 物理 marker 和 NOKOV skeleton_segments 识别点。结果仅是图像辅助诊断，
    不会替换正式手眼标定。

inspect_nokov_hand_export.py
    检查 TRC、C3D 或 SDK CSV 的点名、数量、帧率和缺失率。

inspect_ego_output.py
    检查 accepted.jsonl 的 21×3 结构、左右手数量、时间戳和无效关节。

check_session.py
    检查一次 session 的采集数据、点名、同步、头部刚体、外参和映射。

check_project.sh
    使用随附参考数据检查项目结构和解析器。

check_capture.sh
    检查现场采集阶段所需文件并生成 evaluation 报告。

check_calibrated_session.sh
    对映射、同步和外参完成后的 session 执行严格检查。

test_capture_core.py
    回归检查 NOKOV 9999999.0 缺失哨兵的有效性判断。

test_camera_alignment_core.py
    回归检查Double-Sphere投影、90/30 Hz Marker插值和无序双手中心关联。
```

对已经生成坐标链诊断的 session，可进一步检查反光球是否真正落在 RGB 手套上：

```bash
python3 tools/refine_nokov_marker_camera_alignment.py \
  --session-dir sessions/session_head_sync_004 \
  --dataset sessions/session_head_sync_004/visualization/normalized_multiview_300 \
  --fusion sessions/session_head_sync_004/visualization/fusion_multiview_300_ignore_handedness \
  --diagnostic-summary sessions/session_head_sync_004/visualization/coordinate_chain_diagnostic/summary.json \
  --camera camera2 \
  --output-dir sessions/session_head_sync_004/visualization/marker_ball_refinement \
  --calibration-frame-index 152 \
  --preview-count 8 \
  --render-videos
```

输出目录中的 `calibration_marker_ball_observations.jpg` 用白色亮斑标出自动匹配的
实际球心；`camera2_marker_skeleton_refined_alignment.mp4` 同时显示三种点位；
`camera2_before_after_marker_refinement.mp4` 用左右两列对比图像辅助修正前后结果。
静态预览包含 `--preview-count` 个时间采样，另加校准帧（若校准帧不在采样中）。
该方法依赖 RGB 中反光球可见，且只修正当前相机的诊断链，正式使用前仍应以独立
标定板/Marker 外参验证。

后续 Hand(24) 真值评价仍待实现：

```text
convert_nokov_24_to_ego21.py
evaluate_wilor_against_nokov.py
render_nokov_wilor_comparison.py
```

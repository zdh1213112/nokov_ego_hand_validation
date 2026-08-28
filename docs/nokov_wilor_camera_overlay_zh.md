# NOKOV Hand(24) 与 EGO/WiLoR Hand(21) 相机投影可视化

## 1. 目标与边界

本流程把同一次录制中的两套结果画到 DAS-Ego 原始相机画面：

- NOKOV：左右手各24个物理反光 Marker；
- EGO：六目 WiLoR 融合得到的左右手各21个解剖关节。

两套点位定义不同，因此本工具用于检查时间、空间和投影是否整体一致，不把24点和21点逐点计算误差。报告中的 `coarse_centroid_gap_px` 只是两只手整体位置的粗略诊断量。

坐标链：

```text
NOKOV手点 p_Wm
  -> 同时刻 head_rigidbody 的逆变换
  -> 头环刚体系 p_B
  -> 手眼静态外参
  -> EGO base
  -> MCAP camera_info.T_b_c 的逆变换
  -> 相机光学坐标
  -> GEN Double-Sphere 鱼眼投影
  -> 原始相机像素
```

图中颜色：

```text
EGO/WiLoR Left：青色       EGO/WiLoR Right：绿色
NOKOV Left：     洋红色     NOKOV Right：     橙色
```

## 2. 准备输入

一个完整 session 至少包含：

```text
sessions/SESSION_NAME/
├── ego/原始.mcap
├── ego/*_ego_vio.mcap
├── ego/pose.txt
├── nokov/nokov_markers.csv
├── nokov/nokov_rigid_bodies.csv
├── synchronization/imu_nokov_sync.json
└── calibration/T_nokov_ego_vio_provisional.json
```

另外需要 `/home/zdh/ego_hand_system` 的 WiLoR 模型和 `ego-hand` Conda 环境。

## 3. 解出六目图像和相机标定

以下以 session004 的前300帧、camera2 主视角为例：

```bash
cd /home/zdh/nokov_ego_hand_validation

conda run --no-capture-output -n ego-hand \
  env PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH=/home/zdh/ego_hand_system/scripts \
  python /home/zdh/ego_hand_system/scripts/normalize_multiview_recording.py \
  --input sessions/session_head_sync_004/ego/DAS-Ego_20260827175924_none_none_689985_a924cf7d.mcap \
  --output sessions/session_head_sync_004/visualization/normalized_multiview_300 \
  --cameras camera0 camera1 camera2 camera3 camera4 camera5 \
  --reference-camera camera2 \
  --max-delta-us 1500 \
  --max-frames 300
```

## 4. 运行六目 WiLoR

```bash
conda run --no-capture-output -n ego-hand \
  env PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH=/home/zdh/ego_hand_system/scripts \
  python /home/zdh/ego_hand_system/scripts/wilor_multiview_inference.py \
  --dataset sessions/session_head_sync_004/visualization/normalized_multiview_300 \
  --output sessions/session_head_sync_004/visualization/wilor_multiview_300 \
  --device cuda \
  --gpu-profile compatible \
  --batch-size 4 \
  --frame-batch-size 1 \
  --preprocess-workers 1 \
  --max-detections-per-class 1 \
  --compile-backbone 0 \
  --max-frames 0 \
  --camera-confidence camera0=0.2 \
  --camera-confidence camera1=0.3 \
  --camera-confidence camera2=0.3 \
  --camera-confidence camera3=0.3 \
  --camera-confidence camera4=0.1 \
  --camera-confidence camera5=0.1
```

session004 使用带大量反光点的黑色手套，检测器的左右手类别明显失效。严格 handedness 模式会拒绝全部帧，因此本次只在几何融合阶段使用 `ignore`，并在最终报告中保留 `detector_handedness_mismatch_observation_count`：

```bash
conda run --no-capture-output -n ego-hand \
  env PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH=/home/zdh/ego_hand_system/scripts \
  python /home/zdh/ego_hand_system/scripts/fuse_multiview_wilor_guided.py \
  --dataset sessions/session_head_sync_004/visualization/normalized_multiview_300 \
  --predictions sessions/session_head_sync_004/visualization/wilor_multiview_300 \
  --output sessions/session_head_sync_004/visualization/fusion_multiview_300_ignore_handedness \
  --cameras camera0 camera1 camera2 camera3 camera4 camera5 \
  --anchor-cameras camera2 camera3 \
  --detector-handedness ignore \
  --workers 4 \
  --max-frames 0
```

`ignore` 只允许几何算法解决左右观测关联，不能证明分类器的左右手类别正确。普通无反光手套数据应优先使用 `strict`。

## 5. 生成共同画面

```bash
python3 tools/render_nokov_wilor_camera_alignment.py \
  --session-dir sessions/session_head_sync_004 \
  --dataset sessions/session_head_sync_004/visualization/normalized_multiview_300 \
  --fusion sessions/session_head_sync_004/visualization/fusion_multiview_300_ignore_handedness \
  --hand-eye-json sessions/session_head_sync_004/calibration/T_nokov_ego_vio_provisional.json \
  --camera camera2 \
  --output-dir sessions/session_head_sync_004/visualization/camera2_alignment
```

输出：

```text
camera2_alignment/
├── camera2_nokov24_wilor21_alignment.mp4
├── camera2_alignment_preview.jpg
├── projection_frames.csv
└── summary.json
```

## 6. session004 实测结果

前299个六目同步帧：

| 指标 | 结果 |
|---|---:|
| 六目 WiLoR 接受帧 | 291/299，97.3% |
| WiLoR 六目重投影中位数 | 5.24 px |
| 同时具有 WiLoR 与 NOKOV 的 camera2 帧 | 290/299，97.0% |
| NOKOV→EGO 粗略手中心偏差中位数 | 404.2 px |
| 粗略手中心偏差 P95 | 432.6 px |

结论：

- 时间同步足以让两套手势随时间一致运动；
- WiLoR 自身六目融合质量正常；
- 当前手眼矩阵尚不能支持像素级 NOKOV→camera2 投影，叠加中存在稳定的大空间偏移；
- 这不是24点与21点位置定义不同能够解释的量级，必须进一步校准“头环刚体→EGO base/相机”的静态外参；
- 在该外参修正前，不能把当前投影当作像素级真值标签。

工具的 `auto` 模式只比较 JSON 约定方向和历史直接方向，选择粗偏差较小者；它不会利用手部数据拟合新变换，因此不会为了让画面好看而把 NOKOV 手点强行拉到 WiLoR 手上。

下一步推荐使用刚体 Marker 与相机图像共同可见的独立标定物，直接最小化多帧重投影误差，求 `T_camera_head_rigidbody`，并用保留序列验证。这样得到的相机外参才能用于正式像素级叠加。

## 7. 后续逐层诊断修正

进一步检查发现，404 px 偏差不能笼统归为 `T_B_E` 精度不足。更具体的错误是：

- AX=XB 的 `E` 是官方 VIO 局部 body/IMU frame；
- `camera_info.T_b_c` 与 WiLoR `joints_base_m` 使用 GEN base/rig frame；
- 原渲染代码漏掉了固定的 `T_GENbase_VIObody`，把两个 base/body frame 当成同一个。

在不改动时间映射、NOKOV 刚体 pose、相机外参和 DS 内参的情况下，只补上接近官方
坐标轴定义的 `VIO body -> GEN base` 重标定，粗略手中心偏差就从 404.2 px 降到
25.0 px；用 WiLoR 手中心进行诊断性完整刚体拟合后为 22.6 px。完整证据、逐帧 CSV
和四宫格对比见 [`coordinate_chain_diagnosis_zh.md`](coordinate_chain_diagnosis_zh.md)。

上述拟合只用于定位错误，不是正式标定。生产外参仍需由独立标定物求得。

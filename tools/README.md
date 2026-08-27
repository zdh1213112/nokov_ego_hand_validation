# Tools

本目录包含 NOKOV 实时采集、NOKOV 导出检查、EGO 输出检查和 session 完整性检查。
所有默认路径均相对于项目根目录计算，移动整个项目后仍可运行。

## Commands

项目自检：

```bash
cd /home/zdh/nokov_ego_hand_validation
./tools/check_project.sh
```

列出现场 MarkerSet 和刚体：

```bash
python3 tools/capture_nokov_hand24.py \
  --server 10.1.1.198 \
  --list-only
```

只采集头环刚体，不采 Hand(24)：

```bash
python3 tools/capture_nokov_hand24.py \
  --server 10.1.1.198 \
  --output sessions/session_head_sync_001/nokov \
  --rigid-only \
  --head-rigidbody head_rigidbody \
  --duration 0
```

使用 EGO IMU 和 NOKOV 头环刚体角速度估计时间偏移：

```bash
python3 tools/synchronize_ego_imu_nokov.py \
  --ego-mcap sessions/session_head_sync_001/ego/recording.mcap \
  --nokov-csv sessions/session_head_sync_001/nokov/nokov_rigid_bodies.csv \
  --rigid-body head_rigidbody \
  --output-dir sessions/session_head_sync_001/synchronization \
  --nokov-time-field device_timestamp_raw \
  --nokov-time-scale 0.001
```

完整现场步骤见 `docs/head_rigidbody_imu_time_sync_zh.md`。

采集右手 30 秒：

```bash
python3 tools/capture_nokov_hand24.py \
  --server 10.1.1.198 \
  --hand-markerset "Right Hand(24)" \
  --head-rigidbody "head_rigidbody" \
  --duration 30 \
  --start-delay 5
```

持续采集直到按 `Ctrl+C`：

```bash
python3 tools/capture_nokov_hand24.py \
  --server 10.1.1.198 \
  --hand-markerset "Right Hand(24)" \
  --head-rigidbody "head_rigidbody" \
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
python3 tools/inspect_nokov_hand_export.py \
  sessions/session_001/nokov/hand24.trc \
  --expected-markers 24
```

单独检查 SDK CSV：

```bash
python3 tools/inspect_nokov_hand_export.py \
  sessions/session_001/nokov/nokov_markers.csv \
  --markerset "Right Hand(24)" \
  --expected-markers 24
```

C3D 检查需要可选依赖：

```bash
python3 -m pip install -r tools/requirements-validation.txt
```

Windows 入口：

```text
list_nokov_assets.cmd
capture_nokov_30s.cmd
```

## Files

```text
capture_nokov_hand24.py
    通过 NOKOV SDK 记录 MarkerSet、刚体、刚体 marker、骨骼和时间戳。

synchronize_ego_imu_nokov.py
    通过 EGO IMU 与 NOKOV 头环刚体角速度模长估计第一阶段时间偏移。

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
```

当前仍待真实数据确认后实现：

```text
calibrate_nokov_to_ego.py
convert_nokov_24_to_ego21.py
evaluate_wilor_against_nokov.py
render_nokov_wilor_comparison.py
```

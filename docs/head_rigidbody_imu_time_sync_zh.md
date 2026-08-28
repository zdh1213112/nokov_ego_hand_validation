# 头环四点刚体与 EGO IMU 时间同步操作手册

本流程用于先跑通以下最小闭环：

```text
头环上的4个反光Marker
  -> XINGYING head_rigidbody
  -> NOKOV逐帧四元数
  -> NOKOV角速度模长

EGO TF卡MCAP
  -> /robot0/sensor/imu
  -> EGO角速度模长

两条角速度曲线互相关
  -> 时间偏移 b
  -> nokov_relative_s = ego_relative_s + b
```

第一轮只估计启动时间偏移 `b`，固定时钟比例 `a=1`。这个阶段不采 Hand(24)，不要求标定 `T_head_ego_base`，也不要求 NOKOV 刚体轴与 EGO 轴一致。

## 1. 在头环上安装4个Marker

安装要求：

- 四点必须牢固固定在头环上，录制过程中不能相对头环移动；
- 四点不能排成规则正方形、矩形或一条直线；
- 使用不对称间距，让刚体只有唯一方向；
- 尽量让其中一个点相对其余三点有不同高度，避免四点完全共面；
- 不遮挡六目相机、按键和佩戴者视野；
- 完成刚体创建后不要重新粘贴Marker。

示意：

```text
        M2（稍高）
       /
M1 -------- M3
       \
          M4（偏离中心）
```

四个Marker在头部转动全过程中应尽量保持至少三个可见。第一次实验的目标是先获得连续、无跳变的 `head_rigidbody` 四元数。

## 2. 在 XINGYING 中创建刚体

1. 确认场地内只有需要创建的四个头环Marker，或者能明确框选这四个点；
2. 在3D视图确认四个点都已稳定重建；
3. 冻结一帧并框选四点；
4. 创建刚体，名称统一填写：

```text
head_rigidbody
```

5. 解除冻结，缓慢做左右转头、抬头低头和左右侧倾；
6. 确认刚体不会变成白点、跳转或翻转；
7. 保存场景和刚体资产。

本阶段通过角速度模长同步，刚体局部轴方向可以暂时不与 Ego 对齐。后续进行三轴姿态和空间外参比较时，再标定刚体轴到 Ego `X前、Y左、Z上` 的固定旋转。

## 3. 创建独立测试 Session

```bash
cd /home/zdh/nokov_ego_hand_validation
cp -a sessions/session_001 sessions/session_head_sync_001
```

不要覆盖正式实验或其他 session。

## 4. 确认 SDK 能发现刚体

先启动 XINGYING、加载刚体场景并开启实时数据服务，然后运行：

```bash
cd /home/zdh/nokov_ego_hand_validation

python3 tools/capture_nokov_hand24.py \
  --server 10.1.1.198 \
  --output sessions/session_head_sync_001/nokov \
  --list-only
```

检查输出中的 `rigid_bodies` 是否包含：

```text
head_rigidbody
```

## 5. 同时采集 NOKOV 和 EGO

### 5.1 NOKOV

在 XINGYING 中同时开始保存原始 CAP。另开终端运行 SDK 刚体采集：

```bash
cd /home/zdh/nokov_ego_hand_validation

python3 tools/capture_nokov_hand24.py \
  --server 10.1.1.198 \
  --output sessions/session_head_sync_001/nokov \
  --rigid-only \
  --head-rigidbody head_rigidbody \
  --duration 0
```

`--duration 0` 表示持续采集，最后按 `Ctrl+C` 停止。

### 5.2 EGO

NOKOV 已经开始录制后：

1. 单击 EGO 按键开始录制；
2. 听到开始录制提示后保持静止约3秒；
3. 执行下面的同步动作；
4. 动作结束后保持静止约3秒；
5. 停止 EGO 录制；
6. 再停止 SDK 和 XINGYING CAP。

### 5.3 第一轮推荐动作

动作不要只做周期完全相同的左右摆头，否则互相关可能出现多个相似峰值。建议做一个不对称序列：

```text
静止3秒
快速左转一次，回正，静止1秒
快速右转两次，回正，静止1秒
抬头一次，回正，静止1秒
低头两次，回正，静止1秒
向左侧倾一次，向右侧倾一次，回正
自由缓慢转头约10秒
静止3秒
```

第一轮录制建议持续45～60秒。头部动作应清晰但不要猛烈甩头，并注意人身安全。

## 6. 从TF卡复制 EGO MCAP

把同一次录制的原始 MCAP 复制为：

```text
/home/zdh/nokov_ego_hand_validation/
sessions/session_head_sync_001/ego/recording.mcap
```

不要使用文件修改时间进行同步。程序读取 MCAP 消息内部的 IMU 时间戳。

## 7. 安装同步程序依赖

当前机器已经安装所需依赖。其他机器首次运行时执行：

```bash
python3 -m pip install -r tools/requirements-sync.txt
```

## 8. 计算时间偏移

```bash
cd /home/zdh/nokov_ego_hand_validation

python3 tools/synchronize_ego_imu_nokov.py \
  --ego-mcap sessions/session_head_sync_001/ego/recording.mcap \
  --nokov-csv sessions/session_head_sync_001/nokov/nokov_rigid_bodies.csv \
  --rigid-body head_rigidbody \
  --output-dir sessions/session_head_sync_001/synchronization \
  --nokov-time-field device_timestamp_raw \
  --nokov-time-scale 0.001 \
  --max-offset-s 30
```

本项目在2026-08-27采集的四组真实数据中确认：

```text
device_timestamp_raw 为 Unix epoch 毫秒时间戳
相邻90 Hz帧的典型增量为11 ms
```

因此这台 NOKOV/XINGYING 设备优先使用 `device_timestamp_raw × 0.001`。`receive_perf_ns` 仍保留为诊断和兼容入口，但它代表 SDK 客户端接收时刻，不是相机测量时刻。

程序生成：

```text
synchronization/
├── imu_nokov_sync.json
├── imu_nokov_aligned_signals.csv
├── nokov_pose_at_ego_imu_timestamps.csv
├── ego_nokov_interpolation_validation.json
└── imu_nokov_sync.png
```

### 8.1 90 Hz NOKOV 与 200 Hz EGO 如何对应

两套设备不需要具有相同采样率。程序不会按 CSV 行号配对，而是对每条 EGO IMU 原始时间戳执行：

```text
EGO时间戳 -> 使用 a、b 映射到 NOKOV 时间轴
            -> 找到前后两帧 NOKOV 刚体数据
            -> 位置线性插值、四元数 SLERP
```

结果写入 `nokov_pose_at_ego_imu_timestamps.csv`。`valid_interpolation=1` 才能使用。默认 `--max-interpolation-gap-s 0.05`，即 NOKOV 前后样本相隔超过50 ms时拒绝插值，以免跨越较长的刚体丢失区间。

验证报告 `ego_nokov_interpolation_validation.json` 给出：

- EGO 和 NOKOV 实测频率；
- 共同录制区间内的有效插值比例；
- NOKOV 插值括号间隔的中位数、P95和最大值；
- 在 EGO 精确时间戳上重新比较的角速度相关系数；
- 因较大数据空洞被拒绝的行数。

如果要改变允许的最大插值间隔，例如限制为30 ms：

```bash
python3 tools/synchronize_ego_imu_nokov.py \
  --ego-mcap sessions/SESSION_NAME/ego/recording.mcap \
  --nokov-csv sessions/SESSION_NAME/nokov/nokov_rigid_bodies.csv \
  --rigid-body head_rigidbody \
  --output-dir sessions/SESSION_NAME/synchronization \
  --nokov-time-field device_timestamp_raw \
  --nokov-time-scale 0.001 \
  --max-offset-s 30 \
  --max-interpolation-gap-s 0.03
```

插值只是求 NOKOV 在目标时刻的连续位姿，不会把90 Hz动捕实际变成200 Hz传感器。

## 9. 如何读取结果

核心公式：

```text
nokov_relative_s = a * ego_relative_s + b
```

第一阶段：

```text
a = 1
b = imu_nokov_sync.json 中 time_mapping.b_s
```

例如：

```text
b = -3.250 s
```

表示同一个动作在 NOKOV 相对时间上比 EGO 相对时间早3.250秒：

```text
EGO相对时间 10.000 s
对应NOKOV相对时间 6.750 s
```

不要直接把 `b` 理解成网络延迟；它主要包含两套设备开始录制时刻的差异。

结果置信度：

- `strong`：曲线峰值清楚，可以进入人工检查；
- `usable`：可以用于第一轮联调，仍需查看PNG；
- `weak`：动作不足、动作过于周期性、刚体丢失或搜索范围不足，应重新录制。

必须打开 `imu_nokov_sync.png`，检查对齐后的两条角速度曲线峰值是否同时出现，不能只看一个相关系数。

## 10. 为什么这里不需要先做 IMU 换轴

同步程序比较的是：

```text
||gyro_ego|| 与 ||omega_nokov||
```

向量模长不受固定坐标旋转影响，因此本阶段不需要应用：

```text
IMU -> Ego：[-imu_z, +imu_x, -imu_y]
```

以后比较 X/Y/Z 三轴正负方向、积分姿态或标定旋转外参时，才必须按 [`das_ego_imu_coordinate_system_zh.md`](das_ego_imu_coordinate_system_zh.md) 换轴。

## 11. 第一阶段的限制

第一轮程序固定 `a=1`，只估计时间偏移 `b`。45～60秒录制适合验证流程，但不适合精确估计两套时钟的长期漂移。

完整实验应录制3～5分钟，并在开头、中间、结尾各执行一次不同的同步动作，然后拟合：

```text
t_nokov = a * t_ego + b
```

第一轮通过后再增加漂移估计，避免同时排查刚体、采集、MCAP解析、时间偏移和时钟漂移。

四组首轮真实数据结果见 [`head_sync_dataset_results_20260827.md`](head_sync_dataset_results_20260827.md)。

## 12. 常见问题

### 找不到 `head_rigidbody`

- 检查 XINGYING 是否加载了正确场景；
- 检查刚体名称大小写和空格；
- 先运行 `--list-only`，复制 SDK 实际返回的名称。

### NOKOV 四元数断断续续

- 检查每个Marker在2D视图中被几台相机看到；
- 改变四点在头环上的高度和不对称程度；
- 检查刚体完整性阈值；
- 不要先通过插值掩盖长时间刚体丢失。

### 最佳偏移位于 ±30秒边界

增加搜索范围，例如：

```bash
--max-offset-s 60
```

### 相关峰值很低

- 确认 EGO MCAP 和 NOKOV CSV 来自同一次录制；
- 检查是否真的转动了头环，而不是只移动身体平移；
- 使用更不对称、更有停顿的 yaw/pitch/roll 动作序列；
- 检查 NOKOV 刚体是否跳变或丢失；
- 检查 `imu_nokov_sync.png`。

# NOKOV 24 点对 EGO/WiLoR 21 点真值验证方案

**编写日期：** 2026-08-26  
**目标：** 使用 XINGYING `Left/Right Hand(24)` 光学动捕结果作为参考真值，验证 `/home/zdh/nokov_ego_hand_validation/ego_wilor` 输出的 WiLoR/EGO 21 点三维手部姿态与 21-DOF 手指角度。

## 1. 结论

这项验证可行，但不能把 NOKOV 24 个反光 marker 与 WiLoR 21 点直接按数组下标相减。完整验证必须解决四件事：

1. 建立 NOKOV 24 点到 WiLoR/MediaPipe 21 点的语义映射；
2. 把 NOKOV 世界坐标动态转换到 EGO 相机或 GEN base 坐标系；
3. 按真实时间戳同步两套系统；
4. 处理反光 marker 中心与真实关节中心之间的安装偏移。

NOKOV 24 点更准确地称为“高精度光学参考”，不是严格的解剖学关节中心真值。报告中建议使用“光学参考真值”或“reference ground truth”。

## 2. 官方 24 点模型提供了什么

NOKOV 官方页面说明：

- 左手和右手分别使用 24 点模型；
- 反光 marker 贴在手指各关节处；
- 创建模型前，3D 视图中必须能看到全部反光点，场地内不能有多余点；
- 创建模型时要冻结一帧；
- Y 轴向上时手指朝 `+Z`，Z 轴向上时手指朝 `+Y`。

官方页面没有给出 24 个 marker 的完整名称和编号表。因此，最终映射必须根据实际导出的 TRC/C3D 文件确认，不能假设前 21 个点就是 WiLoR 21 点。

官方参考：

- [Left/Right Hand(24)](https://docs.nokov.com/xingying/XingYing4.6-CN/jiu-chuang-jian-markerset/er-body-ren-ti/left-right-hand-24/)
- [C3D 文件导出](https://docs.nokov.com/xingying/XingYing4.6-CN/shi-san-shu-ju-dao-chu/er-c3d-wen-jian)
- [TRC 文件导出](https://docs.nokov.com/xingying/XingYing4.6-CN/shi-san-shu-ju-dao-chu/wu-trc-wen-jian)
- [NOKOV SDK](https://docs.nokov.com/xingying/XingYing4.6-CN/shi-qi-cha-jian-sdk/shi-san-sdk)

## 3. 当前目录中哪些文件需要使用

### 3.1 需要保留

| 文件或目录 | 用途 | 是否直接进入评价程序 |
|---|---|---|
| `CalWand/` | 六相机标定、镜头模型、NOKOV 世界坐标定义 | 否，但必须归档 |
| `XINGYING 4.6.0.7923 WINDOW Install Package/` | 创建 24 点模型、采集、回放和导出 | 否 |
| `XING_Python_SDK_4105645/` | 实时接收 MarkerSet、刚体和时间戳 | 仅实时路线需要 |
| 新采集的 TRC/C3D | 24 点逐帧三维坐标和点名 | 是 |
| 新采集的原始 CAP 工程 | 出错时在 XINGYING 中重放和重新导出 | 否，但必须归档 |

### 3.2 这次不能直接使用

以下内容是旧的 NOKOV–MANUS 三指联合标定，不适用于新的 24 点手模型：

```text
NOKOV_MANUS_20260811/
final_calibration.json
ManusLiveBridge.exe
ManusSDK.dll
newTracker2
SinglePoint0 / SinglePoint1 / SinglePoint2
```

旧 B-V4 只覆盖食指、中指、无名指三个光学点，并且目标坐标系是 `ManusWrist`，不能当作 NOKOV 世界到 EGO 相机的外参。

## 4. 每次实验需要保存的完整文件

建议每次采集建立独立目录：

```text
session_001/
├── nokov/
│   ├── hand24.trc
│   ├── hand24.c3d
│   ├── raw_capture/                 # CAP、VC、场景和标定相关文件
│   ├── marker_names.txt             # 24点实际名称和左右手说明
│   └── capture_metadata.json        # 帧率、单位、坐标轴、起止时间
├── ego/
│   ├── recording.mcap               # GEN六目原始录制，优先保留
│   └── normalized_multiview/         # 标准化后的视频、标定和同步表
├── calibration/
│   ├── head_rigidbody_definition.json
│   ├── T_head_ego_base.json
│   └── calibration_observations.csv
├── synchronization/
│   ├── sync_events.csv
│   └── frame_alignment.csv
├── config/
│   └── nokov24_to_ego21.yaml
└── evaluation/
    ├── metrics.json
    ├── per_frame_errors.csv
    ├── per_joint_errors.csv
    └── overlay.mp4
```

### 最小可用数据

如果当前只做第一轮适配，至少需要：

1. 一段约 100～300 帧、包含静态张手和逐指屈曲的 TRC；
2. 同一时段的 EGO `.mcap`；
3. XINGYING 中 24 点名称的截图或导出列表；
4. 两套系统各自的时间戳；
5. NOKOV 到 EGO 的空间外参，或者能够计算该外参的同步标定数据。

## 5. 24 点到 21 点的语义映射

`ego_hand_system` 使用 MediaPipe 顺序：

```text
0      wrist
1-4    thumb:  CMC, MCP, IP, tip
5-8    index:  MCP, PIP, DIP, tip
9-12   middle: MCP, PIP, DIP, tip
13-16  ring:   MCP, PIP, DIP, tip
17-20  pinky:  MCP, PIP, DIP, tip
```

现有实现中的定义位于：

- `/home/zdh/nokov_ego_hand_validation/ego_wilor/scripts/fit_mano_sequence.py`
- `/home/zdh/nokov_ego_hand_validation/ego_wilor/scripts/mano_conventions.py`

官方示意图大致可以分解为：

```text
五根手指 × 4点 = 20点
手掌/腕部参考点 = 4点
总计 = 24点
```

推荐处理：

- 五指的 20 个 marker 映射到 WiLoR 的 1～20；
- 使用腕部两侧 marker 的中点，或者通过四个掌腕点拟合，生成 WiLoR 的 `wrist=0`；
- 多出的三个点保留为手掌刚体、遮挡检测和 MANO 拟合约束；
- 不要简单删除前三个或最后三个点；
- 不要根据导出列顺序猜测语义。

映射配置建议采用以下形式，实际点名取得后再填写：

```yaml
schema: nokov_hand24_to_mediapipe21_v1
units: mm
right_hand:
  wrist_sources: [RIGHT_WRIST_A, RIGHT_WRIST_B]
  joints:
    thumb_cmc: null
    thumb_mcp: null
    thumb_ip: null
    thumb_tip: null
    index_mcp: null
    index_pip: null
    index_dip: null
    index_tip: null
    middle_mcp: null
    middle_pip: null
    middle_dip: null
    middle_tip: null
    ring_mcp: null
    ring_pip: null
    ring_dip: null
    ring_tip: null
    pinky_mcp: null
    pinky_pip: null
    pinky_dip: null
    pinky_tip: null
  auxiliary_palm_markers: []
```

左右手必须分别建立和检查映射。

## 6. 为什么必须在 EGO 头环上建立 NOKOV 刚体

EGO 相机戴在头上，采集期间会相对 NOKOV 世界坐标运动。只使用手部 24 点，无法知道每帧 EGO 相机的位置和朝向。

建议在头环或六目相机支架上固定至少三个不共线 marker，推荐四个，并在 XINGYING 中创建：

```text
head_rigidbody
```

NOKOV 每帧输出：

```text
T_world_head(t)
```

还需要一次性标定头环刚体到 GEN base 的安装外参：

```text
T_head_base
```

于是：

```text
T_world_base(t) = T_world_head(t) @ T_head_base
p_base(t) = inverse(T_world_base(t)) @ p_world_nokov(t)
```

这时 `p_base(t)` 才能与 GEN 六目融合输出中的 `joints_base_m` 直接比较。

如果评价实时双目输出，则需要 `T_head_left_camera`：

```text
T_world_left_camera(t) = T_world_head(t) @ T_head_left_camera
p_left_camera(t) = inverse(T_world_left_camera(t)) @ p_world_nokov(t)
```

没有 `head_rigidbody` 时，只有在 EGO 相机整个实验期间完全固定不动，或者已有可靠的 EGO 轨迹并与 NOKOV 世界完成对齐时，才能做可信的空间评价。

### 6.1 Ego/VIO 与原始 IMU 坐标约定

本项目采用简智官方 Ego/VIO 局部坐标系：

```text
Ego/VIO：X 前、Y 左、Z 上
```

已检查的 DAS Ego V6 MCAP 原始 IMU 安装轴为：

```text
原始 IMU：X 左、Y 下、Z 后
IMU -> Ego：[-imu_z, +imu_x, -imu_y]
```

因此，比较三轴角速度、积分 IMU 姿态或估计 NOKOV 头环刚体旋转外参前，必须先完成该换轴。完整依据、矩阵和代码见 [`das_ego_imu_coordinate_system_zh.md`](das_ego_imu_coordinate_system_zh.md)。

## 7. 时间同步

### 7.1 首选方法

使用共享硬件触发、TTL 或 TimeCode，让 NOKOV 和 EGO 记录同一个触发事件。保存每一帧的设备时间戳，不要只保存帧号。

### 7.2 没有硬件同步时

在录制开头、中间和结尾执行两套系统都能观察到的快速事件，例如：

```text
快速张手 → 握拳 → 张手
```

通过 NOKOV 指尖速度与 EGO 21 点速度的峰值拟合：

```text
t_nokov = a × t_ego + b
```

其中：

- `b` 是启动时间偏移；
- `a` 是两套时钟的速率差；
- 长序列不能只估计一个固定 `b`。

每个 EGO 时间戳都应在相邻 NOKOV 帧之间插值：

- marker 位置使用线性插值；
- 头环刚体旋转使用四元数 SLERP；
- 不要简单按相同帧号比较；
- 不要把最近帧配对当作最终同步结果。

如果使用原始 IMU 与 NOKOV 刚体角速度进行同步，角速度模长不受坐标旋转影响；如果按 X/Y/Z 分轴相关，则必须先按第 6.1 节转换到 Ego/VIO 局部坐标系。

建议生成：

```text
ego_frame_index,ego_timestamp_ns,nokov_frame_before,nokov_frame_after,
interpolation_alpha,time_error_us
```

第一阶段建议把同步残差控制在约 2～5 ms，再根据手部最大运动速度收紧门限。

## 8. 应该使用哪个 EGO 结果

### 8.1 验证 GEN 六目融合 21 点

优先使用：

```text
/home/zdh/nokov_ego_hand_validation/ego_wilor/output/gen6_pose_full_v3/
fusion_handedness_strict_full/accepted.jsonl
```

其中每只手的：

```text
joints_base_m
```

是 GEN base 坐标系下的 21×3 三维结果，单位为米。这是当前最适合与 NOKOV 参考真值比较的输出。

对应的时间表和六目相机标定为：

```text
normalized_multiview/multiview_frames.csv
normalized_multiview/calibration/camera0.json
...
normalized_multiview/calibration/camera5.json
```

### 8.2 验证实时双目 21 点

使用：

```text
/home/zdh/nokov_ego_hand_validation/ego_wilor/output/ego_live/live_landmarks_3d.csv
```

关键字段为：

```text
left_timestamp_us
right_timestamp_us
track_id
handedness
landmark_index
filtered_valid
x_left_camera_m
y_left_camera_m
z_left_camera_m
```

坐标系是左相机光学坐标系，单位为米。

### 8.3 验证 21-DOF 手指角度

使用：

```text
/home/zdh/nokov_ego_hand_validation/ego_wilor/output/ego_live/live_mano_21dof.csv
```

但需要先从 NOKOV 24 点构造相同的手掌坐标系和骨段方向，再计算完全相同定义的屈曲、外展和对掌角。24 个 XYZ 点不能直接与 21 个角度列比较。

## 9. 推荐采集动作

一次正式 session 至少录制以下阶段：

| 阶段 | 建议时长 | 用途 |
|---|---:|---|
| 静态张手 | 5～10 秒 | 检查点名、左右手、单位、外参和固定偏差 |
| 单指依次屈曲 | 每指 3～5 秒 | 确认每条手指链的语义映射 |
| 同时握拳/张手 | 10 秒 | 检查快速运动时间同步 |
| 分指/并指 | 10 秒 | 检查 MCP 外展和外侧手指误差 |
| 拇指对掌 | 10 秒 | 检查拇指 CMC、MCP、IP 和 opposition |
| 手尽量静止、头部转动 | 10 秒 | 验证动态 `head_rigidbody` 外参 |
| 头和手同时运动 | 10～20 秒 | 最终端到端压力测试 |

每个动作阶段写入事件记录，不能只依靠事后看视频猜测。

## 10. 评价指标

### 10.1 三维关节点位置

对每个同步帧和每个关节：

```text
e(t,j) = ||p_ego(t,j) - p_nokov(t,j)||₂ × 1000 mm
```

至少输出：

- 绝对 MPJPE；
- 腕部对齐后的 MPJPE；
- 每个关节的 mean、median、P95、max；
- PCK@10 mm、PCK@20 mm、PCK@30 mm；
- 五个指尖的单独误差；
- 有效点覆盖率；
- 左右手身份错误次数；
- NOKOV 真实观测点与反算/插值点分开统计。

绝对 MPJPE 同时包含手势估计、时间同步和空间外参误差；腕部对齐 MPJPE 更侧重手部内部姿态与骨形状误差。两者都应报告。

### 10.2 21-DOF 角度

至少输出：

- 每个自由度的角度 MAE；
- 角度 P95；
- 静态偏差；
- 快速运动峰值误差；
- 拇指和其余四指分组统计；
- 各手势阶段的误差。

### 10.3 手势分类

如果最终目标是离散手势识别，还应输出：

- 总体准确率；
- 每类 precision、recall、F1；
- 混淆矩阵；
- 识别延迟；
- 从动作开始到稳定识别的时间。

NOKOV 24 点只提供几何轨迹，不自动提供手势类别。手势类别需要事件标注或基于光学关节角定义明确规则。

## 11. 数据划分原则

用于求解以下参数的数据不能再次作为最终测试数据：

- 24→21 marker offset；
- `T_head_base` 或 `T_head_camera`；
- 时间偏移与漂移；
- 手型或 MANO shape；
- 手势分类阈值。

推荐按完整 session 划分：

```text
Calibration：求点位偏移、空间外参和初始同步
Validation：调同步门限、异常值门限和角度定义
Test：固定全部参数后只做一次最终报告
```

不要把相邻帧随机分到训练和测试，否则误差会被低估。

## 12. 需要在 ego_hand_system 中新增的代码

当前项目已经有 WiLoR 推理、六目融合、MANO 拟合和训练标签校验，但还没有读取 XINGYING 24 点导出的适配器。建议新增：

```text
scripts/inspect_nokov_hand_export.py
    检查TRC/C3D点名、数量、单位、帧率和缺失率

scripts/calibrate_nokov_to_ego.py
    求T_head_base/T_head_camera和同步参数a、b

scripts/convert_nokov_24_to_ego21.py
    24→21映射、marker offset、动态坐标变换和时间插值

scripts/evaluate_wilor_against_nokov.py
    计算MPJPE、PCK、逐指/逐关节/逐动作误差

scripts/render_nokov_wilor_comparison.py
    生成3D叠加、EGO图像投影和误差视频
```

推荐第一版使用 TRC，因为它是文本格式，容易检查和复现；同时保留 C3D 作为原始命名点备份。实时 SDK 接入放在离线验证通过之后。

## 13. 建议的实施顺序

```text
步骤1：创建单手24点模型并确认点名稳定
  ↓
步骤2：给EGO头环创建head_rigidbody
  ↓
步骤3：采集100～300帧小样本，导出TRC+C3D，保存EGO MCAP
  ↓
步骤4：检查24点名称、缺失率、单位和左右手
  ↓
步骤5：建立nokov24_to_ego21.yaml
  ↓
步骤6：标定T_head_base并拟合时间t_nokov=a*t_ego+b
  ↓
步骤7：把NOKOV参考点转换到GEN base或左相机坐标
  ↓
步骤8：与joints_base_m或live_landmarks_3d.csv比较
  ↓
步骤9：生成逐关节指标、3D叠加和图像重投影视频
  ↓
步骤10：独立session固定参数测试
```

## 14. 第一轮交付验收标准

第一轮不追求立刻得到最低误差，而是先证明数据链正确：

- TRC/C3D 中每只手稳定存在 24 个命名点；
- 左右手和五根手指没有发生语义交换；
- NOKOV 单位从 mm 正确转换为 EGO 的 m；
- 头部转动时，转换到 GEN base 后的静止手不会整体漂移；
- 快速握拳时两套轨迹没有明显时间拖影；
- 21 点 3D 叠加方向正确，没有镜像、轴交换或前后翻转；
- 每一帧能追溯到原始 NOKOV 帧和 EGO 时间戳；
- 标定集和最终测试集严格分开。

## 15. 当前最需要的数据

要开始实现适配器，下一步请先准备一个小样本目录，至少包含：

```text
hand24.trc
hand24.c3d
对应的EGO recording.mcap
24点名称截图或列表
头环刚体数据
同步事件说明
```

拿到真实点名后，才能最终确定 24→21 映射；在此之前任何具体的 marker 编号映射都只能是假设。

## 16. 项目内已有参考

`ego_hand_system` 已有更偏向训练标签生成的设计说明：

[NOKOV/XINGYING 光学动捕手部模型生成 WiLoR 标签](nokov_optical_mocap_to_wilor_labels.md)

本文件更侧重“真值验证”：先比较同步后的 21 点位置和 21-DOF 角度，不要求第一阶段就生成 WiLoR 的 778 顶点训练标签。

# NOKOV–EGO Hand Validation

用于在 Windows NOKOV 上位机采集头环刚体/Hand(24)，在 Linux 对简智 DAS-Ego TF 卡录制的 MCAP 做时间同步，并为后续 NOKOV 24 点与 EGO/WiLoR 21 点真值评价准备数据。

当前已经用真实设备和数据打通：

- XINGYING/NOKOV → 90 Hz 头环四 Marker 刚体 CSV；
- DAS-Ego TF 卡录制 → MCAP IMU；
- 通过头部转动产生共同角速度事件，估计两个独立时钟的时间偏移；
- 4 组同步相关系数为 `0.8296–0.9983`，`session_head_sync_001` 达到 `0.9983`；
- 使用 `/robot0/vio/eef_pose` 与 `head_rigidbody` 完成 `AX=XB` 空间手眼标定；
- `session_head_sync_001` 的 NOKOV 世界系 ↔ Ego/VIO 世界系变换已经过现场物理测量验证，方向和位置符合实际。

本仓库不要求 NOKOV 和 EGO 同时联网，也不假设两台设备开始录制的系统时间完全一致。

```text
Windows NOKOV 上位机                         Linux 后处理机
XINGYING + nokovpy                          Python + MCAP
        │                                         ▲
        ├─ nokov/*.csv ─┐                         │
EGO 头环 ─ TF卡 *.mcap ─┼─ U盘/受控存储 ─ session_xxx/
XINGYING ─ 原始 *.cap ──┘                         │
                                                  ├─ 时间偏移、对齐曲线、JSON
                                                  └─ T_B_E、T_Wm_We、T_We_Wm
```

## 1. GitHub 中包含什么

仓库包含采集器、Windows 启动脚本、Linux 同步脚本、测试、文档和空 session 模板。

`.gitignore` 会排除真实录制、模型、厂家 SDK、安装包和现场标定。所有被排除文件的官方下载地址、许可证限制、哈希和 U 盘路径见：

- [外部文件、下载地址与 U 盘清单](docs/required_assets_and_downloads_zh.md)
- [机器可读资产清单](docs/external_assets_manifest.json)

关键结论：

- Windows 刚体/Hand24 采集只需补充 NOKOV `nokovpy` wheel，并确保 XINGYING 已安装；
- Linux 时间同步不需要 WiLoR、MANO、MediaPipe、CUDA、Orbbec SDK 或 NOKOV SDK；
- 完整 WiLoR/MANO 模型只在后续手部推理/评价时需要。

## 2. Windows 上位机：从克隆到可采集

### 2.1 安装公开软件

安装：

- [Git for Windows x64](https://git-scm.com/install/windows)
- [Python 3.11.9 Windows x64](https://www.python.org/downloads/release/python-3119/)；安装时保留 Python Launcher
- [Microsoft VC++ x64 Runtime](https://aka.ms/vc14/vc_redist.x64.exe)；仅 NOKOV DLL 加载失败时需要

建议使用 Python 3.11；项目也接受 64 位 Python 3.10，不使用系统中更新的 3.12–3.14。

### 2.2 克隆代码

在 PowerShell 中执行，将地址换成实际 GitHub 仓库：

```powershell
cd D:\
git clone https://github.com/zdh1213112/nokov_ego_hand_validation
cd D:\nokov-ego-hand-validation
```

### 2.3 从厂商包或 U 盘补充 NOKOV wheel

GitHub 不包含 `nokovpy` 和 XINGYING 安装包。把厂家提供的文件复制为：

```text
vendor\nokov_python_sdk\nokovpy-3.0.1-py3-none-any.whl
```

例如 U 盘为 `E:`：

```powershell
New-Item -ItemType Directory -Force vendor\nokov_python_sdk
Copy-Item E:\nokov_windows_assets\nokovpy-3.0.1-py3-none-any.whl `
  vendor\nokov_python_sdk\nokovpy-3.0.1-py3-none-any.whl
py -3.11 tools\verify_external_assets.py --profile windows-capture
```

如果上位机尚未安装 XINGYING 4.6，也从 NOKOV 交付包/U 盘安装。公开仓库不能替代现场的 XINGYING 工程、相机标定和 `head_rigidbody` 刚体定义。

### 2.4 一键创建 Windows 环境

在资源管理器双击，或在 PowerShell 中运行：

```powershell
tools\setup_nokov_windows.cmd
```

成功结束时应显示：

```text
[OK] NOKOV SDK native load: ...
NOKOV Windows environment: OK
```

### 2.5 检查 XINGYING 资产

在 XINGYING 中：

1. 打开已经完成相机标定的工程；
2. 加载头环四点刚体 `head_rigidbody`；
3. 确认 4 个 Marker 持续可见；
4. 开启 Data Adapter/Python SDK 数据广播；
5. 确认上位机和 NOKOV 服务网段互通。

然后运行：

```powershell
tools\list_nokov_assets.cmd
```

输入 SDK 服务地址，例如 `10.1.1.198`。输出保存在：

```text
sessions\_discovery\nokov\asset_descriptions.json
```

必须能看到 `head_rigidbody`、刚体 ID 和 4 个 Marker，才继续正式采集。

### 2.6 正式录制 NOKOV 刚体与 EGO

以下使用 `session_head_sync_001` 演示。正式实验必须换成唯一名称，避免覆盖以前的数据。

先在 PowerShell 中创建固定目录：

```powershell
cd D:\nokov_ego_hand_validation

New-Item -ItemType Directory -Force `
  sessions\session_head_sync_001\ego, `
  sessions\session_head_sync_001\nokov, `
  sessions\session_head_sync_001\synchronization, `
  sessions\session_head_sync_001\calibration, `
  sessions\session_head_sync_001\nokov\raw_capture
```

目录结构：

```text
D:\nokov_ego_hand_validation\sessions\session_head_sync_001\
├── ego\
│   └── recording.mcap
├── nokov\
│   └── raw_capture\
├── synchronization\
└── calibration\
```

在 XINGYING 中：

1. 加载 `head_rigidbody`；
2. 确认 4 个 Marker 都能看到且刚体跟踪稳定；
3. 开启 SDK/Data Adapter 数据广播；
4. 开始 CAP 原始录制，并保留 `.cap` 文件。

如果已经运行过 `tools\setup_nokov_windows.cmd`，可以直接使用项目虚拟环境：

```powershell
.\.venv\Scripts\python.exe tools\capture_nokov_hand24.py `
  --server 10.1.1.198 `
  --output sessions\session_head_sync_001\nokov `
  --rigid-only `
  --head-rigidbody head_rigidbody `
  --duration 0 `
  --start-delay 5 `
  --queue-size 1024
```

如果已经激活 `.venv`，也可以使用用户熟悉的写法：

```powershell
python tools\capture_nokov_hand24.py `
  --server 10.1.1.198 `
  --output sessions\session_head_sync_001\nokov `
  --rigid-only `
  --head-rigidbody head_rigidbody `
  --duration 0 `
  --start-delay 5 `
  --queue-size 1024
```

参数含义：

- `--rigid-only`：只采集头环刚体，不采集手部 24 点；
- `--duration 0`：持续录制，按 `Ctrl+C` 停止；
- `--start-delay 5`：等待 5 秒后正式写入数据；
- `--queue-size 1024`：增加回调缓存，降低高负载时丢帧风险。

看到等待提示后启动 EGO TF 卡录制。EGO MCAP 必须使用 DAS-Ego 官方设备端录制功能生成，本项目的 Python 采集器只负责 NOKOV。

建议动作顺序：

1. XINGYING 开始 CAP；
2. 执行上面的 NOKOV 命令；
3. 在 5 秒等待期间启动 EGO；
4. 保持头部静止 3 秒；
5. 快速左转一次并回正；
6. 快速右转两次并回正；
7. 抬头一次、低头两次，均回正；
8. 左右侧倾各一次；
9. 缓慢自由转头约 10 秒；
10. 最后静止 3 秒；
11. 停止 EGO；
12. PowerShell 中按 `Ctrl+C` 停止 NOKOV；
13. 停止 XINGYING CAP。

从 TF 卡或 EGO 数据目录复制 MCAP：

```powershell
Copy-Item `
  D:\你的EGO录制目录\DAS-Ego_*.mcap `
  sessions\session_head_sync_001\ego\recording.mcap
```

如果已经明确知道文件名，也可以直接指定：

```powershell
Copy-Item `
  D:\data\recording.mcap `
  sessions\session_head_sync_001\ego\recording.mcap
```

若通配符匹配多个 MCAP，必须手动选择本次录制对应的文件，不能把多次录制混入同一个 session。将 XINGYING CAP 放入 `nokov\raw_capture\`，或在其中记录 CAP 的受控存储位置。

### 2.7 Windows 采集后检查

```powershell
Get-ChildItem sessions\SESSION_NAME\nokov

$rows = Import-Csv sessions\SESSION_NAME\nokov\nokov_rigid_bodies.csv
"总刚体行数: $($rows.Count)"
"有效行数: $(($rows | Where-Object { $_.valid_numeric -eq '1' }).Count)"

Get-Item sessions\SESSION_NAME\ego\recording.mcap
```

至少应有：

```text
session_xxx/
├── ego/recording.mcap
├── nokov/nokov_frames.csv
├── nokov/nokov_rigid_bodies.csv
├── nokov/nokov_rigid_body_markers.csv
├── nokov/asset_descriptions.json
├── nokov/capture_metadata.json
└── synchronization/
```

只采集刚体时 `nokov_markers.csv` 中没有 Hand(24) 数据是正常的。左右手正式采集按下一节操作。

### 2.8 同时录制左右手 Hand(24) 和头部刚体

先在 XINGYING 中同时加载：

- `head_rigidbody`；
- 左手 Hand(24) MarkerSet；
- 右手 Hand(24) MarkerSet。

确保左右手各有 24 个已命名 Marker，头环 4 个 Marker 可见，然后开启 SDK/Data Adapter 广播。先读取实际资产名称：

```powershell
.\.venv\Scripts\python.exe tools\capture_nokov_hand24.py `
  --server 10.1.1.198 `
  --output sessions\_discovery\nokov `
  --list-only

Get-Content sessions\_discovery\nokov\asset_descriptions.json
```

下面假设 JSON 中的准确名称为 `Left Hand(24)` 和 `Right Hand(24)`。如果现场名称不同，必须使用 JSON 中的名称，不能猜测。

创建双手采集 session：

```powershell
$SESSION_NAME = "session_bimanual_hand24_001"

New-Item -ItemType Directory -Force `
  "sessions\$SESSION_NAME\ego", `
  "sessions\$SESSION_NAME\nokov", `
  "sessions\$SESSION_NAME\nokov\raw_capture", `
  "sessions\$SESSION_NAME\synchronization", `
  "sessions\$SESSION_NAME\calibration"
```

先在 XINGYING 中开始 CAP，然后同时采集左右手 24 点和头部刚体：

```powershell
.\.venv\Scripts\python.exe tools\capture_nokov_hand24.py `
  --server 10.1.1.198 `
  --output "sessions\$SESSION_NAME\nokov" `
  --hand-markerset "Left Hand(24)" `
  --hand-markerset "Right Hand(24)" `
  --head-rigidbody head_rigidbody `
  --expected-hand-markers 24 `
  --duration 0 `
  --start-delay 5 `
  --queue-size 1024
```

已经激活 `.venv` 时，可以把命令开头改成
`python tools\capture_nokov_hand24.py`，其余参数保持不变。

双手采集命令中不能加入 `--rigid-only`，否则手部 MarkerSet 不会写入。两个 `--hand-markerset` 可以重复使用，这里分别选择左手和右手。

只录制一只手时，仅保留对应参数。例如只录右手：

```powershell
.\.venv\Scripts\python.exe tools\capture_nokov_hand24.py `
  --server 10.1.1.198 `
  --output sessions\session_right_hand24_001\nokov `
  --hand-markerset "Right Hand(24)" `
  --head-rigidbody head_rigidbody `
  --expected-hand-markers 24 `
  --duration 0 `
  --start-delay 5 `
  --queue-size 1024
```

推荐录制顺序：

1. XINGYING 开始 CAP；
2. 执行双手采集命令；
3. 5 秒等待期间启动 EGO；
4. 双手自然张开并静止 3 秒；
5. 做一组明显的左右转头动作，用于 EGO–NOKOV 时间同步；
6. 左右手分别完成张手、握拳、五指依次屈曲、拇指对掌、分指、并指和手腕旋转；
7. 双手同时完成几组动作；
8. 再做一组明显的头部同步动作；
9. 双手张开并静止 3 秒；
10. 停止 EGO，按 `Ctrl+C` 停止 NOKOV，最后停止 XINGYING CAP。

头部刚体和左右手 MarkerSet 共享 NOKOV 的 `frame_no` 和 `device_timestamp_raw`，因此它们在 NOKOV 内部已经位于同一时间轴。跨设备同步仍使用 `head_rigidbody` 与 EGO IMU/VIO 完成。

主要输出：

```text
sessions\session_bimanual_hand24_001\nokov\
├── nokov_frames.csv
├── nokov_markers.csv
├── nokov_rigid_bodies.csv
├── nokov_rigid_body_markers.csv
├── asset_descriptions.json
├── capture_metadata.json
├── events.csv
└── marker_names.txt
```

`nokov_markers.csv` 中：

- `markerset_name` 区分左手和右手；
- `marker_index`、`marker_name` 区分每个反光点；
- `valid` 表示该 Marker 在当前帧是否有效；
- `x_mm/y_mm/z_mm` 是 NOKOV 世界坐标，单位 mm；
- 个别 Marker 暂时丢失时，后处理必须保留逐点 `valid` 掩码，不能把无效坐标当作零点。

采集后检查左右手行数、点名和有效率：

```powershell
$markers = Import-Csv "sessions\$SESSION_NAME\nokov\nokov_markers.csv"

$markers |
  Group-Object markerset_name |
  Select-Object Name, Count

$markers |
  Group-Object markerset_name, marker_name |
  Select-Object Name, Count

$markers |
  Where-Object { $_.valid -eq "1" } |
  Group-Object markerset_name |
  Select-Object Name, Count

Get-Content "sessions\$SESSION_NAME\nokov\capture_metadata.json"
```

重点确认 `queue_dropped_frames = 0`、`callback_errors = 0`，并检查两个 MarkerSet 都有数据。正式评价时不能把 `nokov_markers.csv` 的数组前 21 项直接当成 WiLoR 21 关节，仍需使用项目中的 24→21 语义映射和有效性规则。

## 3. Windows → Linux：用 U 盘传 session

真实 session 被 Git 忽略，不能靠 `git push/pull` 传输。推荐复制整个目录。

Windows PowerShell：

```powershell
New-Item -ItemType Directory -Force E:\nokov_ego_sessions
Copy-Item -Recurse sessions\SESSION_NAME E:\nokov_ego_sessions\
Get-FileHash sessions\SESSION_NAME\ego\recording.mcap -Algorithm SHA256
```

安全弹出 U 盘后，在 Linux 中：

```bash
cd /home/zdh/nokov_ego_hand_validation
mkdir -p sessions
cp -a /media/zdh/USB_NAME/nokov_ego_sessions/SESSION_NAME sessions/
sha256sum sessions/SESSION_NAME/ego/recording.mcap
```

两端 MCAP 的 SHA-256 应一致。也可以用受控共享目录或 `scp`，但始终传完整 session，不只传一个 CSV。

## 4. Linux：从克隆到同步结果

### 4.1 初始化

```bash
git clone https://github.com/YOUR_ACCOUNT/nokov-ego-hand-validation.git
cd nokov-ego-hand-validation
./tools/setup_linux_sync.sh
python3 tools/verify_external_assets.py --profile linux-sync
```

Ubuntu/Debian 如果提示无法创建 venv：

```bash
sudo apt update
sudo apt install -y python3-venv
./tools/setup_linux_sync.sh
```

### 4.2 执行同步

把完整 session 放入 `sessions/` 后：

```bash
./tools/run_linux_sync.sh SESSION_NAME head_rigidbody
```

脚本固定使用已经由真实数据确认的 NOKOV 时间字段：

```text
device_timestamp_raw × 0.001 = seconds
```

输出：

```text
sessions/SESSION_NAME/synchronization/
├── imu_nokov_sync.json
├── imu_nokov_aligned_signals.csv
└── imu_nokov_sync.png
```

查看结果：

```bash
cat sessions/SESSION_NAME/synchronization/imu_nokov_sync.json
xdg-open sessions/SESSION_NAME/synchronization/imu_nokov_sync.png
```

无桌面环境时只查看 JSON 和 CSV 即可。

### 4.3 判断是否成功

至少检查：

- JSON 成功生成，状态不是失败；
- NOKOV 刚体有效率高、4 个 Marker 跟踪稳定；
- `imu_nokov_sync.png` 中主要转头峰值一一对应；
- 相关系数建议 `≥ 0.8`，正式数据最好 `≥ 0.9`；
- `capture_metadata.json` 没有 queue drop 或 callback error。

算法对 EGO IMU 三轴取角速度模长，并由 NOKOV 刚体四元数计算角速度模长，因此第一阶段估计时间偏移不要求先知道 IMU 与刚体之间的固定旋转外参。它估计的是：

```text
nokov_relative_s = ego_relative_s + offset_s
```

长录制仍建议分段估计并进一步拟合 `t_nokov = a * t_ego + b`，以处理独立时钟漂移。

### 4.4 VIO–NOKOV 空间手眼标定（链路已验证）

先使用 DAS-Ego 官方 VIO/SLAM 后处理生成带有 `/robot0/vio/eef_pose` 的
`*_ego_vio.mcap`。如果官方输出目录已经包含 `pose.txt`，可以直接使用；否则从
VIO MCAP 导出：

```bash
.venv-sync/bin/python tools/export_ego_vio_pose.py \
  --mcap sessions/SESSION_NAME/ego/DAS-Ego_xxx_ego_vio.mcap \
  --output sessions/SESSION_NAME/ego/pose.txt
```

先安装空间标定依赖：

```bash
.venv-sync/bin/python -m pip install -r tools/requirements-calibration.txt
```

然后使用 `/robot0/vio/eef_pose` 导出的 TUM 格式 `pose.txt`：

```bash
.venv-sync/bin/python tools/calibrate_ego_vio_nokov.py \
  --ego-pose sessions/SESSION_NAME/ego/pose.txt \
  --nokov-csv sessions/SESSION_NAME/nokov/nokov_rigid_bodies.csv \
  --sync-json sessions/SESSION_NAME/synchronization/imu_nokov_sync.json \
  --rigid-body head_rigidbody \
  --output sessions/SESSION_NAME/calibration/T_nokov_ego_vio_provisional.json
```

`session_head_sync_001` 已进一步确认时间修正为 `+0.01 s`，该 session 的完整复算命令为：

```bash
.venv-sync/bin/python tools/calibrate_ego_vio_nokov.py \
  --ego-pose sessions/session_head_sync_001/ego/pose.txt \
  --nokov-csv sessions/session_head_sync_001/nokov/nokov_rigid_bodies.csv \
  --sync-json sessions/session_head_sync_001/synchronization/imu_nokov_sync.json \
  --rigid-body head_rigidbody \
  --time-correction-s 0.01 \
  --output sessions/session_head_sync_001/calibration/T_nokov_ego_vio_provisional.json
```

`T_A_B` 表示把 B 系坐标变换到 A 系。脚本输出 `T_B_E`、`T_Wm_We` 和
`T_We_Wm`。其中 `T_We_Wm` 用于把 NOKOV 世界坐标变换到 Ego/VIO 世界坐标。

当前结果已经过现场物理测量验证，在当前安装与当前 session 中方向、位置符合实际。
算法仍保留误差统计和 `provisional_rotation_only` 状态，提醒后续评价不要忽略平移残差。

`session_head_sync_001` 当前验证通过的 NOKOV 世界坐标到 Ego/VIO 世界坐标变换为
（输入输出单位均为米，NOKOV 原始 mm 必须先乘 `0.001`）：

```text
p_We = T_We_Wm @ p_Wm

T_We_Wm =
[[ 0.1746872616,  0.9824355326,  0.0656108601, -0.0693047093],
 [-0.9844311951,  0.1755833991, -0.0081050691,  0.1237448602],
 [-0.0194828858, -0.0631735251,  0.9978123686, -0.3396085837],
 [ 0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000]]
```

必须遵守以下复用规则：

- `T_B_E` 只有在反光球刚体相对头环完全没有移动时才可复用；
- `T_Wm_We` 和 `T_We_Wm` 与本次 VIO 世界原点绑定，每次重新启动 VIO/重新录制都要重新计算；
- NOKOV 重新标定世界坐标、修改刚体原点/轴向或重新粘贴 Marker 后，全部空间变换重新计算；
- `--time-correction-s 0.01` 只属于 `session_head_sync_001`，不能直接套到新 session。

### 4.5 时间同步与空间标定一键代码

推荐直接运行统一入口。它会依次执行：

```text
原始 EGO MCAP + NOKOV rigid CSV
  -> IMU/刚体角速度时间同步
  -> imu_nokov_sync.json
  -> 从 *_ego_vio.mcap 导出 pose.txt（已有时复用）
  -> AX=XB 空间手眼标定
  -> T_B_E、T_Wm_We、T_We_Wm
  -> ego_nokov_alignment_summary.json
```

新 session 默认命令：

```bash
.venv-sync/bin/python tools/run_ego_nokov_alignment.py \
  --session-dir sessions/SESSION_NAME \
  --rigid-body head_rigidbody \
  --max-offset-s 30
```

`session_head_sync_001` 的完整复现命令包含该 session 专属的 `+0.01 s` 细化量：

```bash
.venv-sync/bin/python tools/run_ego_nokov_alignment.py \
  --session-dir sessions/session_head_sync_001 \
  --rigid-body head_rigidbody \
  --max-offset-s 30 \
  --fine-time-correction-s 0.01
```

如果 `ego/pose.txt` 不存在，程序会自动查找唯一的 `*_ego_vio.mcap` 并导出；
如果原始 MCAP 或 VIO MCAP 不止一个，应使用 `--ego-mcap`、`--ego-vio-mcap`
明确指定。所有相对输入路径都相对于 `--session-dir`。

最终输出：

```text
sessions/SESSION_NAME/
├── ego/pose.txt
├── synchronization/
│   ├── imu_nokov_sync.json
│   ├── imu_nokov_aligned_signals.csv
│   └── imu_nokov_sync.png
└── calibration/
    ├── T_nokov_ego_vio_provisional.json
    └── ego_nokov_alignment_summary.json
```

其中 `ego_nokov_alignment_summary.json` 同时包含最终时间映射、相关系数、
空间矩阵和空间残差，是后续 NOKOV Hand(24) → Ego/WiLoR 评价应读取的统一入口。

## 5. 坐标系约定

项目采用：

```text
官方 Ego/VIO：X 前、Y 左、Z 上
原始 MCAP IMU：X 左、Y 下、Z 后
IMU -> Ego：[-imu_z, +imu_x, -imu_y]
```

时间同步第一阶段使用角速度模长，对固定换轴不敏感；后续按轴比较、外参标定和轨迹变换必须应用上述映射。依据见 [DAS-Ego IMU 坐标系说明](docs/das_ego_imu_coordinate_system_zh.md)。

## 6. 目录结构

```text
.
├── README.md
├── assets/                    # 现场标定；本地文件被忽略
├── docs/
├── ego_wilor/                 # 后续 EGO/WiLoR/MANO 处理
├── reference_data/            # 本地参考数据；被忽略
├── sessions/
│   ├── README.md
│   └── session_001/           # 仅模板
├── tools/                     # Windows 采集与 Linux 同步入口
└── vendor/                    # NOKOV/XINGYING 厂商包；被忽略
```

每次实验创建新的 session。不要混合不同被试、重新佩戴、不同 NOKOV 标定或不同 EGO 录制的数据。

## 7. GitHub 发布

在 GitHub 创建一个空仓库后：

```bash
git remote add origin git@github.com:YOUR_ACCOUNT/nokov-ego-hand-validation.git
git push -u origin main
```

发布前检查：

```bash
git status --short
git ls-files | grep -E '\.(mcap|cap|c3d|trc|ckpt|pt|pkl|whl|exe)$' && echo '检查到不应提交的二进制'
```

不要用 `git add -f` 绕过 `.gitignore`。建议先使用 GitHub 私有仓库；被试数据和许可证受限文件即使仓库是私有的，也应按实验与供应商规则单独管理。

## 8. 详细文档

- [Windows 部署与 GitHub 发布](docs/github_and_nokov_windows_deployment_zh.md)
- [外部文件、下载地址与 U 盘清单](docs/required_assets_and_downloads_zh.md)
- [头环刚体—IMU 时间同步](docs/head_rigidbody_imu_time_sync_zh.md)
- [2026-08-27 四组真实数据结果](docs/head_sync_dataset_results_20260827.md)
- [NOKOV 24 点 → EGO/WiLoR 21 点验证方案](docs/nokov24_to_ego21_validation_plan_zh.md)
- [DAS-Ego IMU 坐标系](docs/das_ego_imu_coordinate_system_zh.md)

## 9. 当前边界

- 已完成“NOKOV 头部刚体 ↔ EGO IMU/VIO”的采集、时间同步和空间变换闭环；
- `session_head_sync_001` 的空间矩阵已由用户现场测量验证，但它仍是本次安装和本次 VIO 世界原点下的结果；
- 24 个反光 Marker 与 WiLoR 21 个解剖关节不要求物理重合，后续应按骨段、角度、尺度归一化和可见性做评价；
- NOKOV 为 mm，EGO/WiLoR 通常为 m；
- 新 session 仍需逐帧头部刚体、对应 VIO 位姿和该 session 重新求解的世界变换；
- 时间同步不能替代相机内参、NOKOV↔EGO 外参和 24→21 语义映射。

# 外部文件、下载地址与 U 盘清单

本文说明被 `.gitignore` 排除的 SDK、模型、安装包和实验数据应从哪里取得。不要使用 `git add -f` 把这些文件强行提交到公开仓库。

机器可读清单位于 [`external_assets_manifest.json`](external_assets_manifest.json)，文件完整性可用 `tools/verify_external_assets.py` 检查。

## 1. 只做 Windows NOKOV 采集时

### 可公开下载

| 软件 | 用途 | 官方地址 |
|---|---|---|
| Git for Windows x64 | 克隆和更新项目 | <https://git-scm.com/install/windows> |
| Python 3.11.9 x64 | 运行 NOKOV Python SDK；安装时保留 Python Launcher | <https://www.python.org/downloads/release/python-3119/> |
| Microsoft VC++ x64 Runtime | NOKOV 原生 DLL 缺少运行库时安装 | <https://aka.ms/vc14/vc_redist.x64.exe> |

不要让 `setup_nokov_windows.cmd` 使用 Python 3.14。当前项目明确选择 64 位 Python 3.11，找不到时才选择 3.10。

### 必须由 NOKOV 厂商包或 U 盘提供

| 文件/软件 | Git 克隆后的目标位置 | 是否必须 |
|---|---|---|
| `nokovpy-3.0.1-py3-none-any.whl` | `vendor\nokov_python_sdk\nokovpy-3.0.1-py3-none-any.whl` | 必须 |
| XINGYING 4.6 | 安装到 Windows 上位机，不放入仓库 | 必须，但已安装则不用再复制 |
| `VC_redist.x64.exe` | 不要求固定位置，管理员运行安装 | 仅 DLL 加载失败时；也可用上面的微软官方下载 |
| 现场 XINGYING 工程、相机标定、`head_rigidbody` 定义 | 由 XINGYING 打开或导入 | 现场必需，不能由代码仓库替代 |

目前没有把 NOKOV Python wheel 和 XINGYING 安装程序写成公共直链：它们来自交付包，并可能受版本与再分发许可限制。需要向 NOKOV 技术支持索取与 XINGYING 4.6 匹配的 SDK，或从已经验证过的 Linux 工作站复制。

本机已经验证过的文件指纹：

| 文件 | 大小 | SHA-256 |
|---|---:|---|
| `nokovpy-3.0.1-py3-none-any.whl` | 403,145 B | `214f321e2149346e5def41d4804f9b97ee0bce458d14858ef065a62a4dc779cc` |
| `XINGYING_4.6.0.7923_Windows_x64.exe` | 576,292,570 B | `106e88e0c7ea003e952fb536d719b5d14f6ea1f26df2b6f2bdc15a5b1a5733ed` |
| 厂商包内 `VC_redist.x64.exe` | 25,168,640 B | `a1592d3da2b27230c087a3b069409c1e82c2664b0d4c3b511701624702b2e2a3` |

如果厂商提供了不同版本，不应为了匹配这里的哈希而从非官方网站寻找旧文件；先核对版本，再更新清单并重新做一次 `--list-only` 和短采集验证。

### 推荐 U 盘布局

```text
USB_ROOT\nokov_windows_assets\
├── nokovpy-3.0.1-py3-none-any.whl
├── VC_redist.x64.exe                 # 可选
├── XINGYING_4.6.0.7923_Windows_x64.exe  # 上位机未安装时
└── site_assets\                     # 现场工程/标定/刚体定义，按实际情况
```

Windows PowerShell 中复制 wheel：

```powershell
New-Item -ItemType Directory -Force vendor\nokov_python_sdk
Copy-Item E:\nokov_windows_assets\nokovpy-3.0.1-py3-none-any.whl `
  vendor\nokov_python_sdk\nokovpy-3.0.1-py3-none-any.whl
python tools\verify_external_assets.py --profile windows-capture
```

把 `E:` 换成实际 U 盘盘符。

## 2. 只做 Linux 时间同步时

不需要 NOKOV SDK、XINGYING、WiLoR、MANO、MediaPipe、CUDA 或 Orbbec SDK。需要的只有：

- GitHub 仓库中的代码；
- 从 Windows/U 盘传回的整个 `sessions/session_xxx/`；
- Linux 的 Python 3、`venv` 和联网安装 PyPI 依赖的能力。

Ubuntu/Debian 缺少 `venv` 时：

```bash
sudo apt update
sudo apt install -y python3 python3-venv
```

离线 Linux 环境可在另一台同架构、同 Python 版本机器上预下载 `tools/requirements-sync.txt` 中的 wheels，再从 U 盘安装。

## 3. 后续完整 WiLoR/MANO 处理时

这些资产与当前“头部刚体—IMU 时间同步”无关，只在以后运行 EGO 六目/WiLoR/MANO 链路时补充。

| 资产 | 获取方式 | 目标位置 |
|---|---|---|
| MediaPipe Hand Landmarker | [Google 官方模型](https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task)，也可运行 `ego_wilor/scripts/install_mediapipe_model.sh` | `ego_wilor/models/hand_landmarker.task` |
| WiLoR `detector.pt` | [作者发布地址](https://huggingface.co/spaces/rolpotamias/WiLoR/resolve/main/pretrained_models/detector.pt) | `ego_wilor/models/wilor/detector.pt` |
| WiLoR `wilor_final.ckpt` | [作者发布地址](https://huggingface.co/spaces/rolpotamias/WiLoR/resolve/main/pretrained_models/wilor_final.ckpt) | `ego_wilor/models/wilor/wilor_final.ckpt` |
| WiLoR `model_config.yaml` | [作者发布地址](https://huggingface.co/spaces/rolpotamias/WiLoR/resolve/main/pretrained_models/model_config.yaml) | `ego_wilor/models/wilor/model_config.yaml` |
| MANO 左/右手模型 | [MANO 官网](https://mano.is.tue.mpg.de/)注册并接受许可证后下载 | `ego_wilor/models/mano/MANO_LEFT.pkl`、`MANO_RIGHT.pkl` |
| WiLoR 源码 | [官方实现](https://github.com/rolpotamias/WiLoR) | `ego_wilor/third_party/WiLoR/` |
| Orbbec SDK | [Orbbec 官方 SDK 页面](https://www.orbbec.com/developers/orbbec-sdk/) | 仅设备直连/对应代码需要，普通 MCAP 时间同步不需要 |

WiLoR 官方说明模型使用 CC-BY-NC-ND，MANO 也有独立许可证。公开发布项目时保留下载步骤，不直接重新分发这些模型。

完整模型检查：

```bash
python3 tools/verify_external_assets.py --profile linux-full
```

`wilor_final.ckpt` 约 2.56 GB，超过 GitHub 普通 Git 的单文件限制，也超过 GitHub Free 的单个 LFS 文件上限；它应从作者地址下载或通过受控存储/U 盘传输，而不是提交进仓库。GitHub 限制见：

- <https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github>
- <https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage>

## 4. 实验数据必须单独传输

以下文件默认被 `.gitignore` 排除，也不应通过公开 GitHub 传输：

- EGO：`.mcap`；
- XINGYING：`.cap`、`.vc*`；
- 导出：`.c3d`、`.trc`；
- 所有真实 `sessions/session_xxx/` 数据与同步输出；
- 被试相关数据、标定和结果。

每次通过 U 盘复制完整 session，至少保留：

```text
session_head_sync_xxx/
├── ego/
│   └── recording.mcap
├── nokov/
│   ├── nokov_frames.csv
│   ├── nokov_rigid_bodies.csv
│   ├── nokov_rigid_body_markers.csv
│   ├── asset_descriptions.json
│   ├── capture_metadata.json
│   └── raw_capture/                 # CAP 或其说明
└── synchronization/                # 可为空，Linux 后处理生成
```

复制完成后不要只检查文件名，还要比较哈希：

```powershell
# Windows
Get-FileHash sessions\session_head_sync_xxx\ego\recording.mcap -Algorithm SHA256
```

```bash
# Linux
sha256sum sessions/session_head_sync_xxx/ego/recording.mcap
```

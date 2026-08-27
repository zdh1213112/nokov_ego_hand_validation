# GitHub 发布与 NOKOV Windows 上位机部署

从全新克隆开始的可执行主流程以根目录 [`README.md`](../README.md) 为准；
所有 `.gitignore` 外部文件、官方下载地址和 U 盘布局见
[`required_assets_and_downloads_zh.md`](required_assets_and_downloads_zh.md)。本文保留发布策略和 Windows 部署要点。

## 1. 仓库策略

建议首先创建 GitHub 私有仓库。Git 仓库只保存代码、文档、配置模板和启动脚本，不保存：

- EGO MCAP；
- XINGYING CAP、VC、TRC、C3D；
- 被试数据和评价结果；
- 场地相机标定；
- XINGYING 安装包；
- WiLoR/MANO/MediaPipe 模型；
- 未确认再分发许可的 NOKOV SDK 二进制。

上述内容已经由根目录 `.gitignore` 排除。不要使用 `git add -f` 绕过规则。

GitHub 普通 Git 对大于100 MiB的单文件会阻止上传。虽然可以使用 Git LFS，但设备录制和第三方安装包仍建议通过受控存储或厂家渠道分发：

- <https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github>
- <https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage>

## 2. 第一次发布

当前目录已经初始化为 Git 仓库。先在 GitHub 网页创建一个空的私有仓库，例如：

```text
nokov-ego-hand-validation
```

不要在网页端自动创建 README、License 或 `.gitignore`，然后在本机执行：

```bash
cd /home/zdh/nokov_ego_hand_validation

git remote add origin git@github.com:YOUR_ACCOUNT/nokov-ego-hand-validation.git
git push -u origin main
```

也可以使用 HTTPS：

```bash
git remote add origin https://github.com/YOUR_ACCOUNT/nokov-ego-hand-validation.git
git push -u origin main
```

HTTPS 登录应使用 GitHub 凭据管理器或 Personal Access Token，不要把 token 写进脚本、URL、README 或 `.env` 并提交。

## 3. 在 NOKOV Windows 上位机克隆

安装 [Git for Windows](https://git-scm.com/install/windows) 和64位 Python 3.11（也支持3.10）后：

```bat
git clone https://github.com/YOUR_ACCOUNT/nokov-ego-hand-validation.git
cd nokov-ego-hand-validation
```

从厂家 NOKOV Python SDK 包复制：

```text
nokovpy-3.0.1-py3-none-any.whl
```

到：

```text
vendor\nokov_python_sdk\nokovpy-3.0.1-py3-none-any.whl
```

如果 SDK 包要求 VC++ 运行库，请使用厂家提供的 `VC_redist.x64.exe` 安装；不要从非官方来源下载 DLL。

## 4. Windows 一键初始化

双击：

```text
tools\setup_nokov_windows.cmd
```

脚本会：

1. 创建项目专用 `.venv`；
2. 安装 NOKOV wheel；
3. 安装 MCAP、NumPy 和绘图依赖；
4. 检查64位 Python；
5. 检查 NOKOV 原生 DLL 能否加载。

## 5. 上位机执行顺序

1. 启动 XINGYING；
2. 加载并播放包含 `head_rigidbody` 的场景；
3. 开启 SDK 数据广播；
4. 双击 `tools\list_nokov_assets.cmd` 检查刚体；
5. 在 XINGYING 中开始录制 CAP；
6. 双击 `tools\capture_head_rigidbody.cmd` 开始 SDK CSV 采集；
7. 开始 EGO TF卡录制并执行同步动作；
8. 停止 EGO、SDK 和 CAP；
9. 将 EGO MCAP 复制成 `sessions\SESSION_NAME\ego\recording.mcap`；
10. 双击 `tools\sync_head_imu_nokov.cmd`；
11. 查看 `synchronization\imu_nokov_sync.png` 和 JSON。

完整动作要求见 [`head_rigidbody_imu_time_sync_zh.md`](head_rigidbody_imu_time_sync_zh.md)。

## 6. 数据回传规则

实验 session 默认被 Git 忽略。不要通过普通 Git commit/push 上传被试和设备数据。

需要把数据传回 Linux 分析机时，复制整个 session 目录到受控存储，包括：

```text
session_xxx/
├── ego/recording.mcap
├── nokov/nokov_rigid_bodies.csv
├── nokov/nokov_rigid_body_markers.csv
├── nokov/capture_metadata.json
├── nokov/raw_capture/...
└── synchronization/...
```

代码更新和数据传输应分开管理。

Linux 收到数据后执行：

```bash
./tools/setup_linux_sync.sh
./tools/run_linux_sync.sh session_head_sync_001 head_rigidbody
```

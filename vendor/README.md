# Local vendor packages

第三方安装包和 SDK 二进制默认不上传 GitHub。

在 Linux NOKOV 工作站运行前，将厂家提供的 Python wheel 复制为：

```text
vendor/nokov_python_sdk/nokovpy-3.0.1-py3-none-any.whl
```

该 wheel 已确认同时包含 Linux x86-64 的 `libnokov_sdk.so`。也可以安装厂家 SDK 到当前 Python 环境；采集器会优先导入已经安装的 `nokov.nokovsdk`。

Linux 初始化：

```bash
./tools/setup_nokov_linux.sh
conda activate nokov-ego-validation
```

Windows DLL/VC++ 说明仅适用于保留的兼容工作流：

如果 Windows 报 DLL 加载错误，请使用 NOKOV SDK 包中厂家提供的 `VC_redist.x64.exe` 安装 Microsoft Visual C++ 运行库。是否有权重新分发这些文件应以厂家许可证为准。

微软官方 x64 运行库下载：<https://aka.ms/vc14/vc_redist.x64.exe>。

完整的下载地址、经过验证的 SHA-256 和 U 盘布局见
[`docs/required_assets_and_downloads_zh.md`](../docs/required_assets_and_downloads_zh.md)。

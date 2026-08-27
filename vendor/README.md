# Local vendor packages

第三方安装包和 SDK 二进制默认不上传 GitHub。

在 NOKOV Windows 上位机运行前，将厂家提供的 Python wheel 复制为：

```text
vendor/nokov_python_sdk/nokovpy-3.0.1-py3-none-any.whl
```

也可以安装厂家 SDK 到当前 Python 环境；采集器会优先导入已经安装的 `nokov.nokovsdk`。

如果 Windows 报 DLL 加载错误，请使用 NOKOV SDK 包中厂家提供的 `VC_redist.x64.exe` 安装 Microsoft Visual C++ 运行库。是否有权重新分发这些文件应以厂家许可证为准。

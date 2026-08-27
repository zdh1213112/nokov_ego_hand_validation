# Session Template

每次实验复制一份 `session_001` 并改名，例如：

```text
session_20260827_subject01_right_hand
```

从项目根目录复制：

```bash
cp -a sessions/session_001 sessions/session_20260827_subject01_right_hand
```

不要在同一个 session 目录里混放不同被试、不同佩戴或不同相机标定的数据。

推荐 session 结构：

```text
session_name/
├── nokov/
├── ego/
├── calibration/
├── synchronization/
├── config/
└── evaluation/
```

采集结束后应同时保存 NOKOV SDK CSV、原始 CAP 工程、TRC、C3D 和 EGO
MCAP。SDK CSV/TRC 用于第一版解析，C3D 和原始工程用于追溯与重新导出。

所有模板文件中带有 `TODO`、`null` 或 `PLACEHOLDER_DO_NOT_USE` 的字段均不可直接用于评价。

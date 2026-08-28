# session004 坐标链逐层诊断

分析对象：`session_head_sync_004`、`camera2`、前 300 个六目同步帧。

## 结论先行

造成约 404 px 稳定偏移的主因不是 Double-Sphere 投影，也不是 90 Hz/30 Hz
时间插值，而是把两个不同的“机身坐标系”直接串接了：

```text
现有 AX=XB 结果：NOKOV head_rigidbody B -> DAS VIO body/IMU E
GEN camera_info / WiLoR 融合：camera -> GEN base/rig G
```

原渲染链将 `B -> E` 的结果直接当成 `B -> G` 使用，漏掉了固定的 `E -> G`
安装坐标变换。加入一个接近官方轴定义的 `E -> G` 轴重标定后，手中心粗略像素
间隙从 404 px 降到约 25 px；再用 WiLoR 手中心拟合完整刚体后约 23 px。

这两个“几十像素”结果只是诊断证据，不是正式真值外参。正式流程仍应使用相机和
NOKOV 同时可见的标定板/Marker，直接优化 `T_GENbase_head_rigidbody` 的多帧重投影误差。

## 官方文档核对

已联网核对 [GenRobot DAS Ego 手册](https://docs.genrobot.ai/zh/products/das-ego)：

- 世界坐标原点是初始采集时刻 IMU 位置；Z 轴向上；X 轴为初始化时设备前向水平投影；Y 轴按右手定则确定。
- 局部坐标原点是 IMU 几何中心；X 为设备前向，Y 为设备左侧，Z 为设备上向。
- `/robot0/vio/eef_pose` 是世界坐标中的 DAS-Ego pose；`/robot0/vio/relative_eef_pose` 是相对初始姿态的 pose。
- 六路 `/camera*_compressed` 为 30 Hz，`/robot0/sensor/imu` 为 200 Hz，`camera_info` 为单帧标定消息。

官方设备坐标图也与文字一致：红色 X 从设备正面向前，绿色 Y 指向佩戴者左侧，蓝色 Z
向上。官方 Tilted/Aligned 图强调 `eef_pose` 的 world frame 与随设备运动的 local frame
是两个不同 frame，不能因为数据中都出现 `body/base` 字样就默认原点和轴完全相同。

公开手册没有在页面正文中给出 `T_b_c` 的矩阵方向；本项目通过以下独立证据判定其为
`camera -> GEN base`：字段名、六目射线求交实现、相机中心布局，以及将
`joints_base_m` 用 `inverse(T_b_c)` 投回原观测时只有约 5.6 px 中位误差。

因此 `pose.txt` 中的 VIO pose 可作为 EGO 世界/局部 body 轨迹输入，但不能仅凭
topic 名称推断它与 GEN `camera_info.T_b_c` 的 base 完全同名同原点。项目内部对
`T_base_camera` 的实现和六目融合也明确使用 `GEN base/rig`。

## 每一步检查结果

| 层级 | 实际检查 | 结果 |
|---|---|---|
| NOKOV 数据 | `device_timestamp_raw` 为 Unix epoch ms；90.0 Hz；4 个头环 Marker；9999999 哨兵已过滤 | 正常 |
| 时间映射 | `t_nokov = 1.0*t_ego + 0.349934461 s`；290/299 帧两套手同时存在 | 正常 |
| 头刚体插值 | 位置线性插值、四元数 SLERP；最大括号间隔 50 ms | 正常 |
| AX=XB | `T_B_E` 旋转误差中位数 0.86°；逐帧 Y 平移误差中位数 11.7 mm | 可用但平移 provisional |
| 相机内参 | 六路 1600×1300；D 顺序 `[fx,fy,cx,cy,xi,alpha]` | 正常 |
| DS 投影 | GEN 正向公式与代码一致；光轴回投影到主点；正逆模型单测通过 | 正常 |
| `T_base_camera` 方向 | `camera -> GEN base`；用逆矩阵投影融合点，camera2 重投影中位数约 5.6 px | 正常 |
| NOKOV→GEN 链 | 直接漏 `E->G` 时 404 px；补轴重标定后 25 px；拟合后 23 px | 错在坐标系拼接 |

六目标定参数本身也通过了结构检查：

| 项目 | session004 实测 |
|---|---|
| 分辨率 | 六路均为 1600×1300 |
| DS 参数与 K | 每路 `D[0:4] == [K.fx,K.fy,K.cx,K.cy]`，完全一致 |
| 旋转矩阵 | 六路 `det(R)=1`，正交误差约 `1e-15` |
| camera2/3 主双目基线 | 59.37 mm |
| 六路相机中心横向跨度 | camera0 到 camera5 为 199.65 mm |
| 主相机光轴 | camera2/3 在 GEN base 中几乎均为 `(0,0,-1)` |

相机中心从 camera0 到 camera5 沿 GEN base X 轴从 `+101.17 mm` 单调变化到
`-98.47 mm`，主相机光轴指向 GEN `-Z`。结合官方 EGO local 的 `X前、Y左、Z上`，
可直接得到近似轴关系：

```text
GEN +X ≈ EGO +Y（左）
GEN +Y ≈ EGO -Z（下）
GEN +Z ≈ EGO -X（后）
```

这正是诊断拟合得到的主要 90° 轴置换；完整拟合相对该纯轴置换只差约 4–5°。

## 关键数值证据

诊断脚本输出的 `camera2` 候选中位数/ P95：

| 候选链 | median | P95 |
|---|---:|---:|
| `documented_B_to_E_only`（按 JSON 方向） | 786.7 px | 820.8 px |
| `legacy_direct_matrix`（历史直接用法） | 404.2 px | 432.6 px |
| `axis_relabel_E_to_GEN` | 25.0 px | 36.3 px |
| `fitted_E_to_GEN`（WiLoR 手中心代理拟合） | 22.6 px | 34.8 px |

`fitted_E_to_GEN` 的手中心三维残差为 median 21.2 mm、P95 40.3 mm。它解释了
为什么图像中能看到骨架形状和动作已经同步，但仍存在稳定的小偏差：24 个反光点
是皮肤/手套表面 Marker，WiLoR 点是解剖关节，二者不应逐点相等；手中心拟合还会
吸收 Marker 偏移、手部形状差异和 WiLoR 误差。

## 可视化产物

运行：

```bash
PYTHONPATH=tools python3 tools/diagnose_nokov_coordinate_chain.py \
  --session-dir sessions/session_head_sync_004 \
  --dataset sessions/session_head_sync_004/visualization/normalized_multiview_300 \
  --fusion sessions/session_head_sync_004/visualization/fusion_multiview_300_ignore_handedness \
  --hand-eye-json sessions/session_head_sync_004/calibration/T_nokov_ego_vio_provisional.json \
  --camera camera2 \
  --output-dir sessions/session_head_sync_004/visualization/coordinate_chain_diagnostic \
  --preview-count 8 \
  --render-videos
```

输出：

- `coordinate_chain_diagnostic_preview.jpg`：四宫格，依次为漏变换、历史直接、轴重标定、手中心拟合；
- `coordinate_chain_candidate_gaps.png`：299 帧各候选链的粗略手中心像素间隙曲线；
- `coordinate_chain_frames.csv`：每帧 Wm/B/E 中间中心、相机深度、像素中心和候选误差；
- `summary.json`：机器可读的完整诊断报告。
- `selected_corrected_frames_contact_sheet.jpg`：从 8 个时间区间各挑一帧低残差画面，按“原错误链/轴修正/拟合修正”三列排列；
- `selected_frames/`：8 组全分辨率轴修正版、拟合修正版和三联对比图；
- `camera2_axis_relabel_alignment.mp4`：固定官方轴关系、仅用手中心估计平移的保守诊断版视频；
- `camera2_fitted_alignment.mp4`：使用诊断性 `E->GEN` 刚体拟合的视频；
- `camera2_before_axis_fitted_comparison.mp4`：原错误链、轴修正版、拟合修正版的三联视频。

三段视频均为 1600×1300 或 1920×520、299 帧、约 9.97 秒、29.998 fps。静态帧
不是从全序列挑绝对最低误差，而是先按时间分成 8 段，再在每段选低残差帧，确保
同时覆盖序列前、中、后段和不同手势。选中帧的拟合版粗略间隙为 1.6–26.1 px。

## 反光球与 NOKOV 识别骨架的进一步核对

在 RGB 原图中可以直接看到手套上的白色反光球。原来的坐标链诊断使用
`nokov_markers.csv` 的 24 个物理球心，但只用 WiLoR 手中心估计平移，因此在
`sync152` 处球心投影整体仍低约 11–19 px。新增的
`tools/refine_nokov_marker_camera_alignment.py` 以当前拟合链为初值，在原图中匹配
亮斑并优化一个相机局部 SE(3) 修正；同时读取 `nokov_skeleton_segments.csv`，把
NOKOV 识别骨架点与物理球心分层绘制。

session004 的 `camera2` 结果：

| 项目 | 结果 |
|---|---:|
| 修正帧匹配球心 | 44 |
| 修正帧内点中位误差 | 0.35 px |
| 修正帧内点 P95 | 0.77 px |
| 全段抽样球心最近亮斑中位误差 | 0.77 px |
| 全段抽样球心最近亮斑 P95 | 4.95 px |
| 修正帧 40 个手指 marker 与 skeleton 对应点中位距离 | 1.9 mm |
| 修正帧 40 个手指 marker 与 skeleton 对应点 P95 | 5.3 mm |

对应输出位于：

```text
sessions/session_head_sync_004/visualization/marker_ball_refinement/
```

其中洋红/橙色环是 24 个物理反光球，紫色/黄色细线和方框是 NOKOV
`skeleton_segments` 识别点，青色/绿色粗线是 WiLoR 21 点。物理球心与识别骨架点
不应被强制认为是同一组解剖关节点；该输出只证明相机投影和球心位置已经对齐，
图像辅助修正仍不是正式手眼外参。

## 对现有转换流程的判断

```text
NOKOV marker Wm(mm)
  -> inverse(T_Wm_B(t))                         [正确]
  -> inverse(T_B_E)                              [正确，但结果在 E]
  -> T_E_G（当前缺失）                           [主错误]
  -> inverse(T_G_Ck) = inverse(camera.T_base_camera) [正确]
  -> GEN Double-Sphere                            [正确]
```

现有 `T_B_E` 文件的 JSON 语义和 AX=XB 方程是一致的；问题在于文件中的 `E` 被标注为
“DAS-Ego local body/IMU-center frame”，而渲染代码后续需要的是 GEN `base/rig`。
不能把 `T_B_E` 的逆矩阵直接当成 `T_G_B`。

## 下一步

1. 从 GEN URDF/飞书文档取得 `T_G_E`（若官方定义了明确的 body/base 固定外参），替换诊断中的轴重标定。
2. 若没有可复用的 `T_G_E`，采集刚体 Marker 与相机可见标定板，直接求 `T_G_B`，分别验证 camera0…camera5。
3. 在独立保留序列上验收：手中心 median < 5 px、P95 < 10–15 px，再考虑正式标签导出。
4. 不要把诊断中的 `fitted_E_to_GEN` 写回正式标定文件；它只用于证明漏掉的 frame bridge 是主问题。

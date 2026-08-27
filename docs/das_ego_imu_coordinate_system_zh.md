# DAS-Ego 原始 IMU 与 Ego/VIO 坐标系映射

本文记录 DAS-Ego MCAP 原始 IMU、Ego/VIO 局部坐标系以及 NOKOV 头环刚体之间的坐标约定。后续进行时间同步、姿态比较和空间外参标定时，必须使用本文定义，不能把原始 IMU 三轴直接当作 Ego/VIO 三轴。

## 1. 最终结论

```text
官方 Ego/VIO 局部坐标系：+X 前、+Y 左、+Z 上
MCAP 原始 IMU 坐标系：   +X 左、+Y 下、+Z 后

IMU -> Ego：[-imu_z, +imu_x, -imu_y]
```

也就是：

```text
ego_x = -imu_z
ego_y = +imu_x
ego_z = -imu_y
```

该换轴关系同时适用于陀螺仪角速度和加速度计三轴向量。

## 2. 官方 Ego/VIO 坐标系

简智官方 DAS-Ego 文档定义的 Ego 局部坐标系为：

- 原点：IMU 几何中心；
- `+X`：设备前方；
- `+Y`：设备左方；
- `+Z`：设备上方；
- 坐标系满足右手定则。

官方世界坐标系的 `+Z` 与重力方向相反、指向上方；初始化时，设备前向在水平面的投影定义世界 `+X`。

参考：

- <https://docs.genrobot.ai/zh/products/das-ego>
- <https://docs.genrobot.ai/zh/guides/das-ego-data-introduction>

## 3. 原始 MCAP IMU 到 Ego/VIO 的转换

近似的轴交换矩阵为：

```python
import numpy as np

R_ego_from_imu = np.array([
    [0.0,  0.0, -1.0],
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
], dtype=np.float64)

gyro_ego = R_ego_from_imu @ gyro_imu
accel_ego = R_ego_from_imu @ accel_imu
```

如果输入采用批量数组 `vectors_imu.shape == (N, 3)`，可以写成：

```python
vectors_ego = vectors_imu @ R_ego_from_imu.T
```

针对本项目已检查的 DAS Ego V6 数据，使用原始 IMU 角速度与 VIO 姿态差分拟合得到的精确旋转矩阵为：

```python
R_ego_from_imu_fitted = np.array([
    [ 0.003759, -0.002359, -0.999990],
    [ 0.999991,  0.001892,  0.003754],
    [ 0.001883, -0.999995,  0.002367],
], dtype=np.float64)
```

该矩阵与理想轴交换矩阵的差异小于约 1 度。一般数据处理可以使用理想矩阵；需要高精度姿态外参时，应保存并使用精确拟合矩阵，或使用厂家 URDF/标定外参重新确认。

## 4. 数据依据和适用范围

本次检查使用的数据为：

```text
/home/zdh/ego_genrobot/data/output/20260820/1/
DAS-Ego_20260820112438_none_none_689985_1088ffc0_ego_vio.mcap
```

设备和处理信息：

```text
设备：DAS Ego V6
VIO：v0.2.21_ego_opt_v1-20260727115537
IMU topic：/robot0/sensor/imu，约 200 Hz
VIO topic：/robot0/vio/eef_pose，约 30 Hz
```

原始 IMU 消息含角速度和线加速度，但没有提供 `frame_id`。因此：

- Ego/VIO 坐标系采用官方定义；
- 原始 IMU 安装轴采用本数据中 IMU 与 VIO 运动的实测拟合结果；
- 更换硬件版本、固件、VIO 镜像或厂家标定文件后，应重新验证映射。

实测陀螺仪单位为 `rad/s`。本样本加速度模长静止附近约为 `1`，因此该数据中的加速度数值表现为以 `g` 为单位；使用前仍应根据具体固件和数据格式再次确认。

## 5. camera2 不是设备前向坐标系

相机光学坐标通常表示为：

```text
camera +X：图像向右
camera +Y：图像向下
camera +Z：镜头光轴方向
```

本数据中 camera2 的 `T_b_c` 表明其安装方向近似为：

```text
camera2 +X -> Ego -X
camera2 +Y -> Ego +Y
camera2 +Z -> Ego -Z
```

因此不能把 `camera2 +Z` 解释成头环整体的“设备前方”。原始 IMU 与 camera2 光学坐标之间的换轴关系，不能直接用于命名设备的前、后、左、右、上、下。

## 6. 用于 NOKOV–EGO 时间同步

如果只使用陀螺仪角速度模长进行时间偏移拟合：

```python
omega_norm = np.linalg.norm(gyro_imu, axis=-1)
```

旋转不会改变向量模长，所以只做角速度模长峰值匹配时可以不换轴。

以下操作必须先把原始 IMU 转换到 Ego 坐标系：

- 比较三轴角速度的正负方向；
- 分别对 yaw、pitch、roll 信号做相关；
- 使用 IMU 积分姿态；
- 估计 Ego/VIO 与 NOKOV 头环刚体之间的旋转外参；
- 将重力方向或加速度方向与 VIO/NOKOV 比较。

推荐处理顺序：

```text
MCAP raw IMU
  -> R_ego_from_imu
Ego 局部角速度
  -> 时间映射 t_nokov = a * t_ego + b
  -> 固定旋转外参 R_nokov_from_ego
NOKOV 世界/头环刚体坐标
```

## 7. NOKOV 头环刚体建议

在 XINGYING 中创建 `head_rigidbody` 时，建议让刚体局部轴尽量接近 Ego 官方局部坐标系：

```text
+X：头环/佩戴者前方
+Y：头环/佩戴者左方
+Z：头环/佩戴者上方
```

如果 XINGYING 中实际建立的刚体轴向不同，必须标定固定旋转：

```text
R_nokov_rigidbody_from_ego
```

不能只因为两套坐标系都叫 `X/Y/Z` 就直接比较四元数或三轴角速度。

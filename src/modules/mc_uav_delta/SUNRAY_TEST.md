# Sunray 150：UAV Delta 向下 90 度机械臂角测试

本分支用于在没有机械臂的 Sunray 150 上测试原 UAV Delta checkpoint。机载 `delta_arm_fake` 以 50 Hz 发布 `delta_arm_state`，消息满足 `valid=true`、`sequence` 递增、`q_actual=[pi/2,pi/2,pi/2]`，与真实驱动的 DOWN 角度一致，策略本体和四路电机输出不做替换。

固定向下 90 度只表示机械臂观测为该角度，不会模拟训练模型中的机械臂质量和惯量。本配置验证机载控制链路以及策略对 Sunray 的适应性，不等价于重新训练一个无机械臂策略。

## Mac 获取分支并烧录 7-Nano

先安装 PX4 构建依赖和 `arm-none-eabi-gcc/g++`，然后执行：

```bash
git fetch origin
git switch --track origin/sunray-uav-delta-test
# 如果本地已经有该分支，改用：git switch sunray-uav-delta-test
git pull --ff-only origin sunray-uav-delta-test
make cuav_7-nano_sunray_uav_delta upload
```

该构建不包含 Dynamixel 驱动和 RAPTOR，保留 Position、Control Allocator、DDS、MAVLink 及传感器链路。

## 上传策略

策略不嵌入固件，刷写后需单独上传。将 `<MAVLINK>` 换成实际的串口或 UDP 连接：

```bash
mavproxy.py --master <MAVLINK>
ftp mkdir /fs/microsd/uav_delta
ftp put src/modules/mc_uav_delta/blob/policy.tar /fs/microsd/uav_delta/policy.tar
```

更换 checkpoint 后重启飞控，或重启 `mc_uav_delta`。启动日志必须出现 checkpoint 数值自检通过和 `UAV_DELTA mode registration successful`。

## 选择机型

在 QGC 中选择 `Sunray 150 UAV Delta down-90-arm test`，或在 MAVLink Console 中执行：

```sh
param set SYS_AUTOSTART 4070
param save
reboot
```

机型默认设置：

```text
DELTA_FAKE_EN=1
MC_UAVD_ENABLE=1
MC_UAVD_GYRO_HZ=0
MC_UAVD_Q_MAXAGE=100
```

`MC_UAVD_GYRO_HZ=0` 表示读取 `IMU_GYRO_RATEMAX`。实际 `vehicle_angular_velocity` 频率必须是 100 Hz 的整数倍；仅当实际频率与该参数不一致时才显式覆盖。

若要保留现有机型，不设置 `SYS_AUTOSTART`，改为设置上述前三个参数、`param save` 并重启。无论采用哪种方式，都要检查实际机架的电机输出函数、PWM/DSHOT、传感器校准、遥控器校准和位置估计配置，不能只依赖机型默认值。

## 上电检查

使用 QGC MAVLink Console 或：

```bash
Tools/mavlink_shell.py <MAVLINK>
```

检查：

```sh
delta_arm_fake status
mc_uav_delta status
listener delta_arm_state
listener uav_delta_status
listener uav_delta_input
commander status
```

确认：

- fake-arm 序号持续增加，三个角度均为 `pi/2`（约 1.57080 rad）且 `valid=true`；
- `uav_delta_status.can_arm=true`，`delta_arm_state_stale=false`；
- `native_step_count` 对应约 100 Hz 的 GRU 更新；
- `commander status` 显示 `UAV_DELTA` 对应的外部模式编号；
- `uav_delta_input` 的 25 维观测和 4 维动作均为有限值。

在 QGC 的 Flight Modes 中，将一个拨杆位置设为 Position，另一个设为 UAV_DELTA 对应的 External Mode。专用构建没有 RAPTOR，正常只有一个外部模式时为 External Mode 1，但仍以 `commander status` 为准。

断桨检查电机编号、转向和输出函数，并验证 kill。`RC_MAP_KILL_SW` 对应通道触发后，电机输出应立即回到 disarmed 值。kill 会使飞机下落；策略异常且位置估计正常时，首先切回 Position。

## 首次实飞

1. 使用 Position 起飞并确认稳定悬停。
2. 摇杆回中，切入 UAV_DELTA，短暂保持后切回 Position。
3. 确认切换无明显跳变，再进行持续悬停测试。
4. 保存日志，检查 `uav_delta_input`、`uav_delta_status`、`delta_arm_state`、`actuator_motors`、姿态、位置和角速度。

默认日志已包含三个 UAV Delta/假机械臂话题。需要分析电机瞬态时，再通过 SD 卡 logging 配置提高 `actuator_motors` 的记录频率。

## SITL 复现

```bash
make px4_sitl_sunray_uav_delta
mkdir -p build/px4_sitl_sunray_uav_delta/rootfs/uav_delta
cp src/modules/mc_uav_delta/blob/policy.tar \
  build/px4_sitl_sunray_uav_delta/rootfs/uav_delta/policy.tar
make px4_sitl_sunray_uav_delta gz_sunray_150_vision
```

Sunray SITL 自动启用 fake-arm 和 UAV Delta，设置 `MC_UAVD_GYRO_HZ=500`，并仅为该仿真配置设置 `NAV_DLL_ACT=0` 和 `COM_DISARM_LAND=-1`，避免无 QGC 的终端测试被连接检查和落地自动上锁打断。手动 Position 仍要求有效摇杆输入，可在 QGC 中启用虚拟摇杆并回中；通过 MAVLink `MANUAL_CONTROL` 注入时，油门中位为 `z=500`。等待 EKF 就绪后，在 PX4 shell 中执行：

```sh
commander takeoff
commander mode posctl
commander status
commander mode ext1
commander mode posctl
```

运行单元测试：

```bash
make tests TESTFILTER=DeltaArmFake
make tests TESTFILTER=PolicyObservation
```

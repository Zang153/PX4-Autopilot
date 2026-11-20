# 7 Nano Dynamixel 机械臂控制与通信配置

本模块用于 UAV+Delta 项目中的三个 Dynamixel 舵机。`dxl_arm` 在飞控本地完成 Protocol 2.0 通信、实际角度反馈、平滑回中、保持和有界随机运动，并向 `mc_uav_delta` 提供 `delta_arm_state` 观测。

## 为什么必须关闭 TELEM1 上的串口 MAVLink

7 Nano 的 TELEM1 对应 `/dev/ttyS5`。PX4 的 MAVLink 实例 0 默认也绑定在 TELEM1 上。`dxl_arm` 与串口 MAVLink 不能共享这个物理串口：双方可能都能打开设备，但 MAVLink 帧与 Dynamixel Protocol 2.0 帧会混在一起，导致 CRC 错误、反馈超时、参数读写失败和不可预测的动作。

因此，启用 `DXL_ARM_ENABLE` 时，`ROMFS/px4fmu_common/init.d/rc.mc_apps` 会执行：

```sh
param set MAV_0_CONFIG 0
```

这会关闭实例 0 的串口 MAVLink，让 `dxl_arm` 独占 TELEM1。该设置在每次启动时生效，不需要人工执行 `param set`。未启用 Dynamixel 驱动时，不会改变实例 0 的配置。

PX4 的串口编号由构建系统生成：TEL1=101、TEL2=102、Ethernet=1000。`rc.mc_apps` 在 `rc.serial` 之前执行，因此 `MAV_0_CONFIG=0` 会在串口 MAVLink 启动前生效；即使参数文件中保存过旧的 101，也会被纠正，而不只是修改出厂默认值。

`MAV_2_CONFIG=1000` 和 `RC_PORT_CONFIG=102` 继续由 `boards/cuav/7-nano/init/rc.board_defaults` 配置，没有关闭或改走 TELEM1。

同一板级文件还明确设置 DDS 默认值，避免新刷固件后依赖手工补参数：

| 参数 | 默认值 | 用途 |
|---|---|---|
| `UXRCE_DDS_CFG` | `1000` | 使用 Ethernet，而不是串口 |
| `UXRCE_DDS_AG_IP` | `170461697` | Agent 地址 `10.41.10.1`，即机载电脑 |
| `UXRCE_DDS_PRT` | `8888` | Micro XRCE-DDS UDP 端口 |
| `UXRCE_DDS_DOM_ID` | `42` | 与 `uav_ws` 的 ROS_DOMAIN_ID 一致 |
| `UXRCE_DDS_SYNCT` | `1` | 保持 PX4 与 ROS 2 时间同步 |

这些值与 `uav_ws/firmware/px4/cuav-7-nano/runtime-configuration.md` 的目标配置一致。机载电脑需运行 `MicroXRCEAgent udp4 -p 8888`，有线网络沿用机载电脑 `10.41.10.1/24`、飞控 `10.41.10.2/24`。**DDS 参数不会自动配置两端的网卡地址**；若实机尚未配置静态网络，仍需按 `uav_ws` 的 `net.cfg.example` 在部署阶段配置飞控和机载电脑。

**注意：这里只关闭 TELEM1 上的串口 MAVLink，不会关闭以太网 MAVLink，也不会关闭 DDS。**

## 保留的通信链路

| 链路 | 当前用途 | 是否占用 TELEM1 |
|---|---|---|
| TELEM1 `/dev/ttyS5` | 三个 Dynamixel 舵机 | 是，机械臂独占 |
| TELEM2 `/dev/ttyS3` | RC 接收机，`RC_PORT_CONFIG=102` | 否 |
| Ethernet MAVLink 实例 2，`MAV_2_CONFIG=1000` | QGC、参数、命令、MAVLink Console | 否 |
| Micro XRCE-DDS，UDP 8888 | ROS 2 高频话题、动捕、策略消息 | 否 |
| VRPN 动捕网络 | 动捕服务器到机载电脑 | 否 |

室内动捕数据链路为：

```text
VRPN 动捕服务器
  → 机载电脑 uav_mocap / fusion
  → /fmu/in/vehicle_visual_odometry
  → Micro XRCE-DDS
  → PX4 EKF2
```

Mac QGC 的链路为：

```text
PX4 Ethernet MAVLink（实例 2）
  → 机载电脑 mavlink-router
  → UDP 14550 到 Mac QGC
  → UDP 14551 到 fusion / policy 参数读取
```

所以关闭实例 0 不会妨碍上述动捕和 QGC 流程。不要把 `MAV_2_CONFIG` 一并关闭，因为 QGC 仍需要 MAVLink。

## 上电后的自动行为

`DXL_ARM_ENABLE` 默认开启，因此不需要执行 `dxl_arm start` 或其他启动命令。

启动流程为：

1. `rc.mc_apps` 自动启动 `dxl_arm`。
2. 驱动打开 TELEM1，请求三个舵机的实际位置。
3. 收到第一个 `vehicle_status`，确认 PX4 整机状态管理已经运行。
4. 三个舵机都返回有效位置后，先把目标写成当前实际位置，再开启力矩。
5. 从实际角度平滑运动到零位。
6. 回中完成后等待 RC 输入。

`vehicle_status` 不包含本功能所需的无人机位置安全判断。当前默认 `DXL_ALLOW_ARMED=true`，所以自动回中并不等同于“只在未解锁或落地后执行”。需要这种限制时，必须另外增加解锁和 landed 检查。

舵机反馈必须同时满足：

- 三个 ID 都回复；
- 每个回复包含 4 字节位置数据；
- 错误码为 0；
- 原始位置在 `0..4095` 范围内；
- 回复 ID 与配置一致。

任何一台无回复、超时、数据不完整或返回错误，都不会开始或继续回中/随机运动。故障后必须重新回中，不能自动续跑随机轨迹。

## 默认舵机和角度契约

| ID | 物理位置 | 策略关节索引 |
|---|---|---|
| 1 | 正前方 | `delta_motor_1_hinge` |
| 2 | 左后方 | `delta_motor_2_hinge` |
| 3 | 右后方 | `delta_motor_3_hinge` |

三个零点均为 2048，方向均为正：

```text
q = (raw - 2048) × 2π / 4096
```

因此：

```text
1707 → -29.9707°，约 -30°
2048 → 0°
3072 → +90°
```

默认命令角度范围与当前训练 XML 一致：

```text
DXL_Q_MIN ≈ -30°
DXL_Q_MAX = +90°
```

上电测试前必须实测确认这些值，不能仅凭模型名称判断零点、方向和机构位置。

## 默认 RC 约定

与同事的硬件操作方式一致，遥控器的 **SC 三档拨杆映射到 CH7**，不再使用 CH8。`DXL_RC_CHANNEL=7`，档位需稳定 150 ms（`DXL_RC_DB_MS`）。

| SC 档位 | CH7 输入范围 | 三个舵机的动作 |
|---|---|---|
| 上档 | `2000±100`，即 1900～2100 | 平滑向上到 `DXL_Q_MIN`，默认约 `-0.523 rad`，对应原始位置约 1707 |
| 中档 | `1503±100`，即 1403～1603 | 平滑回中到 `0 rad`，对应原始位置 2048 |
| 下档 | `999±100`，即 899～1099 | 持续执行有界平滑随机运动 |

具体行为：

- 上档、下档都必须已经完成一次实际回中；外档之间可以直接平滑切换。若回中过程中已经选择外档，会在实际回中完成后进入对应动作。
- 切到上档或中档会停止随机轨迹；保持下档则持续产生随机目标，不是同事实现的正弦往复。
- 不在上述三个窗口内的输入，稳定 150 ms 后按无效档位处理并保持当前位置。
- RC 失联会保持当前位置并清除外档资格；恢复后必须重新回中，不能直接恢复随机运动。
- 停止轨迹不等于关闭舵机力矩。`DXL_ALLOW_ARMED` 默认开启，允许飞行实验中的机械臂动作，可按实验安全策略调整。

`DXL_FB_HZ` 和 `DXL_CMD_HZ` 默认请求 50 Hz，实际频率受串口波特率和总线负载影响，应通过 `dxl_arm_status` 实测，不能仅凭参数认定已经达到。

## 构建与烧录

在有 ARM 工具链的电脑上：

```bash
git fetch origin
git checkout pc2-dev
git pull --ff-only origin pc2-dev
make cuav_7-nano_uav_delta upload
```

`uav_delta` 配置已经包含：

```text
CONFIG_DRIVERS_DXL_ARM=y
CONFIG_LIB_RL_TOOLS=y
CONFIG_MODULES_MC_UAV_DELTA=y
```

当前工作站的验证范围为 POSIX/SITL 构建和单元测试。7 Nano 固件构建、烧录和实机动作仍需在部署电脑上验证。

## 实机检查顺序

1. 断开螺旋桨，确认舵机独立供电、共地和半双工接线正确。
2. 确认 TELEM1 只连接 Dynamixel，不连接串口数传或其他设备。
3. 确认 `MAV_0_CONFIG=0`、`MAV_2_CONFIG=1000`、`RC_PORT_CONFIG=102`。
4. 确认 Mac QGC 能通过 mavlink-router 收到心跳和参数。
5. 确认 `/fmu/in/vehicle_visual_odometry` 持续到达，EKF 的外部视觉位置、海拔和 yaw 融合正常。
6. 将 SC 放在中档，使用 `dxl_arm status` 和 `listener dxl_arm_status` 检查实际反馈、周期、错误以及 `rc_mode=2`，等待实际回中完成。
7. 使用 `listener delta_arm_state` 确认三个弧度值的 ID 顺序、零点和方向。
8. 将 SC 拨到上档，确认 `rc_mode=1`，三个实际位置向上到约 1707；拨回中档，确认 `rc_mode=2` 且实际位置回到 2048 附近。
9. 将 SC 拨到下档，确认 `rc_mode=3` 且三个角度平滑随机变化；拨回中档应停止随机轨迹并回中。调试时也可使用 `dxl_arm raise`、`dxl_arm random` 和 `dxl_arm hold` 检查相同动作。

`dxl_arm stop` 会停止并关闭串口，但不等于舵机力矩已关闭。需要失能时必须有独立的硬件安全措施。

## 当前边界

- 本模块不包含 Delta 几何、FK/IK 或末端 XYZ 控制。
- 当前随机轨迹是关节空间内的有界连续目标，不是工作空间采样。
- 当前没有 landed/未解锁专用的自动回中限制。
- 实机总线时延、实际反馈频率、机械限位和 RC 通道映射必须通过台架测试确认。

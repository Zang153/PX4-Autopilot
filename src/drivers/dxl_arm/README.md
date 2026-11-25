# 7 Nano Dynamixel 机械臂控制与通信配置

本模块用于 UAV+Delta 项目中的三个 Dynamixel 舵机。`dxl_arm` 在飞控本地完成 Protocol 2.0 通信、实际角度反馈、CH7 上档固定随机轨迹播放、中档回中和下档固定向下位置控制，并向 `mc_uav_delta` 提供 `delta_arm_state` 观测。

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
2. 驱动打开 TELEM1，默认目标设为 `DXL_Q_MAX`（默认 +90°，对应 3072）并开启力矩，三个电机直接向下，不进行平滑运动。
3. 有效 CH7 档位可以覆盖上电默认目标：中档直接回中，下档直接写入 `DXL_Q_MAX`；上档直接启动固定随机轨迹，不检查 home 位置或反馈有效性。
4. 按 `DXL_CMD_HZ` 的频率向三个舵机同步写入目标位置，默认 50 Hz。

固定位置和随机轨迹命令都不检查 `vehicle_status`，也没有解锁限制。上档后直接从轨迹首样本开始播放；如果轨迹已经处于播放状态，重复保持上档不会重新开始。

三个舵机的实际位置仍按 `DXL_FB_HZ` 读取并发布到 `delta_arm_state`，反馈只用于观测和诊断，不影响随机播放。随机播放期间 RC 失联或输入离开有效档位只更新 `rc_mode`，不会停止轨迹；中档、下档或 `hold` 命令仍可主动中止播放。

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

## 默认 RC 与固定随机轨迹

遥控器的 **SC 三档拨杆映射到 CH7**，`DXL_RC_CHANNEL=7`，不使用 CH8，也没有档位防抖等待。

| SC 档位 | CH7 输入范围 | 行为 |
|---|---|---|
| 上档 | `2000±100`，即 1900～2100 | 从 home 单次播放固定 60 秒随机轨迹 |
| 中档 | `1503±100`，即 1403～1603 | 中止随机播放并直接写入 `0 rad`，对应 2048 |
| 下档 | `999±100`，即 899～1099 | 中止随机播放并直接写入 `DXL_Q_MAX`，默认 `+90°`，对应 3072，不做平滑 |

固定轨迹由 `Tools/dxl_arm/generate_fixed_trajectory.py` 生成，使用 Gazebo 真实机械臂几何和 Foundation V1 `balanced_v2` 算法。当前冻结参数为 seed=1、速度档 3、归一化速度 0.45，平均末端速度约 0.135 m/s；轨迹由 58 秒随机运动和 2 秒五次多项式制动组成，没有前 4 秒静止段，最终速度和加速度为零。

- 直接按 50 Hz 生成 3001 个三关节位置样本，位置表为 36,012 字节并编译进固件。
- 样本采用 standard 角度顺序，直接对应 ID 1/2/3，不应用 Gazebo SDF 的换序和符号转换。
- 播放器按启动后的绝对时间每 20 ms 选择样本，串口延迟不会累计成轨迹时长漂移。
- 60 秒结束后保持最终位置，不自动循环。离开上档后再拨上即可重新播放，不检查 home，也不因启动条件变化而自动回中。
- `dxl_arm random` 与 CH7 上档执行同一轨迹；`dxl_arm down` 与 CH7 下档一样直接写入 `DXL_Q_MAX`。
- `dxl_arm_status` 的 `trajectory_active` 和 `trajectory_sample` 分别显示播放状态和当前样本，`state=4` 表示随机播放中。

## 为什么当前使用 50 Hz

`DXL_CMD_HZ` 和 `DXL_FB_HZ` 默认均为 50 Hz，`DXL_ARM_BAUD` 保持 57600。50 Hz 每 20 ms 写入一次，按当前协议包长度估算：

```text
一次三电机位置写入约 5.035 ms
一次完整三电机反馈交换约 10.764 ms
合计约 15.799 ms，小于 20 ms
```

理论上每个周期还剩约 4.2 ms，因此当前选择 50 Hz，不提高到 100 Hz。上述数值只是串口字节时间，不包括舵机响应延迟、调度和丢包；实际写入周期、反馈周期和错误计数仍须通过 `dxl_arm status` 与 `listener dxl_arm_status` 实测。

## 轨迹重新生成、构建与烧录

交付的固定表已经生成。需要复现一致性检查时执行：

```bash
PYTHONPATH=Tools/dxl_arm python3 Tools/dxl_arm/generate_fixed_trajectory.py --check
```

只有明确需要改变固定轨迹时，才不带 `--check` 运行生成器覆盖 `dxl_random_trajectory.hpp/.cpp/.json`。轨迹的几何、seed、活跃度、导数极值和校验哈希保存在 `dxl_random_trajectory.json`。

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

为节省内部 Flash，该变体还关闭了固定翼、VTOL 和空速选择模块；RAPTOR、相机、云台和动捕链路保持启用。

实机烧录和动作仍需在部署电脑上验证。

## 实机检查顺序

1. 断开螺旋桨，确认舵机独立供电、共地和半双工接线正确。
2. 确认 TELEM1 只连接 Dynamixel，不连接串口数传或其他设备；上电未操作 CH7 时，确认三个电机目标约为 `1.571 rad` 并向约 3072 运动，上电默认不再是回中。
3. 确认 `MAV_0_CONFIG=0`、`MAV_2_CONFIG=1000`、`RC_PORT_CONFIG=102`。
4. 确认 Mac QGC 能通过 mavlink-router 收到心跳和参数。
5. 确认 `/fmu/in/vehicle_visual_odometry` 持续到达，EKF 的外部视觉位置、海拔和 yaw 融合正常。
6. 将 SC 放在中档，使用 `dxl_arm status` 和 `listener dxl_arm_status` 检查 `rc_mode=2`、`q_target=0`，并观察实际位置是否回到 2048 附近。
7. 使用 `listener delta_arm_state` 确认三个弧度值的 ID 顺序、零点和方向。
8. 将 SC 拨到下档，确认 `rc_mode=3`、目标约为 `1.571 rad`，实际位置向约 3072 运动，不做平滑；拨回中档应直接下发 2048。
9. 将 SC 拨到上档，确认 `rc_mode=1`、`trajectory_active=true`、`trajectory_sample` 持续增加，实际位置跟随轨迹。60 秒后应变为 `trajectory_active=false` 并保持最终目标。
10. 随机播放中短暂离开有效档位或切断 RC，轨迹不应因此停止；恢复后若仍在播放则继续原轨迹，中档或下档仍可主动接管。
11. 需要单独测试固定下位置或上位置时，分别使用 `dxl_arm down`、`dxl_arm raise`；需要中止随机播放时切到中档或下档。调试命令还包括 `dxl_arm center`、`dxl_arm random` 和 `dxl_arm hold`。

`dxl_arm stop` 会停止并关闭串口，但不等于舵机力矩已关闭。需要失能时必须有独立的硬件安全措施。

## 当前边界

- 飞控端只播放编译进固件的一条固定轨迹，不在线构建工作空间、生成新随机轨迹或执行 FK/IK 和末端 XYZ 控制。
- 轨迹生成时的几何和导数检查是理想运动学约束，不是完整杆体碰撞证明，也不是 Dynamixel 硬件额定能力认证。
- 实机总线时延、实际反馈频率、机械限位和 RC 通道映射必须通过断桨台架测试确认。

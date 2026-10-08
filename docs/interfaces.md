# 接口参考

本工作区通过 ROS 2 与 `drone_px4_node` 交互。飞控连接、MAVLink、坐标变换都在该
节点内部完成；这里只使用 `/drone/*`。

## 话题（订阅）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/drone/state` | `drone_interfaces/msg/VehicleState` | 连接/解锁/飞行模式/高度/健康状态 |
| `/drone/telemetry/odometry` | `nav_msgs/msg/Odometry` | 位置、速度、姿态（ENU/FLU） |
| `/drone/telemetry/imu` | `sensor_msgs/msg/Imu` | 原始 IMU |
| `/drone/telemetry/gps/fix` | `sensor_msgs/msg/NavSatFix` | 经纬度、海拔 |
| `/drone/telemetry/battery` | `sensor_msgs/msg/BatteryState` | 电压、电流、剩余电量 |

### VehicleState 字段

```
connected armed in_air landed offboard
failsafe failsafe_known           # failsafe_known=false 时 failsafe 不可信
armable health_known              # armable 反映 PX4 pre_flight_checks_pass
local_position_ok global_position_ok home_position_ok
flight_mode                       # 例如 HOLD / OFFBOARD / LAND / AUTO_TAKEOFF
status                            # 连接状态文本
relative_altitude_m               # 相对起飞点高度（SITL 地面约 0.2 m）
```

## 话题（发布）

`/drone/cmd_vel`（`geometry_msgs/msg/TwistStamped`）。**有 0.3 秒看门狗**：超过
0.3 秒没有新消息，节点会把速度归零。因此必须持续补发。`dctl` 帮你完成这件事。

`header.frame_id` 决定坐标系：

- `base_link`（默认）：机体 FLU。`linear.x` 前(+)/后(-)，`linear.y` 左(+)/右(-)，
  `linear.z` 上(+)/下(-)，`angular.z` 偏航角速度（rad/s，逆时针为正）。
- `odom`：世界 ENU。`x` 东，`y` 北，`z` 上，`angular.z` 偏航角速度。

收到非法值或非法 frame 时节点会记录警告并忽略该帧。

## 服务

| 服务 | 类型 | 语义 |
|---|---|---|
| `/drone/control/arm` | `std_srvs/srv/SetBool` | `data=true` 解锁，`false` 上锁 |
| `/drone/control/takeoff` | `drone_interfaces/srv/Takeoff` | 请求 `altitude_m`（0.5–20），返回 `accepted/message`；**不会隐式解锁** |
| `/drone/control/land` | `std_srvs/srv/Trigger` | 降落并停止速度控制 |
| `/drone/control/offboard` | `std_srvs/srv/SetBool` | `true` 进入/`false` 退出 offboard 速度控制 |
| `/drone/control/reboot` | `std_srvs/srv/Trigger` | 重启飞控；仅实机、且已落地已上锁时可用，SITL 不支持 |

通常不要直接调用这些服务，而是使用 `dctl`，因为它已内置上述前置条件与限速。

## 坐标与单位

- 位置/速度（odometry）：**ENU**，米、米/秒。`x`=东，`y`=北，`z`=上。
- 速度指令（cmd_vel, base_link）：**FLU** 机体，米/秒；偏航角速度 rad/s。
- 姿态：`attitude_deg` 为 ENU/FLU 下的 roll/pitch/yaw，单位度。
- 高度：`relative_altitude_m` 相对起飞点，SITL 静止时约 0.2 m。

## 限速与超时

节点参数（`drone_ws/src/drone_bringup/config/sitl.yaml`）：

| 参数 | 仿真 | 实机 |
|---|---|---|
| `max_speed_m_s` | 3.0 | 1.0 |
| `max_yaw_rate_rad_s` | 1.5 | 0.8 |
| `command_timeout_s` | 0.3 | 0.3 |

`dctl` 启动时通过 `/drone_px4_node/get_parameters` 读取这两个限速并据此夹紧输入，
读取失败时回退到 3.0 / 1.5。

## 错误码

见 `AGENTS.md` 第 5 节。稳定约定：命令失败时 stdout 为 JSON，含 `ok:false`、`code`、
`message`，进程退出码非 0。

# AGENTS.md — 无人机 AI 交互工作区

本目录（`drone_online_with_ai`）是给编码智能体使用的无人机控制工作区。你（AI）通过
`bin/dctl` 命令经 ROS 2 控制本机正在运行的 PX4 飞控（仿真或实机），并读取实时状态。

> 本工作区**只做控制与观测**，不碰 MAVLink/MAVSDK、不改飞控参数、不启动/停止仿真系统。
> 底层连接由 `drone_ws` 里的 `drone_px4_node` 负责，对外稳定接口是 `/drone/*`。

## 0. 每次开始前先做这一步

所有命令都必须在一个已初始化 ROS 2 环境的 shell 里运行。`bin/dctl` 会自动 source
`bin/denv.sh`，所以你**直接调用 `./bin/dctl ...` 即可**，无需手动 source。

第一次务必确认链路可用：

```bash
./bin/dctl status
```

- 返回 JSON 且 `"ok": true` → 链路正常，可以继续。
- `code: STATE_UNAVAILABLE` → `drone_px4_node` 没运行，或 ROS 域不对（仿真应为
  `ROS_DOMAIN_ID=42`）。先停下，向人类报告，不要盲目重试。

## 1. 安全红线（必须遵守）

1. **先解锁再起飞**：`takeoff` 永远不会自动 `arm`，必须显式执行 `dctl arm`。
2. **速度控制必须先进入 offboard**：`dctl offboard start` 成功后才能发 `velocity`。
3. **速度命令必须带 `--duration`**：底层 `cmd_vel` 有 0.3 秒看门狗，`dctl` 会在
   时长内以 20 Hz 补发，并在到点自动归零。不要试图用单条 `ros2 topic pub` 控制速度。
4. **`dctl` 已强制限速**（SITL 为 3 m/s、1.5 rad/s）并做前置状态检查，被拒绝时会
   返回 `code`，不要绕过它去直接调 ROS 服务。
5. **不确定就急停**：`./bin/dctl stop` 会立即把速度归零并退出 offboard。
6. **飞行前确认空域安全**，并且只在人类明确要求时才实际起飞。
7. 实机（非仿真）默认 domain 可能不同（常为 0）。不要擅自切换域去控制实机。

## 2. 命令速查

默认输出为单行 JSON；加 `--human` 输出可读文本。命令失败时进程退出码非 0，并给出
`code` 与 `message`。

| 命令 | 作用 | 关键参数 |
|---|---|---|
| `dctl status` | 一次读取状态+位姿+电池 | — |
| `dctl watch` | 实时流式输出（每行一个 JSON） | `--rate HZ` `--count N` |
| `dctl wait` | 阻塞等待某状态成立 | `--for landed\|in_air\|armed\|...` `--timeout` |
| `dctl arm` / `dctl disarm` | 解锁 / 上锁 | — |
| `dctl takeoff` | 起飞并等待到达高度 | `--alt M` `--timeout` |
| `dctl offboard` | 进入/退出手动速度控制 | `start` / `stop` |
| `dctl velocity` | 持续输出机体速度，到点自动归零 | `--forward` `--left` `--up` `--yaw` `--duration` |
| `dctl hover` | 零速度悬停一段时间 | `--duration` |
| `dctl land` | 降落并等待落地 | `--timeout` |
| `dctl stop` | **急停**：归零并退出 offboard | — |

速度方向（默认 `base_link` 机体 FLU）：`--forward` 前(+)/后(-)，`--left` 左(+)/右(-)，
`--up` 上(+)/下(-)，`--yaw` 偏航角速度 rad/s（逆时针为正）。加 `--frame odom` 则使用
东/北/上世界系。

## 3. 标准飞行流程

```bash
./bin/dctl status                 # 1. 确认 ok:true，landed:true，armable:true
./bin/dctl arm                    # 2. 解锁
./bin/dctl takeoff --alt 2        # 3. 起飞到 2 m（阻塞到到达）
./bin/dctl offboard start         # 4. 进入速度控制
./bin/dctl velocity --forward 1 --duration 2   # 5. 向前 1 m/s 飞 2 秒
./bin/dctl hover --duration 1     # 6. 悬停 1 秒
./bin/dctl stop                   # 7. 停止速度控制
./bin/dctl land                   # 8. 降落（阻塞到落地）
./bin/dctl disarm                 # 9. 上锁
```

任何一步失败或取消，先执行 `./bin/dctl stop`，必要时 `./bin/dctl land`。

## 4. 实时读取

```bash
./bin/dctl watch --rate 5 --count 10   # 采 10 帧状态，每帧一行 JSON
./bin/dctl status                       # 单帧快照
```

`status`/`watch` 的 JSON 结构：

```json
{
  "ok": true,
  "state": {"connected":true,"armed":false,"in_air":false,"landed":true,"offboard":false,
            "flight_mode":"HOLD","relative_altitude_m":0.3, "armable":true, "...":"..."},
  "odometry": {"position_m":{"east":0,"north":0,"up":0},
               "velocity_m_s":{"east":0,"north":0,"up":0},
               "attitude_deg":{"roll":0,"pitch":0,"yaw":0}},
  "battery": {"voltage_v":16.2,"current_a":-1.0,"remaining":1.0},
  "ages_s": {"state":0.02,"odometry":0.02,"battery":0.02}
}
```

判断状态是否可信：看 `ages_s`（应 < 1 秒）；`failsafe_known=false` 表示失效保护状态
当前不可读，不要把 `failsafe=false` 当作“确认无失效”。

## 5. 错误码

| code | 含义与处理 |
|---|---|
| `STATE_UNAVAILABLE` | 状态缺失/过期；检查 `drone_px4_node` 与 ROS 域 |
| `NOT_CONNECTED` | 飞控未连接，等待或报告人类 |
| `NOT_ARMABLE` | 预检未通过（位置/校准/安全开关），不要反复重试 |
| `NOT_ARMED` | 未解锁，先 `arm` |
| `NOT_AIRBORNE` | 未升空，先 `takeoff` |
| `NOT_LANDED` | 不在确认的地面状态，先 `land` |
| `OFFBOARD_INACTIVE` | 未进入速度控制，先 `offboard start` |
| `ALREADY_ARMED` / `ALREADY_DISARMED` | 幂等成功（`ok:true`）：已是目标状态，无需重复 |
| `ALREADY_ACTIVE` / `ALREADY_INACTIVE` | 幂等成功：offboard 已是目标状态 |
| `ALREADY_LANDED` | 幂等成功：已在地面 |
| `ALREADY_AIRBORNE` | 起飞被拒绝：已在空中 |
| `EXECUTION_FAILED` | 飞控拒绝了动作，看 `message`，必要时停止并上报 |
| `SERVICE_UNAVAILABLE` / `TIMEOUT` | 服务不可用/超时，检查链路 |
| `INTERRUPTED` | 命令被中断，速度已归零、offboard 已停止 |

## 6. 更多

- 接口、坐标与单位细节：`docs/interfaces.md`
- 只读自检：`tests/smoke.sh`
- 完整飞行自检（会真实起飞）：`tests/flight_smoke.sh`
- 本工作区只依赖本地 ROS 2 图，不依赖公网 Hub。

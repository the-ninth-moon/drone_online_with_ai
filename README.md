# drone_online_with_ai

给编码智能体（AI）使用的无人机控制工作区。AI 读取 `AGENTS.md` 与
`.agents/skills/drone-flight/SKILL.md` 后，即可通过 `bin/dctl` 经 ROS 2 控制本机
运行的 PX4 飞控（仿真或实机），并实时读取状态。

## 快速开始

```bash
cd ~/drone_online_with_ai

# 1. 确认链路（仿真需先跑好 drone_ws 的 run_sim.sh）
./bin/dctl status

# 2. 标准飞行
./bin/dctl arm
./bin/dctl takeoff --alt 2
./bin/dctl offboard start
./bin/dctl velocity --forward 1 --duration 2
./bin/dctl stop
./bin/dctl land
./bin/dctl disarm
```

## 目录

```
AGENTS.md                      AI 入口：环境、安全红线、命令、标准流程
.agents/skills/drone-flight/   可被 agent 自动发现的 skill
bin/dctl                       核心 CLI（自动初始化 ROS 环境）
bin/denv.sh                    手动 source 用的环境脚本
lib/drone_api.py               rclpy 薄封装，可被 Python 直接 import
docs/interfaces.md             话题/服务/坐标/单位/错误码参考
tests/smoke.sh                 只读自检（安全）
tests/flight_smoke.sh          完整飞行自检（会真实起飞）
```

## 前置条件

- 已运行 PX4 栈：仿真用 `drone_ws/run_sim.sh`（ROS Domain 42），或实机 `run_hardware.sh`。
- `drone_ws` 已 `colcon build`（提供 `drone_interfaces` 与 `drone_px4_node`）。
- 默认目标 ROS 域为 42；可用 `ROS_DOMAIN_ID=<n>` 覆盖（实机常为 0）。

## 安全

`dctl` 是严格接口：强制 `arm → takeoff → offboard → velocity` 顺序、按飞控参数限速、
速度按 `--duration` 自动归零、并提供 `stop` 急停。详见 `AGENTS.md`。

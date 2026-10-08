---
name: drone-flight
description: Fly and monitor the local PX4 drone over ROS 2 using the dctl CLI. Use when the user asks to arm, take off, control velocity, land, read live drone state/telemetry, or run a flight sequence in the drone_online_with_ai workspace.
---

# Drone Flight (dctl)

Control and observe the locally running PX4 vehicle (SITL or hardware) through the
strict `bin/dctl` command-line interface. All commands talk to `/drone/*` ROS 2
interfaces served by `drone_px4_node`.

## When to use

- "起飞 / take off", "arm / 解锁", "降落 / land"
- "向前飞 / 控速 / velocity", "悬停 / hover"
- "无人机现在状态 / 高度 / 电池 / 位置 / 姿态"
- "实时读取 / 持续监控 drone state"
- Any autonomous flight sequence built from the commands below.

## Setup check (always do this first)

Run from the workspace root (`drone_online_with_ai`). `bin/dctl` self-configures
the ROS 2 environment; do not source anything manually.

```bash
./bin/dctl status
```

Proceed only when the output is JSON with `"ok": true` and, before flying,
`state.landed == true` and `state.armable == true`. Otherwise report the failure
(usually `STATE_UNAVAILABLE`: node down or wrong ROS domain).

## Core workflow

```bash
./bin/dctl status                              # confirm ok:true / landed / armable
./bin/dctl arm                                 # explicit arm
./bin/dctl takeoff --alt 2                     # blocks until 2 m reached
./bin/dctl offboard start                      # enable velocity control
./bin/dctl velocity --forward 1 --duration 2   # body forward 1 m/s for 2 s
./bin/dctl hover --duration 1                   # hold still for 1 s
./bin/dctl stop                                # zero velocity, leave offboard
./bin/dctl land                                # blocks until landed
./bin/dctl disarm
```

Read live state:

```bash
./bin/dctl status                    # one JSON snapshot (state+odometry+battery)
./bin/dctl watch --rate 5 --count 10 # stream JSON lines
./bin/dctl wait --for in_air --timeout 30
```

Output is JSON by default. Add `--human` for text.

## Rules

- `takeoff` never arms; always `arm` first.
- `velocity` requires `offboard start` first and a `--duration`; the CLI refreshes
  the setpoint at 20 Hz and auto-zeroes at the end (the node has a 0.3 s watchdog).
- Velocity is clamped to the node's limits and every command checks preconditions;
  a non-zero exit with a `code` means it was refused — do not bypass the CLI.
- On any doubt or failure: `./bin/dctl stop`, then land if airborne.
- Only fly when the user explicitly asks and airspace is confirmed clear.

## Reference

- Full command/error reference: `AGENTS.md`
- Topics/services/units: `docs/interfaces.md`
- Read-only self test: `tests/smoke.sh`; full flight test: `tests/flight_smoke.sh`

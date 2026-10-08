#!/usr/bin/env python3
"""Implementation of the ``dctl`` command-line interface.

Every command enforces preconditions and prints a single structured result.
By default output is JSON; pass ``--human`` for a readable summary.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from drone_api import DroneClient, DroneError

EXIT_OK = 0
EXIT_FAIL = 1

TAKEOFF_MIN_M = 0.5
TAKEOFF_MAX_M = 20.0
TAKEOFF_TOLERANCE_M = 0.3
MAX_STREAM_DURATION_S = 30.0


# --------------------------------------------------------------------- output
def emit(payload: dict, human: bool) -> None:
    if human:
        print(format_human(payload))
    else:
        print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))


def format_human(payload: dict) -> str:
    if "state" in payload and payload.get("action") is None and "ok" in payload:
        snap = payload
    else:
        snap = payload.get("snapshot", {})
    state = snap.get("state")
    if state is None:
        return json.dumps(payload, ensure_ascii=False, indent=2)
    lines = [
        f"connected={state['connected']} armed={state['armed']} in_air={state['in_air']} "
        f"landed={state['landed']} offboard={state['offboard']} mode={state['flight_mode']} "
        f"alt={state['relative_altitude_m']:.2f}m",
    ]
    odom = snap.get("odometry")
    if odom:
        p = odom["position_m"]
        v = odom["velocity_m_s"]
        a = odom["attitude_deg"]
        lines.append(
            f"pos E/N/U={p['east']:+.2f}/{p['north']:+.2f}/{p['up']:+.2f}m "
            f"vel E/N/U={v['east']:+.2f}/{v['north']:+.2f}/{v['up']:+.2f}m/s "
            f"yaw={a['yaw']:+.1f}deg"
        )
    battery = snap.get("battery")
    if battery:
        pct = battery["remaining"]
        pct_str = "n/a" if pct is None else f"{pct * 100:.0f}%"
        lines.append(
            f"battery {battery['voltage_v']:.1f}V {battery['current_a']:.1f}A {pct_str}"
        )
    action = payload.get("action")
    if action:
        lines.insert(0, f"[{action}] ok={payload.get('ok')} {payload.get('message', '')}")
    return "\n".join(lines)


def fail(code: str, message: str, human: bool, action: str | None = None) -> int:
    emit({"ok": False, "action": action, "code": code, "message": message}, human)
    return EXIT_FAIL


# ----------------------------------------------------------- preconditions
def require_connected(client: DroneClient):
    st = client.require_fresh_state()
    if not st.connected:
        raise DroneError("NOT_CONNECTED", "PX4 is not connected")
    return st


def check_takeoff(client: DroneClient):
    st = require_connected(client)
    if st.in_air:
        raise DroneError("ALREADY_AIRBORNE", "vehicle is already airborne")
    if not st.landed:
        raise DroneError("NOT_LANDED", "vehicle is not confirmed landed")
    if not st.armed:
        raise DroneError("NOT_ARMED", "arm explicitly before takeoff")
    return st


def check_offboard_start(client: DroneClient):
    st = require_connected(client)
    if not st.armed:
        raise DroneError("NOT_ARMED", "vehicle is not armed")
    if not st.in_air:
        raise DroneError("NOT_AIRBORNE", "vehicle is not airborne")
    return st


def check_velocity(client: DroneClient):
    st = require_connected(client)
    if not st.armed:
        raise DroneError("NOT_ARMED", "vehicle is not armed")
    if not st.in_air:
        raise DroneError("NOT_AIRBORNE", "vehicle is not airborne")
    if not st.offboard:
        raise DroneError(
            "OFFBOARD_INACTIVE",
            "velocity control is not active; run 'dctl offboard start' first",
        )
    return st


def clamp(value: float, limit: float):
    if value > limit:
        return limit, True
    if value < -limit:
        return -limit, True
    return value, False


# ------------------------------------------------------------------ commands
def cmd_status(client: DroneClient, args) -> int:
    client.refresh(0.6)
    snap = client.snapshot()
    emit(snap, args.human)
    return EXIT_OK


def cmd_watch(client: DroneClient, args) -> int:
    period = 1.0 / max(0.1, args.rate)
    count = 0
    try:
        while args.count is None or count < args.count:
            snap = client.snapshot()
            emit(snap, args.human)
            sys.stdout.flush()
            count += 1
            client.refresh(period)
    except KeyboardInterrupt:
        pass
    return EXIT_OK


def cmd_arm(client: DroneClient, args, arm: bool) -> int:
    client.refresh(0.4)
    st = require_connected(client)
    action = "arm" if arm else "disarm"
    if arm:
        if st.armed and st.landed:
            emit({"ok": True, "action": action, "code": "ALREADY_ARMED",
                  "message": "already armed", "snapshot": client.snapshot()}, args.human)
            return EXIT_OK
        if not st.landed:
            raise DroneError("NOT_LANDED", "vehicle is not on the ground")
        if not st.armable:
            raise DroneError(
                "NOT_ARMABLE",
                "PX4 preflight checks are not ready (position/home/calibration/safety switch)",
            )
    else:
        if not st.armed:
            emit({"ok": True, "action": action, "code": "ALREADY_DISARMED",
                  "message": "already disarmed", "snapshot": client.snapshot()}, args.human)
            return EXIT_OK
        if st.in_air or not st.landed:
            raise DroneError("NOT_LANDED", "cannot disarm while airborne")
    result = client.set_arm(arm)
    if not result["ok"]:
        raise DroneError("EXECUTION_FAILED", result["message"] or f"{action} rejected")
    client.refresh(0.6)
    emit({"ok": True, "action": action, "message": result["message"],
          "snapshot": client.snapshot()}, args.human)
    return EXIT_OK


def cmd_takeoff(client: DroneClient, args) -> int:
    if not TAKEOFF_MIN_M <= args.alt <= TAKEOFF_MAX_M:
        raise DroneError(
            "BAD_ALTITUDE",
            f"altitude must be within [{TAKEOFF_MIN_M}, {TAKEOFF_MAX_M}] m",
        )
    client.refresh(0.4)
    check_takeoff(client)
    result = client.takeoff(args.alt, timeout=args.timeout)
    if not result["ok"]:
        raise DroneError("EXECUTION_FAILED", result["message"] or "takeoff rejected")
    # Block until the requested relative altitude is actually reached.  In SITL
    # the on-ground relative altitude sits around 0.2 m, so require airborne
    # first and then the altitude target with a small tolerance.
    client.wait_for(lambda s: s.in_air, timeout=args.timeout)
    client.wait_for(
        lambda s: s.relative_altitude_m >= args.alt - TAKEOFF_TOLERANCE_M,
        timeout=args.timeout,
    )
    emit({"ok": True, "action": "takeoff", "message": result["message"],
          "target_altitude_m": args.alt, "snapshot": client.snapshot()}, args.human)
    return EXIT_OK


def cmd_offboard(client: DroneClient, args) -> int:
    enabled = args.mode == "start"
    action = f"offboard_{args.mode}"
    client.refresh(0.4)
    st = require_connected(client)
    if enabled:
        if st.offboard:
            emit({"ok": True, "action": action, "code": "ALREADY_ACTIVE",
                  "message": "velocity control already active",
                  "snapshot": client.snapshot()}, args.human)
            return EXIT_OK
        check_offboard_start(client)
    else:
        if not st.offboard:
            client.zero_velocity()
            emit({"ok": True, "action": action, "code": "ALREADY_INACTIVE",
                  "message": "velocity control already inactive",
                  "snapshot": client.snapshot()}, args.human)
            return EXIT_OK
    result = client.set_offboard(enabled)
    if not result["ok"]:
        raise DroneError("EXECUTION_FAILED", result["message"] or "offboard rejected")
    client.refresh(0.6)
    emit({"ok": True, "action": action, "message": result["message"],
          "snapshot": client.snapshot()}, args.human)
    return EXIT_OK


def cmd_velocity(client: DroneClient, args) -> int:
    duration = args.duration
    if duration <= 0 or duration > MAX_STREAM_DURATION_S:
        raise DroneError(
            "BAD_DURATION",
            f"duration must be within (0, {MAX_STREAM_DURATION_S:.0f}] s",
        )
    client.refresh(0.4)
    check_velocity(client)

    forward, cx = clamp(args.forward, client.max_speed)
    left, cy = clamp(args.left, client.max_speed)
    up, cz = clamp(args.up, client.max_speed)
    yaw, cyaw = clamp(args.yaw, client.max_yaw_rate)
    clamped = cx or cy or cz or cyaw

    frame = args.frame
    client.stream_velocity(forward, left, up, yaw, duration, frame=frame)
    emit({
        "ok": True,
        "action": "velocity",
        "message": "streamed and zeroed",
        "frame": frame,
        "commanded": {"forward": forward, "left": left, "up": up, "yaw": yaw},
        "requested": {"forward": args.forward, "left": args.left, "up": args.up, "yaw": args.yaw},
        "clamped": clamped,
        "duration_s": duration,
        "snapshot": client.snapshot(),
    }, args.human)
    return EXIT_OK


def cmd_hover(client: DroneClient, args) -> int:
    return cmd_velocity(client, argparse.Namespace(
        forward=0.0, left=0.0, up=0.0, yaw=0.0,
        duration=args.duration, frame="base_link", human=args.human,
    ))


def cmd_land(client: DroneClient, args) -> int:
    client.refresh(0.4)
    st = require_connected(client)
    if st.landed and not st.in_air:
        emit({"ok": True, "action": "land", "code": "ALREADY_LANDED",
              "message": "already landed", "snapshot": client.snapshot()}, args.human)
        return EXIT_OK
    result = client.land(timeout=args.timeout)
    if not result["ok"]:
        raise DroneError("EXECUTION_FAILED", result["message"] or "land rejected")
    client.wait_for(lambda s: s.landed and not s.in_air, timeout=args.timeout)
    emit({"ok": True, "action": "land", "message": result["message"],
          "snapshot": client.snapshot()}, args.human)
    return EXIT_OK


def cmd_stop(client: DroneClient, args) -> int:
    client.stop()
    client.refresh(0.4)
    emit({"ok": True, "action": "stop", "message": "velocity zeroed and offboard stopped",
          "snapshot": client.snapshot()}, args.human)
    return EXIT_OK


WAIT_TARGETS = {
    "connected": lambda s: s.connected,
    "armed": lambda s: s.armed,
    "disarmed": lambda s: not s.armed,
    "in_air": lambda s: s.in_air,
    "landed": lambda s: s.landed,
    "offboard": lambda s: s.offboard,
    "offboard_off": lambda s: not s.offboard,
}


def cmd_wait(client: DroneClient, args) -> int:
    if args.target == "altitude":
        if args.value is None:
            raise DroneError("BAD_ARGUMENT", "--value is required with --for altitude")
        target = args.value
        predicate = lambda s: s.relative_altitude_m >= target  # noqa: E731
    else:
        predicate = WAIT_TARGETS[args.target]
    client.wait_for(predicate, timeout=args.timeout)
    emit({"ok": True, "action": "wait", "target": args.target, "value": args.value,
          "snapshot": client.snapshot()}, args.human)
    return EXIT_OK


# --------------------------------------------------------------------- parser
def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--human", action="store_true", help="human-readable output instead of JSON")

    parser = argparse.ArgumentParser(
        prog="dctl",
        description="Strict ROS 2 control interface for the local drone_px4_node.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status", parents=[common], help="one-shot state+odometry+battery snapshot")

    p_watch = sub.add_parser("watch", parents=[common], help="stream snapshots (JSON lines)")
    p_watch.add_argument("--rate", type=float, default=5.0, help="samples per second (default 5)")
    p_watch.add_argument("--count", type=int, default=None, help="stop after N samples")

    sub.add_parser("arm", parents=[common], help="arm the vehicle (must be on the ground)")
    sub.add_parser("disarm", parents=[common], help="disarm the vehicle (must be landed)")

    p_takeoff = sub.add_parser("takeoff", parents=[common], help="arm-checked takeoff to an altitude")
    p_takeoff.add_argument("--alt", type=float, required=True, help="target relative altitude (m)")
    p_takeoff.add_argument("--timeout", type=float, default=30.0, help="seconds to wait for climb")

    p_off = sub.add_parser("offboard", parents=[common], help="enter/leave offboard velocity control")
    p_off.add_argument("mode", choices=["start", "stop"])

    p_vel = sub.add_parser("velocity", parents=[common], help="stream a body velocity for a duration")
    p_vel.add_argument("--forward", type=float, default=0.0, help="body forward m/s (+ = nose)")
    p_vel.add_argument("--left", type=float, default=0.0, help="body left m/s (+ = left)")
    p_vel.add_argument("--up", type=float, default=0.0, help="body up m/s (+ = up)")
    p_vel.add_argument("--yaw", type=float, default=0.0, help="yaw rate rad/s (+ = CCW)")
    p_vel.add_argument("--duration", type=float, required=True, help="seconds to hold the velocity")
    p_vel.add_argument("--frame", choices=["base_link", "odom"], default="base_link",
                       help="base_link = body FLU (default); odom = ENU world")

    p_hover = sub.add_parser("hover", parents=[common], help="hold zero body velocity for a duration")
    p_hover.add_argument("--duration", type=float, required=True, help="seconds to hover")

    p_land = sub.add_parser("land", parents=[common], help="land and wait until on the ground")
    p_land.add_argument("--timeout", type=float, default=60.0, help="seconds to wait for landing")

    sub.add_parser("stop", parents=[common], help="EMERGENCY: zero velocity and leave offboard")

    p_wait = sub.add_parser("wait", parents=[common], help="block until a state condition holds")
    p_wait.add_argument("--for", dest="target", required=True,
                        choices=["connected", "armed", "disarmed", "in_air", "landed",
                                 "offboard", "offboard_off", "altitude"])
    p_wait.add_argument("--value", type=float, default=None, help="altitude target (m)")
    p_wait.add_argument("--timeout", type=float, default=30.0)

    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    client = DroneClient()
    try:
        client.load_limits()
        client.refresh(0.4)
        command = args.command
        if command == "status":
            return cmd_status(client, args)
        if command == "watch":
            return cmd_watch(client, args)
        if command == "arm":
            return cmd_arm(client, args, arm=True)
        if command == "disarm":
            return cmd_arm(client, args, arm=False)
        if command == "takeoff":
            return cmd_takeoff(client, args)
        if command == "offboard":
            return cmd_offboard(client, args)
        if command == "velocity":
            return cmd_velocity(client, args)
        if command == "hover":
            return cmd_hover(client, args)
        if command == "land":
            return cmd_land(client, args)
        if command == "stop":
            return cmd_stop(client, args)
        if command == "wait":
            return cmd_wait(client, args)
        return fail("BAD_COMMAND", f"unknown command: {command}", getattr(args, "human", False))
    except DroneError as exc:
        return fail(exc.code, exc.message, getattr(args, "human", False), args.command)
    except KeyboardInterrupt:
        # Safety first: never leave a stale velocity command behind.
        try:
            client.stop()
        finally:
            emit({"ok": False, "action": args.command, "code": "INTERRUPTED",
                  "message": "interrupted; velocity zeroed and offboard stopped"},
                 getattr(args, "human", False))
        return EXIT_FAIL
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())

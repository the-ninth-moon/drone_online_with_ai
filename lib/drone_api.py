"""Thin rclpy client for the drone_online_with_ai workspace.

This module only talks to the local ``drone_px4_node`` over ROS 2.  It does
not know anything about MAVLink, MAVSDK, connection URIs or PX4 internals;
those stay behind the ``/drone/*`` ROS 2 contract.

Coordinate conventions (do not change without updating docs/interfaces.md):

* ``/drone/telemetry/odometry`` pose/twist are ENU: x=East, y=North, z=Up.
* ``/drone/cmd_vel`` with ``frame_id: base_link`` is FLU body:
  x=forward, y=left, z=up, ``angular.z`` = yaw rate (rad/s, CCW positive).
* ``/drone/cmd_vel`` with ``frame_id: odom`` is ENU: x=East, y=North, z=Up.
"""

from __future__ import annotations

import math
import time
from typing import Callable, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool, Trigger

from drone_interfaces.msg import VehicleState
from drone_interfaces.srv import Takeoff


# Fallback limits if /drone_px4_node parameters cannot be read.
DEFAULT_MAX_SPEED_M_S = 3.0
DEFAULT_MAX_YAW_RATE_RAD_S = 1.5

# The node zeroes any command whose age exceeds command_timeout_s (0.3 s).
STREAM_RATE_HZ = 20.0


class DroneError(RuntimeError):
    """A refusal or failure with a stable machine-readable ``code``."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def quaternion_to_euler_deg(x: float, y: float, z: float, w: float):
    """Return (roll, pitch, yaw) in degrees from an ENU/FLU quaternion."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class DroneClient:
    """Synchronous facade over the ``/drone/*`` interfaces."""

    def __init__(self, node_name: str = "dctl") -> None:
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)

        self._state: Optional[VehicleState] = None
        self._state_at = 0.0
        self._odom: Optional[Odometry] = None
        self._odom_at = 0.0
        self._battery: Optional[BatteryState] = None
        self._battery_at = 0.0

        self.node.create_subscription(VehicleState, "/drone/state", self._on_state, 10)
        self.node.create_subscription(Odometry, "/drone/telemetry/odometry", self._on_odom, 10)
        self.node.create_subscription(BatteryState, "/drone/telemetry/battery", self._on_battery, 10)

        self._cmd_pub = self.node.create_publisher(TwistStamped, "/drone/cmd_vel", 10)
        self._arm_cli = self.node.create_client(SetBool, "/drone/control/arm")
        self._takeoff_cli = self.node.create_client(Takeoff, "/drone/control/takeoff")
        self._land_cli = self.node.create_client(Trigger, "/drone/control/land")
        self._offboard_cli = self.node.create_client(SetBool, "/drone/control/offboard")

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)

        self._max_speed = DEFAULT_MAX_SPEED_M_S
        self._max_yaw_rate = DEFAULT_MAX_YAW_RATE_RAD_S

    # ------------------------------------------------------------------ spin
    def spin(self, seconds: float) -> None:
        """Process callbacks for up to ``seconds`` wall-clock time."""
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self._executor.spin_once(timeout_sec=min(0.05, max(0.0, remaining)))

    def refresh(self, seconds: float = 0.5) -> None:
        self.spin(seconds)

    # -------------------------------------------------------------- callbacks
    def _on_state(self, msg: VehicleState) -> None:
        self._state = msg
        self._state_at = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_at = time.monotonic()

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery = msg
        self._battery_at = time.monotonic()

    # ------------------------------------------------------------------ state
    @property
    def state(self) -> Optional[VehicleState]:
        return self._state

    def state_age(self) -> Optional[float]:
        return None if self._state is None else time.monotonic() - self._state_at

    def odom_age(self) -> Optional[float]:
        return None if self._odom is None else time.monotonic() - self._odom_at

    def battery_age(self) -> Optional[float]:
        return None if self._battery is None else time.monotonic() - self._battery_at

    def require_fresh_state(self, max_age: float = 1.0) -> VehicleState:
        age = self.state_age()
        if self._state is None or age is None or age > max_age:
            raise DroneError(
                "STATE_UNAVAILABLE",
                "vehicle state is unavailable or stale; is drone_px4_node running "
                "and is ROS_DOMAIN_ID correct?",
            )
        return self._state

    # ---------------------------------------------------------------- limits
    def load_limits(self, timeout: float = 2.0) -> None:
        """Best-effort read of the node's clipping limits; fall back silently."""
        try:
            from rcl_interfaces.srv import GetParameters

            client = self.node.create_client(GetParameters, "/drone_px4_node/get_parameters")
            req = GetParameters.Request()
            req.names = ["max_speed_m_s", "max_yaw_rate_rad_s"]
            resp = self._call(client, req, timeout, "get_parameters")
            values = list(resp.values)
            if len(values) >= 2:
                speed = float(values[0].double_value)
                yaw = float(values[1].double_value)
                if speed > 0.0:
                    self._max_speed = speed
                if yaw > 0.0:
                    self._max_yaw_rate = yaw
        except Exception:
            # Keep the conservative defaults; never fail a command over this.
            pass

    @property
    def max_speed(self) -> float:
        return self._max_speed

    @property
    def max_yaw_rate(self) -> float:
        return self._max_yaw_rate

    # -------------------------------------------------------------- snapshot
    def snapshot(self) -> dict:
        st = self._state
        odom = self._odom
        battery = self._battery

        state = None
        if st is not None:
            state = {
                "connected": bool(st.connected),
                "armed": bool(st.armed),
                "in_air": bool(st.in_air),
                "landed": bool(st.landed),
                "offboard": bool(st.offboard),
                "failsafe": bool(st.failsafe),
                "failsafe_known": bool(st.failsafe_known),
                "armable": bool(st.armable),
                "health_known": bool(st.health_known),
                "local_position_ok": bool(st.local_position_ok),
                "global_position_ok": bool(st.global_position_ok),
                "home_position_ok": bool(st.home_position_ok),
                "flight_mode": str(st.flight_mode),
                "status": str(st.status),
                "relative_altitude_m": float(st.relative_altitude_m),
            }

        odometry = None
        if odom is not None:
            p = odom.pose.pose.position
            q = odom.pose.pose.orientation
            t = odom.twist.twist.linear
            roll, pitch, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
            odometry = {
                "frame_id": str(odom.header.frame_id),
                "child_frame_id": str(odom.child_frame_id),
                "position_m": {"east": p.x, "north": p.y, "up": p.z},
                "velocity_m_s": {"east": t.x, "north": t.y, "up": t.z},
                "attitude_deg": {"roll": roll, "pitch": pitch, "yaw": yaw},
            }

        battery_out = None
        if battery is not None:
            battery_out = {
                "voltage_v": float(battery.voltage),
                "current_a": float(battery.current),
                "remaining": None if math.isnan(battery.percentage) else float(battery.percentage),
            }

        def age(at: float, present: bool):
            return None if not present else round(time.monotonic() - at, 3)

        return {
            "ok": bool(state and state["connected"]),
            "state": state,
            "odometry": odometry,
            "battery": battery_out,
            "ages_s": {
                "state": age(self._state_at, st is not None),
                "odometry": age(self._odom_at, odom is not None),
                "battery": age(self._battery_at, battery is not None),
            },
        }

    # -------------------------------------------------------------- services
    def _call(self, client, request, timeout: float, label: str):
        deadline = time.monotonic() + timeout
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise DroneError(
                    "SERVICE_UNAVAILABLE",
                    f"{label} service is not available; is drone_px4_node running?",
                )
            self.spin(0.05)
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                raise DroneError("TIMEOUT", f"{label} call timed out after {timeout}s")
            self.spin(0.05)
        return future.result()

    def set_arm(self, arm: bool, timeout: float = 5.0) -> dict:
        req = SetBool.Request()
        req.data = bool(arm)
        resp = self._call(self._arm_cli, req, timeout, "arm")
        return {"ok": bool(resp.success), "message": str(resp.message)}

    def takeoff(self, altitude_m: float, timeout: float = 8.0) -> dict:
        req = Takeoff.Request()
        req.altitude_m = float(altitude_m)
        resp = self._call(self._takeoff_cli, req, timeout, "takeoff")
        return {"ok": bool(resp.accepted), "message": str(resp.message)}

    def land(self, timeout: float = 8.0) -> dict:
        resp = self._call(self._land_cli, Trigger.Request(), timeout, "land")
        return {"ok": bool(resp.success), "message": str(resp.message)}

    def set_offboard(self, enabled: bool, timeout: float = 5.0) -> dict:
        req = SetBool.Request()
        req.data = bool(enabled)
        resp = self._call(self._offboard_cli, req, timeout, "offboard")
        return {"ok": bool(resp.success), "message": str(resp.message)}

    # -------------------------------------------------------------- velocity
    def _twist(self, forward: float, left: float, up: float, yaw: float, frame: str) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.twist.linear.x = float(forward)
        msg.twist.linear.y = float(left)
        msg.twist.linear.z = float(up)
        msg.twist.angular.z = float(yaw)
        return msg

    def zero_velocity(self, frame: str = "base_link", repeats: int = 5) -> None:
        for _ in range(repeats):
            self._cmd_pub.publish(self._twist(0.0, 0.0, 0.0, 0.0, frame))
            self.spin(1.0 / STREAM_RATE_HZ)

    def stream_velocity(
        self,
        forward: float,
        left: float,
        up: float,
        yaw: float,
        duration: float,
        frame: str = "base_link",
        stop_at_end: bool = True,
        progress: Optional[Callable[[float], None]] = None,
    ) -> None:
        """Publish a constant body velocity at ~20 Hz for ``duration`` seconds.

        The node zeroes stale commands after 0.3 s, so a single message is not
        enough: the stream must be refreshed for the whole duration.
        """
        period = 1.0 / STREAM_RATE_HZ
        deadline = time.monotonic() + duration
        start = time.monotonic()
        try:
            while True:
                self._cmd_pub.publish(self._twist(forward, left, up, yaw, frame))
                if progress is not None:
                    progress(time.monotonic() - start)
                now = time.monotonic()
                if now >= deadline:
                    break
                self.spin(min(period, deadline - now))
        finally:
            if stop_at_end:
                self.zero_velocity(frame=frame)

    def stop(self) -> None:
        """Emergency stop: zero the command stream and leave offboard."""
        self.zero_velocity()
        try:
            st = self._state
            if st is not None and st.offboard:
                self.set_offboard(False)
        except DroneError:
            pass

    # ------------------------------------------------------------------ wait
    def wait_for(self, predicate: Callable[[VehicleState], bool], timeout: float) -> VehicleState:
        deadline = time.monotonic() + timeout
        while True:
            self.refresh(0.1)
            st = self._state
            if st is not None and predicate(st):
                return st
            if time.monotonic() >= deadline:
                raise DroneError("TIMEOUT", f"condition not met within {timeout}s")

    def close(self) -> None:
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

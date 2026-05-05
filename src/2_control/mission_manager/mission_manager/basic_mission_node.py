#!/usr/bin/env python3

from enum import Enum
from math import sqrt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


# ---------------------------------------------------------------------------
# State definition
# ---------------------------------------------------------------------------

class MissionState(Enum):
    IDLE           = 0
    PREFLIGHT      = 1
    START_OFFBOARD = 2
    TAKEOFF        = 3
    WAYPOINTS      = 4
    LAND           = 5
    COMPLETE       = 6
    EMERGENCY      = 7


# ---------------------------------------------------------------------------
# Timeouts (seconds) — tune for your hardware / sim speed
# ---------------------------------------------------------------------------

STATE_TIMEOUTS = {
    MissionState.PREFLIGHT:      60.0,   # wait for MAVROS + AirSim connection
    MissionState.START_OFFBOARD: 15.0,   # wait for ARM + OFFBOARD ack
    MissionState.TAKEOFF:        20.0,   # time to reach takeoff altitude
    MissionState.WAYPOINTS:     180.0,   # total mission budget
    MissionState.LAND:           30.0,   # time to touch down
}


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class BasicMissionNode(Node):
    """
    AeroSAR mission manager — MAVROS backend.

    Drop-in replacement for the px4_msgs version.
    Same state machine, same abort API, same upgrade path.
    """

    def __init__(self):
        super().__init__("basic_mission_node")

        # QoS for MAVROS topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subscribers ---
        self.create_subscription(
            State,
            "/mavros/state",
            self._on_mavros_state,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_local_pose,
            qos,
        )

        # --- Publisher ---
        # MAVROS requires continuous setpoint stream before OFFBOARD is accepted
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10,
        )

        # --- Service clients ---
        self.arming_client   = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode,     "/mavros/set_mode")
        self.land_client     = self.create_client(CommandTOL,  "/mavros/cmd/land")

        # --- Vehicle state ---
        self.mavros_state: State | None = None
        self.local_pose: PoseStamped | None = None

        # --- Mission state machine ---
        self.state = MissionState.IDLE
        self.previous_state = None
        self._state_entry_time: float = self._now()

        # --- Mission parameters ---
        self.timer_period_s             = 0.1  # 10 Hz — MAVROS needs fast setpoints
        self.takeoff_altitude           = 10.0  # ENU: positive = up
        self.position_acceptance_radius = 0.6  # metres

        # ENU waypoints: x=East, y=North, z=Up
        self.waypoints = [
            [0.0, 0.0, 3.0],
            [5.0, 0.0, 3.0],
            [5.0, 5.0, 3.0],
            [0.0, 5.0, 3.0],
            [0.0, 0.0, 3.0],
        ]
        self.current_waypoint_index = 0

        # Active setpoint sent every tick
        self.active_setpoint = [0.0, 0.0, self.takeoff_altitude]

        # One-shot flags
        self._offboard_arm_sent = False
        self._land_sent         = False

        # Throttled logging
        self._last_log_time: dict[MissionState, float] = {}

        self.timer = self.create_timer(self.timer_period_s, self._timer_callback)
        self.get_logger().info("AeroSAR basic mission node started (MAVROS backend).")

    # -----------------------------------------------------------------------
    # Subscribers
    # -----------------------------------------------------------------------

    def _on_mavros_state(self, msg: State):
        self.mavros_state = msg

    def _on_local_pose(self, msg: PoseStamped):
        self.local_pose = msg

    # -----------------------------------------------------------------------
    # Main loop
    # -----------------------------------------------------------------------

    def _timer_callback(self):
        # MAVROS requires continuous setpoint stream to enter/stay in OFFBOARD
        if self.state not in (MissionState.COMPLETE, MissionState.EMERGENCY):
            self._publish_setpoint(self.active_setpoint)

        self._check_timeout()

        match self.state:
            case MissionState.IDLE:           self._handle_idle()
            case MissionState.PREFLIGHT:      self._handle_preflight()
            case MissionState.START_OFFBOARD: self._handle_start_offboard()
            case MissionState.TAKEOFF:        self._handle_takeoff()
            case MissionState.WAYPOINTS:      self._handle_waypoints()
            case MissionState.LAND:           self._handle_land()
            case MissionState.COMPLETE:       self._handle_complete()
            case MissionState.EMERGENCY:      self._handle_emergency()

    # -----------------------------------------------------------------------
    # State handlers
    # -----------------------------------------------------------------------

    def _handle_idle(self):
        self._transition_to(MissionState.PREFLIGHT)

    def _handle_preflight(self):
        if self.mavros_state is None or self.local_pose is None:
            self._log_throttled("Waiting for MAVROS state and position...", interval=2.0)
            return

        if not self.mavros_state.connected:
            self._log_throttled("Waiting for MAVROS connection to PX4...", interval=2.0)
            return

        # Wait 3 s after connection is confirmed
        if self._time_in_state() < 3.0:
            return

        self.get_logger().info(
            f"MAVROS connected. mode={self.mavros_state.mode} "
            f"armed={self.mavros_state.armed}"
        )
        self._transition_to(MissionState.START_OFFBOARD)

    def _handle_start_offboard(self):
        self.active_setpoint = [0.0, 0.0, self.takeoff_altitude]

        if not self._offboard_arm_sent:
            # Stream setpoints for 1 s before requesting OFFBOARD
            if self._time_in_state() < 1.0:
                return
            self._set_offboard_mode()
            self._arm()
            self._offboard_arm_sent = True
            self.get_logger().info("Sent OFFBOARD mode + ARM requests.")

        if self.mavros_state is not None:
            self._log_throttled(
                f"Waiting — mode={self.mavros_state.mode} "
                f"armed={self.mavros_state.armed}",
                interval=2.0,
            )
            if self.mavros_state.armed and self.mavros_state.mode == "OFFBOARD":
                self.get_logger().info("Armed and OFFBOARD confirmed.")
                self._transition_to(MissionState.TAKEOFF)
            elif self._time_in_state() > 8.0:
                # Retry in case first attempt was too early
                self._set_offboard_mode()
                self._arm()

    def _handle_takeoff(self):
        self.active_setpoint = [0.0, 0.0, self.takeoff_altitude]

        if self.local_pose is None:
            return

        z = self.local_pose.pose.position.z
        self._log_throttled(
            f"Taking off — z={z:.2f} m (target {self.takeoff_altitude})", interval=1.0
        )

        if z > self.takeoff_altitude - 0.5:
            self.get_logger().info(f"Takeoff altitude reached (z={z:.2f}).")
            self.current_waypoint_index = 0
            self._transition_to(MissionState.WAYPOINTS)

    def _handle_waypoints(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed.")
            self._transition_to(MissionState.LAND)
            return

        target = self.waypoints[self.current_waypoint_index]
        self.active_setpoint = target
        dist = self._distance_to(target)

        self._log_throttled(
            f"WP {self.current_waypoint_index + 1}/{len(self.waypoints)} "
            f"target={target}  dist={dist:.2f} m",
            interval=1.0,
        )

        if dist < self.position_acceptance_radius:
            self.get_logger().info(
                f"Reached WP {self.current_waypoint_index + 1}: {target}"
            )
            self.current_waypoint_index += 1

    def _handle_land(self):
        if not self._land_sent:
            self._land()
            self._land_sent = True
            self.get_logger().info("Land command sent.")

        if self.local_pose is not None:
            z = self.local_pose.pose.position.z
            disarmed = (
                self.mavros_state is not None
                and not self.mavros_state.armed
            )
            if z < 0.15 or disarmed:
                self.get_logger().info("Touchdown confirmed. Mission complete.")
                self._transition_to(MissionState.COMPLETE)

    def _handle_complete(self):
        self._log_throttled("Mission complete. Stop node with Ctrl+C.", interval=5.0)

    # -----------------------------------------------------------------------
    # EMERGENCY handler
    # -----------------------------------------------------------------------

    def _handle_emergency(self):
        if not self._land_sent:
            self._land()
            self._land_sent = True

        # Re-send land every 3 s in case it was missed
        if self._time_in_state() % 3.0 < self.timer_period_s:
            self._land()

        self._log_throttled("EMERGENCY — landing in progress...", interval=2.0)

        if self.local_pose is not None:
            if self.local_pose.pose.position.z < 0.15:
                self._log_throttled("EMERGENCY landing complete.", interval=5.0)

    # -----------------------------------------------------------------------
    # Public abort API
    # -----------------------------------------------------------------------

    def abort(self, reason: str = "unspecified"):
        """
        Trigger emergency landing from anywhere.
        Future use: call from perception node when obstacle detected.

        Example:
            node.abort("obstacle detected")
        """
        self.get_logger().error(f"ABORT triggered: {reason}")
        self._land_sent = False
        self._transition_to(MissionState.EMERGENCY)

    # -----------------------------------------------------------------------
    # MAVROS service calls
    # -----------------------------------------------------------------------

    def _set_offboard_mode(self):
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn("set_mode service not ready yet.")
            return
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.set_mode_client.call_async(req)

    def _arm(self):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn("arming service not ready yet.")
            return
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)

    def _land(self):
        if not self.land_client.service_is_ready():
            self.get_logger().warn("land service not ready yet.")
            return
        req = CommandTOL.Request()
        req.altitude  = 0.0
        req.latitude  = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw       = 0.0
        self.land_client.call_async(req)

    # -----------------------------------------------------------------------
    # Publisher
    # -----------------------------------------------------------------------

    def _publish_setpoint(self, position: list[float]):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = 1.0  # facing East, yaw = 0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        self.setpoint_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Time helpers
    # -----------------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _time_in_state(self) -> float:
        return self._now() - self._state_entry_time

    def _check_timeout(self):
        timeout = STATE_TIMEOUTS.get(self.state)
        if timeout is None:
            return
        elapsed = self._time_in_state()
        if elapsed > timeout:
            self.abort(
                f"timeout in {self.state.name} after {elapsed:.1f}s "
                f"(limit {timeout}s)"
            )

    def _log_throttled(self, msg: str, interval: float = 2.0):
        now  = self._now()
        last = self._last_log_time.get(self.state, 0.0)
        if now - last >= interval:
            self.get_logger().info(msg)
            self._last_log_time[self.state] = now

    # -----------------------------------------------------------------------
    # State transitions
    # -----------------------------------------------------------------------

    def _transition_to(self, new_state: MissionState):
        if new_state == self.state:
            return
        self.get_logger().info(
            f"[{self._time_in_state():.1f}s in state] "
            f"{self.state.name} → {new_state.name}"
        )
        self.previous_state    = self.state
        self.state             = new_state
        self._state_entry_time = self._now()

    # -----------------------------------------------------------------------
    # Geometry
    # -----------------------------------------------------------------------

    def _distance_to(self, target: list[float]) -> float:
        if self.local_pose is None:
            return float("inf")
        dx = target[0] - self.local_pose.pose.position.x
        dy = target[1] - self.local_pose.pose.position.y
        dz = target[2] - self.local_pose.pose.position.z
        return sqrt(dx * dx + dy * dy + dz * dz)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BasicMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
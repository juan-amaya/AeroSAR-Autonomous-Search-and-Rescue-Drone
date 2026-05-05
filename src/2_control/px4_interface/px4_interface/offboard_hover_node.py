import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class OffboardHoverNode(Node):
    """
    MAVROS-based offboard hover test:
    - stream position setpoints at 10 Hz
    - switch to OFFBOARD mode
    - arm
    - hover at 3 m for 10 s
    - land
    """

    def __init__(self):
        super().__init__("offboard_hover_node")

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            qos_sub,
        )
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_pos_callback,
            qos_sub,
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10,
        )

        # ── Service clients ───────────────────────────────────────────────────
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

        # ── State ─────────────────────────────────────────────────────────────
        self.current_state = State()
        self.current_pose = PoseStamped()

        self.target_altitude_m = 3.0
        self.hover_duration_s = 10.0
        self.timer_period_s = 0.1   # 10 Hz — MAVROS requires >2 Hz

        self.counter = 0
        self.offboard_started = False
        self.land_sent = False

        # Build the setpoint message once
        self.setpoint = PoseStamped()
        self.setpoint.header.frame_id = "map"
        self.setpoint.pose.position.x = 0.0
        self.setpoint.pose.position.y = 0.0
        self.setpoint.pose.position.z = self.target_altitude_m
        self.setpoint.pose.orientation.w = 1.0  # facing forward

        self.timer = self.create_timer(self.timer_period_s, self.timer_callback)
        self.get_logger().info("Offboard hover node started (MAVROS).")
        self.get_logger().info(
            f"Target: takeoff to {self.target_altitude_m} m, "
            f"hover {self.hover_duration_s} s, then land."
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def state_callback(self, msg: State):
        self.current_state = msg

    def local_pos_callback(self, msg: PoseStamped):
        self.current_pose = msg

    # ── Main loop ─────────────────────────────────────────────────────────────

    def timer_callback(self):
        # Always publish setpoint — MAVROS requires continuous stream
        self.setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.setpoint)

        if not self.current_state.connected:
            return  # wait for FCU connection

        # After 20 cycles (~2 s) engage offboard and arm
        if self.counter == 20 and not self.offboard_started:
            self.set_offboard_mode()
            self.arm()
            self.offboard_started = True
            self.get_logger().info("OFFBOARD mode + ARM requested.")

        # After hover duration, land
        hover_ticks = int(self.hover_duration_s / self.timer_period_s)
        if (
            self.offboard_started
            and self.counter >= 20 + hover_ticks
            and not self.land_sent
        ):
            self.send_land()
            self.land_sent = True
            self.get_logger().info("Hover complete. Land command sent.")

        if self.counter < 100_000:
            self.counter += 1

    # ── Service calls ─────────────────────────────────────────────────────────

    def set_offboard_mode(self):
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"SetMode result: {f.result().mode_sent}"
                if f.result()
                else "SetMode call failed"
            )
        )

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Arming result: {f.result().success}"
                if f.result()
                else "Arming call failed"
            )
        )

    def send_land(self):
        req = CommandTOL.Request()
        req.altitude = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        future = self.land_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Land result: {f.result().success}"
                if f.result()
                else "Land call failed"
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardHoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
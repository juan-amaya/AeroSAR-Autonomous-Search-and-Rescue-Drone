import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

class Waypoint:
    def __init__(self, x: float, y: float, z: float, yaw: float = 0.0):
        """
        Define a data class for a single waypoint in the PX4 Frame
        Args:
        x, y, z  — NED metres (z is negative for altitude above ground)
        yaw      — radians, 0 = North
        
        """
        
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
 
    def __repr__(self):
        return f"WP(x={self.x:.1f}, y={self.y:.1f}, z={self.z:.1f}, yaw={math.degrees(self.yaw):.0f}°)"
    
class WaypointSequencerNode(Node):
    """
    Flies a hardcoded list of waypoints in order using PX4 offboard position control.
 
    Flow:
        1. Stream warm up for PRE_ARM_CYCLES
        2. Engage offboard mode + arm
        3. Fly to each waypoint
        4. Advance to next waypoint when within ACCEPTANCE_RADIUS_M of current one
        5. Land after all waypoints are visited
 
    For this version: This node is self contained -> Can be tested alone.
    Not included yet: 
        msg.position   = True
        msg.velocity   = False
        msg.acceleration = False
        msg.attitude   = False
        msg.body_rate  = False
    """
 
    # ------------------------------------------------------------------
    # Tuning constants — adjust these without touching the logic below.
    # ------------------------------------------------------------------
    TIMER_HZ         = 10          # control loop rate
    PRE_ARM_CYCLES   = 10          # cycles to stream before arming (≥1 s at 10 Hz)
    ACCEPTANCE_RADIUS_M = 0.5      # advance to next WP when closer than this
    TAKEOFF_Z_NED    = -3.0        # default takeoff altitude in NED (negative = up)
 
    def __init__(self):
        super().__init__("waypoint_sequencer_node")
 
        # For publishing TO PX4
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # For subscribing FROM PX4
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,)
 
        # --- Publishers ---
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_pub)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_pub)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_pub)
 
        # --- Subscribers ---
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._on_local_position,
            qos_sub,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self._on_vehicle_status,
            qos_sub,
        )
 
        # --- State ---
        self.local_pos: VehicleLocalPosition | None = None
        self.vehicle_status: VehicleStatus | None   = None
 
        self.counter       = 0
        self.armed         = False
        self.land_sent     = False
        self.wp_index      = 0        # index into self.waypoints
 
        # ------------------------------------------------------------------
        # Waypoint list — replace this with planner output later.
        # A simple 3 × 3 grid at 3 m altitude as a smoke-test pattern.
        # All coordinates are in PX4 NED (x = North, y = East, z negative up).
        # ------------------------------------------------------------------
        self.waypoints: list[Waypoint] = [
            Waypoint( 0.0,  0.0, self.TAKEOFF_Z_NED),   # take-off point
            Waypoint( 5.0,  0.0, self.TAKEOFF_Z_NED),
            Waypoint( 5.0,  5.0, self.TAKEOFF_Z_NED),
            Waypoint( 0.0,  5.0, self.TAKEOFF_Z_NED),
            Waypoint( 0.0, 10.0, self.TAKEOFF_Z_NED),
            Waypoint( 5.0, 10.0, self.TAKEOFF_Z_NED),
            Waypoint( 0.0,  0.0, self.TAKEOFF_Z_NED),   # return to origin before land
        ]
 
        self.timer = self.create_timer(1.0 / self.TIMER_HZ, self._control_loop)
        self.get_logger().info("Waypoint sequencer started.")
        self.get_logger().info(f"Mission: {len(self.waypoints)} waypoints, "
                               f"acceptance radius {self.ACCEPTANCE_RADIUS_M} m.")
 
    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _on_local_position(self, msg: VehicleLocalPosition):
        self.local_pos = msg
 
    def _on_vehicle_status(self, msg: VehicleStatus):
        self.vehicle_status = msg
 
    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def _control_loop(self):
 
        # Always publish the offboard heartbeat — PX4 will drop offboard
        # mode if this stops arriving at >2 Hz.
        self._publish_offboard_control_mode()
 
        # 1. Pre arm streaming
        if self.counter < self.PRE_ARM_CYCLES:
            # Stream the first waypoint as setpoint so PX4 is happy
            self._publish_setpoint(self.waypoints[0])
            self.counter += 1
            return
 
        # 2. Engange offboard and arm
        if not self.armed:
            self._engage_offboard_mode()
            self._arm()
            self.armed = True
            self.get_logger().info("Offboard mode requested. Arm command sent.")
 
        # 3. Check if all points were visited
        if self.wp_index >= len(self.waypoints):
            if not self.land_sent:
                self._land()
                self.land_sent = True
                self.get_logger().info("All waypoints complete. Land command sent.")
            # Keep publishing the last setpoint so the offboard heartbeat
            # stays alive until PX4 takes over with the land mode.
            self._publish_setpoint(self.waypoints[-1])
            return
 
        # 4. Fly current setpoint
        current_wp = self.waypoints[self.wp_index]
        self._publish_setpoint(current_wp)
 
        # Check arrival at each point only when we have a valid position estimate.
        if self.local_pos is not None:
            distance = self._distance_to_wp(current_wp)
            if distance < self.ACCEPTANCE_RADIUS_M:
                self.get_logger().info(
                    f"Reached WP {self.wp_index} {current_wp}  "
                    f"(err={distance:.2f} m) → advancing."
                )
                self.wp_index += 1
        else:
        
            self.get_logger().warn("local_pos still None")  # ← add
 
    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp  = self._timestamp_us()
        msg.position   = True
        msg.velocity   = False
        msg.acceleration = False
        msg.attitude   = False
        msg.body_rate  = False
        self.offboard_control_mode_pub.publish(msg)
 
    def _publish_setpoint(self, wp: Waypoint):
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        msg.position  = [wp.x, wp.y, wp.z]
        msg.yaw       = wp.yaw
        self.trajectory_setpoint_pub.publish(msg)
 
    # ------------------------------------------------------------------
    # Vehicle commands
    # ------------------------------------------------------------------
    def _engage_offboard_mode(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                           param1=1.0, param2=6.0)
 
    def _arm(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                           param1=1.0)
 
    def _land(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
 
    def _send_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.timestamp        = self._timestamp_us()
        msg.command          = command
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        for k, v in kwargs.items():
            setattr(msg, k, float(v))
        self.vehicle_command_pub.publish(msg)
 
    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _distance_to_wp(self, wp: Waypoint) -> float:
        """3-D Euclidean distance from current drone position to waypoint."""
        dx = self.local_pos.x - wp.x
        dy = self.local_pos.y - wp.y
        dz = self.local_pos.z - wp.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)
 
    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)
 
 
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WaypointSequencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    
 
 
if __name__ == "__main__":
    main()
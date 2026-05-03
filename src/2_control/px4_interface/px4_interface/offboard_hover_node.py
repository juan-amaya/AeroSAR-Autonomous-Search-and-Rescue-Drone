import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

class OffboardHoverNode(Node):
    """
     Minimal PX4 offboard test:
    - stream offboard heartbeat
    - command hover at 3 m
    - switch to offboard
    - arm
    - hover for 10 s
    - land
    """
    def __init__ (self):
        super().__init__("offboard_hover_node")
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # Define Publishers
        self.offboard_control_mode_pub = self.create_publisher (
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos,
        )
        self.trajectory_setpoint_pub = self.create_publisher (
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos,
        )
        self.vehicle_command_pub = self.create_publisher (
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos,
        )
        
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos,
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos,
        )
        self.vehicle_local_position = None
        self.vehicle_status = None

        self.counter = 0
        self.offboard_started = False
        self.land_sent = False

        self.target_altitude_m = -3.0  # PX4 uses NED: negative z means up
        self.hover_duration_s = 10.0
        self.timer_period_s = 0.1      # 10 Hz

        self.timer = self.create_timer(self.timer_period_s, self.timer_callback)

        self.get_logger().info("Offboard hover node started.")
        self.get_logger().info("Target: takeoff to 3 m, hover 10 s, then land.")
    
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
    
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
    
    def timer_callback(self):
        """
            Main control loop.

            PX4 requires offboard messages before switching to offboard mode.
            Therefore:
            - first 10 cycles: only stream heartbeat + setpoint
            - after 10 cycles: switch to offboard and arm
            - then keep streaming setpoint
            - after hover duration: send land command
        """
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        
        if self.counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_started = True
            self.get_logger().info("Offboard mode requested and arm command sent.")
            
        # After entering offboard, wait hover duration, then land.
        hover_ticks = int(self.hover_duration_s / self.timer_period_s)

        if self.offboard_started and self.counter >= 10 + hover_ticks and not self.land_sent:
            self.land()
            self.land_sent = True
            self.get_logger().info("Hover complete. Land command sent.")

        if self.counter < 100000:
            self.counter += 1
    
    def publish_offboard_control_mode(self):
        
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = False
        msg.thrust_and_torque = False
        
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_trajectory_setpoint(self):
        
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()

        # Hold x=0, y=0, z=-3 m in PX4 NED frame.
        msg.position = [0.0, 0.0, self.target_altitude_m]
        msg.yaw = 0.0

        self.trajectory_setpoint_pub.publish(msg)
        
    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )

    def land(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        )

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()

        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)


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
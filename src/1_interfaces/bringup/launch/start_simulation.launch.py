"""
start_simulation.launch.py
──────────────────────────
AeroSAR simulation bringup for the AirSim + MAVROS stack.
Migrated from px4_avoidance_airsim/launch/start_simulation.launch (ROS 1 XML).

Starts:
  1. MAVROS          — PX4 SITL ↔ ROS 2 bridge
  2. AirSim ROS 2    — publishes camera, depth, IMU, GPS topics from AirSim
  3. Static TF       — base_link → camera_link frame used by planners
  4. RViz2           — optional, controlled by the 'rviz' launch arg

Usage:
  ros2 launch bringup start_simulation.launch.py
  ros2 launch bringup start_simulation.launch.py airsim_host:=192.168.1.10
  ros2 launch bringup start_simulation.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Paths ─────────────────────────────────────────────────────────────────
    bringup_share = get_package_share_directory("bringup")
    mavros_config  = os.path.join(bringup_share, "config", "mavros.yaml")
    rviz_config    = os.path.join(bringup_share, "rviz", "airsim.rviz")

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_airsim_host = DeclareLaunchArgument(
        "airsim_host",
        default_value=EnvironmentVariable("AIRSIM_HOST", default_value="172.17.0.1"),
        description="IP address of the machine running AirSim / Unreal Engine",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz2 visualiser",
    )
    declare_fcu_url = DeclareLaunchArgument(
        "fcu_url",
        default_value="udp://:14540@localhost:14557",
        description="MAVLink FCU URL for MAVROS (e.g. serial:///dev/ttyUSB0:57600)",
    )
    declare_vehicle = DeclareLaunchArgument(
        "vehicle",
        default_value="Drone",
        description="AirSim vehicle name as configured in settings.json",
    )

    # ── MAVROS node ───────────────────────────────────────────────────────────
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
        parameters=[
            mavros_config,
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": "udp://@localhost:14550",
                "tgt_system": 1,
                "tgt_component": 1,
            },
        ],
    )

    # ── AirSim ROS 2 wrapper ──────────────────────────────────────────────────
    # Source: https://github.com/Cosys-Lab/Cosys-AirSim  (ros2 branch)
    # Build airsim_ros_pkgs with colcon before launching.
    airsim_node = Node(
        package="airsim_ros_pkgs",
        executable="airsim_node",
        name="airsim",
        output="screen",
        parameters=[
            {
                "host_ip": LaunchConfiguration("airsim_host"),
                "host_port": 41451,
                "vehicle_name": LaunchConfiguration("vehicle"),
                # Publish depth pointcloud for obstacle avoidance
                "publish_clock": True,
                "is_vulkan": False,
            }
        ],
        remappings=[
            # Remap depth image to the topic PX4-Avoidance / Fast-Planner expect
            ("airsim/DepthPlanar", "/camera/depth/image_rect_raw"),
            ("airsim/DepthPlanar/camera_info", "/camera/depth/camera_info"),
            ("airsim/Scene", "/camera/rgb/image_rect_color"),
        ],
    )

    # ── Static TF: base_link → camera_link ───────────────────────────────────
    # AirSim default camera is mounted at the front centre.
    # Adjust the translation/rotation to match your AirSim settings.json.
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        arguments=[
            "0.15", "0.0", "-0.05",   # x y z  (metres, forward/left/up)
            "0",    "0",   "0",  "1", # qx qy qz qw  (no rotation)
            "base_link",
            "camera_link",
        ],
    )

    # ── RViz2 (optional) ──────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    # ── Startup log ───────────────────────────────────────────────────────────
    startup_log = LogInfo(
        msg=[
            "\n\n",
            "┌─────────────────────────────────────────────┐\n",
            "│  AeroSAR  ·  AirSim simulation bringup     │\n",
            "│                                             │\n",
            "│  Make sure:                                 │\n",
            "│  1. AirSim is running on AIRSIM_HOST        │\n",
            "│  2. PX4 SITL is running (make px4_sitl      │\n",
            "│     none_iris)                              │\n",
            "│  3. AirSim settings.json has correct UDP    │\n",
            "│     ports (14560 for PX4 HIL)               │\n",
            "└─────────────────────────────────────────────┘\n",
        ]
    )

    return LaunchDescription(
        [
            declare_airsim_host,
            declare_rviz,
            declare_fcu_url,
            declare_vehicle,
            startup_log,
            mavros_node,
            airsim_node,
            camera_tf,
            rviz_node,
        ]
    )

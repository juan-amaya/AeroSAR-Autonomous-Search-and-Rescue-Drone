"""
start_avoidance.launch.py
─────────────────────────
Launches obstacle avoidance planners for AeroSAR.
Migrated from px4_avoidance_airsim/launch/start_fast_planner.launch (ROS 1 XML).

Two planners are supported (select with the 'planner' arg):

  fast_planner  — HKUST Fast-Planner (kinodynamic RRT* + B-spline opt.)
                  ROS 2 port: https://github.com/xtark/fast_planner_ros2
                  Best for aggressive, time-optimal trajectories.

  px4_avoidance — PX4-Avoidance local planner (stereo/depth input)
                  ROS 2 port available in PX4 main branch.
                  Best for reactive obstacle avoidance.

Usage:
  ros2 launch bringup start_avoidance.launch.py                      # fast_planner
  ros2 launch bringup start_avoidance.launch.py planner:=px4_avoidance
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ── Paths ─────────────────────────────────────────────────────────────────
    bringup_share = get_package_share_directory("bringup")

    fast_planner_config = os.path.join(
        bringup_share, "config", "fast_planner.yaml"
    )
    px4_avoidance_config = os.path.join(
        bringup_share, "config", "px4_avoidance.yaml"
    )

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_planner = DeclareLaunchArgument(
        "planner",
        default_value="fast_planner",
        choices=["fast_planner", "px4_avoidance"],
        description="Which planning algorithm to run",
    )
    declare_goal_height = DeclareLaunchArgument(
        "goal_height",
        default_value="1.0",
        description="Default flight height (m) when using 2-D goal inputs",
    )
    declare_max_vel = DeclareLaunchArgument(
        "max_vel",
        default_value="3.0",
        description="Maximum cruise velocity (m/s) — matches MPC_XY_CRUISE param",
    )

    use_fast = PythonExpression(
        ["'", LaunchConfiguration("planner"), "' == 'fast_planner'"]
    )
    use_px4  = PythonExpression(
        ["'", LaunchConfiguration("planner"), "' == 'px4_avoidance'"]
    )

    # ── Fast-Planner nodes ────────────────────────────────────────────────────
    # Requires fast_planner_ros2 built in your colcon workspace.
    # Depth image topic remapped to match AirSim output from start_simulation.
    fast_planner_group = GroupAction(
        condition=IfCondition(use_fast),
        actions=[
            # Voxel map builder (occupancy grid from depth)
            Node(
                package="plan_env",
                executable="voxel_mapping_node",
                name="voxel_map",
                output="screen",
                parameters=[fast_planner_config],
                remappings=[
                    ("depth",       "/camera/depth/image_rect_raw"),
                    ("camera_info", "/camera/depth/camera_info"),
                    ("odom",        "/mavros/local_position/odom"),
                ],
            ),
            # Kinodynamic planner
            Node(
                package="plan_manage",
                executable="kino_replan_node",
                name="fast_planner",
                output="screen",
                parameters=[
                    fast_planner_config,
                    {
                        # Override goal height from launch arg
                        "fsm/flight_type": 1,
                        # Publish /move_base_simple/goal for waypoint injection
                    },
                ],
                remappings=[
                    ("odom",  "/mavros/local_position/odom"),
                    ("bspline", "/planning/bspline"),
                ],
            ),
            # Trajectory tracker — converts B-spline to MAVROS setpoints
            Node(
                package="bspline_opt",
                executable="traj_server_node",
                name="traj_server",
                output="screen",
                parameters=[fast_planner_config],
                remappings=[
                    ("bspline",          "/planning/bspline"),
                    ("position_cmd",     "/planning/pos_cmd"),
                    # Output setpoint to MAVROS
                    ("mavros/setpoint_position/local",
                     "/mavros/setpoint_position/local"),
                ],
            ),
        ],
    )

    # ── PX4-Avoidance local planner ───────────────────────────────────────────
    # Uses depth pointcloud; outputs velocity setpoints via MAVROS.
    px4_avoidance_group = GroupAction(
        condition=IfCondition(use_px4),
        actions=[
            Node(
                package="local_planner",
                executable="local_planner_node",
                name="local_planner",
                output="screen",
                parameters=[
                    px4_avoidance_config,
                    {
                        "max_speed": LaunchConfiguration("max_vel"),
                    },
                ],
                remappings=[
                    ("camera/depth/points", "/camera/depth/points"),
                    ("mavros/local_position/pose",
                     "/mavros/local_position/pose"),
                    ("mavros/setpoint_velocity/cmd_vel_unstamped",
                     "/mavros/setpoint_velocity/cmd_vel_unstamped"),
                ],
            ),
        ],
    )

    # ── Startup log ───────────────────────────────────────────────────────────
    startup_log = LogInfo(
        msg=[
            "\nStarting planner: ",
            LaunchConfiguration("planner"),
            "  |  max_vel=",
            LaunchConfiguration("max_vel"),
            " m/s  |  goal_height=",
            LaunchConfiguration("goal_height"),
            " m\n",
        ]
    )

    return LaunchDescription(
        [
            declare_planner,
            declare_goal_height,
            declare_max_vel,
            startup_log,
            fast_planner_group,
            px4_avoidance_group,
        ]
    )

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os


def generate_launch_description():

    replan_enabled_arg = DeclareLaunchArgument(
        "replan_enabled",
        default_value="true",
        description="Enable or disable the DWA replanning feature (true/false)"
    )

    replan_arg = LaunchConfiguration("replan_enabled")

    local_costmap = Node(
        package="pibot_localplan",
        executable="local_costmap",
        name="local_costmap",
        parameters=[
            os.path.join(
                get_package_share_directory("pibot_globalplan"),
                "config",
                "nav2_params.yaml",
            ),
            {"use_sim_time": False}
        ],
    )

    dwa_node_replan_true = Node(
        package="pibot_localplan",
        executable="dwa_cpp_replan",
        name="dwa_planner_node",
        parameters=[
            {"use_sim_time": False},
            {"replan_mode": "periodic"},
            {"replan_period": 1.0},
        ],
        condition=IfCondition(replan_arg)
    )

    dwa_node_replan_false = Node(
        package="pibot_localplan",
        executable="dwa_cpp",
        name="dwa_planner_node",
        parameters=[
            {"use_sim_time": False},
        ],
        condition=UnlessCondition(replan_arg)
    )
 
    return LaunchDescription([ 
        replan_enabled_arg,
        local_costmap,
        dwa_node_replan_true,
        dwa_node_replan_false
    ])
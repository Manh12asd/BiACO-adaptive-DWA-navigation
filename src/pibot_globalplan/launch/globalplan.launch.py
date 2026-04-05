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
    
    replan_config = LaunchConfiguration("replan_enabled")

    ACO_client = Node(
        package="pibot_globalplan",
        executable="planner_client.py",
        name="aco_action_client",
    )

    ACO_server_replan_true = Node(
        package="pibot_globalplan",
        executable="biaco",
        name="aco_planner_action_server",
        condition=IfCondition(replan_config)
    )

    ACO_server_replan_false = Node(
        package="pibot_globalplan",
        executable="aco_test",
        name="aco_planner_action_server",
        condition=UnlessCondition(replan_config)
    )
        
    global_costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        parameters=[
            os.path.join(
                get_package_share_directory("pibot_globalplan"),
                "config",
                "nav2_params.yaml",
            ),
            {"use_sim_time": False}
        ],
    )

    return LaunchDescription([
        replan_enabled_arg,
        ACO_server_replan_true,
        ACO_server_replan_false,
        ACO_client,
        global_costmap
    ])
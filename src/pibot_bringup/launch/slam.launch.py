import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("pibot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )

    pibot_controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("pibot_controller"),
            "launch",
            "pibot_controller.launch.py"
        )
    )

    joy_controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("pibot_controller"),
            "launch",
            "joy_teleop.launch.py"
        )
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("pibot_localization"),
            "launch",
            "ekf_fusion.launch.py"
        )
    )

    slam_toolbox = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("pibot_mappping"),
            "launch",
            "slam_toolbox.launch.py"
        )
    )
    

    return LaunchDescription([
        pibot_controller,
        joy_controller,
        laser_driver,
        slam_toolbox,
        local_localization
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Đường dẫn đến file config bạn vừa tạo ở Bước 2
    # Giả sử bạn để file params.yaml trong thư mục config của package 'obstacle_detector_2'
    config_file = os.path.join(
        get_package_share_directory('obstacle_detector'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='obstacle_detector',
            executable='obstacle_extractor_node',
            name='obstacle_extractor',
            parameters=[config_file],
            remappings=[
                ('scan', '/scan') # Sửa '/scan' thành topic lidar thật của bạn
            ],
            output='screen'
        ),
        
        # Node này thực hiện việc TRACKING (Kalman Filter)
        Node(
            package='obstacle_detector',
            executable='obstacle_tracker_node',
            name='obstacle_tracker',
            parameters=[{
                'frame_id': 'map',
                'loop_rate': 10.0,
                'tracking_duration': 2.0, # Giữ track trong 2s nếu mất dấu
                'min_correspondence_cost': 0.6,
                'std_correspondence_dev': 0.15,
                'process_variance': 0.1,  # Độ tin cậy mô hình chuyển động
                'process_rate_variance': 0.1,
                'measurement_variance': 1.0 
            }],
            remappings=[
                ('raw_obstacles', 'raw_obstacles') # Output của extractor -> Input của tracker
            ],
            output='screen'
        )
    ])
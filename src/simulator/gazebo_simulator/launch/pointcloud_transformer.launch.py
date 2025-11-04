from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        'config_file': '/config/topic_transform.yaml',
        'check_period_sec': 1.0
    }

    transformer_node = Node(
        package='gazebo_simulator',
        executable='pointcloud_transformer',
        name='pointcloud_transformer',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([transformer_node])

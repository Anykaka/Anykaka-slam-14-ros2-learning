import os, socket, time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros import parameter_descriptions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取gazebo_simulator包的路径
    gazebo_simulator_dir = get_package_share_directory('gazebo_simulator')

    # 获取机器人模型的URDF文件的描述信息
    urdf_model_file = os.path.join(gazebo_simulator_dir, 'urdf', 'entrance', 'robot.xacro')
    urdf_model_str = Command('xacro ' + urdf_model_file)
    robot_model_description = parameter_descriptions.ParameterValue(urdf_model_str, value_type=str)
    
    # 获取机器人状态发布节点
    action_robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot_model_description}])
    
    # 获取机器人连接状态发布节点
    action_joint_state_publisher = Node(package='joint_state_publisher', executable='joint_state_publisher')
    
    # Gazebo节点通过加载launch启动
    default_word_file = os.path.join(gazebo_simulator_dir, 'world', 'entrance', 'entrance.world')
    action_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_word_file), ('verbose', 'true')]
    )
    
    time.sleep(2)  # 等待Gazebo完全启动
    
    # Gazebo加载机器人模型
    action_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', '/robot_description', '-entity', 'robot'])
    
    # 获取RViz2节点
    rviz_config_file = os.path.join(gazebo_simulator_dir, 'rviz', 'entrance.rviz')
    action_rviz2 = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_file])
    
    # 加载点云转换节点
    action_point_cloud_transformer_parameters = {
        'config_file': '/config/topic_transform.yaml',
        'check_period_sec': 1.0
    }
    action_point_cloud_transformer = Node(package='gazebo_simulator', executable='pointcloud_transformer', parameters=[action_point_cloud_transformer_parameters])

    return LaunchDescription([
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_gazebo_launch,
        action_spawn_entity,
        action_rviz2,
        action_point_cloud_transformer,
    ])

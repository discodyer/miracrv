import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # 获取包的路径
    miracrv_driver_share = get_package_share_directory('miracrv_driver')

    # 配置文件路径参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([miracrv_driver_share, 'config', 'ardu_rover.yaml']),
        description='Path to the configuration file'
    )

    # 声明启动参数
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm',
        default_value='false',
        description='Automatically arm the vehicle'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    default_mode_arg = DeclareLaunchArgument(
        'default_mode',
        default_value='GUIDED',
        description='Default vehicle mode'
    )
    
    gps_origin_lat_arg = DeclareLaunchArgument(
        'gps_origin_lat',
        default_value='39.979982',
        description='GPS origin latitude'
    )
    
    gps_origin_lon_arg = DeclareLaunchArgument(
        'gps_origin_lon',
        default_value='116.335492',
        description='GPS origin longitude'
    )
    
    # ArduRover节点
    ardu_rover_node = Node(
        package='miracrv_driver',
        executable='ardu_rover_node',
        name='ardu_rover_node',
        output='screen',
        parameters=[
            # 首先加载yaml文件
            LaunchConfiguration('config_file'),
            # 然后覆盖特定参数（如果需要）
            {
                'config_file' : LaunchConfiguration('config_file')
                # 'auto_arm': LaunchConfiguration('auto_arm'),
                # 'default_mode': LaunchConfiguration('default_mode'),
                # 'gps_origin_lat': LaunchConfiguration('gps_origin_lat'),
                # 'gps_origin_lon': LaunchConfiguration('gps_origin_lon')
            }
        ],
        # 话题重映射
        remappings=[
            # ('/cmd_vel', '/miracrv/cmd_vel'),  # 如果需要重映射
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        config_file_arg,
        auto_arm_arg,
        default_mode_arg,
        gps_origin_lat_arg,
        gps_origin_lon_arg,
        # 节点
        ardu_rover_node
    ])
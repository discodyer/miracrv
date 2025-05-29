#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value = PathJoinSubstitution([FindPackageShare('miracrv_bringup'), 'urdf', 'lidar_imu_urdf.urdf']),
        description='Path to the URDF/xacro model file'
    )
    
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM_APM',
        description='FCU connection URL'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # 功能开关参数
    enable_urdf_arg = DeclareLaunchArgument(
        'enable_urdf',
        default_value='true',
        description='Enable URDF visualization'
    )
    
    enable_mavros_arg = DeclareLaunchArgument(
        'enable_mavros',
        default_value='true',
        description='Enable MAVROS'
    )
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable LSLidar driver'
    )
    
    enable_cartographer_arg = DeclareLaunchArgument(
        'enable_cartographer',
        default_value='true',
        description='Enable Cartographer'
    )
    
    # 获取参数
    model_path = LaunchConfiguration('model_path')
    fcu_url = LaunchConfiguration('fcu_url')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_urdf = LaunchConfiguration('enable_urdf')
    enable_mavros = LaunchConfiguration('enable_mavros')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_cartographer = LaunchConfiguration('enable_cartographer')
    
    # 条件启动各个组件
    urdf_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('urdf_tutorial'),
                'launch',
                'display.launch.py'
            ])
        ]),
        launch_arguments={
            'model': model_path,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(enable_urdf)
    )
    
    tf_to_odom_node = Node(
        package='miracrv_driver',
        executable='tf_to_odom.py',
        name='tf_to_odom',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'apm.launch'
            ])
        ]),
        launch_arguments={
            'fcu_url': fcu_url
        }.items(),
        condition=IfCondition(enable_mavros)
    )
    
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lslidar_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(enable_lidar)
    )
    
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('miracrv_bringup'),
                'launch',
                'cartographer.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(enable_cartographer)
    )
    
    return LaunchDescription([
        # 参数声明
        model_path_arg,
        fcu_url_arg,
        use_sim_time_arg,
        enable_urdf_arg,
        enable_mavros_arg,
        enable_lidar_arg,
        enable_cartographer_arg,
        
        # 启动信息
        LogInfo(msg='Starting Ardupilot ROS2 system...'),
        
        # 启动各个组件
        urdf_display,
        tf_to_odom_node,
        TimerAction(period=1.0, actions=[lslidar_launch]),
        TimerAction(period=3.0, actions=[mavros_launch]),
        TimerAction(period=5.0, actions=[cartographer_launch]),
        
        # 完成提示
        TimerAction(
            period=6.0,
            actions=[LogInfo(msg='System startup complete!')]
        )
    ])
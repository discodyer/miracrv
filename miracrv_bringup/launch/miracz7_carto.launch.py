from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    urdf_model_path_arg = DeclareLaunchArgument(
        'urdf_model_path',
        default_value = PathJoinSubstitution([FindPackageShare('miracrv_bringup'), 'urdf', 'lidar_imu_miracz7_urdf.urdf']),
        description='Path to the URDF/xacro model file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_urdf_arg = DeclareLaunchArgument(
        'enable_urdf',
        default_value='true',
        description='Enable URDF visualization'
    )

    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='false',
        description='Enable LSLidar driver'
    )

    enable_cartographer_arg = DeclareLaunchArgument(
        'enable_cartographer',
        default_value='true',
        description='Enable Cartographer'
    )

    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_urdf = LaunchConfiguration('enable_urdf')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_cartographer = LaunchConfiguration('enable_cartographer')

    # 条件启动各个组件
    urdf_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', LaunchConfiguration('urdf_model_path')]),
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(enable_urdf)
    )

    odom_to_tf_broadcaster_node = Node(
        package='miracrv_driver',
        executable='odom_to_tf_broadcaster.py',
        name='odom_to_tf_broadcaster',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lslidar_miracz7_nogui_launch.py'
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
                'cartographer_miracz7.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(enable_cartographer)
    )

    return LaunchDescription([
        # 参数声明
        urdf_model_path_arg,
        use_sim_time_arg,
        enable_urdf_arg,
        enable_lidar_arg,
        enable_cartographer_arg,

        urdf_publisher_node,
        # odom_to_tf_broadcaster_node,
        lslidar_launch,
        cartographer_launch
        ])


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm',
        default_value='false',
        description='Automatically arm the vehicle'
    )
    
    default_mode_arg = DeclareLaunchArgument(
        'default_mode',
        default_value='GUIDED',
        description='Default vehicle mode'
    )
    
    gps_origin_lat_arg = DeclareLaunchArgument(
        'gps_origin_lat',
        default_value='0.0',
        description='GPS origin latitude'
    )
    
    gps_origin_lon_arg = DeclareLaunchArgument(
        'gps_origin_lon',
        default_value='0.0',
        description='GPS origin longitude'
    )
    
    # ArduRover节点
    ardu_rover_node = Node(
        package='libracer',
        executable='ardu_rover_node',
        name='ardu_rover_node',
        output='screen',
        parameters=[{
            'auto_arm': LaunchConfiguration('auto_arm'),
            'default_mode': LaunchConfiguration('default_mode'),
            'gps_origin_lat': LaunchConfiguration('gps_origin_lat'),
            'gps_origin_lon': LaunchConfiguration('gps_origin_lon')
        }]
    )
    
    return LaunchDescription([
        auto_arm_arg,
        default_mode_arg,
        gps_origin_lat_arg,
        gps_origin_lon_arg,
        ardu_rover_node
    ])
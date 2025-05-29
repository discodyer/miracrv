import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz."
    )

    ## ***** Nodes *****
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory",
            FindPackageShare("miracrv_bringup").find("miracrv_bringup")
            + "/config",
            "-configuration_basename",
            "cartographer.lua",
        ],
        output="screen",
        remappings=[
            ("/scan", "/scan"),
            ("/imu", "/mavros/imu/data"),
            ("/odom", "/mavros/local_position/odom"),
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"resolution": 0.05},
        ],
    )

    # Robot description.

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # RViz.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            str(
                Path(
                    FindPackageShare("miracrv_bringup").find(
                        "miracrv_bringup"
                    ),
                    "rviz",
                    "cartographer.rviz",
                )
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            rviz_arg,
            # Nodes
            cartographer_node,
            cartographer_occupancy_grid_node,
            rviz,
        ]
    )

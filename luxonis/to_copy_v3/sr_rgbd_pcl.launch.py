import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver_v3")

    name = LaunchConfiguration("name").perform(context)
    namespace = LaunchConfiguration("namespace", default="").perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "driver.launch.py")
            ),
            launch_arguments={
                "name": name,
                "params_file": params_file,
                "parent_frame": LaunchConfiguration("parent_frame"),
                "cam_pos_x": LaunchConfiguration("cam_pos_x"),
                "cam_pos_y": LaunchConfiguration("cam_pos_y"),
                "cam_pos_z": LaunchConfiguration("cam_pos_z"),
                "cam_roll": LaunchConfiguration("cam_roll"),
                "cam_pitch": LaunchConfiguration("cam_pitch"),
                "cam_yaw": LaunchConfiguration("cam_yaw"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "publish_description": LaunchConfiguration("publish_description"),
                "publish_tf_from_calibration": LaunchConfiguration(
                    "publish_tf_from_calibration"
                ),
            }.items(),
        ),
        # depth is computed in rectified space, so /oak/stereo/image_raw only
        # lines up pixel-for-pixel against the *rectified* right image -- the
        # driver itself only ever publishes raw. Loads into the same
        # {name}_container the driver.launch.py include above just started
        # (no second container process). See
        # docs_demo/OAK_DEPTHAI_ROS2_SETUP.md section 7 ("Real fix #2").
        LoadComposableNodes(
            condition=IfCondition(LaunchConfiguration("rectify_right")),
            target_container=f"{name}_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_right_node",
                    namespace=namespace,
                    remappings=[
                        ("image", f"{name}/right/image_raw"),
                        ("camera_info", f"{name}/right/camera_info"),
                        ("image_rect", f"{name}/right/image_rect"),
                    ],
                )
            ],
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver_v3")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("camera_model", default_value="OAK-D-SR"),
        DeclareLaunchArgument("parent_frame", default_value="oak_parent_frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "sr_rgbd.yaml"),
        ),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        # Default false: PiPER's real URDF (piper_mount_oak_d_sr_handeye)
        # already roots this camera under link6 -- see driver.launch.py's
        # publish_description arg for why both must stay off here.
        DeclareLaunchArgument("publish_description", default_value="false"),
        DeclareLaunchArgument("publish_tf_from_calibration", default_value="false"),
        DeclareLaunchArgument("rectify_right", default_value="true"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    package_share = os.path.join(
        os.getenv('HOME'),'RISET','Aruco_ros','src','aruco'
    )
    config_path = os.path.join(package_share,'config','config.yaml')


    return LaunchDescription([
        # Node untuk static transform (base_link -> camera_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera',
            arguments=['0.45', '0.0', '0.22', '-1.5708', '0', '-1.5708', 'base_link', 'camera_link']
        ),

        # Node ArUco detection kamu
        Node(
            package='aruco',
            executable='aruco_detect',
            name='aruco_detect',
            output='screen',
            parameters=[config_path]
        )
    ])

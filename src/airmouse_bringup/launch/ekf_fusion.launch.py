import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('airmouse_bringup')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file]
    )

    return LaunchDescription([
        ekf_node
    ])

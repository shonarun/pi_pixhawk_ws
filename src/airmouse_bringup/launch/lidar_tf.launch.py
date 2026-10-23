from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '0.0', '0.0', '0.1',
            '0.0', '0.0', '0.0',
            'base_link',
            'laser_frame'
        ]
    )

    return LaunchDescription([
        rplidar_node,
        static_tf_node
    ])

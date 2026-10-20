import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
        ),
        launch_arguments={'fcu_url': 'serial:///dev/ttyUSB0:921600'}.items()
    )

    set_stream_rate = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/mavros/set_stream_rate',
                    'mavros_msgs/srv/StreamRate',
                    '{stream_id: 0, message_rate: 20, on_off: true}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        mavros_launch,
        set_stream_rate
    ])

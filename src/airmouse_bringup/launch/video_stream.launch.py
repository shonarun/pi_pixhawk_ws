from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'host_ip',
            default_value='192.168.1.100',
            description='Target IP address for the UDP video stream'
        ),
        ExecuteProcess(
            cmd=[
                'gst-launch-1.0', 'v4l2src', 'device=/dev/video0', '!',
                'image/jpeg,width=1920,height=1080,framerate=30/1', '!',
                'rtpjpegpay', '!', 
                'udpsink', ['host=', LaunchConfiguration('host_ip')], 'port=5000'
            ],
            output='screen'
        )
    ])
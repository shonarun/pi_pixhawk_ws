from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    receiver_cmd = (
        "\n\n"
        "========================================================================================\n"
        "TO VIEW VIDEO, RUN THIS COMMAND ON YOUR LAPTOP TERMINAL:\n\n"
        "gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink\n"
        "========================================================================================\n"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'host_ip',
            default_value='192.168.1.100',
            description='Target IP address for the UDP video stream'
        ),
        LogInfo(msg=receiver_cmd),
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

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State


class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )
        self.last_state = None
        self.msg_received = False
        self.watchdog_timer = self.create_timer(2.0, self.check_stream)
        self.get_logger().info('State Monitor initialized. Listening to /mavros/state...')

    def state_cb(self, msg: State):
        self.msg_received = True
        current = (msg.connected, msg.mode, msg.armed, msg.system_status)

        if current != self.last_state:
            self.get_logger().info(
                f'FCU Connected: {msg.connected} | '
                f'Mode: {msg.mode} | '
                f'Armed: {msg.armed} | '
                f'System Status: {msg.system_status}'
            )
            self.last_state = current

    def check_stream(self):
        if not self.msg_received:
            self.get_logger().warn('No messages received on /mavros/state in the last 2 seconds.')
        self.msg_received = False


def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

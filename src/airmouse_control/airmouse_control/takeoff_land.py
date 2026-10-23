import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, StreamRate


class TakeoffLand(Node):
    def __init__(self):
        super().__init__('takeoff_land')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            qos
        )

        self.stream_cli = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.current_state = State()
        self.current_z = 0.0
        self.pose_received = False
        self.stream_set = False
        self.target_alt = 2.0
        self.step = 0
        self.hover_ticks = 0

        self.timer = self.create_timer(1.0, self.fsm_step)
        self.get_logger().info('TakeoffLand initialized. Waiting for FCU connection...')

    def state_cb(self, msg: State):
        self.current_state = msg

    def pose_cb(self, msg: PoseStamped):
        if not self.pose_received:
            self.get_logger().info('Local position stream detected!')
        self.pose_received = True
        self.current_z = msg.pose.position.z

    def fsm_step(self):
        if not self.current_state.connected:
            return

        if not self.stream_set:
            if self.stream_cli.service_is_ready():
                req = StreamRate.Request()
                req.stream_id = 0
                req.message_rate = 20
                req.on_off = True
                self.stream_cli.call_async(req)
                self.stream_set = True
                self.get_logger().info('Stream rate 20Hz requested.')
            return

        if self.step == 0:
            if self.current_state.mode != 'GUIDED':
                if not self.mode_cli.service_is_ready():
                    return
                req = SetMode.Request()
                req.custom_mode = 'GUIDED'
                self.mode_cli.call_async(req)
            else:
                self.get_logger().info('GUIDED mode confirmed.')
                self.step = 1

        elif self.step == 1:
            if not self.current_state.armed:
                if not self.arm_cli.service_is_ready():
                    return
                self.get_logger().info('Sending arm command...')
                req = CommandBool.Request()
                req.value = True
                self.arm_cli.call_async(req)
            else:
                self.get_logger().info('Armed! Waiting for local position stream...')
                self.step = 2

        elif self.step == 2:
            if not self.pose_received:
                self.get_logger().warn('Armed, but /mavros/local_position/pose is still empty...')
                return
            if not self.takeoff_cli.service_is_ready():
                return
            self.get_logger().info('Requesting takeoff...')
            req = CommandTOL.Request()
            req.altitude = self.target_alt
            future = self.takeoff_cli.call_async(req)
            future.add_done_callback(self.takeoff_response_cb)
            self.step = 3

        elif self.step == 3:
            self.get_logger().info(f'Climbing... Current Z: {self.current_z:.2f}m')
            if self.current_z >= self.target_alt * 0.90:
                self.get_logger().info('Target altitude reached. Hovering for 5 seconds...')
                self.step = 4

        elif self.step == 4:
            self.hover_ticks += 1
            self.get_logger().info(f'Hovering... ({self.hover_ticks}/5)')
            if self.hover_ticks >= 5:
                self.step = 5

        elif self.step == 5:
            if self.current_state.mode != 'LAND':
                self.get_logger().info('Switching to LAND mode...')
                req = SetMode.Request()
                req.custom_mode = 'LAND'
                self.mode_cli.call_async(req)
            else:
                self.get_logger().info('LAND mode active. Waiting for touchdown and disarm...')
                self.step = 6

        elif self.step == 6:
            if not self.current_state.armed:
                self.get_logger().info('Touchdown and disarm confirmed. Sequence complete.')
                self.timer.cancel()

    def takeoff_response_cb(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().error('Takeoff rejected by FCU. Retrying...')
                self.step = 2
        except Exception as e:
            self.get_logger().error(f'Takeoff call failed: {e}')
            self.step = 2


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate


class TakeoffLand(Node):
    def __init__(self):
        super().__init__('takeoff_land')

        self.cb_group = ReentrantCallbackGroup()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10,
            callback_group=self.cb_group
        )

        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            sensor_qos,
            callback_group=self.cb_group
        )

        self.stream_cli = self.create_client(
            StreamRate, '/mavros/set_stream_rate', callback_group=self.cb_group
        )
        self.mode_cli = self.create_client(
            SetMode, '/mavros/set_mode', callback_group=self.cb_group
        )
        self.arm_cli = self.create_client(
            CommandBool, '/mavros/cmd/arming', callback_group=self.cb_group
        )
        self.takeoff_cli = self.create_client(
            CommandTOL, '/mavros/cmd/takeoff', callback_group=self.cb_group
        )

        self.current_state = State()
        self.relative_alt = 0.0
        self.stream_requested = False

        self.target_alt = 2.0
        self.step = 0
        self.hover_ticks = 0
        self.last_req_time = self.get_clock().now()
        self.takeoff_sent = False

        self.timer = self.create_timer(0.1, self.fsm_step, callback_group=self.cb_group)
        self.get_logger().info('TakeoffLand initialized. Waiting for FCU connection...')

    def state_cb(self, msg: State):
        self.current_state = msg

    def pose_cb(self, msg: PoseStamped):
        self.relative_alt = msg.pose.position.z

    def throttle_ready(self, seconds=1.0) -> bool:
        now = self.get_clock().now()
        # In rclpy, subtracting two Time objects yields a Duration, which has a nanoseconds attribute
        if (now - self.last_req_time).nanoseconds >= seconds * 1e9:
            self.last_req_time = now
            return True
        return False

    def fsm_step(self):
        if not self.current_state.connected:
            return

        if not self.stream_requested and self.stream_cli.service_is_ready():
            req = StreamRate.Request()
            req.stream_id = 0
            req.message_rate = 20
            req.on_off = True
            self.stream_cli.call_async(req)
            self.stream_requested = True
            self.get_logger().info('Stream rate 20Hz requested.')

        if self.step == 0:
            if self.current_state.mode != 'GUIDED':
                if self.mode_cli.service_is_ready() and self.throttle_ready(1.0):
                    self.get_logger().info('Requesting GUIDED mode...')
                    req = SetMode.Request()
                    req.custom_mode = 'GUIDED'
                    self.mode_cli.call_async(req)
            else:
                self.get_logger().info('GUIDED mode confirmed.')
                self.step = 1

        elif self.step == 1:
            if not self.current_state.armed:
                if self.arm_cli.service_is_ready() and self.throttle_ready(1.0):
                    self.get_logger().info('Requesting Arming...')
                    req = CommandBool.Request()
                    req.value = True
                    self.arm_cli.call_async(req)
            else:
                self.get_logger().info('Armed! Ready for takeoff.')
                self.takeoff_sent = False
                self.step = 2

        elif self.step == 2:
            if not self.takeoff_sent and self.takeoff_cli.service_is_ready() and self.throttle_ready(1.0):
                req = CommandTOL.Request()
                req.min_pitch = 0.0
                req.yaw = 0.0
                req.latitude = 0.0
                req.longitude = 0.0
                req.altitude = float(self.target_alt)

                future = self.takeoff_cli.call_async(req)
                future.add_done_callback(self.takeoff_response_cb)
                self.takeoff_sent = True
                self.get_logger().info(f'Sent takeoff request to {self.target_alt}m...')

        elif self.step == 3:
            if self.hover_ticks % 10 == 0:
                self.get_logger().info(f'Climbing... Current Alt: {self.relative_alt:.2f}m')
            self.hover_ticks += 1

            if self.relative_alt >= self.target_alt * 0.90:
                self.get_logger().info('Target altitude reached. Hovering for 5 seconds...')
                self.hover_ticks = 0
                self.step = 4

        elif self.step == 4:
            self.hover_ticks += 1
            if self.hover_ticks % 10 == 0:
                self.get_logger().info(f'Hovering... ({self.hover_ticks // 10}/5s)')

            if self.hover_ticks >= 50:
                self.step = 5

        elif self.step == 5:
            if self.current_state.mode != 'LAND':
                if self.mode_cli.service_is_ready() and self.throttle_ready(1.0):
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
            if res.success:
                self.get_logger().info('Takeoff accepted by FCU.')
                self.hover_ticks = 0
                self.step = 3
            else:
                self.get_logger().error('Takeoff rejected by FCU. Will retry...')
                self.takeoff_sent = False
        except Exception as e:
            self.get_logger().error(f'Takeoff service call failed: {e}')
            self.takeoff_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLand()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
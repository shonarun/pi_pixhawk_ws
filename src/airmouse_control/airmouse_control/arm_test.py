import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class ArmTest(Node):
    def __init__(self):
        super().__init__('arm_test')
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.current_state = State()
        self.step = 0
        self.hold_ticks = 0
        self.timer = self.create_timer(1.0, self.fsm_step)
        self.get_logger().info('ArmTest initialized. Waiting for FCU connection...')

    def state_cb(self, msg: State):
        self.current_state = msg

    def fsm_step(self):
        if not self.current_state.connected:
            return

        if self.step == 0:
            if self.current_state.mode != 'GUIDED':
                if not self.mode_cli.service_is_ready():
                    self.get_logger().warn('Waiting for /mavros/set_mode service...')
                    return
                self.get_logger().info('Requesting GUIDED mode...')
                req = SetMode.Request()
                req.custom_mode = 'GUIDED'
                self.mode_cli.call_async(req)
            else:
                self.get_logger().info('GUIDED mode confirmed.')
                self.step = 1

        elif self.step == 1:
            if not self.current_state.armed:
                if not self.arm_cli.service_is_ready():
                    self.get_logger().warn('Waiting for /mavros/cmd/arming service...')
                    return
                self.get_logger().info('Sending arming command...')
                req = CommandBool.Request()
                req.value = True
                future = self.arm_cli.call_async(req)
                future.add_done_callback(self.arm_response_cb)
                self.step = 2
            else:
                self.get_logger().info('Motors armed! Holding for 3 seconds...')
                self.step = 3

        elif self.step == 2:
            if self.current_state.armed:
                self.get_logger().info('Motors armed! Holding for 3 seconds...')
                self.step = 3

        elif self.step == 3:
            self.hold_ticks += 1
            self.get_logger().info(f'Holding armed state... ({self.hold_ticks}/3)')
            if self.hold_ticks >= 3:
                self.step = 4

        elif self.step == 4:
            if self.current_state.armed:
                self.get_logger().info('Sending disarm command...')
                req = CommandBool.Request()
                req.value = False
                self.arm_cli.call_async(req)
            else:
                self.get_logger().info('Disarmed confirmed. Test complete.')
                self.timer.cancel()

    def arm_response_cb(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().error(
                    f'Arming rejected by FCU (result={res.result}). '
                    'Check GPS lock, safety switch, or pre-arm checks. Retrying...'
                )
                self.step = 1
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.step = 1


def main(args=None):
    rclpy.init(args=args)
    node = ArmTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

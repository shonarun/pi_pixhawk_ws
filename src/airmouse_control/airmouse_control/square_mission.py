import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class SquareMission(Node):
    def __init__(self):
        super().__init__('square_mission')

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
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.pose_received = False

        self.target_alt = 2.0
        self.side_length = 2.0
        self.acceptance_radius = 0.25

        self.waypoints = [
            (self.side_length, 0.0, self.target_alt),
            (self.side_length, self.side_length, self.target_alt),
            (0.0, self.side_length, self.target_alt),
            (0.0, 0.0, self.target_alt)
        ]
        self.wp_index = 0
        self.hold_ticks = 0
        self.step = 0
        self.last_req_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.fsm_step)
        self.get_logger().info('SquareMission initialized. Waiting for FCU and GPS lock...')

    def state_cb(self, msg: State):
        self.current_state = msg

    def pose_cb(self, msg: PoseStamped):
        self.pose_received = True
        self.current_pose = msg

    def throttle_ready(self, seconds=1.0):
        now = self.get_clock().now()
        if (now - self.last_req_time).nanoseconds >= seconds * 1e9:
            self.last_req_time = now
            return True
        return False

    def publish_waypoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def distance_to_target(self, tx, ty, tz):
        px = self.current_pose.pose.position.x
        py = self.current_pose.pose.position.y
        pz = self.current_pose.pose.position.z
        return math.sqrt((tx - px) ** 2 + (ty - py) ** 2 + (tz - pz) ** 2)

    def fsm_step(self):
        if not self.current_state.connected or not self.pose_received:
            return

        if self.step == 0:
            if self.current_state.mode != 'GUIDED':
                if self.mode_cli.service_is_ready() and self.throttle_ready(1.0):
                    req = SetMode.Request()
                    req.custom_mode = 'GUIDED'
                    self.mode_cli.call_async(req)
            else:
                self.get_logger().info('GUIDED mode confirmed.')
                self.step = 1

        elif self.step == 1:
            if not self.current_state.armed:
                if self.arm_cli.service_is_ready() and self.throttle_ready(1.0):
                    req = CommandBool.Request()
                    req.value = True
                    self.arm_cli.call_async(req)
            else:
                self.get_logger().info('Armed. Requesting takeoff...')
                self.step = 2

        elif self.step == 2:
            if self.takeoff_cli.service_is_ready() and self.throttle_ready(1.0):
                req = CommandTOL.Request()
                req.altitude = self.target_alt
                future = self.takeoff_cli.call_async(req)
                future.add_done_callback(self.takeoff_response_cb)
                self.step = 3

        elif self.step == 3:
            if self.current_pose.pose.position.z >= self.target_alt * 0.90:
                self.get_logger().info('Takeoff altitude reached. Starting square trajectory...')
                self.step = 4

        elif self.step == 4:
            if self.wp_index < len(self.waypoints):
                tx, ty, tz = self.waypoints[self.wp_index]
                self.publish_waypoint(tx, ty, tz)
                dist = self.distance_to_target(tx, ty, tz)

                if dist <= self.acceptance_radius:
                    self.hold_ticks += 1
                    if self.hold_ticks >= 20:
                        self.get_logger().info(
                            f'Waypoint {self.wp_index + 1} reached: ({tx}, {ty}, {tz})'
                        )
                        self.wp_index += 1
                        self.hold_ticks = 0
                else:
                    self.hold_ticks = 0
            else:
                self.get_logger().info('Square trajectory complete. Switching to LAND...')
                self.step = 5

        elif self.step == 5:
            if self.current_state.mode != 'LAND':
                if self.mode_cli.service_is_ready() and self.throttle_ready(1.0):
                    req = SetMode.Request()
                    req.custom_mode = 'LAND'
                    self.mode_cli.call_async(req)
            else:
                self.get_logger().info('LAND mode active. Waiting for touchdown and disarm...')
                self.step = 6

        elif self.step == 6:
            if not self.current_state.armed:
                self.get_logger().info('Touchdown and disarm confirmed. Mission complete.')
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
    node = SquareMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

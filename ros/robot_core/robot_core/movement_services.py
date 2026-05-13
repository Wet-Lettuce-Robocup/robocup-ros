import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action.server import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_msgs.action import Move


class MovementNode(Node):
    """Class for movement actions."""

    def __init__(self) -> None:
        super().__init__('movement_node')

        self.declare_parameter('wheel_dist', 0.12)
        self.wheel_dist: float = self.get_parameter('wheel_dist').value

        self.current_pose: tuple[float, float, float] | None = None  # (x, y, yaw)
        self.start_pose: tuple[float, float, float] | None = None

        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.action_server = ActionServer(
            self,
            Move,
            'move',
            self.execute_callback,
            callback_group=self.callback_group,
        )

    def odom_callback(self, msg: Odometry) -> None:
        """Update current robot pose from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.current_pose = (x, y, yaw)

    async def execute_callback(self, goal_handle) -> Move.Result:
        self.get_logger().info('Executing movement action...')

        request = goal_handle.request

        if self.current_pose is None:
            self.get_logger().error('No odometry data available yet!')
            goal_handle.abort()
            return Move.Result(success=False)

        self.start_pose = self.current_pose
        feedback = Move.Feedback()
        result = Move.Result(success=False)

        rate = self.create_rate(20)

        while rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info('Goal was cancelled')
                self.stop_robot()
                return Move.Result(success=False)

            if self.current_pose is None:
                rate.sleep()
                continue

            # Calculate progress
            dist_traveled, angle_traveled = self._get_progress()

            # Publish feedback
            feedback.distance_travelled = (
                dist_traveled  # assuming your feedback has this
            )
            goal_handle.publish_feedback(feedback)

            # Check if we reached the target (with tolerance)
            if (
                abs(dist_traveled - request.distance) < 0.03
                and abs(angle_traveled - request.angle) < 0.08
            ):  # tolerances in m and rad
                self.get_logger().info('Goal reached successfully!')
                goal_handle.succeed()
                result.success = True
                break

            # Send velocity command
            twist = Twist()
            twist.linear.x = request.vel * 0.6  # simple scaling - can be improved
            twist.angular.z = (
                math.copysign(request.vel * 0.8, request.angle)
                if abs(request.angle) > 0.1
                else 0.0
            )

            self.twist_pub.publish(twist)
            rate.sleep()

        self.stop_robot()
        return result

    def _get_progress(self) -> tuple[float, float]:
        """Calculate distance and angle traveled since start."""
        if not self.start_pose or not self.current_pose:
            return 0.0, 0.0

        sx, sy, syaw = self.start_pose
        cx, cy, cyaw = self.current_pose

        distance = math.hypot(cx - sx, cy - sy)
        angle = cyaw - syaw
        # Normalize angle to [-pi, pi]
        angle = (angle + math.pi) % (2 * math.pi) - math.pi

        return distance, angle

    def stop_robot(self) -> None:
        """Stop the robot immediately."""
        stop_msg = Twist()
        self.twist_pub.publish(stop_msg)
        self.get_logger().info('Robot stopped')


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()

    try:
        executor = MultiThreadedExecutor()

        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# Robot Core
# Copyright (C) 2026  Dry Lettuce
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
    """
    Node for movement actions with distance and turn angle.

    Handles action calls for higher level robot movement, specifically
    targetting a particular movement distance and angle in which to move.
    Accepts distance, angle and velocity values for movement, periodically
    updates with distance moved, and returns success status.

    Example Usage:

    .. code-block:: python

        import math

        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from robot_msgs.action import Move

        class ActionClient(Node):
            def __init__(self):
                super().__init__('my_action_client')
                # Initialize the client
                self._action_client = ActionClient(self, Move, 'move')

            def send_goal(self, order):
                goal_msg = Move.Goal()
                goal_msg.distance = 1
                goal_msg.angle = math.pi / 2
                goal_msg.vel = 0.1

                # Wait for the server to spin up
                self._action_client.wait_for_server()

                # Send goal asynchronously and hook up the response callback
                self._send_goal_future = self._action_client.send_goal_async(
                    goal_msg,
                    feedback_callback=self.feedback_callback
                )
                self._send_goal_future.add_done_callback(self.goal_response_callback)

            def goal_response_callback(self, future):
                goal_handle = future.result()

                if not goal_handle.accepted:
                    return

                self.get_logger().info('Goal accepted')
                # Request the final result
                self._get_result_future = goal_handle.get_result_async()
                self._get_result_future.add_done_callback(self.get_result_callback)

            def get_result_callback(self, future):
                result = future.result().result
                self.get_logger().info(f'Result received: {result.success}')

            def feedback_callback(self, feedback_msg):
                feedback = feedback_msg.feedback
                self.get_logger().info(f'Current distance: {feedback.distance_travelled},
                                         Current angle: {feedback.angle_turned}')


    :ivar wheel_dist: Distance between wheels on each side.
    :type wheel_dist: float

    """

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
        """
        Execute movement command.

        First checks if odometry data exists yet, and returns an error if not.
        Then, in a loop, calculates difference in angle between current pose and
        target pose, and moves towards the target. Once the target is reached, sends
        stop command and returns success.

        :param goal_handle: Movement action goal.
        :returns: Result of movement including success.
        :rtype: Move.Result
        """
        self.get_logger().info('Executing movement action...')

        request = goal_handle.request

        if self.current_pose is None:
            self.get_logger().error('No odometry data available yet!')
            goal_handle.abort()
            return Move.Result(success=False)

        self.start_pose = self.current_pose
        feedback = Move.Feedback()
        result = Move.Result(success=False)

        rate = self.create_rate(8)

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

            dist_diff = request.distance - math.copysign(
                dist_traveled, request.distance
            )
            angle_diff = request.angle - angle_traveled

            # Check if we reached the target (with tolerance)
            if abs(dist_diff) < 0.03 and (
                abs(angle_diff) < 0.08 or abs(request.angle) < 0.08
            ):  # tolerances in m and rad
                self.get_logger().info('Goal reached successfully!')
                goal_handle.succeed()
                result.success = True
                break

            linear_vel = (
                math.copysign(request.vel, dist_diff) if abs(dist_diff) > 0.03 else 0.0
            )
            angular_vel = (
                math.copysign(request.vel, angle_diff)
                if abs(angle_diff) > 0.08 and abs(request.angle) > 0.08
                else 0.0
            )

            # Send velocity command
            twist = Twist()
            twist.linear.x = (
                linear_vel  # simple scaling - can be improved
                if abs(linear_vel) > 0.03
                else 0.0
            )
            twist.angular.z = angular_vel

            self.get_logger().info(f'{dist_diff}, {angle_diff}')

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


def main(args=None) -> None:
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

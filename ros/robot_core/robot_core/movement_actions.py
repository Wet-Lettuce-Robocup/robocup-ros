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
from rclpy.task import Future
from rclpy.time import Time
from robot_msgs.action import Move, MoveTime
from robot_msgs.srv import I2CWrite
from std_msgs.msg import Int32, Int8


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
        from robot_msgs.action import MoveTime

        class ActionClient(Node):
            def __init__(self):
                super().__init__('my_action_client')
                # Initialize the client
                self._action_client = ActionClient(self, MoveTime, 'move_time')

            def send_goal(self, order):
                goal_msg = MoveTime.Goal()
                goal_msg.vel = 100.0
                goal_msg.linear_vel = 0.0
                goal_msg.time = 1.0

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
                self.get_logger().info(f'Current time: {feedback.time_elapsed}')


    :ivar wheel_dist: Distance between wheels on each side.
    :type wheel_dist: float

    """

    STM_ADDR = 0x67
    DRIVE_TIME_CMD = 0x04
    MOVING_TIME_STATE = 2

    def __init__(self) -> None:
        super().__init__('movement_node')

        self.declare_parameter('wheel_dist', 0.12)
        self.wheel_dist: float = self.get_parameter('wheel_dist').value

        self.current_pose: tuple[float, float, float] | None = None  # (x, y, yaw)
        self.start_pose: tuple[float, float, float] | None = None

        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.state_sub = self.create_subscription(Int8, 'robot_state', self.state_callback, 10)
        self.move_time_count_sub = self.create_subscription(
            Int32, 'move_time_count', self.move_time_count_callback, 10
        )

        self.write_cli = self.create_client(I2CWrite, 'i2c_write')
        self.move_time_future: Future[I2CWrite.Response] | None = None

        self.robot_state: int = 0
        self.move_time_count: int = 0
        self.last_move_time_count: int = 0

        while not self.write_cli.wait_for_service(timeout_sec=1):
            self.get_logger().info('Waiting for I2C service...')

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.action_server = ActionServer(
            self,
            Move,
            'move',
            self.execute_callback,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            MoveTime,
            'move_time',
            self.time_execute_callback,
            callback_group=self.callback_group,
        )

    def odom_callback(self, msg: Odometry) -> None:
        """Update current robot pose from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.current_pose = (x, y, yaw)

    def state_callback(self, msg: Int8) -> None:
        """Detect the current state of the robot."""
        self.robot_state = msg.data

    def move_time_count_callback(self, msg: Int32) -> None:
        """Detect the current move time count of the robot."""
        self.move_time_count = msg.data
        # self.get_logger().info(f'Move time count: {msg.data}')

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
            feedback.distance_travelled = dist_traveled  # assuming your feedback has this
            goal_handle.publish_feedback(feedback)

            dist_diff = request.distance - math.copysign(dist_traveled, request.distance)
            angle_diff = request.angle - angle_traveled

            # Check if we reached the target (with tolerance)
            if abs(dist_diff) < 0.03 and (
                abs(angle_diff) < 0.08 or abs(request.angle) < 0.08
            ):  # tolerances in m and rad
                self.get_logger().info('Goal reached successfully!')
                goal_handle.succeed()
                result.success = True
                break

            linear_vel = math.copysign(request.vel, dist_diff) if abs(dist_diff) > 0.03 else 0.0
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

    def send_time_command(self, vel: float, angular_vel: float, time: float):
        """
        Write linear and angular velocity, and time to stm32 move time command.

        :param vel: Target velocity as duty cycle for motor PWM timers.
        :type vel: float
        :param angular_vel: Target angular velocity as duty cycle for motor PWM timers.
        :type angular_vel: float
        :param time: Time in seconds to move for.
        :type time: float
        """
        cmd: I2CWrite.Request = I2CWrite.Request()
        cmd.device_address = self.STM_ADDR
        cmd.register_address = self.DRIVE_TIME_CMD

        vel_byte = int(vel)
        angular_vel_byte = int(angular_vel)
        time_byte = int(time * 1000)

        cmd.data = [
            vel_byte >> 24 & 0xFF,
            vel_byte >> 16 & 0xFF,
            vel_byte >> 8 & 0xFF,
            vel_byte & 0xFF,
            0,
            0,
            0,
            0,
            angular_vel_byte >> 24 & 0xFF,
            angular_vel_byte >> 16 & 0xFF,
            angular_vel_byte >> 8 & 0xFF,
            angular_vel_byte & 0xFF,
            time_byte >> 24 & 0xFF,
            time_byte >> 16 & 0xFF,
            time_byte >> 8 & 0xFF,
            time_byte & 0xFF,
        ]

        self.move_time_future = self.write_cli.call_async(cmd)
        self.move_time_future.add_done_callback(self.move_time_callback)

    def move_time_callback(self, future):
        """Check if I2C write command succeeded."""
        try:
            response: I2CWrite.Response | None = future.result()

            if response is None:
                raise Exception('No response')

            if not response.success:
                raise Exception(response.message)

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    async def time_execute_callback(self, goal_handle) -> MoveTime.Result:
        """
        Execute time movement command.

        Moves the robot at the specified linear and angular velocity for the
        specified amount of time. Does not use any odometry data.

        :param goal_handle: Time movement action goal.
        :returns: Result of time movement including success.
        :rtype: MoveTime.Result
        """
        self.get_logger().info('Executing movement action...')

        request = goal_handle.request

        feedback = MoveTime.Feedback()
        result = MoveTime.Result(success=False)

        self.last_move_time_count = self.move_time_count
        self.send_time_command(request.vel, request.angular_vel, request.time)

        start_time: Time = self.get_clock().now()

        rate = self.create_rate(10)

        while rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info('Goal was cancelled')
                self.stop_robot()
                return MoveTime.Result(success=False)

            # Calculate progress
            current_time: Time = self.get_clock().now()
            elapsed_time: float = (current_time - start_time).nanoseconds * 1e-9

            # Publish feedback
            feedback.time_elapsed = elapsed_time
            goal_handle.publish_feedback(feedback)

            time_left: float = request.time - elapsed_time

            if time_left <= -1 and self.robot_state != self.MOVING_TIME_STATE:
                if self.last_move_time_count == self.move_time_count:
                    self.get_logger().error(
                        "Move time command doesn't seem to have been executed. Attempting again..."
                    )
                    self.send_time_command(request.vel, request.angular_vel, request.time)
                    start_time = self.get_clock().now()
                    elapsed_time = 0
                    continue

                self.get_logger().info('Goal reached successfully!')
                goal_handle.succeed()
                result.success = True
                break

            rate.sleep()

        self.stop_robot()
        return result

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

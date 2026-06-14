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
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from robot_msgs.srv import I2CWrite


class TwistSubscriber(Node):
    """
    Node for listening to velocity commands and sending them to STM32 over I2C.

    .. note::

        Only linear X and angular Z velocities are used.

    :cvar STM_ADDR: I2C address of STM32 MCU.
    :type STM_ADDR: int
    :cvar DRIVE_REQUEST: I2C command to drive motors.
    :type DRIVE_REQUEST: int
    :cvar STOP_REQUEST: I2C command to stop motors.
    :type STOP_REQUEST: int

    :ivar wheel_dist: Distance between left and right wheels in meters.
    :type wheel_dist: float
    :ivar counts_per_revolution: Number of encoder counts per revolution of
        each wheel.
    :type counts_per_revolution: float
    :ivar wheel_radius: Radius of wheels in meters.
    :type wheel_radius: float
    :ivar max_counts_per_second: Maximum motor speed in counts per second.
    :type max_counts_per_second: float

    :ivar speed_mult: Multiplier to convert speed from revolutions per second to
        encoder counts per second.
    :type speed_mult: float

    """

    STM_ADDR: int = 0x67
    DRIVE_REQUEST: int = 0x01
    STOP_REQUEST: int = 0x02

    def __init__(self) -> None:
        super().__init__('twist_subscriber')

        self.declare_parameter('wheel_dist', 0.175)
        self.declare_parameter('counts_per_revolution', 480.0)
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('max_counts_per_second', 900.0)

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10
        )

        self.cli = self.create_client(I2CWrite, 'i2c_write')
        self.future: Future[I2CWrite.Response] | None = None

        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().info('Waiting for I2C service...')

        self.wheel_dist = self.get_parameter('wheel_dist').value
        self.counts_per_revolution = self.get_parameter('counts_per_revolution').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_counts_per_second = self.get_parameter('max_counts_per_second').value

        self.speed_mult = self.counts_per_revolution / (self.wheel_radius * 2 * math.pi)

        self.get_logger().info('Twist subscriber node started!')

    def stop(self) -> None:
        """Stop moving all motors."""
        request: I2CWrite.Request = I2CWrite.Request()

        request.device_address = self.STM_ADDR
        request.register_address = self.STOP_REQUEST
        request.data = []

        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.i2c_callback)

    def twist_callback(self, msg: Twist) -> None:
        """
        Handle velocity commands.

        Uses linear x velocity and angular z velocity to calculate required velocity for
        each motor, and converts into encoder counts per second, then sends the command
        over I2C to the STM32.

        :param msg: Twist velocity command.
        :type msg: Twist
        """
        linear_x = int(msg.linear.x * self.speed_mult)
        angular_z = int(msg.angular.z * self.speed_mult)

        def fitted(a):
            return min(max(a, -self.max_counts_per_second), self.max_counts_per_second)

        linear_x: int = int(fitted(linear_x))
        angular_z: int = int(fitted(angular_z))

        if angular_z < -60:
            angular_z = -60

        # self.get_logger().info(f'{linear_x}, {angular_z}')

        request: I2CWrite.Request = I2CWrite.Request()

        request.device_address = self.STM_ADDR
        request.register_address = self.DRIVE_REQUEST
        request.data = [
            linear_x >> 24 & 0xFF,
            linear_x >> 16 & 0xFF,
            linear_x >> 8 & 0xFF,
            linear_x & 0xFF,
            0,
            0,
            0,
            0,
            angular_z >> 24 & 0xFF,
            angular_z >> 16 & 0xFF,
            angular_z >> 8 & 0xFF,
            angular_z & 0xFF,
        ]

        # self.get_logger().info(f'{linear_x}, {angular_z}')

        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.i2c_callback)

    def i2c_callback(self, future: Future[I2CWrite.Response]) -> None:
        """Handle I2C response."""
        try:
            response: I2CWrite.Response | None = future.result()

            if response is None:
                raise Exception('No response')

            if not response.success:
                raise Exception(response.message)

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def __del__(self) -> None:
        """Stop all motors on node exit."""
        self.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    twist_subscriber_node = TwistSubscriber()
    rclpy.spin(twist_subscriber_node)
    twist_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

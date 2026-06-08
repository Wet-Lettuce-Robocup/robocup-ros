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

from gpiozero import OutputDevice
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from robot_msgs.srv import I2CWrite
from std_msgs.msg import Float32


class ServoController(Node):
    """
    Node providing topics to allow for servos to be controlled over I2C.

    Handles controlling a servo through sending commands to STM32 over I2C, and enabling
    servos through a GPIO pin connected to a mosfet in series with the servo power.

    .. note::

        All angles are in radians.

    :ivar listen_topic: The topic subscribed to for angle commands.
    :type listen_topic: str
    :ivar servo_id: The ID of the servo on the STM32 to be controlled.
    :type servo_id: int
    :ivar i2c_address: The I2C address of the STM32.
    :type i2c_address: int
    :ivar servo_cmd: The command sent to the STM32 to set servos.
    :type servo_cmd: int
    :ivar gpio_pin: Pin for controlling mosfet gate to power the servo.
    :type gpio_pin: int

    """

    def __init__(self) -> None:
        super().__init__('servo_controller')

        self.declare_parameter('listen_topic', '/servo/default')
        self.declare_parameter('servo_id', 0)
        self.declare_parameter('i2c_address', 0x67)
        self.declare_parameter('servo_cmd', 0x10)
        self.declare_parameter('gpio_pin', 1)

        self.listen_topic: str = self.get_parameter('listen_topic').value
        self.servo_id: int = self.get_parameter('servo_id').value & 0xFF
        self.i2c_address: int = self.get_parameter('i2c_address').value & 0xFF
        self.servo_cmd: int = self.get_parameter('servo_cmd').value & 0xFF
        self.gpio_pin: int = self.get_parameter('gpio_pin').value

        self.create_subscription(Float32, self.listen_topic, self.servo_callback, 10)
        self.cli = self.create_client(I2CWrite, 'i2c_write')
        self.future: Future[I2CWrite.Response] | None = None

        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().info('Waiting for I2C service...')

        self.gpio_device = OutputDevice(
            self.gpio_pin, active_high=True, initial_value=False
        )

    @staticmethod
    def rads_to_degrees(angle: float) -> float:
        """
        Convert radians to degrees.

        :param angle: Angle in radians.
        :type angle: float

        :returns: Angle in degrees.
        :rtype: float
        """
        return angle * 180 / math.pi

    def servo_callback(self, msg: Float32) -> None:
        """
        Set servo position.

        Receives servo position in radians, and converts it to degrees
        to send to the STM32 over I2C.

        :param msg: Angle to set servo to in radians.
        :type msg: Float32
        """
        degrees = int(self.rads_to_degrees(msg.data))
        degrees = min(max(degrees, 0), 180) & 0xFF

        request = I2CWrite.Request()

        request.device_address = self.i2c_address
        request.register_address = self.servo_cmd
        request.data = [self.servo_id, degrees]

        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.i2c_callback)

        self.gpio_device.on()

    def i2c_callback(self, future: Future[I2CWrite.Response]) -> None:
        """Check if the I2C command was successful."""
        try:
            response: I2CWrite.Response | None = future.result()

            if response is None:
                raise Exception('No response')

            if not response.success:
                raise Exception(response.message)

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def cleanup(self) -> None:
        """Turn off servo on node exit."""
        self.gpio_device.off()


def main(args=None):
    rclpy.init(args=args)
    servo_controller_node = ServoController()
    rclpy.spin(servo_controller_node)
    servo_controller_node.cleanup()
    servo_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_msgs.srv import I2CWrite, ServoCommand


class ServoController(Node):
    """
    Node providing services to allow for servos to be controlled over I2C.

    Handles controlling a servo through sending commands to STM32 over I2C, and enabling
    servos through a GPIO pin connected to a mosfet in series with the servo power.

    .. note::

        All angles are in radians.

    :ivar service_name: The service subscribed to for angle commands.
    :type service_name: str
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

        self.declare_parameter('service_name', '/servo/default')
        self.declare_parameter('servo_id', 0)
        self.declare_parameter('i2c_address', 0x67)
        self.declare_parameter('servo_cmd', 0x10)
        self.declare_parameter('gpio_pin', 1)

        self.service_name: str = self.get_parameter('service_name').value
        self.servo_id: int = self.get_parameter('servo_id').value & 0xFF
        self.i2c_address: int = self.get_parameter('i2c_address').value & 0xFF
        self.servo_cmd: int = self.get_parameter('servo_cmd').value & 0xFF
        self.gpio_pin: int = self.get_parameter('gpio_pin').value

        self.callback_group = ReentrantCallbackGroup()

        self.servo_srv = self.create_service(
            ServoCommand,
            self.service_name,
            self.servo_callback,
            callback_group=self.callback_group,
        )
        self.cli = self.create_client(
            I2CWrite,
            'i2c_write',
            callback_group=self.callback_group,
        )

        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().info('Waiting for I2C service...')

        self.gpio_device = OutputDevice(self.gpio_pin, active_high=True, initial_value=False)

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

    async def servo_callback(
        self, request: ServoCommand.Request, response: ServoCommand.Response
    ) -> ServoCommand.Response:
        """
        Set servo position.

        Receives servo position in radians, and converts it to degrees
        to send to the STM32 over I2C.

        :param msg: Angle to set servo to in radians.
        :type msg: Float32
        """
        degrees = int(self.rads_to_degrees(request.angle))
        degrees = min(max(degrees, 0), 180) & 0xFF

        i2c_request = I2CWrite.Request()

        i2c_request.device_address = self.i2c_address
        i2c_request.register_address = self.servo_cmd
        i2c_request.data = [self.servo_id, degrees]

        self.get_logger().info(f'Servo {self.service_name} called with angle {degrees}')

        try:
            self.gpio_device.on()
            future = self.cli.call_async(i2c_request)
            i2c_response = await future

            if i2c_response is None:
                response.success = False
                response.message = 'No response from I2C service'
                return response

            if not i2c_response.success:
                response.success = False
                response.message = i2c_response.message
                return response

            response.success = True
            response.message = ''

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f'Servo command failed: {e}')

        return response

    def cleanup(self) -> None:
        """Turn off servo on node exit."""
        self.gpio_device.off()


def main(args=None):
    rclpy.init(args=args)
    servo_controller_node = ServoController()

    executor = MultiThreadedExecutor()
    rclpy.spin(servo_controller_node, executor=executor)

    servo_controller_node.cleanup()
    servo_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

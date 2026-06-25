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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class LEDController(Node):
    """
    Node for controlling fan with ros2.

    - Subscribed to fan/target_speed (Int32, 0-100%)
    - RPM is sampled over 3 seconds and published as percentage to fan/speed (Int32, 0-100%)
    """

    PWM_CHANNEL = 0
    PERIOD = 100000

    def __init__(self) -> None:
        super().__init__('front_led_controller')

        self.target_brightness_sub = self.create_subscription(
            # should be int from 0-100 (%)
            Int32,
            'front_led/target_brightness',
            self.target_brightness_callback,
            10,
        )

        # publishers for pwm controller
        self.enable_pub = self.create_publisher(Bool, f'/pwm{self.PWM_CHANNEL}/enable', 10)
        self.period_pub = self.create_publisher(Int32, f'/pwm{self.PWM_CHANNEL}/period', 10)
        self.duty_cycle_pub = self.create_publisher(Int32, f'/pwm{self.PWM_CHANNEL}/duty_cycle', 10)

        self.target_brightness = 0

    def target_brightness_callback(self, msg: Int32) -> None:
        target_brightness = msg.data
        self.get_logger().info(f'Setting brightness to {target_brightness}%')
        self.set_brightness(target_brightness)

    def set_brightness(self, target_brightness: int) -> None:
        if target_brightness < 0 or target_brightness > 100:
            self.get_logger().error('Target brightness must be between 0 and 100!')
            return

        self.target_brightness = target_brightness

        if target_brightness == 0:
            self.enable_pub.publish(Bool(data=False))
            return

        self.period_pub.publish(Int32(data=self.PERIOD))

        # convert percentage to duty cycle (0-100000) for 100kHz period
        self.duty_cycle_pub.publish(Int32(data=int((target_brightness / 100) * self.PERIOD)))

        self.enable_pub.publish(Bool(data=True))
        return


def main(args=None):
    rclpy.init(args=args)
    node = LEDController()
    rclpy.spin(node)
    node.set_brightness(0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

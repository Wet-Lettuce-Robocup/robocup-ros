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

from math import isclose

from gpiozero import DigitalInputDevice
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class FanController(Node):
    """
    Node for controlling fan with ros2.

    - Subscribed to fan/target_speed (Int32, 0-100%)
    - RPM is sampled over 3 seconds and published as percentage to fan/speed (Int32, 0-100%)
    """

    PWM_CHANNEL = 2
    TACH_PIN = 11
    PULSES_PER_REV = 2
    MAX_RPM = 2000  # min 450

    def __init__(self) -> None:
        super().__init__('fan_controller')

        self.target_speed_sub = self.create_subscription(
            # should be int from 0-100 (%)
            Int32,
            'fan/target_speed',
            self.target_rpm_callback,
            10,
        )

        # publishers for pwm controller
        self.enable_pub = self.create_publisher(
            Bool, f'/pwm{self.PWM_CHANNEL}/enable', 10
        )
        self.period_pub = self.create_publisher(
            Int32, f'/pwm{self.PWM_CHANNEL}/period', 10
        )
        self.duty_cycle_pub = self.create_publisher(
            Int32, f'/pwm{self.PWM_CHANNEL}/duty_cycle', 10
        )

        # percentage speed publisher
        self.speed_pub = self.create_publisher(Int32, 'fan/speed', 10)

        # setup tachometer input
        self.tach = DigitalInputDevice(self.TACH_PIN)

        self.target_speed = 0
        self.current_speed = 0
        self.last_set = self.get_clock().now()

        self.count = 0
        self.tach.when_activated = self.tach_interrupt

        # timer to calculate speed every 5 seconds
        self.create_timer(3, self.calculate_speed)

    def target_rpm_callback(self, msg: Int32) -> None:
        target_speed = msg.data
        self.get_logger().info(f'Setting fan target to {target_speed}%')
        self.set_fan_speed(target_speed)

    def set_fan_speed(self, target_speed: int) -> None:
        if target_speed < 0 or target_speed > 100:
            self.get_logger().error('Target speed must be between 0 and 100!')
            return

        self.target_speed = target_speed
        self.last_set = self.get_clock().now()

        if target_speed == 0:
            self.enable_pub.publish(Bool(data=False))
            return

        self.period_pub.publish(Int32(data=100000))

        # convert percentage to duty cycle (0-100000) for 100kHz period
        self.duty_cycle_pub.publish(Int32(data=int((target_speed / 100) * 100000)))

        self.enable_pub.publish(Bool(data=True))
        return

    def tach_interrupt(self):
        self.count += 1

    def calculate_speed(self) -> None:
        freq = self.get_frequency()
        rpm = (freq * 60) / self.PULSES_PER_REV
        time_now = self.get_clock().now()
        dt = (time_now - self.last_set).nanoseconds
        if dt > 5e9 and self.target_speed != 0 and rpm <= 1:
            # if fan is stopped, set target to 0 to prevent excess current draw
            self.set_fan_speed(0)

            self.get_logger().warn('Fan is stalling, disabling fan')
        self.current_speed = int((rpm / self.MAX_RPM) * 100)
        # convert to percentage of max speed (2000 RPM)

        msg = Int32()
        msg.data = int(rpm)
        self.speed_pub.publish(msg)

    def get_frequency(self) -> int:
        hz = self.count / 3.0  # 3s for Hz
        self.count = 0
        return int(hz)

    def check_working(self) -> None:
        # for debugging, check if fan is working by comparing target and current speed
        self.calculate_speed()
        if self.current_speed > 0:
            if isclose(self.current_speed, self.target_speed, abs_tol=10):
                self.get_logger().info(
                    'target is close to current speed, fan is working'
                )
            else:
                self.get_logger().warn('target is not close to current speed')
        else:
            self.get_logger().warn('fan speed is 0')
        self.get_logger().info(
            f'target speed: {self.target_speed}% | current speed: {self.current_speed}%'
        )
        return


def main(args=None):
    rclpy.init(args=args)
    node = FanController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

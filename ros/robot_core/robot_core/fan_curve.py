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

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class FanCurve(Node):
    """
    Node for automatic speed control of robot fan.

    - Publishes to fan/target_speed (Int32, 0-100%) based on temperature of Raspberry Pi.
    - Subscribed to fan/speed (Int32, 0-100%) to ensure that fan is running optimally.
    """

    IDLE_SPEED = 40
    LOW_TEMP, LOW_SPEED = 40, 50
    MED_TEMP, MED_SPEED = 50, 70
    HIGH_TEMP, HIGH_SPEED = 60, 100

    def __init__(self) -> None:
        super().__init__('fan_curve')

        self.target_speed_sub = self.create_publisher(
            # should be int from 0-100 (%)
            Int32,
            'fan/target_speed',
            10,
        )

        self.pi_temp_path = Path('/sys/class/thermal/thermal_zone0/temp')
        self.pi_temp_timer = self.create_timer(10.0, self.pi_temp_timer_callback)

    def pi_temp_timer_callback(self):
        self.pi_temp_c = self._read_pi_temp_c()

        msg = Int32()

        if self.pi_temp_c > self.HIGH_TEMP:
            msg.data = self.HIGH_SPEED
        if self.pi_temp_c > self.MED_TEMP:
            msg.data = self.MED_SPEED
        if self.pi_temp_c > self.LOW_TEMP:
            msg.data = self.LOW_SPEED
        else:
            msg.data = self.IDLE_SPEED

        self.target_speed_pub.publish(msg)

    def _read_pi_temp_c(self) -> float | None:
        try:
            raw = self.pi_temp_path.read_text(encoding='utf-8').strip()
            return float(raw) / 1000.0
        except (OSError, ValueError):
            return None


def main(args=None):
    rclpy.init(args=args)
    node = FanCurve()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

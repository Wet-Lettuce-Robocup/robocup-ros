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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class ClawServices(Node):
    """
    Node to handle claw commands.

    Publish to /claw_command: e.g. claw_open, claw_close, lift_up, lift_down, gate_open, gate_close
    """

    LIFT_UP_ANGLE = 0.0
    LIFT_DOWN_ANGLE = math.pi / 2
    CLAW_OPEN_ANGLE = 0.0
    CLAW_CLOSED_ANGLE = math.pi / 2
    GATE_OPEN_ANGLE = 0.0
    GATE_CLOSED_ANGLE = math.pi / 2

    def __init__(self):
        super().__init__('claw_services')

        # Subscribes to claw commands
        self.commands = self.create_subscription(
            String, '/claw_command', self.claw_command_callback, 10
        )

        # Publish to servo controllers
        self.grab_pub = self.create_publisher(Float32, '/servo/grab', 10)
        self.lift_pub = self.create_publisher(Float32, '/servo/lift', 10)
        self.gate_pub = self.create_publisher(Float32, '/servo/tray_release', 10)

    def claw_command_callback(self, msg: String) -> None:
        if msg.data == 'lift_up':
            self.get_logger().info('Lifting claw up')
            self.lift_pub.publish(Float32(data=self.LIFT_UP_ANGLE))
        elif msg.data == 'lift_down':
            self.get_logger().info('Lifting claw down')
            self.lift_pub.publish(Float32(data=self.LIFT_DOWN_ANGLE))
        elif msg.data == 'claw_open':
            self.get_logger().info('Opening claw')
            self.grab_pub.publish(Float32(data=self.CLAW_OPEN_ANGLE))
        elif msg.data == 'claw_close':
            self.get_logger().info('Closing claw')
            self.grab_pub.publish(Float32(data=self.CLAW_CLOSED_ANGLE))
        elif msg.data == 'gate_open':
            self.get_logger().info('Opening gate')
            self.gate_pub.publish(Float32(data=self.GATE_OPEN_ANGLE))
        elif msg.data == 'gate_close':
            self.get_logger().info('Closing gate')
            self.gate_pub.publish(Float32(data=self.GATE_CLOSED_ANGLE))
        else:
            self.get_logger().warn(f'Unknown claw command: {msg.data}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClawServices()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

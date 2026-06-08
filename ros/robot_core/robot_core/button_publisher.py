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

from gpiozero import Button
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ButtonPublisher(Node):
    """Node for reading button presses and publishing them to ROS2 topics."""

    def __init__(self) -> None:
        super().__init__('button_publisher')

        self.declare_parameter('gpio_pin', 6)
        self.declare_parameter('publish_topic', '/button')
        self.declare_parameter('pull_up', True)

        self.gpio_pin: int = self.get_parameter('gpio_pin').value

        self.publish_topic: str = self.get_parameter('publish_topic').value

        self.pull_up: bool = self.get_parameter('pull_up').value

        self.pub = self.create_publisher(Bool, self.publish_topic, 10)

        self.button = Button(self.gpio_pin, pull_up=self.pull_up)
        self.button.when_activated = self.on_press
        self.button.when_deactivated = self.on_release

    def on_press(self) -> None:
        """Publish that button has been pressed."""
        msg: Bool = Bool()

        msg.data = True

        self.pub.publish(msg)

    def on_release(self) -> None:
        """Publish that button has been released."""
        msg: Bool = Bool()

        msg.data = False

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

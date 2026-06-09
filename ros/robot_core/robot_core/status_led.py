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

import numpy as np
import rclpy
from rclpy.node import Node
from robot_msgs.msg import LEDCommand
from rpi5_ws2812.ws2812 import Color, WS2812SpiDriver
from std_msgs.msg import ColorRGBA


class StatusLED(Node):
    """
    Node for controlling WS2812 leds over SPI interface.

    Uses rpi5_ws2812 library which manipulates the SPI data lines to send RGB data
    to WS2812 leds.

    :ivar spi_bus: The SPI bus to use.
    :type spi_bus: int
    :ivar spi_device: The SPI device to use.
    :type spi_device: int
    :ivar led_count: The number of connected LEDs.
    :type led_count: int
    """

    def __init__(self) -> None:
        super().__init__('status_led')

        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('led_count', 3)

        self.spi_bus: int = self.get_parameter('spi_bus').value
        self.spi_device: int = self.get_parameter('spi_device').value
        self.led_count: int = self.get_parameter('led_count').value

        self.command_sub = self.create_subscription(
            LEDCommand, 'led_command', self.command_callback, 10
        )

        self.led_strip = WS2812SpiDriver(
            self.spi_bus, self.spi_device, self.led_count
        ).get_strip()
        self.led_strip.clear()

    @staticmethod
    def ColorRGBA_to_Color(original: ColorRGBA) -> Color:
        """
        Convert from ROS2 ColorRGBA type to WS2812 Color type.

        .. note::

            The ROS2 color type has an alpha channel but the WS2812 one
            does not, so all other color channels are multiplied by the
            alpha channel.

        :param original: Original color using ROS2 type.
        :type original: ColorRGBA

        :returns: Color as WS2812 Color type.
        :rtype: Color
        """
        color: Color = Color(
            r=np.uint8(original.r * original.a * 255),
            g=np.uint8(original.g * original.a * 255),
            b=np.uint8(original.b * original.a * 255),
        )

        return color

    def command_callback(self, msg: LEDCommand) -> None:
        """Receive and process color command."""
        index: int = msg.index
        color: ColorRGBA = msg.color

        if index >= self.led_count or index < 0:
            return

        processed_color: Color = self.ColorRGBA_to_Color(color)

        self.led_strip.set_pixel_color(index, processed_color)
        self.led_strip.show()

    def cleanup(self) -> None:
        """Clear the LED strip on exit."""
        self.led_strip.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    status_led_node = StatusLED()
    rclpy.spin(status_led_node)
    status_led_node.cleanup()
    status_led_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

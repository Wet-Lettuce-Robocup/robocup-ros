import numpy as np
import rclpy
from rclpy.node import Node
from robot_msgs.msg import LEDCommand
from rpi5_ws2812.ws2812 import Color, WS2812SpiDriver
from std_msgs.msg import ColorRGBA


class StatusLED(Node):
    """Node for controlling WS2812 leds over SPI interface."""

    def __init__(self):
        super().__init__('status_led')

        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('led_count', 3)

        self.spi_bus: int = (
            self.get_parameter('spi_bus').get_parameter_value().integer_value
        )
        self.spi_device: int = (
            self.get_parameter('spi_device').get_parameter_value().integer_value
        )
        self.led_count: int = (
            self.get_parameter('led_count').get_parameter_value().integer_value
        )

        self.command_sub = self.create_subscription(
            LEDCommand, 'led_command', self.command_callback, 10
        )

        self.led_colors: np.ndarray = np.full(
            self.led_count, Color(0, 0, 0), dtype=Color
        )

        for i in range(self.led_count):
            self.led_colors[i] = Color(0, 0, 0)

        self.led_strip = WS2812SpiDriver(self.spi_bus, self.spi_device, self.led_count)
        self.led_strip.clear()

    @staticmethod
    def ColorRGBA_to_Color(original: ColorRGBA) -> Color:
        color: Color = Color(
            r=int(original.r * original.a * 255),
            g=int(original.g * original.a * 255),
            b=int(original.b * original.a * 255),
        )

        return color

    def command_callback(self, msg: LEDCommand):
        index: int = msg.index
        color: ColorRGBA = msg.color

        if index >= self.led_count or index < 0:
            return

        processed_color: Color = self.ColorRGBA_to_Color(color)

        self.led_colors[index] = processed_color

        self.led_strip.write(self.led_colors)

    def cleanup(self):
        self.led_strip.clear()


def main(args=None):
    rclpy.init(args=args)
    status_led_node = StatusLED()
    rclpy.spin(status_led_node)
    status_led_node.cleanup()
    status_led_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

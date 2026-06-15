from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OLEDController(Node):
    """Node for controlling OLED display on robot."""

    def __init__(self):
        super().__init__('oled_controller')

        self.device: ssd1306 | None = None
        self.serial: i2c | None = None

        try:
            self.serial = i2c(port=1, address=0x3C)
            self.device = ssd1306(self.serial, width=128, height=64)
            self.get_logger().info('OLED display initialized!')

        except OSError as e:
            self.get_logger().fatal(f'OLED display not initialized! {e}')

        if self.device is None:
            return

        self.text_sub = self.create_subscription(
            String, 'display_text', self.text_callback, 10
        )

    def update_display(self, text: str):
        with canvas(self.device) as draw:
            draw.text((10, 20), text, fill='white')

    def text_callback(self, msg: String):
        self.update_display(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = OLEDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

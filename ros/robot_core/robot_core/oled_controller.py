from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import ImageFont
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String


class OLEDController(Node):
    """Node for controlling OLED display on robot."""

    def __init__(self):
        super().__init__('oled_controller')

        self.device: ssd1306 | None = None
        self.serial: i2c | None = None

        self.status_value = "-"
        self.error_value = 0
        self.silver_value = 0
        self.black_value = 0

        self.font_large = self._load_font(36)
        self.font_small = self._load_font(12)

        try:
            self.serial = i2c(port=1, address=0x3C)
            self.device = ssd1306(self.serial, width=128, height=64)
            self.get_logger().info('OLED display initialized!')

        except OSError as e:
            self.get_logger().fatal(f'OLED display not initialized! {e}')

        if self.device is None:
            return

        self.status_sub = self.create_subscription(
            String, 'oled_status', self.status_callback, 10
        )
        self.error_sub = self.create_subscription(
            Int32, 'oled_error', self.error_callback, 10
        )
        self.silver_sub = self.create_subscription(
            Int32, 'oled_silver', self.silver_callback, 10
        )
        self.black_sub = self.create_subscription(
            Int32, 'oled_black', self.black_callback, 10
        )

        self.update_display()

    def _load_font(self, size: int) -> ImageFont.ImageFont:
        try:
            return ImageFont.truetype("DejaVuSans.ttf", size)
        except OSError:
            return ImageFont.load_default()

    def update_display(self):
        with canvas(self.device) as draw:
            draw.text((2, 6), self.status_value[:1], font=self.font_large, fill='white')
            draw.text((64, 2), f"Error: {self.error_value}", font=self.font_small, fill='white')
            draw.text((64, 22), f"Silver: {self.silver_value}", font=self.font_small, fill='white')
            draw.text((64, 42), f"Black: {self.black_value}", font=self.font_small, fill='white')

    def status_callback(self, msg: String):
        self.status_value = msg.data.strip() or "-"
        self.update_display()

    def error_callback(self, msg: Int32):
        self.error_value = msg.data
        self.update_display()

    def silver_callback(self, msg: Int32):
        self.silver_value = msg.data
        self.update_display()

    def black_callback(self, msg: Int32):
        self.black_value = msg.data
        self.update_display()


def main(args=None):
    rclpy.init(args=args)
    node = OLEDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
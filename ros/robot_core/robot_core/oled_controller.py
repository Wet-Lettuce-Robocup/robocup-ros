from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import ImageFont
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
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

        self.current_page = 0
        self.console_lines: list[str] = []
        self.max_console_lines = 50

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
        self.rosout_sub = self.create_subscription(
            Log, '/rosout', self.rosout_callback, 50
        )

        self.declare_parameter('page_change', 5.0)
        page_change = float(self.get_parameter('page_change').value)
        self.page_timer = self.create_timer(page_change, self.page_timer_callback)

        self.update_display()

    def _load_font(self, size: int) -> ImageFont.ImageFont:
        try:
            return ImageFont.truetype("DejaVuSans.ttf", size)
        except OSError:
            return ImageFont.load_default()

    def update_display(self):
        with canvas(self.device) as draw:
            if self.current_page == 0:
                draw.text((2, 6), self.status_value[:1], font=self.font_large, fill='white')
                draw.text((64, 2), f"Error: {self.error_value}", font=self.font_small, fill='white')
                draw.text((64, 22), f"Silver: {self.silver_value}", font=self.font_small, fill='white')
                draw.text((64, 42), f"Black: {self.black_value}", font=self.font_small, fill='white')
            else:
                self._draw_console_page(draw)

    def _draw_console_page(self, draw):
        lines = self.console_lines[-4:]
        padded = [""] * max(0, 4 - len(lines)) + lines
        y_positions = [0, 16, 32, 48]
        for line, y in zip(padded, y_positions):
            draw.text((0, y), line, font=self.font_small, fill='white')

    def _truncate_line(self, text: str, max_len: int) -> str:
        return text if len(text) <= max_len else text[: max_len - 1] + '…'

    def page_timer_callback(self):
        self.current_page = 1 - self.current_page
        self.update_display()

    def rosout_callback(self, msg: Log):
        level = self._level_to_letter(msg.level)
        name = msg.name.split('/')[-1]
        line = f"{level} {name}: {msg.msg}"
        self.console_lines.append(self._truncate_line(line, 21))
        if len(self.console_lines) > self.max_console_lines:
            self.console_lines = self.console_lines[-self.max_console_lines :]
        if self.current_page == 1:
            self.update_display()

    def _level_to_letter(self, level: int) -> str:
        if level >= Log.FATAL:
            return 'F'
        if level >= Log.ERROR:
            return 'E'
        if level >= Log.WARN:
            return 'W'
        if level >= Log.INFO:
            return 'I'
        return 'D'

    def status_callback(self, msg: String):
        self.status_value = msg.data.strip() or "-"
        if self.current_page == 0:
            self.update_display()

    def error_callback(self, msg: Int32):
        self.error_value = msg.data
        if self.current_page == 0:
            self.update_display()

    def silver_callback(self, msg: Int32):
        self.silver_value = msg.data
        if self.current_page == 0:
            self.update_display()

    def black_callback(self, msg: Int32):
        self.black_value = msg.data
        if self.current_page == 0:
            self.update_display()


def main(args=None):
    rclpy.init(args=args)
    node = OLEDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
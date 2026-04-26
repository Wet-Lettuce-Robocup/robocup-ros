import math

from smbus import SMBus
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.declare_parameter('listen_topic', '/servo/default')
        self.declare_parameter('servo_id', 0)
        self.declare_parameter('i2c_address', 0x67)

        self.listen_topic: str = (
            self.get_parameter('listen_topic').get_parameter_value().string_value
        )
        self.servo_id: int = (
            self.get_parameter('servo_id').get_parameter_value().integer_value & 0xFF
        )
        self.i2c_address: int = (
            self.get_parameter('servo_id').get_parameter_value().integer_value & 0xFF
        )

        self.create_subscription(Float32, self.listen_topic, self.servo_callback, 10)

        self.bus = SMBus(1)

    @staticmethod
    def rads_to_degrees(angle: float):
        return angle * 180 / math.pi

    def servo_callback(self, msg: Float32) -> None:
        degrees = int(self.rads_to_degrees(msg.data))
        degrees = min(max(degrees, 0), 180) & 0xFF

        self.bus.write_i2c_block_data(self.i2c_address, self.servo_id, [degrees])


def main(args=None):
    rclpy.init(args=args)
    servo_controller_node = ServoController()
    rclpy.spin(servo_controller_node)
    servo_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

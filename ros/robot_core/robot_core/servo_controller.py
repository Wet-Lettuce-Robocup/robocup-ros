import math

from gpiozero import OutputDevice
import rclpy
from rclpy.node import Node
from smbus import SMBus
from std_msgs.msg import Float32


class ServoController(Node):
    """
    Node providing topics to allow for servos to be controlled over i2c.

    Attributes
    ----------
        listen_topic (str): The topic subscribed to for angle commands. All angles are in radians.
        servo_id (int): The ID of the servo on the STM32 to be controlled.
        i2c_address (int): The I2C address of the STM32.
        servo_cmd (int): The command sent to the STM32 to set servos.

    """

    def __init__(self):
        super().__init__('servo_controller')

        self.declare_parameter('listen_topic', '/servo/default')
        self.declare_parameter('servo_id', 0)
        self.declare_parameter('i2c_address', 0x67)
        self.declare_parameter('servo_cmd', 0x10)
        self.declare_parameter('gpio_pin', 1)

        self.listen_topic: str = (
            self.get_parameter('listen_topic').get_parameter_value().string_value
        )
        self.servo_id: int = (
            self.get_parameter('servo_id').get_parameter_value().integer_value & 0xFF
        )
        self.i2c_address: int = (
            self.get_parameter('i2c_address').get_parameter_value().integer_value & 0xFF
        )
        self.servo_cmd: int = (
            self.get_parameter('servo_cmd').get_parameter_value().integer_value & 0xFF
        )
        self.gpio_pin: int = (
            self.get_parameter('gpio_pin').get_parameter_value().integer_value
        )

        self.create_subscription(Float32, self.listen_topic, self.servo_callback, 10)

        self.bus = SMBus(1)
        self.gpio_device = OutputDevice(
            self.gpio_pin, active_high=True, initial_value=False
        )

    @staticmethod
    def rads_to_degrees(angle: float):
        return angle * 180 / math.pi

    def servo_callback(self, msg: Float32) -> None:
        degrees = int(self.rads_to_degrees(msg.data))
        degrees = min(max(degrees, 0), 180) & 0xFF

        self.bus.write_i2c_block_data(
            self.i2c_address, self.servo_cmd, [self.servo_id, degrees]
        )

        self.gpio_device.on()

    def cleanup(self):
        self.gpio_device.off()


def main(args=None):
    rclpy.init(args=args)
    servo_controller_node = ServoController()
    rclpy.spin(servo_controller_node)
    servo_controller_node.cleanup()
    servo_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

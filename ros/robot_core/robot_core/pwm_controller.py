import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class PWMController(Node):
    """Node for controlling pwm channels through sysfs."""

    def __init__(self):
        super().__init__('pwm_controller')

        self.declare_parameter('pwm_chip', 0)
        self.declare_parameter('pwm_channel', 0)
        self.declare_parameter('subscribe_topic', 'pwm0')

        self.pwm_chip: int = (
            self.get_parameter('pwm_chip').get_parameter_value().integer_value
        )
        self.pwm_channel: int = (
            self.get_parameter('pwm_channel').get_parameter_value().integer_value
        )
        self.subscribe_topic: str = (
            self.get_parameter('subscribe_topic').get_parameter_value().string_value
        )

        self.chip_path = os.path.join('/sys/class/pwm', f'pwmchip{self.pwm_chip}')
        self.channel_path = os.path.join(self.chip_path, f'pwm{self.pwm_channel}')

        if os.path.isdir(self.channel_path):
            with open(os.path.join(self.channel_path, 'enable'), 'w') as f:
                f.write('0')

        else:
            with open(os.path.join(self.chip_path, 'export'), 'w') as f:
                f.write(str(self.pwm_channel))

        self.enable_sub = self.create_subscription(
            Bool, f'{self.subscribe_topic}/enable', self.enable_callback, 10
        )
        self.period_sub = self.create_subscription(
            Int32, f'{self.subscribe_topic}/period', self.period_callback, 10
        )
        self.duty_cycle_sub = self.create_subscription(
            Int32, f'{self.subscribe_topic}/duty_cycle', self.duty_cycle_callback, 10
        )

        self.period = 0

    def enable_callback(self, msg: Bool) -> None:
        with open(os.path.join(self.channel_path, 'enable'), 'w') as f:
            f.write(str(int(msg.data)))

    def period_callback(self, msg: Int32) -> None:
        with open(os.path.join(self.channel_path, 'period'), 'w') as f:
            f.write(str(msg.data))
            self.period = msg.data

    def duty_cycle_callback(self, msg: Int32) -> None:
        duty_cycle = msg.data
        if duty_cycle > self.period:
            duty_cycle = self.period

        with open(os.path.join(self.channel_path, 'duty_cycle'), 'w') as f:
            f.write(str(duty_cycle))


def main(args=None):
    rclpy.init(args=args)
    node = PWMController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

from gpiozero import DigitalInputDevice
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class FanController(Node):
    """Node for controlling fan with ros2."""

    PWM_CHANNEL = 0
    TACH_PIN = 11
    PULSES_PER_REV = 2

    def __init__(self) -> None:
        super().__init__('fan_controller')

        self.enable_pub = self.create_publisher(
            Bool, f'/pwm{self.PWM_CHANNEL}/enable', 10
        )
        self.period_pub = self.create_publisher(
            Int32, f'/pwm{self.PWM_CHANNEL}/period', 10
        )
        self.duty_cycle_pub = self.create_publisher(
            Int32, f'/pwm{self.PWM_CHANNEL}/duty_cycle', 10
        )

        self.tach = DigitalInputDevice(self.TACH_PIN)

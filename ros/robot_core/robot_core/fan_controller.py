from math import isclose

from gpiozero import DigitalInputDevice
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class FanController(Node):
    
    """Node for controlling fan with ros2.
    - Subscribed to fan/target_speed (Int32, 0-100%)
    - RPM is sampled over 3 seconds and published as percentage to fan/speed (Int32, 0-100%)
    """

    PWM_CHANNEL = 0
    TACH_PIN = 11
    PULSES_PER_REV = 2
    MAX_RPM = 2000

    def __init__(self) -> None:
        super().__init__('fan_controller')

        self.target_speed_sub = self.create_subscription(  # should be int from 0-100 (%)
            Int32, 'fan/target_speed', self.target_rpm_callback, 10
        )

        # publishers for pwm controller
        self.enable_pub = self.create_publisher(
            Bool, f'/pwm{self.PWM_CHANNEL}/enable', 10
        )
        self.period_pub = self.create_publisher(
            Int32, f'/pwm{self.PWM_CHANNEL}/period', 10
        )
        self.duty_cycle_pub = self.create_publisher(
            Int32, f'/pwm{self.PWM_CHANNEL}/duty_cycle', 10
        )

        # percentage speed publisher
        self.speed_pub = self.create_publisher(Int32, 'fan/speed', 10)

        # setup tachometer input
        self.tach = DigitalInputDevice(self.TACH_PIN)

        self.target_speed = 0
        self.current_speed = 0

        # timer to calculate speed every second
        self.create_timer(1.0, self.calculate_speed)

    def target_rpm_callback(self, msg: Int32) -> None:
        target_speed = msg.data
        self.get_logger().info(f'Setting fan target to {target_speed}%')
        self.set_fan_speed(target_speed)

    def set_fan_speed(self, target_speed: int) -> None:
        if target_speed < 0 or target_speed > 100:
            self.get_logger().error('Target speed must be between 0 and 100!')
            return

        self.target_speed = target_speed

        if target_speed == 0:
            self.enable_pub.publish(Bool(data=False))
            return

        self.enable_pub.publish(Bool(data=True))
        self.period_pub.publish(Int32(data=999))
        self.duty_cycle_pub.publish(Int32(data=int(target_speed * 10000)))
        return

    def calculate_speed(self) -> None:
        freq = self.get_frequency()
        rpm = (freq * 60) / self.PULSES_PER_REV
        self.current_speed = int((rpm / self.MAX_RPM) * 100)  # convert to percentage of max speed (2000 RPM)

    def get_frequency(self) -> int:
        count = 0
        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time).seconds < 3.0:
            if self.tach.is_active:  # wait for the signal to go high
                count += 1
                while self.tach.is_active:  # wait for the signal to go low
                    pass
                
        hz = count / 3.0  # /3s for Hz
        return hz
    
    def check_working(self) -> None:  
        # for debugging, check if fan is working by comparing target and current speed
        self.calculate_speed()
        if self.current_speed > 0:
            if isclose(self.current_speed, self.target_speed, abs_tol=10):
                self.get_logger().info('target is close to current speed, fan is working')
            else:
                self.get_logger().warn('target is not close to current speed')
        else:
            self.get_logger().warn('fan speed is 0')
        self.get_logger().info(f'target speed: {self.target_speed}% | current speed: {self.current_speed}%')
        return
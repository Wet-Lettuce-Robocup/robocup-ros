import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from smbus import SMBus


class TwistSubscriber(Node):
    DRIVE_REQUEST: bytes = b"\x01"
    STOP_REQUEST: bytes = b"\x02"

    def __init__(self):
        super().__init__("twist_subscriber")

        self.declare_parameter("wheel_dist", 0.175)
        self.declare_parameter("counts_per_revolution", 480.0)
        self.declare_parameter("wheel_radius", 0.04)
        self.declare_parameter("max_counts_per_second", 900.0)

        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.twist_callback, 10
        )

        self.bus = SMBus(1)
        self.addr = 0x67

        self.wheel_dist = self.get_parameter("wheel_dist").value
        self.counts_per_revolution = self.get_parameter("counts_per_revolution").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.max_counts_per_second = self.get_parameter("max_counts_per_second").value

        self.speed_mult = self.counts_per_revolution / (self.wheel_radius * 2 * math.pi)

        self.get_logger().info("Twist subscriber node started!")

    def stop(self):
        self.bus.write_i2c_block_data(self.addr, self.STOP_REQUEST)

    def twist_callback(self, msg):
        linear_x = int(
            (msg.linear.x * self.speed_mult * 127) / self.max_counts_per_second
        )
        angular_z = int(
            (msg.angular.z * self.speed_mult * 127) / self.max_counts_per_second
        )

        def fitted(a):
            return min(max(a, -127), 127)

        linear_x = fitted(linear_x)
        angular_z = fitted(angular_z)

        if angular_z < -60:
            angular_z = -60

        linear_x_byte = int(linear_x) & 0xFF
        angular_z_byte = int(angular_z) & 0xFF

        self.get_logger().info(f"Bytes: {linear_x_byte}, {angular_z_byte}")

        self.bus.write_i2c_block_data(
            self.addr, self.DRIVE_REQUEST, [linear_x_byte, 0, angular_z_byte]
        )

    def __del__(self):
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    twist_subscriber_node = TwistSubscriber()
    rclpy.spin(twist_subscriber_node)
    twist_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

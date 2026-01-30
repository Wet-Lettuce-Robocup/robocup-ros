import math

# from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import serial


class OdomPublisher(Node):

    START_FLAG: bytes = b'\xA5'

    ENCODER_REQUEST: bytes = b'\x40'
    ENCODER_RESPONSE: bytes = b'\x41'

    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz
        self.start_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0  # m/s
        self.vth = 0.0  # rad/s

        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        self.get_logger().info(f"Serial connected: {'y' if self.ser.is_open else 'n'}")
        self.get_logger().info(str(self.ser))

        self.wheel_dist = 0.185  # m
        self.counts_per_revolution = 480
        self.wheel_radius = 0.04
        self.wheel_circumefrence = math.pi * (self.wheel_radius ** 2)

    def get_encoders(self):
        self.ser.write(self.START_FLAG + self.ENCODER_REQUEST)
        msg = self.ser.read(18)

        if len(msg) < 18:
            self.get_logger().warn(f'Wrong Size ({len(msg)})')
            return

        if msg[0:1] != self.START_FLAG:
            self.get_logger().warn(f'Incorrect Start Flag ({msg[0]})')
            return

        if msg[1:2] != self.ENCODER_RESPONSE:
            self.get_logger().warn(f'Incorrect Response Byte ({msg[1]})')
            return

        enc_a = int.from_bytes(msg[2:6], signed=True)
        enc_b = int.from_bytes(msg[6:10], signed=True)

        speed_a = int.from_bytes(msg[10:14], signed=True)
        speed_b = int.from_bytes(msg[14:18], signed=True)

        return enc_a, enc_b, speed_a, speed_b

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = 0.1  # seconds since last update

        enc_data = self.get_encoders()

        if enc_data is None:
            return

        enc_a, enc_b, speed_a, speed_b = enc_data

        angular_vel_mult = 2 * math.pi * self.wheel_radius / self.counts_per_revolution

        vel_a = speed_a * angular_vel_mult
        vel_b = speed_b * angular_vel_mult

        self.vx = (vel_a + vel_b) / 2
        self.vth = (vel_a - vel_b) / self.wheel_dist

        # Update position
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        # Orientation as quaternion
        odom_quat = self.euler_to_quaternion(0, 0, self.th)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set the position
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = Quaternion(
            x=odom_quat[0], y=odom_quat[1], z=odom_quat[2], w=odom_quat[3]
        )

        # Set the velocity
        odom.twist.twist = Twist(
            linear=Vector3(x=self.vx, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.vth)
        )

        self.publisher_.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
            math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
            math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

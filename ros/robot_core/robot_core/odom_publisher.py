import math

from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from smbus import SMBus


class OdomPublisher(Node):
    ENCODER_REQUEST: bytes = b'\x40'
    ENCODER_RESPONSE: bytes = b'\x41'

    def __init__(self):
        super().__init__('odom_publisher')

        self.declare_parameter('wheel_dist', 0.175)
        self.declare_parameter('counts_per_revolution', 480.0)
        self.declare_parameter('wheel_radius', 0.04)

        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz
        self.start_time = self.get_clock().now()

        self.last_enc_fl = 0.0
        self.last_enc_fr = 0.0
        self.last_enc_bl = 0.0
        self.last_enc_br = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0  # m/s
        self.vth = 0.0  # rad/s

        self.addr = 0x67
        self.bus = SMBus(1)

        self.wheel_dist = self.get_parameter('wheel_dist').value
        self.counts_per_revolution = self.get_parameter('counts_per_revolution').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.wheel_circumefrence = math.pi * (self.wheel_radius**2)

    def get_vel(self):
        vel_msg = self.bus.read_i2c_block_data(self.addr, 0x81, 16)

        vel_fl = int.from_bytes(vel_msg[0:4], signed=True)
        vel_fr = int.from_bytes(vel_msg[4:8], signed=True)
        vel_bl = int.from_bytes(vel_msg[8:12], signed=True)
        vel_br = int.from_bytes(vel_msg[12:16], signed=True)

        return vel_fl, vel_fr, vel_bl, vel_br

    def get_encoders(self):
        enc_msg = self.bus.read_i2c_block_data(self.addr, 0x82, 16)

        enc_fl = int.from_bytes(enc_msg[0:4], signed=True)
        enc_fr = int.from_bytes(enc_msg[4:8], signed=True)
        enc_bl = int.from_bytes(enc_msg[8:12], signed=True)
        enc_br = int.from_bytes(enc_msg[12:16], signed=True)

        return enc_fl, enc_fr, enc_bl, enc_br

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = 0.1

        # vel_data = self.get_vel()

        # if vel_data is None:
        #     return

        # vel_fl, vel_fr, vel_bl, vel_br = vel_data

        enc_data = self.get_encoders()

        if enc_data is None:
            return

        enc_fl, enc_fr, enc_bl, enc_br = enc_data

        d_l = (enc_fl - self.last_enc_fl + enc_bl - self.last_enc_bl) / 2
        self.last_enc_fl = enc_fl
        self.last_enc_bl = enc_bl

        d_r = enc_fr - self.last_enc_fr + enc_br - self.last_enc_br
        self.last_enc_fr = enc_fr
        self.last_enc_br = enc_br

        angular_mult = 2 * math.pi * self.wheel_radius / self.counts_per_revolution

        angle_change_l = d_l * angular_mult
        angle_change_r = d_r * angular_mult

        self.dx = (angle_change_l + angle_change_r) / 2
        self.dth = (angle_change_l - angle_change_r) / self.wheel_dist

        # Update position
        self.x += self.dx * math.cos(self.th)
        self.y += self.dx * math.sin(self.th)
        self.th += self.dth

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
            angular=Vector3(x=0.0, y=0.0, z=self.dth / dt),
        )

        self.publisher_.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

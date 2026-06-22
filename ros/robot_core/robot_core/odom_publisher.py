# Robot Core
# Copyright (C) 2026  Dry Lettuce
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import math

from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from robot_msgs.srv import I2CRead


class OdomPublisher(Node):
    """
    Node for reading and publishing odometry data.

    Reads encoder values from STM32 and uses the change in values to
    calculate movement velocity and direction. Publishes this data to
    /odom topic.

    :cvar STM_ADDR: I2C address of STM32 MCU.
    :type STM_ADDR: int
    :cvar VEL_CMD: I2C command to read velocity data from STM32.
    :type VEL_CMD: int
    :cvar VEL_LEN: Length of velocity data in bytes to read over I2C.
    :type VEL_LEN: int
    :cvar ENC_CMD: I2C command to read encoder data from STM32.
    :type ENC_CMD: int
    :cvar ENC_LEN: Length of encoder data in bytes to read over I2C.
    :type ENC_LEN: int

    :ivar wheel_dist: Distance between left and right wheels in meters.
    :type wheel_dist: float
    :ivar counts_per_revolution: Number of encoder counts per revolution of
        each wheel.
    :type counts_per_revolution: float
    :ivar wheel_radius: Radius of wheels in meters.
    :type wheel_radius: float

    :ivar velocities: Velocities of all 4 motors in encoder counts per second.
        Order: front left, front right, back left, back right.
    :type velocities: tuple[int, int, int, int]
    :ivar encoders: Encoder counts for all 4 motors.
        Order: front left, front right, back left, back right.
    :type encoders: tuple[int, int, int, int]

    """

    STM_ADDR = 0x67
    VEL_CMD = 0x81
    VEL_LEN = 16
    ENC_CMD = 0x82
    ENC_LEN = 16

    def __init__(self):
        super().__init__('odom_publisher')

        self.declare_parameter('wheel_dist', 0.175)
        self.declare_parameter('counts_per_revolution', 1000.0)
        self.declare_parameter('wheel_radius', 0.04)

        self.last_enc_fl = 0.0
        self.last_enc_fr = 0.0
        self.last_enc_bl = 0.0
        self.last_enc_br = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0  # m/s
        self.vth = 0.0  # rad/s

        self.velocities: tuple[int, int, int, int] = 0, 0, 0, 0
        self.encoders: tuple[int, int, int, int] = 0, 0, 0, 0

        self.cli = self.create_client(I2CRead, 'i2c_read')
        self.vel_future: Future[I2CRead.Response] | None = None
        self.enc_future: Future[I2CRead.Response] | None = None

        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().info('Waiting for I2C service...')

        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.pub_timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz
        self.start_time = self.get_clock().now()

        self.wheel_dist = self.get_parameter('wheel_dist').value
        self.counts_per_revolution = self.get_parameter('counts_per_revolution').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.wheel_circumefrence = math.pi * (self.wheel_radius**2)

        self.encoders_set: bool = False

    def request_vel(self) -> None:
        """Request velocity data from the STM32 over I2C."""
        request: I2CRead.Request = I2CRead.Request()

        request.device_address = self.STM_ADDR
        request.register_address = self.VEL_CMD
        request.length = self.VEL_LEN

        self.vel_future = self.cli.call_async(request)
        self.vel_future.add_done_callback(self.vel_callback)

    def vel_callback(self, future: Future[I2CRead.Response]) -> None:
        """
        Handle received velocity data.

        Converts received data into integers for all 4 motors and updates
        velocities attribute with new values.
        """
        try:
            response: I2CRead.Response | None = future.result()

            if response is None:
                raise Exception('No response')

            if not response.success:
                raise Exception(response.message)

            vel_msg = response.data

            vel_fl = int.from_bytes(vel_msg[0:4], signed=True)
            vel_fr = int.from_bytes(vel_msg[4:8], signed=True)
            vel_bl = int.from_bytes(vel_msg[8:12], signed=True)
            vel_br = int.from_bytes(vel_msg[12:16], signed=True)

            self.velocities = vel_fl, vel_fr, vel_bl, vel_br

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def request_enc(self) -> None:
        """Request encoder data from the STM32 over I2C."""
        request: I2CRead.Request = I2CRead.Request()

        request.device_address = self.STM_ADDR
        request.register_address = self.ENC_CMD
        request.length = self.ENC_LEN

        self.enc_future = self.cli.call_async(request)
        self.enc_future.add_done_callback(self.enc_callback)

    def enc_callback(self, future: Future[I2CRead.Response]) -> None:
        """
        Handle received encoder data.

        Converts received data into integers for all 4 motors and updates
        encoders attribute with new values.
        """
        try:
            response: I2CRead.Response | None = future.result()

            if response is None:
                raise Exception('No response')

            if not response.success:
                raise Exception(response.message)

            enc_msg = response.data

            enc_fl = int.from_bytes(enc_msg[2:4], signed=True)
            enc_fr = int.from_bytes(enc_msg[6:8], signed=True)
            enc_bl = int.from_bytes(enc_msg[10:12], signed=True)
            enc_br = int.from_bytes(enc_msg[14:16], signed=True)

            self.encoders = enc_fl, enc_fr, enc_bl, enc_br

            if not self.encoders_set:
                (
                    self.last_enc_fl,
                    self.last_enc_fr,
                    self.last_enc_bl,
                    self.last_enc_br,
                ) = self.encoders
                self.encoders_set = True

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def publish_odom(self) -> None:
        """
        Calculate and publish current odometry data.

        Finds average velocity of wheels on each side using encoder data.
        Uses the velocity of each side to calculate both linear and angular velocity.
        Updates current position and angle, and publishes position and velocity to
        odometry topic.
        """
        self.request_enc()

        current_time = self.get_clock().now()
        dt = 0.1

        if not self.encoders_set:
            return

        # vel_data = self.get_vel()

        # if vel_data is None:
        #     return

        # vel_fl, vel_fr, vel_bl, vel_br = vel_data

        enc_fl, enc_fr, enc_bl, enc_br = self.encoders

        # Find average velocity of each side
        d_l = (enc_fl - self.last_enc_fl + enc_bl - self.last_enc_bl) / 2
        self.last_enc_fl = enc_fl
        self.last_enc_bl = enc_bl

        d_r = (enc_fr - self.last_enc_fr + enc_br - self.last_enc_br) / 2
        self.last_enc_fr = enc_fr
        self.last_enc_br = enc_br

        # Convert encoder counts per second to radians per second
        angular_mult = 2 * math.pi * self.wheel_radius / self.counts_per_revolution

        angle_change_l = d_l * angular_mult
        angle_change_r = d_r * angular_mult

        self.vx = (angle_change_l + angle_change_r) / 2
        self.vth = (angle_change_l - angle_change_r) / self.wheel_dist

        # Update position
        self.x += self.vx * math.cos(self.th)
        self.y += self.vx * math.sin(self.th)
        self.th += self.vth

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
            linear=Vector3(x=self.vx / dt, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.vth / dt),
        )

        self.publisher_.publish(odom)

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> list[float]:
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

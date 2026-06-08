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

try:
    import adafruit_vl53l1x
    import board
    import busio

except ImportError:
    from unittest.mock import MagicMock

    adafruit_vl53l1x = MagicMock()
    board = MagicMock()
    busio = MagicMock()

from gpiozero import OutputDevice
import rclpy
from rclpy.node import Node
from robot_msgs.srv import I2CRead, I2CWrite
from smbus2 import SMBus
from std_msgs.msg import Float32, Int32


class I2CBusController(Node):
    """
    Central I2C control node.

    Listens to services to handle I2C read and write commands,
    and periodically reads STM32 MCU for sensor readings and publishes them
    to ROS topics. The main purpose of this node is to prevent I2C collisions
    by routing all I2C requests through one node instead of each individual
    node which requires I2C using SMBUS.

    :cvar STM_ADDR: I2C address of STM32 MCU.
    :type STM_ADDR: int
    :cvar ULTRASONIC_CMD: I2C command to read ultrasonic data from STM32.
    :type ULTRASONIC_CMD: int
    :cvar TEMP_CMD: I2C command to read temperature data from STM32.
    :type TEMP_CMD: int

    :ivar read_srv: Service for reading I2C data.
    :type read_srv: rclpy.service
    :ivar write_srv: Service for writing I2C data.
    :type write_srv: rclpy.service

    :ivar claw_tof_en: Enable pin of claw TOF sensor.
    :type claw_tof_en: OutputDevice
    :ivar right_tof_en: Enable pin of right TOF sensor.
    :type right_tof_en: OutputDevice
    :ivar front_tof_en: Enable pin of front TOF sensor.
    :type right_tof_en: OutputDevice

    :ivar claw_tof_addr: Target I2C address of claw TOF sensor.
    :type claw_tof_addr: int
    :ivar right_tof_addr: Target I2C address of right TOF sensor.
    :type right_tof_addr: int
    :ivar front_tof_addr: Target I2C address of front TOF sensor.
    :type front_tof_addr: int

    :ivar claw_tof_enabled: Status of whether claw TOF sensor is connected and
        successfully started.
    :type claw_tof_enabled: bool
    :ivar right_tof_enabled: Status of whether right TOF sensor is connected and
        successfully started.
    :type right_tof_enabled: bool
    :ivar front_tof_enabled: Status of whether front TOF sensor is connected and
        successfully started.
    :type front_tof_enabled: bool

    """

    STM_ADDR = 0x67
    ULTRASONIC_CMD = 0x83
    TEMP_CMD = 0x84

    def __init__(self) -> None:
        super().__init__('i2c_controller')

        try:
            self.bus = SMBus(1)

        except OSError as e:
            self.get_logger().fatal(f'Failed to initialize I2C device! {e}')
            return

        self.read_srv = self.create_service(I2CRead, 'i2c_read', self.handle_read)
        self.write_srv = self.create_service(I2CWrite, 'i2c_write', self.handle_write)

        self.claw_tof_pub = self.create_publisher(Int32, 'tof/claw', 10)
        self.right_tof_pub = self.create_publisher(Int32, 'tof/right', 10)
        self.front_tof_pub = self.create_publisher(Int32, 'tof/front', 10)
        self.ultrasonic_pub = self.create_publisher(Int32, 'ultrasonic', 10)
        self.stm_temp_pub = self.create_publisher(Float32, 'stm_temp', 10)

        self.claw_tof_en = OutputDevice(20, active_high=True, initial_value=False)
        self.right_tof_en = OutputDevice(19, active_high=True, initial_value=False)
        self.front_tof_en = OutputDevice(7, active_high=True, initial_value=False)

        self.claw_tof_addr = 0x30
        self.right_tof_addr = 0x31
        self.front_tof_addr = 0x32

        self.claw_tof_enabled = False
        self.right_tof_enabled = False
        self.front_tof_enabled = False

        self.init_tof()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def init_tof(self) -> None:
        """
        Attempt to initialize all TOF sensors.

        For each TOF sensor, sets the enable pin high and attempts to
        connect via I2C. If it cannot be connected, it is marked as
        disabled. If it is connected, its I2C address is changed to a
        previously defined value.
        """
        self.adafruit_i2c = busio.I2C(board.SCL, board.SDA)

        try:
            self.claw_tof_en.on()
            self.claw_tof = adafruit_vl53l1x.VL53L1X(self.adafruit_i2c)
            self.claw_tof.set_address(self.claw_tof_addr)
            self.claw_tof.start_ranging()
            self.claw_tof_enabled = True
        except Exception as e:
            self.claw_tof_en.off()
            self.claw_tof_enabled = False
            self.get_logger().error(f'Claw TOF not enabled! {e}')

        try:
            self.right_tof_en.on()
            self.right_tof = adafruit_vl53l1x.VL53L1X(self.adafruit_i2c)
            self.right_tof.set_address(self.right_tof_addr)
            self.right_tof.start_ranging()
            self.right_tof_enabled = True
        except Exception as e:
            self.right_tof_en.off()
            self.right_tof_enabled = False
            self.get_logger().error(f'Right TOF not enabled! {e}')

        try:
            self.front_tof_en.on()
            self.front_tof = adafruit_vl53l1x.VL53L1X(self.adafruit_i2c)
            self.front_tof.set_address(self.front_tof_addr)
            self.front_tof.start_ranging()
            self.front_tof_enabled = True
        except Exception as e:
            self.front_tof_en.off()
            self.front_tof_enabled = False
            self.get_logger().error(f'Front TOF not enabled! {e}')

    def handle_read(
        self, request: I2CRead.Request, response: I2CRead.Response
    ) -> I2CRead.Response:
        """
        Attempt to read data over I2C.

        Connects to the device given by addr, and reads length bytes from
        location cmd. Returns either the received data, or an empty data
        array with success set to False.
        """
        addr = request.device_address
        cmd = request.register_address
        data_len = request.length

        try:
            data = self.bus.read_i2c_block_data(addr, cmd, data_len)

            response.success = True
            response.message = ''
            response.data = data

        except IOError as e:
            response.success = False
            response.message = e
            response.data = []

        return response

    def handle_write(
        self, request: I2CWrite.Request, response: I2CWrite.Response
    ) -> I2CWrite.Response:
        """
        Attempt to write data over I2C.

        Connects to the device given by addr, and writes data to location
        cmd. Returns either success True or success False with an error
        message.
        """
        addr = request.device_address
        cmd = request.register_address
        data = request.data

        try:
            self.bus.write_i2c_block_data(addr, cmd, data)

            response.success = True
            response.message = ''

        except IOError as e:
            response.success = False
            response.message = e

        return response

    def read_ultrasonic(self) -> int | None:
        """Attempt to read ultrasonic sensor data from STM32."""
        try:
            msg = self.bus.read_i2c_block_data(self.STM_ADDR, self.ULTRASONIC_CMD, 4)

            dist: int = int.from_bytes(msg)

            return dist

        except IOError as e:
            self.get_logger().error(f'I2C read failed! {e}')

    def read_temp(self) -> float | None:
        """Attempt to read temperature data from STM32."""
        try:
            msg = self.bus.read_i2c_block_data(self.STM_ADDR, self.TEMP_CMD, 4)

            dist: float = int.from_bytes(msg) / 100

            return dist

        except IOError as e:
            self.get_logger().error(f'I2C read failed! {e}')

    def publish_tof(self) -> None:
        """
        Attempt to read and publish data from all TOF sensors.

        For each sensor, checks if it is enabled. Then, attempts to read the
        distance data over I2C. If data is received, converts to millimetres
        and publishes to ROS2 topic.
        """
        msg = Int32()

        if self.claw_tof_enabled:
            try:
                claw_dist: float | None = self.claw_tof.distance
            except OSError:
                return

            if claw_dist is None:
                msg.data = -1
            else:
                msg.data = int(claw_dist * 10)

            self.claw_tof_pub.publish(msg)

        if self.right_tof_enabled:
            try:
                right_dist: float | None = self.right_tof.distance
            except OSError:
                return

            if right_dist is None:
                msg.data = -1
            else:
                msg.data = int(right_dist * 10)

            self.right_tof_pub.publish(msg)

        if self.front_tof_enabled:
            try:
                front_dist: float | None = self.front_tof.distance
            except OSError:
                return

            if front_dist is None:
                msg.data = -1
            else:
                msg.data = int(front_dist * 10)

            self.front_tof_pub.publish(msg)

    def timer_callback(self) -> None:
        """
        Periodically read sensor data.

        Reads and publishes TOF sensors, ultrasonic sensor,
        and STM32 temperature.
        """
        self.publish_tof()

        msg = Int32()
        ultrasonic_dist: int | None = self.read_ultrasonic()

        if ultrasonic_dist is None:
            return

        msg.data = ultrasonic_dist
        self.ultrasonic_pub.publish(msg)

        temp_msg = Float32()
        temp: float | None = self.read_temp()

        if temp is None:
            return

        temp_msg.data = temp
        self.stm_temp_pub.publish(temp_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = I2CBusController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

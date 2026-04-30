from rclpy.node import Node
from robot_msgs.srv import I2CRead, I2CWrite
from smbus2 import SMBus


class I2CBusController(Node):
    """Class for controlling I2C bus to prevent collisions."""

    def __init__(self):
        super().__init__('i2c_controller')
        self.bus = SMBus(1)

        self.create_service(I2CRead, 'i2c_read', self.handle_read)
        self.create_service(I2CWrite, 'i2c_write', self.handle_write)

    def handle_read(
        self, request: I2CRead.Request, response: I2CRead.Response
    ) -> I2CRead.Response:
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

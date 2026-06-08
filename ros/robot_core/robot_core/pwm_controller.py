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

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class PWMController(Node):
    """
    Node for controlling PWM channels through sysfs interface.

    Listens to period, duty cycle, and enable topics in order to configure the configured
    PWM channel. This allows for nodes to use PWM without interacting with the sysfs
    interface, potentially conflicting with other nodes and creating permission problems,
    or resorting to using software pwm emulation.

    .. warning::

        Period must be set first, then duty cycle, then enable. Failing to use this order
        can result in undefined behaviour and errors.

    :ivar pwm_chip: The PWM chip to use in /sys/class/pwm (eg. '0' for pwmchip0).
    :type pwm_chip: int
    :ivar pwm_channel: The PWM channel to use in the specified chip (eg. '1' for pwm1).
    :type pwm_channel: int
    """

    def __init__(self) -> None:
        super().__init__('pwm_controller')

        self.declare_parameter('pwm_chip', 0)
        self.declare_parameter('pwm_channel', 0)

        self.pwm_chip: int = self.get_parameter('pwm_chip').value
        self.pwm_channel: int = self.get_parameter('pwm_channel').value

        self.chip_path = os.path.join('/sys/class/pwm', f'pwmchip{self.pwm_chip}')
        self.channel_path = os.path.join(self.chip_path, f'pwm{self.pwm_channel}')

        if os.path.isdir(self.channel_path):
            with open(os.path.join(self.channel_path, 'enable'), 'w') as f:
                f.write('0')

        else:
            with open(os.path.join(self.chip_path, 'export'), 'w') as f:
                f.write(str(self.pwm_channel))

        self.enable_on_set_period: bool = False

        self.enable_sub = self.create_subscription(
            Bool, 'enable', self.enable_callback, 10
        )
        self.period_sub = self.create_subscription(
            Int32, 'period', self.period_callback, 10
        )
        self.duty_cycle_sub = self.create_subscription(
            Int32, 'duty_cycle', self.duty_cycle_callback, 10
        )

        self.period = 0

    def enable_callback(self, msg: Bool) -> None:
        """
        Enable or disable the PWM channel.

        First checks if the period has been set. If not, does not enable to
        prevent errors (writing 1 to enable throws an error if period is 0),
        but sets enable_on_set_period to enable the channel automatically
        once the period has been set. If the period has been set, writes data
        to enable.

        :param msg: Data to write to enable.
        :type msg: Bool
        """
        if self.period == 0:
            self.enable_on_set_period = True
            return

        with open(os.path.join(self.channel_path, 'enable'), 'w') as f:
            f.write(str(int(msg.data)))

    def period_callback(self, msg: Int32) -> None:
        """
        Set the desired period of the PWM channel.

        First sets the period as specified. Then checks if the
        enable_on_set_period variable has been set to True, in which cas it
        will also enable the channel.
        """
        with open(os.path.join(self.channel_path, 'period'), 'w') as f:
            f.write(str(msg.data))
            self.period = msg.data

        if not self.enable_on_set_period:
            return

        with open(os.path.join(self.channel_path, 'enable'), 'w') as f:
            f.write('1')
            self.enable_on_set_period = False

    def duty_cycle_callback(self, msg: Int32) -> None:
        """Set the desired duty cycle of the PWM channel."""
        duty_cycle = msg.data
        if duty_cycle > self.period:
            duty_cycle = self.period

        with open(os.path.join(self.channel_path, 'duty_cycle'), 'w') as f:
            f.write(str(duty_cycle))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PWMController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

from enum import Enum

from gpiozero import OutputDevice
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.node import Node
from rclpy.subscription import RCLError
from std_msgs.msg import Bool  # , Int32


class State(Enum):
    """Current state of the robot."""

    INIT = 0
    LINE_FOLLOWING = 1
    RESCUE = 2
    IDLE = 3
    STOP = 4


class StateMachineNode(Node):
    """
    Switches between line follow, rescue and idle states.

    Configure names for line follow and rescue nodes in config file.
    Defaults are line_follower and rescue.

    .. note::

        Topics for changing states:

        * **Rescue**: /rescue_active (Bool)
        * **Idle**: /idle_button (Bool)

    .. note::

        Line follow and rescue nodes must be lifecycle notes so that they
        can be enabled and disabled depending on whichever section of the
        course the robot is on.

    :ivar line_follow_node: Name of line follow node.
    :type line_follow_node: str
    :ivar rescue_node: Name of rescue node.
    :type rescue_node: str

    :ivar en_3v3: Enable pin for 3.3V voltage regulator.
    :type en_3v3: OutputDevice
    :ivar en_5v: Enable pin for 5V voltage regulator.
    :type en_5v: OutputDevice

    """

    def __init__(self) -> None:
        super().__init__('state_machine')
        self.current_state = State.INIT

        self.rescue_active = False
        self.idle_toggle = True

        self.declare_parameter('line_follow_node', 'line_follower')
        self.declare_parameter('rescue_node', 'rescue')
        self.declare_parameter('ml_rescue_node', 'ml_rescue_node')

        self.line_follow_node: str = self.get_parameter('line_follow_node').value
        self.rescue_node: str = self.get_parameter('rescue_node').value
        self.ml_rescue_node: str = self.get_parameter('ml_rescue_node').value

        self.rescue_active_sub = self.create_subscription(
            Bool, '/rescue_active', self.rescue_active_callback, 10
        )
        self.idle_button_sub = self.create_subscription(
            Bool, '/idle_button', self.idle_button_callback, 10
        )

        self.en_3v3 = OutputDevice(16, active_high=True, initial_value=True)
        self.en_5v = OutputDevice(17, active_high=True, initial_value=True)

        # Lifecycle service clients
        self.line_follower_client = self.create_client(
            ChangeState, f'{self.line_follow_node}/change_state'
        )

        while not self.line_follower_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('Waiting for line follower node...')

        self.rescue_client = self.create_client(ChangeState, f'{self.rescue_node}/change_state')

        while not self.rescue_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('Waiting for rescue node...')

        self.ml_rescue_client = self.create_client(
            ChangeState, f'{self.ml_rescue_node}/change_state'
        )

        while not self.ml_rescue_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('Waiting for ml rescue node...')

        self.timer = self.create_timer(0.05, self.state_loop)
        self.transition_future = None
        self.transitioning_num: int = 0

    def change_node_state(self, client, transition_id) -> None:
        """
        Change the state of a lifecycle node.

        .. warning::

            Lifecycle node state changes must follow the correct order.
            For example, lifecycle nodes must be configured before being activated.
            Failing to do so will result in errors.

        :param client: The lifecycle node to change.
        :type client: rclpy.client
        :param transition_id: Target lifecycle node state.
        :type transition_id: Transition
        """
        try:
            self.transitioning_num += 1
            req = ChangeState.Request()
            req.transition.id = transition_id
            self.transition_future = client.call_async(req)
            self.transition_future.add_done_callback(self.transition_completed_callback)
            self.get_logger().info(f'Active transitions: {self.transitioning_num}')
        except RCLError as e:
            self.get_logger().error(f'Could not transition node! {e}')

    def transition_completed_callback(self, _):
        self.transitioning_num -= 1
        self.get_logger().info(f'Active transitions: {self.transitioning_num}')

    def rescue_active_callback(self, msg: Bool) -> None:
        """
        Set whether or not rescue should be active.

        Should be called by the line follow node when a rescue zone
        is detected, and by the rescue node after exiting rescue.

        :param msg: Rescue status.
        :type msg: Bool
        """
        self.rescue_active = msg.data

    def idle_button_callback(self, msg: Bool):
        """
        Read idle button status.

        Toggles idle state once button is released.

        :param msg: Idle button status.
        :type msg: Bool
        """
        if not msg.data:
            self.idle_toggle = not self.idle_toggle

    # def black_callback(self, msg: Bool):
    #     if msg.data and self.current_state == State.LINE_FOLLOWING:
    #         self.rescue_active = True

    # def silver_callback(self, msg: Bool):
    #     if msg.data and self.current_state == State.RESCUE_EXIT:
    #         self.change_node_state(self.line_follower_client, Transition.TRANSITION_ACTIVATE)
    #         self.change_node_state(self.rescue_client, Transition.TRANSITION_DEACTIVATE)
    #         self.rescue_active = False
    #         self.current_state = State.LINE_FOLLOWING

    def clean_exit(self):
        """Disable voltage regulators on exit."""
        self.en_3v3.off()
        self.en_5v.off()

    def state_loop(self):
        """
        Run main control loop for robot.

        Begins in idle state.

        **Idle**: checks if idle button has been toggled. If it has, transitions
        into line follow state.

        **Line Follow**: checks if rescue_active or idle_toggle is active,
        in which case it will transition to rescue or idle state respectively.

        **Rescue state**, checks if rescue_active is inactive or idle_toggle is
        active, in which case it will transition to line follow or idle state respectively.
        """
        if self.transitioning_num > 0:
            return

        if self.current_state == State.INIT:
            self.change_node_state(self.line_follower_client, Transition.TRANSITION_CONFIGURE)
            self.change_node_state(self.rescue_client, Transition.TRANSITION_CONFIGURE)
            self.change_node_state(self.ml_rescue_client, Transition.TRANSITION_CONFIGURE)

            self.current_state = State.IDLE

        elif self.current_state == State.IDLE:
            if not self.idle_toggle:
                self.get_logger().info('Exiting idle')
                self.change_node_state(self.line_follower_client, Transition.TRANSITION_ACTIVATE)

                self.current_state = State.LINE_FOLLOWING

        elif self.idle_toggle:
            self.get_logger().info('Idling all nodes')
            if self.rescue_active:
                self.change_node_state(self.rescue_client, Transition.TRANSITION_DEACTIVATE)
                self.change_node_state(self.ml_rescue_client, Transition.TRANSITION_DEACTIVATE)
            else:
                self.change_node_state(self.line_follower_client, Transition.TRANSITION_DEACTIVATE)

            self.rescue_active = False
            self.current_state = State.IDLE

        elif self.current_state == State.LINE_FOLLOWING:
            self.get_logger().info('Activating line follow')
            if self.rescue_active:
                self.get_logger().info('Deactivating line follow')
                self.change_node_state(self.line_follower_client, Transition.TRANSITION_DEACTIVATE)
                self.change_node_state(self.rescue_client, Transition.TRANSITION_ACTIVATE)
                self.change_node_state(self.ml_rescue_client, Transition.TRANSITION_ACTIVATE)
                self.current_state = State.RESCUE

        elif self.current_state == State.RESCUE:
            self.get_logger().info('Activating rescue')
            if not self.rescue_active:
                self.get_logger().info('Deactivating rescue')
                self.change_node_state(self.rescue_client, Transition.TRANSITION_DEACTIVATE)
                self.change_node_state(self.ml_rescue_client, Transition.TRANSITION_DEACTIVATE)
                self.change_node_state(self.line_follower_client, Transition.TRANSITION_ACTIVATE)
                self.current_state = State.LINE_FOLLOWING

        elif self.current_state == State.STOP:
            self.get_logger().info('Deactivating all nodes')
            if self.rescue_active:
                self.change_node_state(self.rescue_client, Transition.TRANSITION_DEACTIVATE)
                self.change_node_state(self.ml_rescue_client, Transition.TRANSITION_DEACTIVATE)
            else:
                self.change_node_state(self.line_follower_client, Transition.TRANSITION_DEACTIVATE)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.clean_exit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

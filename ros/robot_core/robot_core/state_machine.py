from enum import Enum

from gpiozero import OutputDevice
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class State(Enum):
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

    Topics for changing states:
        Rescue: /rescue_active (Bool)
        Idle: /idle_button (Bool)
    """

    def __init__(self):
        super().__init__('state_machine')
        self.current_state = State.INIT

        self.rescue_active = False
        self.idle_toggle = True

        self.declare_parameter('line_follow_node', 'line_follower')
        self.declare_parameter('rescue_node', 'rescue')

        self.line_follow_node: str = self.get_parameter('line_follow_node').value
        self.rescue_node: str = self.get_parameter('rescue_node').value

        self.rescue_active_sub = self.create_subscription(
            Bool, '/rescue_active', self.rescue_active_callback, 10
        )
        self.idle_button_sub = self.create_subscription(
            Bool, '/idle_button', self.idle_button_callback, 10
        )
        self.fan_pub = self.create_publisher(Int32, '/fan/target_speed', 10)

        # Lifecycle service clients
        self.line_follower_client = self.create_client(
            ChangeState, f'/{self.line_follow_node}/change_state'
        )
        self.rescue_client = self.create_client(
            ChangeState, f'/{self.rescue_node}/change_state'
        )

        self.en_3v3 = OutputDevice(16, active_high=True, initial_value=True)
        self.en_5v = OutputDevice(17, active_high=True, initial_value=True)

        self.timer = self.create_timer(0.05, self.state_loop)

    def change_node_state(self, client, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id  # e.g., Transition.TRANSITION_ACTIVATE
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def rescue_active_callback(self, msg):
        self.rescue_active = msg.data

    def idle_button_callback(self, msg: Bool):
        if not msg.data:
            self.idle_toggle = not self.idle_toggle

    def clean_exit(self):
        self.en_3v3.off()
        self.en_5v.off()

    def state_loop(self):
        from lifecycle_msgs.msg import Transition

        if self.current_state == State.INIT:
            # Activate motor control for all states
            self.current_state = State.IDLE

        elif self.current_state == State.IDLE:
            if not self.idle_toggle:
                fan_msg = Int32()
                fan_msg.data = 100
                self.fan_pub.publish(fan_msg)

                # self.change_node_state(
                #     self.line_follower_client, Transition.TRANSITION_ACTIVATE
                # )

                self.current_state = State.LINE_FOLLOWING

        elif self.idle_toggle:
            fan_msg = Int32()
            fan_msg.data = 0
            self.fan_pub.publish(fan_msg)

            # self.change_node_state(
            #     self.line_follower_client, Transition.TRANSITION_DEACTIVATE
            # )
            # self.change_node_state(self.rescue_client, Transition.TRANSITION_DEACTIVATE)
            self.current_state = State.IDLE

        elif self.current_state == State.LINE_FOLLOWING:
            if self.rescue_active:
                self.change_node_state(
                    self.line_follower_client, Transition.TRANSITION_DEACTIVATE
                )
                self.change_node_state(
                    self.rescue_client, Transition.TRANSITION_ACTIVATE
                )

        elif self.current_state == State.RESCUE:
            if not self.rescue_active:
                self.change_node_state(
                    self.rescue_client, Transition.TRANSITION_DEACTIVATE
                )
                self.change_node_state(
                    self.line_follower_client, Transition.TRANSITION_ACTIVATE
                )

        elif self.current_state == State.STOP:
            # Deactivate all nodes
            self.change_node_state(
                self.line_follower_client, Transition.TRANSITION_DEACTIVATE
            )
            self.change_node_state(self.rescue_client, Transition.TRANSITION_DEACTIVATE)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.clean_exit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

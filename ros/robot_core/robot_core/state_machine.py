from enum import Enum

from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State as LifecycleState
from std_msgs.msg import Bool


class State(Enum):
    INIT = 0
    LINE_FOLLOWING = 1
    RESCUE = 2
    IDLE = 3
    STOP = 4


class StateMachineNode(LifecycleNode):

    def __init__(self):
        super().__init__('state_machine')
        self.current_state = State.INIT
        self.begin_rescue_sub = None
        self.end_rescue_sub = None
        self.idle_button_sub = None

        self.begin_rescue = False
        self.end_rescue = False
        self.idle_button_pressed = False

        # Lifecycle service clients
        self.line_follower_client = self.create_client(ChangeState, '/line_follower/change_state')
        self.rescue_client = self.create_client(ChangeState, '/rescue_node/change_state')
        self.camera_client = self.create_client(ChangeState, '/camera_node/change_state')
        self.motor_client = self.create_client(ChangeState, '/motor_control/change_state')

        self.timer = self.create_timer(0.05, self.state_loop)

    def change_node_state(self, client, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id  # e.g., Transition.TRANSITION_ACTIVATE
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def on_configure(self, state: LifecycleState):
        self.begin_rescue_sub = self.create_subscription(Bool, '/rescue/begin',
                                                         self.begin_rescue_callback, 10)
        self.end_rescue_sub = self.create_subscription(Bool, '/rescue/end',
                                                       self.end_rescue_callback, 10)
        self.idle_button_sub = self.create_subscription(Bool, '/button/idle',
                                                        self.idle_button_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def begin_rescue_callback(self, msg):
        self.begin_rescue = msg.data

    def end_rescue_callback(self, msg):
        self.end_rescue = msg.data

    def idle_button_callback(self, msg):
        self.idle_button_pressed = msg.data

    def state_loop(self):
        from lifecycle_msgs.msg import Transition
        if self.current_state == State.INIT:
            # Activate motor control for all states
            self.change_node_state(self.motor_client, Transition.TRANSITION_ACTIVATE)
            self.current_state = State.IDLE

        elif self.current_state == State.IDLE:
            if self.idle_button_pressed:
                self.change_node_state(self.line_follower_client, Transition.TRANSITION_ACTIVATE)
                self.current_state = State.LINE_FOLLOWING

        elif self.idle_button_pressed:
            self.change_node_state(self.line_follower_client, Transition.TRANSITION_DEACTIVATE)
            self.change_node_state(self.rescue_client, Transition.TRANSITION_DEACTIVATE)
            self.current_state = State.IDLE

        elif self.current_state == State.LINE_FOLLOWING:
            if self.begin_rescue:
                self.change_node_state(self.line_follower_client, Transition.TRANSITION_DEACTIVATE)
                self.change_node_state(self.rescue_client, Transition.TRANSITION_ACTIVATE)

        elif self.current_state == State.RESCUE:
            if self.end_rescue:
                self.change_node_state(self.rescue_client, Transition.TRANSITION_DEACTIVATE)
                self.change_node_state(self.line_follower_client, Transition.TRANSITION_ACTIVATE)

        elif self.current_state == State.STOP:
            # Deactivate all nodes
            self.change_node_state(self.line_follower_client, Transition.TRANSITION_DEACTIVATE)
            self.change_node_state(self.rescue_client, Transition.TRANSITION_DEACTIVATE)
            self.change_node_state(self.camera_client, Transition.TRANSITION_DEACTIVATE)
            self.change_node_state(self.motor_client, Transition.TRANSITION_DEACTIVATE)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

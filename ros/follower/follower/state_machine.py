from enum import Enum

from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State as LifecycleState
from std_msgs.msg import Bool, Float64


class State(Enum):
    INIT = 0
    LINE_FOLLOWING = 1
    TURNING_BACK = 2
    SEARCHING_VICTIMS = 3
    APPROACHING_VICTIM = 4
    LOWERING_CLAW = 5
    GRABBING_VICTIM = 6
    RAISING_CLAW = 7
    SEARCHING_EVACUATION_POINT = 8
    APPROACHING_EVACUATION_POINT = 9
    RELEASING_VICTIM = 10
    SEARCHING_EXIT = 11
    APPROACHING_EXIT = 12
    STOP = 13


class StateMachineNode(LifecycleNode):

    def __init__(self):
        super().__init__('state_machine')
        self.current_state = State.INIT
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.line_sub = None
        self.can_sub = None
        self.line_error = 0.0
        self.can_detected = False

        # Lifecycle service clients
        self.line_sensor_client = self.create_client(ChangeState, '/line_sensor/change_state')
        self.camera_client = self.create_client(ChangeState, '/camera_node/change_state')
        self.motor_client = self.create_client(ChangeState, '/motor_control/change_state')

        self.timer = self.create_timer(0.1, self.state_loop)

    def change_node_state(self, client, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id  # e.g., Transition.TRANSITION_ACTIVATE
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def on_configure(self, state: LifecycleState):
        self.line_sub = self.create_subscription(Float64, '/line_error', self.line_callback, 10)
        self.can_sub = self.create_subscription(Bool, '/can_detected', self.can_callback, 10)
        return TransitionCallbackReturn.SUCCESS

    def line_callback(self, msg):
        self.line_error = msg.data

    def can_callback(self, msg):
        self.can_detected = msg.data

    def state_loop(self):
        from lifecycle_msgs.msg import Transition
        if self.current_state == State.INIT:
            # Activate motor control for all states
            self.change_node_state(self.motor_client, Transition.TRANSITION_ACTIVATE)
            self.current_state = State.LINE_FOLLOWING

        elif self.current_state == State.LINE_FOLLOWING:
            # Ensure line sensor is active, camera is inactive
            self.change_node_state(self.line_sensor_client, Transition.TRANSITION_ACTIVATE)
            self.change_node_state(self.camera_client, Transition.TRANSITION_DEACTIVATE)
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -self.line_error * 1.0
            self.cmd_pub.publish(twist)
            if abs(self.line_error) > 1.0 or self.can_detected:
                self.current_state = State.SEARCHING_CAN

        elif self.current_state == State.SEARCHING_CAN:
            # Activate camera, deactivate line sensor
            self.change_node_state(self.camera_client, Transition.TRANSITION_ACTIVATE)
            self.change_node_state(self.line_sensor_client, Transition.TRANSITION_DEACTIVATE)
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_pub.publish(twist)
            if self.can_detected:
                self.current_state = State.LINE_FOLLOWING  # Simplified

        elif self.current_state == State.STOP:
            # Deactivate all nodes
            self.change_node_state(self.line_sensor_client, Transition.TRANSITION_DEACTIVATE)
            self.change_node_state(self.camera_client, Transition.TRANSITION_DEACTIVATE)
            self.change_node_state(self.motor_client, Transition.TRANSITION_DEACTIVATE)
            twist = Twist()
            self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

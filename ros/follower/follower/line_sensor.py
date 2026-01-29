# import cv2
# import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class LineSensorNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__('line_sensor')
        self.publisher_ = None
        self.subscriber_ = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.publisher_ = self.create_publisher(Float64, '/line_error', 10)
        self.subscriber_ = self.create_subscription(Image, '/line_camera/image_raw',
                                                    self.image_callback, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating LineSensorNode')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating LineSensorNode')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisher_)
        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg):
        pass

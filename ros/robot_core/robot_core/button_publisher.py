from gpiozero import Button
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ButtonPublisher(Node):
    """Node for detecting and publishing button presses."""

    def __init__(self) -> None:
        super().__init__('button_publisher')

        self.declare_parameter('gpio_pin', 6)
        self.declare_parameter('publish_topic', '/button')
        self.declare_parameter('pull_up', True)

        self.gpio_pin: int = self.get_parameter('gpio_pin').value

        self.publish_topic: str = self.get_parameter('publish_topic').value

        self.pull_up: bool = self.get_parameter('pull_up').value

        self.pub = self.create_publisher(Bool, self.publish_topic, 10)

        self.button = Button(self.gpio_pin, pull_up=self.pull_up)
        self.button.when_activated = self.on_press
        self.button.when_deactivated = self.on_release

    def on_press(self) -> None:
        msg: Bool = Bool()

        msg.data = True

        self.pub.publish(msg)

    def on_release(self) -> None:
        msg: Bool = Bool()

        msg.data = False

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

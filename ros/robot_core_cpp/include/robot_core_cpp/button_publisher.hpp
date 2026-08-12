#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <gpiod.hpp>
#include <thread>
#include <std_msgs/msg/bool.hpp>

class ButtonPublisher : public rclcpp::Node {
public:
  explicit ButtonPublisher(const rclcpp::NodeOptions & options);
  ~ButtonPublisher();

private:
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> pub;
  unsigned int gpio_pin;
  std::string pub_topic;
  bool pull_up;

  ::gpiod::chip chip;
  ::gpiod::line input_line;

  std::thread interrupt_thread_;
  std::atomic<bool> keep_running_{true};
  void worker_loop();
};

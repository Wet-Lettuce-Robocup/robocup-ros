#include "robot_core_cpp/button_publisher.hpp"
#include <chrono>
#include <gpiod.hpp>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

ButtonPublisher::ButtonPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("button_publisher", options)
{
  this->declare_parameter<std::string>("gpio_chip", "gpiochip4");
  this->declare_parameter<int>("gpio_pin", 6);
  this->declare_parameter<std::string>("publish_topic", "/button");
  this->declare_parameter<bool>("pull_up", false);

  this->chip = ::gpiod::chip(this->get_parameter("gpio_chip").as_string());
  this->gpio_pin = this->get_parameter("gpio_pin").as_int();
  this->pub_topic = this->get_parameter("publish_topic").as_string();
  this->pull_up = this->get_parameter("pull_up").as_bool();

  this->input_line = this->chip.get_line(this->gpio_pin);

  if (this->pull_up) {
    this->input_line.request({
      "ButtonPublisher",
      ::gpiod::line_request::EVENT_BOTH_EDGES,
      ::gpiod::line_request::FLAG_BIAS_PULL_UP
  });
  } else {
    this->input_line.request({
      "ButtonPublisher",
      ::gpiod::line_request::EVENT_BOTH_EDGES,
      0
  });
  }

  interrupt_thread_ = std::thread(&ButtonPublisher::worker_loop, this);
}

ButtonPublisher::~ButtonPublisher()
{
  keep_running_ = false;
  if (interrupt_thread_.joinable()) {
    interrupt_thread_.join();
  }
}

void ButtonPublisher::worker_loop()
{
  while (rclcpp::ok() && keep_running_) {
    if (input_line.event_wait(std::chrono::milliseconds(500))) {
      ::gpiod::line_event event = input_line.event_read();

      bool pressed = event.event_type == ::gpiod::line_event::RISING_EDGE ? true : false;

      std_msgs::msg::Bool msg = std_msgs::msg::Bool();
      msg.data = pressed;

      this->pub->publish(msg);
    }
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ButtonPublisher>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

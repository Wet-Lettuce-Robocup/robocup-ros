#include "robot_core_cpp/movement_actions.hpp"
#include <bit>
#include <fcntl.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>
#include <sys/ioctl.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}
#include <unistd.h>

MovementActions::MovementActions(const rclcpp::NodeOptions & options)
: rclcpp::Node("movement_actions", options)
{
  this->declare_parameter<double>("wheel_dist", 0.12);
  this->wheel_dist = this->get_parameter("wheel_dist").as_double();

  auto handle_goal = [this](
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveTime::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with vel %f, angular_vel %f, time %f",
      goal->vel, goal->angular_vel, goal->time);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  auto handle_cancel = [this](
    const std::shared_ptr<GoalMoveTime> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

  auto handle_accepted = [this](
    const std::shared_ptr<GoalMoveTime> goal_handle)
    {
      auto execute_in_thread = [this, goal_handle](){return this->execute_goal(goal_handle);};
      std::thread{execute_in_thread}.detach();
    };

  this->action_server_ = rclcpp_action::create_server<MoveTime>(
      this,
      "move_time",
      handle_goal,
      handle_cancel,
      handle_accepted);
}

void MovementActions::send_movement_action(double vel, double angular_vel, double time)
{
  const char * device = "/dev/i2c-1";
  int i2c_fd = open(device, O_RDWR);

  if (i2c_fd < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus: %d", i2c_fd);
    return;
  }

  int slave_addr = 0x67;
  if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus/talk to slave.");
    close(i2c_fd);
    return;
  }

  int32_t vel_byte = int(vel);
  int32_t angular_vel_byte = int(angular_vel);
  uint32_t time_byte = int(time * 1000);

  uint8_t buffer[12];

  buffer[0] = vel_byte >> 24 & 0xFF;
  buffer[1] = vel_byte >> 16 & 0xFF;
  buffer[2] = vel_byte >> 8 & 0xFF;
  buffer[3] = vel_byte & 0xFF;

  buffer[4] = angular_vel_byte >> 24 & 0xFF;
  buffer[5] = angular_vel_byte >> 16 & 0xFF;
  buffer[6] = angular_vel_byte >> 8 & 0xFF;
  buffer[7] = angular_vel_byte & 0xFF;

  buffer[8] = time_byte >> 24 & 0xFF;
  buffer[9] = time_byte >> 16 & 0xFF;
  buffer[10] = time_byte >> 8 & 0xFF;
  buffer[11] = time_byte & 0xFF;

  if (i2c_smbus_write_block_data(i2c_fd, 0x04, 12, buffer) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write to I2C bus!");
  }

  close(i2c_fd);
}

void MovementActions::stop_robot()
{
  const char * device = "/dev/i2c-1";
  int i2c_fd = open(device, O_RDWR);

  if (i2c_fd < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus: %d", i2c_fd);
    return;
  }

  int slave_addr = 0x67;
  if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus/talk to slave.");
    close(i2c_fd);
    return;
  }

  if (i2c_smbus_write_block_data(i2c_fd, 0x02, 0, nullptr) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write to I2C bus!");
  }

  close(i2c_fd);
}

uint8_t MovementActions::get_robot_state()
{
  const char * device = "/dev/i2c-1";
  int i2c_fd = open(device, O_RDWR);

  if (i2c_fd < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus: %d", i2c_fd);
    return 0x00;
  }

  int slave_addr = 0x67;
  if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus/talk to slave.");
    close(i2c_fd);
    return 0;
  }

  uint8_t rx[1];
  i2c_smbus_read_i2c_block_data(i2c_fd, 0x80, 1, rx);
  close(i2c_fd);

  return rx[0];
}

unsigned int MovementActions::get_action_count()
{
  const char * device = "/dev/i2c-1";
  int i2c_fd = open(device, O_RDWR);

  if (i2c_fd < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus: %d", i2c_fd);
    return 0x00;
  }

  int slave_addr = 0x67;
  if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus/talk to slave.");
    close(i2c_fd);
    return 0;
  }

  uint8_t rx[4];
  i2c_smbus_read_i2c_block_data(i2c_fd, 0x85, 4, rx);
  close(i2c_fd);

  unsigned int result = std::bit_cast<unsigned int>(rx);

  return result;
}

robot_msgs::action::MoveTime::Result MovementActions::execute_goal(
  const std::shared_ptr<GoalMoveTime> goal_handle)
{
  MoveTime::Feedback feedback;
  MoveTime::Result result;
  result.success = false;

  MoveTime::Goal request = *goal_handle->get_goal();

  double vel = request.vel;
  double angular_vel = request.angular_vel;
  double time = request.time;

  unsigned int count = this->get_action_count();
  this->send_movement_action(vel, angular_vel, time);

  rclcpp::Time start_time = this->get_clock()->now();
  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    if (!goal_handle->is_active()) {
      RCLCPP_INFO(this->get_logger(), "Movement action cancelled!");
      this->stop_robot();
      return result;
    }

    rclcpp::Time current_time = this->get_clock()->now();
    rclcpp::Duration time_diff = current_time - start_time;

    feedback.time_elapsed = time_diff.nanoseconds() * 1e-9;
    goal_handle->publish_feedback(std::make_shared<MoveTime::Feedback>(feedback));

    uint8_t state = this->get_robot_state();
    int time_left = time - time_diff.nanoseconds() * 1e-9;

    if (time_left < -1 && state != 2) {
      unsigned int new_count = this->get_action_count();

      if (new_count == count) {
        RCLCPP_ERROR(this->get_logger(), "Movement service not executed. Retrying...");
        this->send_movement_action(vel, angular_vel, time);
        start_time = this->get_clock()->now();
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      result.success = true;
      break;
    }

    rate.sleep();
  }

  this->stop_robot();
  return result;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovementActions>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

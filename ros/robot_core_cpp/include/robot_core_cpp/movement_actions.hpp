#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/action/move_time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class MovementActions : public rclcpp::Node {
public:
  using MoveTime = robot_msgs::action::MoveTime;
  using GoalMoveTime = rclcpp_action::ServerGoalHandle<MoveTime>;
  explicit MovementActions(const rclcpp::NodeOptions & options);

private:
  rclcpp_action::Server<MoveTime>::SharedPtr action_server_;
  void send_movement_action(double vel, double angular_vel, double time);
  void stop_robot();
  uint8_t get_robot_state();
  unsigned int get_action_count();
  MoveTime::Result execute_goal(const std::shared_ptr<GoalMoveTime> goal_handle);

  double wheel_dist;
};

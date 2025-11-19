#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <system_interfaces/action/follow_trajectory.hpp>

namespace target_system {
using FollowTrajectory = system_interfaces::action::FollowTrajectory;

class TargetSystemNode : public rclcpp::Node {
public:
  explicit TargetSystemNode();
  bool is_action_in_progress();
  int get_trajectory_index();
  void set_position(double x, double y, double z);
  void set_result(bool result);

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const FollowTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>>
          goal_handle);
  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>>
          goal_handle);
  void execute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>>
          goal_handle);
  void reset();

  rclcpp_action::Server<FollowTrajectory>::SharedPtr action_server_;
  std::jthread thread_;
  std::mutex mutex_;
  bool is_action_in_progress_{false};
  int trajectory_index_{0};
  double x_, y_, z_{0.0};
  bool result_{false};
  bool is_action_finished_{false};
  bool is_position_updated_{false};
};
} // namespace target_system

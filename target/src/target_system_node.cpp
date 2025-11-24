#include "target_system_node.hpp"

namespace target_system {
using FollowTrajectory = system_interfaces::action::FollowTrajectory;

TargetSystemNode::TargetSystemNode() : rclcpp::Node("target") {
  action_server_ = rclcpp_action::create_server<FollowTrajectory>(
      this, "follow_trajectory",
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const FollowTrajectory::Goal> goal) {
        return handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<
             rclcpp_action::ServerGoalHandle<FollowTrajectory>>
                 goal_handle) { return handle_cancel(goal_handle); },
      [this](const std::shared_ptr<
             rclcpp_action::ServerGoalHandle<FollowTrajectory>>
                 goal_handle) { return handle_accepted(goal_handle); });
}

bool TargetSystemNode::is_action_in_progress() {
  std::lock_guard<std::mutex> guard(mutex_);
  return is_action_in_progress_;
}

int TargetSystemNode::get_trajectory_index() {
  std::lock_guard<std::mutex> guard(mutex_);
  return trajectory_index_;
}

void TargetSystemNode::set_odometry(double pos_x, double pos_y, double pos_z,
                                    double twist_x, double twist_y,
                                    double twist_z) {
  std::lock_guard<std::mutex> guard(mutex_);
  pos_x_ = pos_x;
  pos_y_ = pos_y;
  pos_z_ = pos_z;
  twist_x_ = twist_x;
  twist_y_ = twist_y;
  twist_z_ = twist_z;
  is_odometry_updated_ = true;
}

void TargetSystemNode::set_result(bool result) {
  std::lock_guard<std::mutex> guard(mutex_);
  result_ = result;
  is_action_finished_ = true;
}

rclcpp_action::GoalResponse TargetSystemNode::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowTrajectory::Goal> goal) {
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TargetSystemNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>>
        goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TargetSystemNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>>
        goal_handle) {
  thread_ =
      std::jthread([this, goal_handle]() { return execute(goal_handle); });
}

void TargetSystemNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>>
        goal_handle) {
  rclcpp::Rate loop_rate(1);
  reset();

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<FollowTrajectory::Feedback>();
  auto result = std::make_shared<FollowTrajectory::Result>();

  {
    std::lock_guard<std::mutex> guard(mutex_);
    is_action_in_progress_ = true;
    trajectory_index_ = goal->trajectory_index;
  }

  while (true) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      reset();
      return;
    }

    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (is_action_finished_) {
        result->success = result_;
        goal_handle->succeed(result);
        reset();
        return;
      }

      if (is_odometry_updated_) {
        geometry_msgs::msg::Pose pose{};
        pose.position.x = pos_x_;
        pose.position.y = pos_y_;
        pose.position.z = pos_z_;

        geometry_msgs::msg::Twist twist{};
        twist.linear.x = twist_x_;
        twist.linear.y = twist_y_;
        twist.linear.z = twist_z_;

        std_msgs::msg::Header header{};
        header.stamp = get_clock()->now();
        header.frame_id = "map";

        feedback->header = header;
        feedback->pose = pose;
        feedback->twist = twist;
        goal_handle->publish_feedback(feedback);

        is_odometry_updated_ = false;
      }
    }

    loop_rate.sleep();
  }
}

void TargetSystemNode::reset() {
  std::lock_guard<std::mutex> guard(mutex_);
  is_action_in_progress_ = false;
  is_action_finished_ = false;
  is_odometry_updated_ = false;
  trajectory_index_ = 0;
}

} // namespace target_system
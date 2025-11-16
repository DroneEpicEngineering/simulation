#include "target_system.hpp"

#include <iostream>

#include <rclcpp/rclcpp.hpp>

using namespace target_system;

TargetSystem::TargetSystem() {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = std::make_shared<TargetSystemNode>();
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  node_thread_ = std::jthread([this]() {
    if (rclcpp::ok()) {
      executor_->add_node(node_);
      executor_->spin();
    }
  });
}

TargetSystem::~TargetSystem() {}

void TargetSystem::PreUpdate(const gz::sim::UpdateInfo &info,
                             gz::sim::EntityComponentManager &ecm) {
  if (node_->is_action_in_progress() &&
      !trajectory_reader_.is_trajectory_read() &&
      node_->get_trajectory_index() != 0) {
    std::stringstream filename;
    filename << "/home/dee/ws/src/simulation/data/target_trajectories/"
                "resampled_trajectory_data_"
             << node_->get_trajectory_index() << ".csv";
    trajectory_reader_.read(
        filename.str()); // TODO: parametrized filename and directory
  }
}

void TargetSystem::Update(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm) {
  if (!node_->is_action_in_progress()) {
    return;
  }

  if (trajectory_reader_.is_trajectory_finished()) {
    node_->set_result(true);
    return;
  }

  TrajectoryPoint tp = trajectory_reader_.next_point();

  // TODO: send world/basic/set_pose message

  node_->set_position(tp.pos_x, tp.pos_y, tp.pos_z);
}

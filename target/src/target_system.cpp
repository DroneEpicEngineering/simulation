#include "target_system.hpp"

#include <iostream>

#include <rclcpp/rclcpp.hpp>

using namespace target_system;
using namespace std::chrono_literals;

TargetSystem::TargetSystem() {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = std::make_shared<TargetSystemNode>();
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  target_system_node_thread_ = std::jthread([this]() {
    if (rclcpp::ok()) {
      executor_->add_node(node_);
      executor_->spin();
    }
  });

  request_.set_id(8);
  request_.set_name("ufo");
}

TargetSystem::~TargetSystem() {}

void TargetSystem::PreUpdate(const gz::sim::UpdateInfo &info,
                             gz::sim::EntityComponentManager &ecm) {
  if (node_->is_action_in_progress() &&
      !trajectory_reader_.is_trajectory_read() &&
      node_->get_trajectory_index() != 0) {
    std::stringstream filename;
    filename << "/home/dee/ws/src/sim/data/target_trajectories/"
                "resampled_trajectory_data_"
             << node_->get_trajectory_index() << ".csv";
    trajectory_reader_.read(
        filename.str()); // TODO: parametrized filename and directory
  }
}

void TargetSystem::Update(const gz::sim::UpdateInfo &info,
                          gz::sim::EntityComponentManager &ecm) {
  if (!node_->is_action_in_progress() || !trajectory_reader_.is_trajectory_read()) {
    return;
  }

  if (trajectory_reader_.is_trajectory_finished()) {
    node_->set_result(true);
    return;
  }

  if (std::chrono::steady_clock::now() - previous_update_ < 20ms) {
    return;
  }

  TrajectoryPoint tp = trajectory_reader_.next_point();
  send_set_pose_req(tp.pos_x, tp.pos_y, tp.pos_z);
  node_->set_odometry(tp.pos_x, tp.pos_y, tp.pos_z, tp.vel_x, tp.vel_y, tp.vel_z);
  previous_update_ = std::chrono::steady_clock::now();
}

void TargetSystem::send_set_pose_req(double x, double y, double z) {
  auto position = request_.mutable_position();
  position->set_x(x);
  position->set_y(y);
  position->set_z(z);
  bool result;
  transport_node_.Request("/world/basic/set_pose", request_, 5000, response_, result);
}

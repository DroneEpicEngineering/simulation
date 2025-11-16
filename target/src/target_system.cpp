#include "target_system.hpp"

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
                             gz::sim::EntityComponentManager &ecm) {}

void TargetSystem::Update(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm) {}

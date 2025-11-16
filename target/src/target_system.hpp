#include "target_system_node.hpp"
#include "trajectory_reader.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>

#include <rclcpp/executors.hpp>

namespace target_system {
class TargetSystem : public gz::sim::System,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemUpdate {
public:
  TargetSystem();
  ~TargetSystem() override;
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;
  void Update(const gz::sim::UpdateInfo &info,
              gz::sim::EntityComponentManager &ecm) override;

private:
  TrajectoryReader trajectory_reader_{};
  TargetSystemNode::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::jthread node_thread_;
};

}; // namespace target_system

GZ_ADD_PLUGIN(target_system::TargetSystem, gz::sim::System,
              target_system::TargetSystem::ISystemPreUpdate,
              target_system::TargetSystem::ISystemUpdate)
#include "target_system_node.hpp"
#include "trajectory_reader.hpp"

#include <chrono>
#include <string>

#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport.hh>

#include <rclcpp/executors.hpp>

namespace target_system {
class TargetSystem : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemUpdate {
public:
  TargetSystem();
  ~TargetSystem() override;
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &event_manager) override;
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;
  void Update(const gz::sim::UpdateInfo &info,
              gz::sim::EntityComponentManager &ecm) override;

private:
  void send_set_pose_req(double x, double y, double z);

  TrajectoryReader trajectory_reader_{};
  std::chrono::steady_clock::time_point previous_update_;

  std::shared_ptr<TargetSystemNode> node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::jthread target_system_node_thread_;

  gz::transport::Node transport_node_;
  gz::msgs::Pose request_;
  gz::msgs::Vector3d request_position_;
  gz::msgs::Boolean response_;

  std::string world_name_;
  std::string model_name_;
  std::string trajectories_directory_;
  std::string trajectories_filename_;
  double update_rate_{0.02};
};

}; // namespace target_system

GZ_ADD_PLUGIN(target_system::TargetSystem, gz::sim::System,
              target_system::TargetSystem::ISystemConfigure,
              target_system::TargetSystem::ISystemPreUpdate,
              target_system::TargetSystem::ISystemUpdate)
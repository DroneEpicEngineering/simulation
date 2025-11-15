#include "trajectory_reader.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>

namespace target_system {
class TargetSystem : public gz::sim::System, public gz::sim::ISystemUpdate {
public:
  TargetSystem();
  ~TargetSystem() override;
  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

private:
  TrajectoryReader trajectory_reader_{};
};

}; // namespace target_system

GZ_ADD_PLUGIN(target_system::TargetSystem, gz::sim::System,
              target_system::TargetSystem::ISystemUpdate)
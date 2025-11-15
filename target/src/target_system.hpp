#include "trajectory_reader.hpp"

#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

namespace target_system
{
    class TargetSystem : public gz::sim::System, public gz::sim::ISystemUpdate
    {
    public:
        TargetSystem();
        ~TargetSystem() override;
        void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;

    private:
        TrajectoryReader trajectory_reader_{};
    };

};

GZ_ADD_PLUGIN(
    target_system::TargetSystem,
    gz::sim::System,
    target_system::TargetSystem::ISystemUpdate)
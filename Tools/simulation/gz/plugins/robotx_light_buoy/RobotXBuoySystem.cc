#include "RobotXBuoySystem.hh"

#include <chrono>
#include <cmath>
#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>

namespace robotx
{

void RobotXBuoySystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  this->modelEntity = _entity;

  if (_sdf->HasElement("mode"))
    this->mode = _sdf->Get<std::string>("mode");
  if (_sdf->HasElement("on_time"))
    this->onTime = _sdf->Get<double>("on_time");
  if (_sdf->HasElement("off_time"))
    this->offTime = _sdf->Get<double>("off_time");

  if (this->mode != "flashing" && this->mode != "solid" && this->mode != "off")
  {
    std::cerr << "[RobotXBuoySystem] Unknown mode '" << this->mode
              << "', using 'flashing'" << std::endl;
    this->mode = "flashing";
  }
  if (this->onTime <= 0.0 || this->offTime <= 0.0)
  {
    std::cerr << "[RobotXBuoySystem] on_time and off_time must be > 0, using 1.0 s each"
              << std::endl;
    this->onTime = 1.0;
    this->offTime = 1.0;
  }

  std::cout << "[RobotXBuoySystem] model=" << this->modelEntity
            << " mode=" << this->mode
            << " on_time=" << this->onTime
            << " off_time=" << this->offTime << std::endl;
}

bool RobotXBuoySystem::FindVisuals(gz::sim::EntityComponentManager &_ecm)
{
  // Search only the links of this model, not the whole world
  const gz::sim::Model model(this->modelEntity);
  for (const auto &linkEntity : model.Links(_ecm))
  {
    const gz::sim::Link link(linkEntity);
    if (this->onVisual == gz::sim::kNullEntity)
      this->onVisual = link.VisualByName(_ecm, "beacon_light_on");
    if (this->offVisual == gz::sim::kNullEntity)
      this->offVisual = link.VisualByName(_ecm, "beacon_light_off");
  }

  if (this->onVisual == gz::sim::kNullEntity || this->offVisual == gz::sim::kNullEntity)
    return false;

  // The SDF pose of the lit visual is where the beacon is shown (on or off)
  const auto *pose = _ecm.Component<gz::sim::components::Pose>(this->onVisual);
  if (pose == nullptr)
    return false;
  this->activePose = pose->Data();
  return true;
}

void RobotXBuoySystem::SetPose(
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::Entity _entity,
  const gz::math::Pose3d &_pose)
{
  auto *poseComp = _ecm.Component<gz::sim::components::Pose>(_entity);
  if (poseComp == nullptr)
  {
    _ecm.CreateComponent(_entity, gz::sim::components::Pose(_pose));
    return;
  }
  *poseComp = gz::sim::components::Pose(_pose);
  _ecm.SetChanged(
    _entity, gz::sim::components::Pose::typeId, gz::sim::ComponentState::OneTimeChange);
}

void RobotXBuoySystem::ApplyState(gz::sim::EntityComponentManager &_ecm, bool _on)
{
  SetPose(_ecm, this->onVisual, _on ? this->activePose : this->hiddenPose);
  SetPose(_ecm, this->offVisual, _on ? this->hiddenPose : this->activePose);
}

void RobotXBuoySystem::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  bool desiredOn = (this->mode == "solid");
  if (this->mode == "flashing")
  {
    // Cycle: on_time lit, then off_time dark, starting lit at sim time 0
    const double t = std::chrono::duration<double>(_info.simTime).count();
    desiredOn = std::fmod(t, this->onTime + this->offTime) < this->onTime;
  }

  if (!this->visualsFound)
  {
    this->visualsFound = this->FindVisuals(_ecm);
    if (!this->visualsFound)
      return;
    // Force the initial state (the SDF starts lit, with the off visual hidden)
    this->ApplyState(_ecm, desiredOn);
    this->currentOn = desiredOn;
    return;
  }

  if (desiredOn == this->currentOn)
    return;

  this->ApplyState(_ecm, desiredOn);
  this->currentOn = desiredOn;
}

}  // namespace robotx

GZ_ADD_PLUGIN(
  robotx::RobotXBuoySystem,
  gz::sim::System,
  robotx::RobotXBuoySystem::ISystemConfigure,
  robotx::RobotXBuoySystem::ISystemPreUpdate
)

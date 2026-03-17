#include "RobotXLightBuoySystem.hh"

#include <chrono>
#include <cmath>
#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>

namespace robotx
{

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static gz::math::Color ColorFromString(const std::string &name)
{
  if (name == "red")   return gz::math::Color(1.0f, 0.0f, 0.0f, 1.0f);
  if (name == "green") return gz::math::Color(0.0f, 1.0f, 0.0f, 1.0f);
  return gz::math::Color(0.0f, 0.0f, 1.0f, 1.0f);  // default: blue
}

// ---------------------------------------------------------------------------
// ISystemConfigure
// ---------------------------------------------------------------------------

void RobotXLightBuoySystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  this->modelEntity = _entity;

  if (_sdf->HasElement("color"))
    this->color = _sdf->Get<std::string>("color");
  if (_sdf->HasElement("mode"))
    this->mode = _sdf->Get<std::string>("mode");
  if (_sdf->HasElement("blink_period"))
    this->blinkPeriod = _sdf->Get<double>("blink_period");

  // Apply color to the active poses (just stored for debug; color comes from SDF material)
  (void)ColorFromString(this->color);

  std::cout << "[RobotXLightBuoySystem] ========== Configure() =========="
            << "\n  entity       = " << this->modelEntity
            << "\n  color        = " << this->color
            << "\n  mode         = " << this->mode
            << "\n  blink_period = " << this->blinkPeriod
            << "\n  strategy     = pose-toggle (no create/delete)"
            << std::endl;

  this->initialized = true;
}

// ---------------------------------------------------------------------------
// Entity discovery
// ---------------------------------------------------------------------------

bool RobotXLightBuoySystem::FindEntities(
  gz::sim::EntityComponentManager &_ecm)
{
  // --- Find light entity ---
  if (this->lightEntity == gz::sim::kNullEntity)
  {
    _ecm.Each<gz::sim::components::Light,
               gz::sim::components::Name,
               gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &_ent,
          gz::sim::components::Light *,
          gz::sim::components::Name *_name,
          gz::sim::components::ParentEntity *) -> bool
      {
        if (_name->Data() == "beacon_light")
        {
          this->lightEntity = _ent;
          std::cout << "[RobotXLightBuoySystem] Found beacon_light: "
                    << _ent << std::endl;
          return false;
        }
        return true;
      });
  }

  // --- Find dome visual entities ---
  if (this->domeOnEntity  == gz::sim::kNullEntity ||
      this->domeOffEntity == gz::sim::kNullEntity)
  {
    _ecm.Each<gz::sim::components::Visual,
               gz::sim::components::Name,
               gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &_ent,
          gz::sim::components::Visual *,
          gz::sim::components::Name *_name,
          gz::sim::components::ParentEntity *) -> bool
      {
        const auto &n = _name->Data();
        if (n == "beacon_dome_on_visual")
        {
          this->domeOnEntity = _ent;
          std::cout << "[RobotXLightBuoySystem] Found beacon_dome_on_visual: "
                    << _ent << std::endl;
        }
        else if (n == "beacon_dome_off_visual")
        {
          this->domeOffEntity = _ent;
          std::cout << "[RobotXLightBuoySystem] Found beacon_dome_off_visual: "
                    << _ent << std::endl;
        }
        return (this->domeOnEntity  == gz::sim::kNullEntity ||
                this->domeOffEntity == gz::sim::kNullEntity);
      });
  }

  bool allFound = (this->lightEntity  != gz::sim::kNullEntity &&
                   this->domeOnEntity != gz::sim::kNullEntity &&
                   this->domeOffEntity != gz::sim::kNullEntity);

  if (!allFound)
  {
    std::cerr << "[RobotXLightBuoySystem] Entities not found yet: "
              << "light=" << this->lightEntity
              << " domeOn=" << this->domeOnEntity
              << " domeOff=" << this->domeOffEntity
              << std::endl;
  }

  return allFound;
}

// ---------------------------------------------------------------------------
// SetEntityPose
// ---------------------------------------------------------------------------

void RobotXLightBuoySystem::SetEntityPose(
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::Entity _entity,
  const gz::math::Pose3d &_pose)
{
  if (_entity == gz::sim::kNullEntity)
    return;

  auto *poseComp =
    _ecm.Component<gz::sim::components::Pose>(_entity);

  if (!poseComp)
  {
    // Create the Pose component if it doesn't exist
    _ecm.CreateComponent(_entity, gz::sim::components::Pose(_pose));
  }
  else
  {
    *poseComp = gz::sim::components::Pose(_pose);
    _ecm.SetChanged(
      _entity,
      gz::sim::components::Pose::typeId,
      gz::sim::ComponentState::OneTimeChange);
  }
}

// ---------------------------------------------------------------------------
// ApplyState
// ---------------------------------------------------------------------------

void RobotXLightBuoySystem::ApplyState(
  gz::sim::EntityComponentManager &_ecm,
  bool on)
{
  if (on)
  {
    // Light and dome_on → active positions
    // dome_off → hidden underground
    SetEntityPose(_ecm, this->lightEntity,   this->lightActivePose);
    SetEntityPose(_ecm, this->domeOnEntity,  this->domeActivePose);
    SetEntityPose(_ecm, this->domeOffEntity, this->hiddenPose);
    std::cout << "[RobotXLightBuoySystem] -> ON  (pose restored)" << std::endl;
  }
  else
  {
    // Light and dome_on → hidden underground
    // dome_off → active position (shows "off" dome)
    SetEntityPose(_ecm, this->lightEntity,   this->hiddenPose);
    SetEntityPose(_ecm, this->domeOnEntity,  this->hiddenPose);
    SetEntityPose(_ecm, this->domeOffEntity, this->domeActivePose);
    std::cout << "[RobotXLightBuoySystem] -> OFF (pose hidden)" << std::endl;
  }
}

// ---------------------------------------------------------------------------
// ISystemPreUpdate
// ---------------------------------------------------------------------------

void RobotXLightBuoySystem::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!this->initialized || _info.paused)
    return;

  // --- Entity discovery (once) ---
  if (!this->entitiesFound)
  {
    this->entitiesFound = this->FindEntities(_ecm);
    if (!this->entitiesFound)
      return;

    // Force initial state on first successful find
    this->ApplyState(_ecm, this->currentOn);
    return;
  }

  // --- Compute desired state ---
  bool desiredOn = true;

  if (this->mode == "flashing")
  {
    const double t =
      std::chrono::duration<double>(_info.simTime).count();
    const double phase = std::fmod(t, this->blinkPeriod);
    desiredOn = (phase < (this->blinkPeriod / 2.0));
  }

  if (desiredOn == this->currentOn)
    return;

  // --- Apply state change ---
  this->ApplyState(_ecm, desiredOn);
  this->currentOn = desiredOn;
}

}  // namespace robotx

GZ_ADD_PLUGIN(
  robotx::RobotXLightBuoySystem,
  gz::sim::System,
  robotx::RobotXLightBuoySystem::ISystemConfigure,
  robotx::RobotXLightBuoySystem::ISystemPreUpdate
)

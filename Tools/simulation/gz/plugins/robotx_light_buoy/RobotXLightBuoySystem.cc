#include "RobotXLightBuoySystem.hh"

#include <chrono>
#include <cmath>
#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightType.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
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

  // Pre-build the sdf::Light data (used each time we create the entity)
  auto onColor = ColorFromString(this->color);

  this->lightSdf.SetName("beacon_light_dyn");
  this->lightSdf.SetType(sdf::LightType::POINT);
  this->lightSdf.SetLightOn(true);
  this->lightSdf.SetIntensity(1.5);
  this->lightSdf.SetDiffuse(onColor);
  this->lightSdf.SetSpecular(gz::math::Color(
    onColor.R() * 0.5f, onColor.G() * 0.5f, onColor.B() * 0.5f, 1.0f));
  this->lightSdf.SetAttenuationRange(10.0);
  this->lightSdf.SetConstantAttenuationFactor(0.5);
  this->lightSdf.SetLinearAttenuationFactor(0.05);
  this->lightSdf.SetQuadraticAttenuationFactor(0.01);
  this->lightSdf.SetCastShadows(false);
  this->lightSdf.SetVisualize(false);   // no helper verde
  // Pose relative to link: same position as the beacon dome
  this->lightSdf.SetRawPose(gz::math::Pose3d(0, 0, 0.85, 0, 0, 0));

  std::cout << "[RobotXLightBuoySystem] ========== Configure() =========="
            << "\n  entity       = " << this->modelEntity
            << "\n  color        = " << this->color
            << "\n  mode         = " << this->mode
            << "\n  blink_period = " << this->blinkPeriod
            << "\n  strategy     = create/delete entity"
            << std::endl;

  this->initialized = true;
}

// ---------------------------------------------------------------------------
// Link entity discovery
// ---------------------------------------------------------------------------

void RobotXLightBuoySystem::FindLinkEntity(
  gz::sim::EntityComponentManager &_ecm)
{
  // Find "base_link" that is a direct child of our model entity
  _ecm.Each<gz::sim::components::Link,
             gz::sim::components::Name,
             gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_ent,
        gz::sim::components::Link *,
        gz::sim::components::Name *_name,
        gz::sim::components::ParentEntity *_parent) -> bool
    {
      if (_parent->Data() == this->modelEntity)
      {
        std::cout << "[RobotXLightBuoySystem] Found link '"
                  << _name->Data() << "' entity=" << _ent << std::endl;

        if (_name->Data() == "base_link")
        {
          this->linkEntity = _ent;
          return false;  // stop — found our link
        }
      }
      return true;
    });

  if (this->linkEntity == gz::sim::kNullEntity)
  {
    std::cerr << "[RobotXLightBuoySystem] WARNING: base_link not found!"
              << std::endl;
  }
}

// ---------------------------------------------------------------------------
// CreateDynamicLight / RemoveDynamicLight
// ---------------------------------------------------------------------------

void RobotXLightBuoySystem::CreateDynamicLight(
  gz::sim::EntityComponentManager &_ecm)
{
  if (this->linkEntity == gz::sim::kNullEntity)
    return;

  // Create a new entity for the dynamic light
  this->dynLightEntity = _ecm.CreateEntity();

  _ecm.CreateComponent(this->dynLightEntity,
    gz::sim::components::Light(this->lightSdf));

  _ecm.CreateComponent(this->dynLightEntity,
    gz::sim::components::LightType(std::string("point")));

  _ecm.CreateComponent(this->dynLightEntity,
    gz::sim::components::Name("beacon_light_dyn"));

  _ecm.CreateComponent(this->dynLightEntity,
    gz::sim::components::ParentEntity(this->linkEntity));

  _ecm.CreateComponent(this->dynLightEntity,
    gz::sim::components::Pose(
      gz::math::Pose3d(0, 0, 0.85, 0, 0, 0)));

  std::cout << "[RobotXLightBuoySystem] Created dynamic light entity: "
            << this->dynLightEntity << std::endl;
}

void RobotXLightBuoySystem::RemoveDynamicLight(
  gz::sim::EntityComponentManager &_ecm)
{
  if (this->dynLightEntity == gz::sim::kNullEntity)
    return;

  _ecm.RequestRemoveEntity(this->dynLightEntity);

  std::cout << "[RobotXLightBuoySystem] Removed dynamic light entity: "
            << this->dynLightEntity << std::endl;

  this->dynLightEntity = gz::sim::kNullEntity;
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

  // --- Find link entity once ---
  if (this->linkEntity == gz::sim::kNullEntity)
  {
    this->FindLinkEntity(_ecm);
    if (this->linkEntity == gz::sim::kNullEntity)
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

  // --- Apply transition ---
  if (desiredOn)
  {
    this->CreateDynamicLight(_ecm);
  }
  else
  {
    this->RemoveDynamicLight(_ecm);
  }

  this->currentOn = desiredOn;
}

}  // namespace robotx

GZ_ADD_PLUGIN(
  robotx::RobotXLightBuoySystem,
  gz::sim::System,
  robotx::RobotXLightBuoySystem::ISystemConfigure,
  robotx::RobotXLightBuoySystem::ISystemPreUpdate
)

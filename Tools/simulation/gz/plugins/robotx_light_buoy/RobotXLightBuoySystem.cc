#include "RobotXLightBuoySystem.hh"

#include <chrono>
#include <cmath>
#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Color.hh>
#include <sdf/Light.hh>

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

  this->onColor  = ColorFromString(this->color);
  this->offColor = gz::math::Color(0.0f, 0.0f, 0.0f, 1.0f);

  std::cout << "[RobotXLightBuoySystem] ========== Configure() =========="
            << "\n  entity       = " << this->modelEntity
            << "\n  color        = " << this->color
            << "\n  onColor      = " << this->onColor
            << "\n  mode         = " << this->mode
            << "\n  blink_period = " << this->blinkPeriod
            << std::endl;

  this->initialized = true;
}

// ---------------------------------------------------------------------------
// Light entity discovery
// ---------------------------------------------------------------------------

void RobotXLightBuoySystem::FindLightEntity(
  gz::sim::EntityComponentManager &_ecm)
{
  this->findAttempts++;

  _ecm.Each<gz::sim::components::Light,
             gz::sim::components::Name>(
    [&](const gz::sim::Entity &_ent,
        gz::sim::components::Light *_lightComp,
        gz::sim::components::Name *_name) -> bool
    {
      if (this->findAttempts <= 3)
      {
        std::cout << "[RobotXLightBuoySystem] Scanning light entity "
                  << _ent << " name='" << _name->Data() << "'"
                  << std::endl;
      }

      if (_name->Data() == "beacon_light")
      {
        this->lightEntity = _ent;
        std::cout << "[RobotXLightBuoySystem] >>> Found beacon_light entity: "
                  << _ent
                  << " | LightOn=" << _lightComp->Data().LightOn()
                  << " | Intensity=" << _lightComp->Data().Intensity()
                  << " | Diffuse=" << _lightComp->Data().Diffuse()
                  << std::endl;
        return false;  // stop iteration
      }
      return true;
    });

  if (this->lightEntity == gz::sim::kNullEntity && this->findAttempts <= 5)
  {
    std::cerr << "[RobotXLightBuoySystem] WARNING: beacon_light NOT found "
              << "(attempt " << this->findAttempts << ")" << std::endl;
  }
}

// ---------------------------------------------------------------------------
// SetLightState — modifica sdf::Light directamente y llama SetChanged
// ---------------------------------------------------------------------------

void RobotXLightBuoySystem::SetLightState(
  gz::sim::EntityComponentManager &_ecm,
  bool on)
{
  auto *lightComp =
    _ecm.Component<gz::sim::components::Light>(this->lightEntity);

  if (!lightComp)
  {
    std::cerr << "[RobotXLightBuoySystem] ERROR: Light component not found "
              << "on entity " << this->lightEntity << std::endl;
    return;
  }

  // Get a mutable copy of the sdf::Light data
  sdf::Light lightData = lightComp->Data();

  if (on)
  {
    lightData.SetLightOn(true);
    lightData.SetIntensity(1.0);
    lightData.SetDiffuse(this->onColor);
    lightData.SetSpecular(gz::math::Color(
      this->onColor.R() * 0.5f,
      this->onColor.G() * 0.5f,
      this->onColor.B() * 0.5f,
      1.0f));
  }
  else
  {
    lightData.SetLightOn(false);
    lightData.SetIntensity(0.0);
    lightData.SetDiffuse(this->offColor);
    lightData.SetSpecular(this->offColor);
  }

  // Write back the modified data to the component
  *lightComp = gz::sim::components::Light(lightData);

  // Signal the ECM that this component changed so the renderer picks it up
  _ecm.SetChanged(
    this->lightEntity,
    gz::sim::components::Light::typeId,
    gz::sim::ComponentState::OneTimeChange);

  std::cout << "[RobotXLightBuoySystem] SetLightState -> "
            << (on ? "ON " : "OFF")
            << " | LightOn=" << lightData.LightOn()
            << " | Intensity=" << lightData.Intensity()
            << " | Diffuse=" << lightData.Diffuse()
            << std::endl;
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

  // --- Entity discovery ---
  if (this->lightEntity == gz::sim::kNullEntity)
  {
    this->FindLightEntity(_ecm);
    if (this->lightEntity == gz::sim::kNullEntity)
      return;

    // Force-apply the initial state on the very first frame we find the light
    this->currentOn = !this->currentOn;  // flip to force first update
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

  // --- Apply change ---
  this->SetLightState(_ecm, desiredOn);
  this->currentOn = desiredOn;
}

}  // namespace robotx

GZ_ADD_PLUGIN(
  robotx::RobotXLightBuoySystem,
  gz::sim::System,
  robotx::RobotXLightBuoySystem::ISystemConfigure,
  robotx::RobotXLightBuoySystem::ISystemPreUpdate
)

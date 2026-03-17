#pragma once

#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Color.hh>

namespace robotx
{

/// \brief Plugin visual-first (con light real) para la boya luminosa de RobotX.
///
/// El beacon se implementa como:
///   1. Una esfera visual con emissive activo (representación visual del color)
///   2. Una point light real anclada al mismo punto
///
/// El blinking se implementa modificando DIRECTAMENTE el componente
/// gz::sim::components::Light (sdf::Light) — sin LightCmd.
/// Se llama SetChanged() para que el renderer Ogre2 propague el cambio.
///
/// Parámetros SDF:
///   <color>        blue | red | green   (default: blue)
///   <mode>         solid | flashing     (default: solid)
///   <blink_period> segundos             (default: 1.0)
class RobotXLightBuoySystem :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

private:
  /// Busca la entidad de la luz una sola vez por nombre "beacon_light".
  void FindLightEntity(gz::sim::EntityComponentManager &_ecm);

  /// Activa o desactiva la luz modificando el componente Light directamente.
  void SetLightState(gz::sim::EntityComponentManager &_ecm, bool on);

private:
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity lightEntity{gz::sim::kNullEntity};

  std::string color{"blue"};
  std::string mode{"solid"};
  double blinkPeriod{1.0};

  gz::math::Color onColor{0.0f, 0.0f, 1.0f, 1.0f};   ///< Color encendido
  gz::math::Color offColor{0.0f, 0.0f, 0.0f, 1.0f};  ///< Color apagado

  bool initialized{false};
  bool currentOn{true};  ///< Estado actual del beacon

  /// Cuántas veces intentamos buscar la entidad (para evitar spam de logs)
  int findAttempts{0};
};

}  // namespace robotx

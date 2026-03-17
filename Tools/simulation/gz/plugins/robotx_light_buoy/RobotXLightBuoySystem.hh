#pragma once

#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Light.hh>

namespace robotx
{

/// \brief Plugin de boya RobotX — blinking sin flash, via pose toggle.
///
/// Estrategia: todas las entidades (luz + domos) son permanentes.
/// Para "apagar": se mueven a z=-10000 (fuera del mundo visible).
/// Para "encender": se restauran a su posición original.
///
/// Esto elimina el flash negro causado por create/delete de entidades,
/// que disparaba un re-broadcast completo del SceneBroadcaster.
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
  /// Busca todas las entidades que controlamos (luz + domos).
  /// Retorna true cuando las encuentra todas.
  bool FindEntities(gz::sim::EntityComponentManager &_ecm);

  /// Mueve una entidad a la pose activa o a la pose oculta.
  void SetEntityPose(
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity _entity,
    const gz::math::Pose3d &_pose);

  /// Aplica el estado ON o OFF moviendo las poses.
  void ApplyState(gz::sim::EntityComponentManager &_ecm, bool on);

private:
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};

  // Entidades controladas por el plugin
  gz::sim::Entity lightEntity{gz::sim::kNullEntity};
  gz::sim::Entity domeOnEntity{gz::sim::kNullEntity};
  gz::sim::Entity domeOffEntity{gz::sim::kNullEntity};

  // Poses: activa (real) y oculta (underground)
  static constexpr double kHiddenZ = -10000.0;
  gz::math::Pose3d lightActivePose{0, 0, 0.85, 0, 0, 0};
  gz::math::Pose3d domeActivePose {0, 0, 0.80, 0, 0, 0};
  gz::math::Pose3d hiddenPose     {0, 0, kHiddenZ, 0, 0, 0};

  std::string color{"blue"};
  std::string mode{"solid"};
  double blinkPeriod{1.0};

  bool initialized{false};
  bool entitiesFound{false};
  bool currentOn{true};  ///< Arranca en true (luz visible por defecto en SDF)
};

}  // namespace robotx

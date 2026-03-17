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

/// \brief Plugin de boya RobotX — blinking por creación/eliminación dinámica
/// de una entidad luz en el ECM.
///
/// La estrategia es:
///   - ON : _ecm.CreateEntity() + componentes de luz → SceneManager::CreateLight
///   - OFF: _ecm.RequestRemoveEntity() → SceneManager remueve la luz de Ogre2
///
/// Esto usa el camino CreateLight/Remove del SceneManager que SÍ funciona,
/// a diferencia de modificar propiedades de una luz existente.
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
  /// Busca la entidad del link "base_link" dentro del modelo.
  void FindLinkEntity(gz::sim::EntityComponentManager &_ecm);

  /// Crea dinámicamente la entidad de luz y adjunta los componentes.
  void CreateDynamicLight(gz::sim::EntityComponentManager &_ecm);

  /// Solicita la eliminación de la entidad de luz dinámica.
  void RemoveDynamicLight(gz::sim::EntityComponentManager &_ecm);

private:
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};

  /// Entidad de luz creada dinámicamente (kNullEntity = apagada)
  gz::sim::Entity dynLightEntity{gz::sim::kNullEntity};

  /// Datos de la luz (pose, atenuación, etc.) pre-construidos en Configure
  sdf::Light lightSdf;

  std::string color{"blue"};
  std::string mode{"solid"};
  double blinkPeriod{1.0};

  bool initialized{false};
  bool currentOn{false};  ///< Arranca en false para forzar creación inicial
};

}  // namespace robotx

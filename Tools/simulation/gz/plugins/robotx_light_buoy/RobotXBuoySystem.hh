#pragma once

#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>

namespace robotx
{

/// \brief Beacon plugin for the robotx_buoy_<color> models (top beacon seen by the UAV).
///
/// Toggles the beacon by swapping two visuals of the model it is attached to:
///   beacon_light_on   emissive sphere (the lit beacon)
///   beacon_light_off  dark sphere (the beacon turned off)
/// The inactive visual is moved underground (z = -10000) instead of being
/// deleted, so the scene is not re-broadcast on every toggle.
///
/// Visuals are searched only inside the model that owns the plugin, so many
/// buoys can share the same visual names in one world. The beacon color comes
/// from each model's material (one model per color).
///
/// SDF parameters:
///   <mode>      flashing | solid | off   (default: flashing)
///   <on_time>   seconds lit per cycle    (default: 1.0, RobotX Task 1)
///   <off_time>  seconds dark per cycle   (default: 1.0, RobotX Task 1)
class RobotXBuoySystem :
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
  /// Finds the on/off visuals inside this model. Returns true when both exist.
  bool FindVisuals(gz::sim::EntityComponentManager &_ecm);

  /// Shows the on or off visual and hides the other one.
  void ApplyState(gz::sim::EntityComponentManager &_ecm, bool _on);

  /// Sets an entity pose and marks it changed so the renderer picks it up.
  static void SetPose(
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity _entity,
    const gz::math::Pose3d &_pose);

private:
  static constexpr double kHiddenZ = -10000.0;

  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity onVisual{gz::sim::kNullEntity};
  gz::sim::Entity offVisual{gz::sim::kNullEntity};

  /// Pose of the beacon in its link, read from the SDF of beacon_light_on.
  gz::math::Pose3d activePose;
  gz::math::Pose3d hiddenPose{0, 0, kHiddenZ, 0, 0, 0};

  std::string mode{"flashing"};
  double onTime{1.0};
  double offTime{1.0};

  bool visualsFound{false};
  bool currentOn{false};
};

}  // namespace robotx

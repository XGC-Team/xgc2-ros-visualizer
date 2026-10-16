#pragma once

#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <json/json.h>
#include <xgc2_ros_visualizer/config.hpp>
#include <xgc2_ros_visualizer/scene_contract.hpp>

namespace xgc2_ros_visualizer {

constexpr std::size_t kMaxRobots = 256;
constexpr std::size_t kMaxRelays = 128;

// The URDF a robot publishes as its visual description.
struct DescriptionProfile {
  std::string package, file;
  bool state_publisher{false};
  std::string joint_state_topic{"joint_states"};
};

struct LabelProfile {
  std::string color{"#00a2ff"};
  bool scale_invariant{false};
  // Meters, or pixels when the label keeps a fixed screen size.
  double font_size{0.24};
  double opacity{1.0};
};

struct AnimationProfile {
  // Freshness windows of the pose, the MAVROS state and the ground motion inputs.
  double pose_timeout{0.5}, state_timeout{2.0}, motion_timeout{0.5};
  // FS150 rotor speeds by flight state, rad/s.
  double rotor_ground{25.0}, rotor_transition{90.0}, rotor_airborne{60.0};
  // Ground wheels.
  double wheel_radius{0.08}, track_width{0.416}, wheelbase_plus_track{0.30};
  double wheel_deadband{0.02}, wheel_max_speed{35.0};
};

// Which outputs the robot contributes to. The scene decides which shared
// outputs exist at all (markers are off unless the scene asks for them); a robot
// can only narrow what its scene publishes, so every contribution defaults to on.
// Scene paths and the per-robot Path topics belong to the robot alone.
struct PublicationProfile {
  bool markers{true}, transforms{true}, scene{true}, scene_paths{false}, paths{true};
};

// Everything the visualizer needs to draw one robot, fully resolved. A profile
// is a value: applying a profile that differs from the applied one rebuilds the
// resources of that robot and nothing else.
struct RobotProfile {
  std::string id;                       // the ROS namespace without its slash
  RateKind kind{RateKind::Global};      // renderer class and row of the rate table
  std::string scene_model;              // scene identity; empty: no scene presence
  double mesh_scale{1.0};
  std::string height_projection_color;  // FS150 only; empty disables the projection
  DescriptionProfile description;
  std::string pose_topic;               // the one pose that drives the scene robot
  std::string ar_pose_topic;            // FS150 identity in the camera pane; empty: none
  std::array<double, 3> world_offset{{0.0, 0.0, 0.0}};
  std::string frame_id{"world"};
  double label_offset{0.55};            // height of the label anchor above the robot
  std::string path_topic{"path"}, ar_path_topic;  // below the robot namespace
  LabelProfile labels;
  AnimationProfile animation;
  PublicationProfile publication;

  std::string rosNamespace() const { return "/" + id; }
  RobotModelKind modelKind() const { return xgc2_ros_visualizer::modelKind(kind); }
  bool hasScene() const { return kind != RateKind::Global && !scene_model.empty(); }
  SceneLabelStyle labelStyle() const;
  // The resolved document: every applicable field is present.
  Json::Value json() const;
  bool operator==(const RobotProfile& other) const;
  bool operator!=(const RobotProfile& other) const { return !(*this == other); }
};
// Strict and complete: unknown fields, fields that do not apply to the kind and
// values outside their range fail. Omitted fields take the kind's defaults.
// Required: model.kind and model.description.package/file.
RobotProfile parseRobotProfile(const std::string& id, const Json::Value& document);

// What the members of an instance share: the optional outputs, the world
// boundary and the display relays.
struct ScenePolicy {
  bool publish_markers{false}, publish_transforms{true}, publish_scene{true};
  std::string boundary_json;  // canonical Experiment worldBoundary, empty: none
  WorldBoundaryDisplay boundary;
  WorldBoundaryDisplayMode boundary_mode{WorldBoundaryDisplayMode::kWalls};
  std::vector<RelayConfig> relays;
  bool operator==(const ScenePolicy& other) const;
  bool operator!=(const ScenePolicy& other) const { return !(*this == other); }
};

// The desired state of one instance.
struct InstanceSpec {
  ScenePolicy scene;
  std::map<std::string, RobotProfile> robots;  // by robot id
  // Panel ceilings. They are not a robot profile and they do not rebuild one.
  double history_max_rate_hz{2};
  bool history_visible{true};
  double prediction_max_rate_hz{2};
};
// What applying a desired state to the current one changes, by robot id.
struct Change {
  std::vector<std::string> added, rebuilt, removed, unchanged;
  bool scene{false};  // the shared outputs or the world boundary change
  bool settings{false};  // history or prediction ceilings, applied without a rebuild
  std::size_t relays_added{0}, relays_removed{0};
  bool empty() const { return added.empty() && rebuilt.empty() && removed.empty() && !scene && !settings && !relays_added && !relays_removed; }
  Json::Value json() const;
};
// Only a robot whose resolved profile differs is rebuilt; a changed display
// relay is replaced by a new one.
Change diffInstance(const InstanceSpec& current, const InstanceSpec& next);

// Whole-instance rules a single profile cannot check: size limits, unique scene
// models and unique path topics.
void checkInstance(const InstanceSpec& spec);
// The ROS names an instance owns. Two instances of one server may not claim the
// same name.
std::set<std::string> claims(const InstanceSpec& spec);

} // namespace xgc2_ros_visualizer

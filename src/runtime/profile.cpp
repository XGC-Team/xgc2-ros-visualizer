#include <xgc2_ros_visualizer/profile.hpp>
#include "../transport/zenoh_input.hpp"
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace xgc2_ros_visualizer {
namespace {
using Names = std::initializer_list<const char*>;

bool identifier(const std::string& value) {
  static const std::regex pattern("^[A-Za-z_][A-Za-z0-9_]*$");
  return !value.empty() && value.size() <= 127 && std::regex_match(value, pattern);
}
bool packageName(const std::string& value) {
  static const std::regex pattern("^[a-z][a-z0-9_]*$");
  return std::regex_match(value, pattern);
}
bool relativeName(const std::string& value) {
  if (value.empty() || value.front() == '/' || value.back() == '/' || value.size() > 499) return false;
  std::stringstream stream(value);
  std::string segment;
  while (std::getline(stream, segment, '/')) if (!identifier(segment)) return false;
  return true;
}
bool absoluteTopic(const std::string& value) {
  return value.size() > 1 && value.front() == '/' && relativeName(value.substr(1));
}
bool descriptionFile(const std::string& value) {
  static const std::regex pattern("^[A-Za-z0-9_.-]+(/[A-Za-z0-9_.-]+)*\\.urdf$");
  if (!std::regex_match(value, pattern) || value.front() == '/') return false;
  std::stringstream stream(value);
  std::string segment;
  while (std::getline(stream, segment, '/')) if (segment == "." || segment == "..") return false;
  return true;
}
bool canonicalColor(const std::string& value) {
  return value.size() == 7 && value[0] == '#' &&
      value.find_first_not_of("0123456789abcdef", 1) == std::string::npos;
}

void objectOf(const Json::Value& value, const std::string& name) {
  if (!value.isObject()) throw std::invalid_argument(name + " must be an object");
}
// Reads an optional section: absent or null is an empty object.
Json::Value section(const Json::Value& parent, const char* name, Names allowed) {
  Json::Value result = parent.isMember(name) ? parent[name] : Json::Value();
  if (result.isNull()) return Json::Value(Json::objectValue);
  objectOf(result, name);
  for (const auto& member : result.getMemberNames())
    if (std::find_if(allowed.begin(), allowed.end(), [&](const char* item){return member == item;}) == allowed.end())
      throw std::invalid_argument(std::string("unknown or inapplicable profile field: ") + name + "." + member);
  return result;
}
bool present(const Json::Value& value, const char* key) {
  return value.isMember(key) && !value[key].isNull();
}
std::string text(const Json::Value& value, const char* key, const std::string& fallback, const std::string& label) {
  if (!present(value, key)) return fallback;
  if (!value[key].isString()) throw std::invalid_argument(label + " must be a string");
  return value[key].asString();
}
bool flag(const Json::Value& value, const char* key, bool fallback, const std::string& label) {
  if (!present(value, key)) return fallback;
  if (!value[key].isBool()) throw std::invalid_argument(label + " must be boolean");
  return value[key].asBool();
}
double number(const Json::Value& value, const char* key, double fallback, double minimum, double maximum,
              const std::string& label) {
  if (!present(value, key)) return fallback;
  const Json::Value& item = value[key];
  if (item.isBool() || !item.isNumeric() || !std::isfinite(item.asDouble()))
    throw std::invalid_argument(label + " must be finite numeric");
  const double result = item.asDouble();
  if (result < minimum || result > maximum)
    throw std::invalid_argument(label + " is outside its range");
  return result;
}
double defaultLabelOffset(RateKind kind) {
  return kind == RateKind::Scout ? 0.65 : kind == RateKind::Mecanum ? 0.32 : 0.55;
}
// The ground wheel geometry differs by kind; the rest of the animation inputs are shared.
AnimationProfile defaultAnimation(RateKind kind) {
  AnimationProfile result;
  if (kind == RateKind::Mecanum) result.wheel_radius = 0.05;
  return result;
}
Names animationFields(RateKind kind) {
  static const Names fs150 = {"poseTimeout", "stateTimeout", "rotorGround", "rotorTransition", "rotorAirborne"};
  static const Names scout = {"poseTimeout", "motionTimeout", "wheelRadius", "trackWidth", "wheelDeadband", "wheelMaxSpeed"};
  static const Names mecanum = {"poseTimeout", "motionTimeout", "wheelRadius", "wheelbasePlusTrack", "wheelDeadband", "wheelMaxSpeed"};
  static const Names none = {};
  return kind == RateKind::Fs150 ? fs150 : kind == RateKind::Scout ? scout : kind == RateKind::Mecanum ? mecanum : none;
}
}

SceneLabelStyle RobotProfile::labelStyle() const {
  return sceneLabelStyleFromMarkerColor(labels.color, labels.scale_invariant, labels.font_size, labels.opacity);
}

Json::Value RobotProfile::json() const {
  Json::Value result(Json::objectValue);
  auto& model = result["model"];
  model["kind"] = kindName(kind);
  model["scene"] = scene_model;
  model["meshScale"] = mesh_scale;
  if (kind == RateKind::Fs150) model["heightProjectionColor"] = height_projection_color;
  auto& visual = model["description"];
  visual["package"] = description.package;
  visual["file"] = description.file;
  visual["statePublisher"] = description.state_publisher;
  visual["jointStateTopic"] = description.joint_state_topic;
  auto& state = result["state"];
  state["poseTopic"] = pose_topic;
  if (kind == RateKind::Fs150) {
    state["arPoseTopic"] = ar_pose_topic;
    state["worldOffset"] = Json::Value(Json::arrayValue);
    for (double coordinate : world_offset) state["worldOffset"].append(coordinate);
  }
  if (!remote_sources.empty()) {
    for (const auto& entry : remote_sources) {
      state["sources"][entry.first]["listen"] = entry.second.endpoint;
      state["sources"][entry.first]["key"] = entry.second.key;
    }
  }
  result["frames"]["world"] = frame_id;
  result["frames"]["labelOffset"] = label_offset;
  result["path"]["topic"] = path_topic;
  if (kind == RateKind::Fs150) result["path"]["arTopic"] = ar_path_topic;
  result["labels"]["color"] = labels.color;
  result["labels"]["scaleInvariant"] = labels.scale_invariant;
  result["labels"]["fontSize"] = labels.font_size;
  result["labels"]["opacity"] = labels.opacity;
  auto& animation_json = result["animation"] = Json::Value(Json::objectValue);
  if (kind != RateKind::Global) animation_json["poseTimeout"] = animation.pose_timeout;
  if (kind == RateKind::Fs150) {
    animation_json["stateTimeout"] = animation.state_timeout;
    animation_json["rotorGround"] = animation.rotor_ground;
    animation_json["rotorTransition"] = animation.rotor_transition;
    animation_json["rotorAirborne"] = animation.rotor_airborne;
  } else if (kind != RateKind::Global) {
    animation_json["motionTimeout"] = animation.motion_timeout;
    animation_json["wheelRadius"] = animation.wheel_radius;
    if (kind == RateKind::Scout) animation_json["trackWidth"] = animation.track_width;
    else animation_json["wheelbasePlusTrack"] = animation.wheelbase_plus_track;
    animation_json["wheelDeadband"] = animation.wheel_deadband;
    animation_json["wheelMaxSpeed"] = animation.wheel_max_speed;
  }
  result["publication"]["markers"] = publication.markers;
  result["publication"]["transforms"] = publication.transforms;
  result["publication"]["scene"] = publication.scene;
  result["publication"]["scenePaths"] = publication.scene_paths;
  result["publication"]["paths"] = publication.paths;
  return result;
}

bool RobotProfile::operator==(const RobotProfile& other) const {
  return id == other.id && json() == other.json();
}

RobotProfile parseRobotProfile(const std::string& id, const Json::Value& document) {
  if (!identifier(id)) throw std::invalid_argument("robot id must be a canonical ROS identifier");
  objectOf(document, "profile");
  for (const auto& member : document.getMemberNames())
    if (member != "model" && member != "state" && member != "frames" && member != "path" &&
        member != "labels" && member != "animation" && member != "publication")
      throw std::invalid_argument("unknown profile section: " + member);
  RobotProfile result;
  result.id = id;

  const auto model = section(document, "model", {"kind", "scene", "meshScale", "heightProjectionColor", "description"});
  if (!present(model, "kind")) throw std::invalid_argument("model.kind is required");
  if (!model["kind"].isString()) throw std::invalid_argument("model.kind must be a string");
  result.kind = parseKind(model["kind"].asString());
  result.scene_model = text(model, "scene", result.kind == RateKind::Global ? "" : id, "model.scene");
  if (result.kind == RateKind::Global && !result.scene_model.empty())
    throw std::invalid_argument("a robot of kind global has no scene model");
  if (!result.scene_model.empty() && !identifier(result.scene_model))
    throw std::invalid_argument("model.scene must be a canonical ROS identifier");
  result.mesh_scale = number(model, "meshScale", result.kind == RateKind::Mecanum ? 0.001 : 1.0, 1e-6, 1e6, "model.meshScale");
  result.height_projection_color = text(model, "heightProjectionColor", "", "model.heightProjectionColor");
  if (!result.height_projection_color.empty() &&
      (result.kind != RateKind::Fs150 || !canonicalColor(result.height_projection_color)))
    throw std::invalid_argument("model.heightProjectionColor is an FS150 #rrggbb color");
  if (!model.isMember("description") || !model["description"].isObject())
    throw std::invalid_argument("model.description is required");
  const auto visual = section(model, "description", {"package", "file", "statePublisher", "jointStateTopic"});
  result.description.package = text(visual, "package", "", "model.description.package");
  result.description.file = text(visual, "file", "", "model.description.file");
  result.description.state_publisher = flag(visual, "statePublisher", false, "model.description.statePublisher");
  result.description.joint_state_topic = text(visual, "jointStateTopic", "joint_states", "model.description.jointStateTopic");
  if (!packageName(result.description.package) || !descriptionFile(result.description.file) ||
      !relativeName(result.description.joint_state_topic))
    throw std::invalid_argument("model.description is not canonical");

  const bool scene = result.hasScene();
  const auto state = section(document, "state", {"poseTopic", "arPoseTopic", "worldOffset", "sources"});
  result.pose_topic = text(state, "poseTopic", scene ? slotVisualizationPoseTopic(result.modelKind(), result.rosNamespace()) : "", "state.poseTopic");
  if (scene != !result.pose_topic.empty() || (scene && !absoluteTopic(result.pose_topic)))
    throw std::invalid_argument("state.poseTopic is an absolute topic of a robot with a scene");
  result.ar_pose_topic = text(state, "arPoseTopic", "", "state.arPoseTopic");
  if (!result.ar_pose_topic.empty() && (!scene || result.kind != RateKind::Fs150 || !absoluteTopic(result.ar_pose_topic)))
    throw std::invalid_argument("state.arPoseTopic is an absolute topic of an FS150 scene robot");
  if (present(state, "worldOffset")) {
    const auto& offset = state["worldOffset"];
    if (!offset.isArray() || offset.size() != 3) throw std::invalid_argument("state.worldOffset requires three coordinates");
    for (Json::ArrayIndex i = 0; i < 3; ++i) {
      if (offset[i].isBool() || !offset[i].isNumeric() || !std::isfinite(offset[i].asDouble()))
        throw std::invalid_argument("state.worldOffset must be finite");
      result.world_offset[i] = offset[i].asDouble();
    }
    if (result.kind != RateKind::Fs150 && result.world_offset != std::array<double, 3>{{0.0, 0.0, 0.0}})
      throw std::invalid_argument("state.worldOffset applies to the FS150 AR pose");
  }

  if (present(state, "sources")) {
    const auto& sources=state["sources"];
    if (!scene || !sources.isObject()) throw std::invalid_argument("state.sources requires a scene robot and an object");
    const std::set<std::string> allowed=result.kind==RateKind::Fs150
      ? std::set<std::string>{"pose", "arPose", "state", "extendedState"}
      : std::set<std::string>{"pose", "twist", "cmdVel"};
    for (const auto& name:sources.getMemberNames()) {
      const auto& source=sources[name];
      if (!allowed.count(name) || !source.isObject() ||
          source.getMemberNames().size()!=2 || !source["listen"].isString() || !source["key"].isString())
        throw std::invalid_argument("invalid state.sources entry");
      RemoteSource remote{source["listen"].asString(),source["key"].asString()};
      validateZenohSource(remote.endpoint,remote.key);
      result.remote_sources.emplace(name,std::move(remote));
    }
  }

  const auto frames = section(document, "frames", {"world", "labelOffset"});
  result.frame_id = text(frames, "world", "world", "frames.world");
  if (!isWorldFixedFrame(result.frame_id)) throw std::invalid_argument("frames.world must be the world fixed frame");
  result.label_offset = number(frames, "labelOffset", defaultLabelOffset(result.kind), -10.0, 10.0, "frames.labelOffset");

  const auto path = section(document, "path", {"topic", "arTopic"});
  result.path_topic = text(path, "topic", "path", "path.topic");
  const bool has_ar_source = !result.ar_pose_topic.empty() || result.remote_sources.count("arPose") != 0;
  result.ar_path_topic = text(path, "arTopic", has_ar_source ? "ar_path" : "", "path.arTopic");
  if (!relativeName(result.path_topic) || (!result.ar_path_topic.empty() && !relativeName(result.ar_path_topic)) ||
      has_ar_source == result.ar_path_topic.empty() || result.path_topic == result.ar_path_topic)
    throw std::invalid_argument("path topics are distinct relative names; arTopic requires a local or remote AR pose source");

  const auto labels = section(document, "labels", {"color", "scaleInvariant", "fontSize", "opacity"});
  result.labels.color = text(labels, "color", result.labels.color, "labels.color");
  result.labels.scale_invariant = flag(labels, "scaleInvariant", false, "labels.scaleInvariant");
  result.labels.font_size = number(labels, "fontSize", result.labels.scale_invariant ? 16.0 : 0.24, 0.0, 256.0, "labels.fontSize");
  result.labels.opacity = number(labels, "opacity", 1.0, 0.0, 1.0, "labels.opacity");
  result.labelStyle();  // validates color, size and opacity together

  const auto animation = section(document, "animation", animationFields(result.kind));
  result.animation = defaultAnimation(result.kind);
  auto& a = result.animation;
  a.pose_timeout = number(animation, "poseTimeout", a.pose_timeout, 0.0, 3600.0, "animation.poseTimeout");
  a.state_timeout = number(animation, "stateTimeout", a.state_timeout, 0.0, 3600.0, "animation.stateTimeout");
  a.motion_timeout = number(animation, "motionTimeout", a.motion_timeout, 0.0, 3600.0, "animation.motionTimeout");
  a.rotor_ground = number(animation, "rotorGround", a.rotor_ground, 0.0, 1e4, "animation.rotorGround");
  a.rotor_transition = number(animation, "rotorTransition", a.rotor_transition, 0.0, 1e4, "animation.rotorTransition");
  a.rotor_airborne = number(animation, "rotorAirborne", a.rotor_airborne, 0.0, 1e4, "animation.rotorAirborne");
  a.wheel_radius = number(animation, "wheelRadius", a.wheel_radius, 1e-6, 100.0, "animation.wheelRadius");
  a.track_width = number(animation, "trackWidth", a.track_width, 1e-6, 100.0, "animation.trackWidth");
  a.wheelbase_plus_track = number(animation, "wheelbasePlusTrack", a.wheelbase_plus_track, 1e-6, 100.0, "animation.wheelbasePlusTrack");
  a.wheel_deadband = number(animation, "wheelDeadband", a.wheel_deadband, 0.0, 100.0, "animation.wheelDeadband");
  a.wheel_max_speed = number(animation, "wheelMaxSpeed", a.wheel_max_speed, 0.0, 1e4, "animation.wheelMaxSpeed");

  const auto publication = section(document, "publication", {"markers", "transforms", "scene", "scenePaths", "paths"});
  result.publication.markers = flag(publication, "markers", true, "publication.markers");
  result.publication.transforms = flag(publication, "transforms", true, "publication.transforms");
  result.publication.scene = flag(publication, "scene", true, "publication.scene");
  result.publication.scene_paths = flag(publication, "scenePaths", false, "publication.scenePaths");
  result.publication.paths = flag(publication, "paths", true, "publication.paths");

  if (scene) {
    if (result.description.state_publisher)
      throw std::invalid_argument("a scene robot publishes its description through the visualizer, not a state publisher");
    // The label namespace contract must fail in the cold request path, never
    // later in the shared publication thread.
    visualization_msgs::MarkerArray label_contract;
    applyRobotMarkerLabel(&label_contract, 0, result.modelKind(), result.rosNamespace());
  }
  return result;
}

bool ScenePolicy::operator==(const ScenePolicy& other) const {
  return publish_markers == other.publish_markers && publish_transforms == other.publish_transforms &&
      publish_scene == other.publish_scene && boundary_json == other.boundary_json &&
      boundary_mode == other.boundary_mode && relays == other.relays;
}

namespace {
bool sameOutputs(const ScenePolicy& a, const ScenePolicy& b) {
  return a.publish_markers == b.publish_markers && a.publish_transforms == b.publish_transforms &&
      a.publish_scene == b.publish_scene && a.boundary_json == b.boundary_json && a.boundary_mode == b.boundary_mode;
}
Json::Value ids(const std::vector<std::string>& values) {
  Json::Value result(Json::arrayValue);
  for (const auto& value : values) result.append(value);
  return result;
}
}

Json::Value Change::json() const {
  Json::Value result(Json::objectValue);
  result["added"] = ids(added);
  result["rebuilt"] = ids(rebuilt);
  result["removed"] = ids(removed);
  result["unchanged"] = ids(unchanged);
  result["scene"] = scene;
  result["settings"] = settings;
  result["relaysAdded"] = Json::UInt64(relays_added);
  result["relaysRemoved"] = Json::UInt64(relays_removed);
  return result;
}

Change diffInstance(const InstanceSpec& current, const InstanceSpec& next) {
  Change change;
  for (const auto& entry : current.robots)
    if (!next.robots.count(entry.first)) change.removed.push_back(entry.first);
  for (const auto& entry : next.robots) {
    const auto found = current.robots.find(entry.first);
    if (found == current.robots.end()) change.added.push_back(entry.first);
    else if (found->second != entry.second) change.rebuilt.push_back(entry.first);
    else change.unchanged.push_back(entry.first);
  }
  change.scene = !sameOutputs(current.scene, next.scene);
  change.settings = current.history_max_rate_hz != next.history_max_rate_hz ||
      current.history_visible != next.history_visible ||
      current.prediction_max_rate_hz != next.prediction_max_rate_hz;
  std::set<std::string> wanted;
  for (const auto& relay : next.scene.relays) {
    wanted.insert(relay.source);
    const auto found = std::find_if(current.scene.relays.begin(), current.scene.relays.end(),
        [&](const RelayConfig& item) { return item.source == relay.source; });
    if (found == current.scene.relays.end()) {
      ++change.relays_added;
    } else if (!(*found == relay)) {
      ++change.relays_added;
      ++change.relays_removed;
    }
  }
  for (const auto& relay : current.scene.relays) if (!wanted.count(relay.source)) ++change.relays_removed;
  return change;
}

std::string remoteListen(const InstanceSpec& spec) {
  std::string listen;
  const auto add=[&listen](const std::string& value) {
    if(value.empty()) return;
    if(!listen.empty() && listen!=value) throw std::invalid_argument("all remote inputs must share one Zenoh listen endpoint");
    listen=value;
  };
  for(const auto& robot:spec.robots) for(const auto& entry:robot.second.remote_sources) add(entry.second.endpoint);
  for(const auto& relay:spec.scene.relays) add(relay.zenoh_endpoint);
  return listen;
}

void checkInstance(const InstanceSpec& spec) {
  remoteListen(spec);
  if (spec.robots.size() > kMaxRobots) throw std::invalid_argument("an instance has at most 256 robots");
  if (spec.scene.relays.size() > kMaxRelays)
    throw std::invalid_argument("an instance has at most " + std::to_string(kMaxRelays) + " display relays");
  std::set<std::string> models, topics, sources;
  for (const auto& entry : spec.scene.relays)
    if (!sources.insert(entry.source).second) throw std::invalid_argument("display relay sources are unique");
  for (const auto& entry : spec.robots) {
    const RobotProfile& robot = entry.second;
    if (!robot.hasScene()) continue;
    if (!models.insert(robot.scene_model).second)
      throw std::invalid_argument("scene model " + robot.scene_model + " is used by more than one robot");
    if (!robot.publication.paths) continue;
    if (!topics.insert(robot.rosNamespace() + "/" + robot.path_topic).second ||
        (!robot.ar_path_topic.empty() && !topics.insert(robot.rosNamespace() + "/" + robot.ar_path_topic).second))
      throw std::invalid_argument("robot path topic collision");
  }
}

std::set<std::string> claims(const InstanceSpec& spec) {
  std::set<std::string> result;
  const ScenePolicy& scene = spec.scene;
  const bool scene_robots = std::any_of(spec.robots.begin(), spec.robots.end(),
      [](const std::map<std::string, RobotProfile>::value_type& entry){return entry.second.hasScene();});
  if (scene.publish_transforms) result.insert("scene-frame-tree");
  if (scene.publish_scene) result.insert("scene-global-topics");
  if (scene.publish_markers && scene_robots) result.insert("topic:/markers");
  if (scene.publish_transforms || !spec.robots.empty()) result.insert("topic:/tf_static");
  for (const auto& entry : spec.robots) {
    const RobotProfile& robot = entry.second;
    if (robot.hasScene() && robot.publication.paths) {
      result.insert("topic:" + robot.rosNamespace() + "/" + robot.path_topic);
      if (!robot.ar_path_topic.empty()) result.insert("topic:" + robot.rosNamespace() + "/" + robot.ar_path_topic);
    }
    result.insert("param:" + robot.rosNamespace() + "/visual_robot_description");
    result.insert("topic:" + robot.rosNamespace() + "/visual_robot_description");
    if (robot.description.state_publisher) {
      result.insert("param:" + robot.rosNamespace() + "/robot_description");
      result.insert("frame-prefix:" + robot.id);
    }
  }
  for (const auto& relay : scene.relays) result.insert("topic:" + relay.topic);
  return result;
}
} // namespace xgc2_ros_visualizer

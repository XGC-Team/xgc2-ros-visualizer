#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <json/json.h>
#include <ros/ros.h>
#include <foxglove_msgs/SceneUpdate.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/MarkerArray.h>
#include <xgc2_ros_visualizer/profile.hpp>
#include "input_pool.hpp"

namespace xgc2_ros_visualizer {

// Cumulative publications by kind and channel, monotonic for the life of an instance.
using Counters = std::array<std::array<std::atomic<std::uint64_t>, kChannelCount>, kKindCount>;

// Drift-free pacing of one output channel.
struct RateGate {
  ros::Time last;
  bool initialized{false};
  bool take(const ros::Time& now, double hz);
};

// What every robot sees in one scheduler tick. The flags say which shared
// outputs exist and are enabled by the scene.
struct TickContext {
  const Rates& table;
  bool transforms, scene, markers;
  const ros::Time& now;
  Counters& totals;
  double history_max_rate_hz;
  bool history_visible;
};

// The messages one tick collects from all robots; the instance publishes each
// once. The scene layers are complete: robots report what they changed, and the
// instance composes the layer from what every robot currently shows.
struct TickBatch {
  tf2_msgs::TFMessage transforms, joints, description_joints;
  visualization_msgs::MarkerArray markers, scratch;
  std::vector<geometry_msgs::TransformStamped> scratch_tf;
  foxglove_msgs::SceneUpdate scene, scene_ar, height, height_ar;  // updates; height holds deletions only
  bool scene_ar_changed{false}, height_changed{false}, height_ar_changed{false};
  void clear();
};

// What a robot currently shows on the shared scene outputs.
struct Shown {
  const foxglove_msgs::SceneEntity* label{nullptr};
  const foxglove_msgs::SceneEntity* ar_label{nullptr};
  const foxglove_msgs::SceneEntity* height{nullptr};
  const foxglove_msgs::SceneEntity* height_ar{nullptr};
  bool streams_scene_paths{false};
};

// Entity deletions for the outputs a robot shared with its siblings.
struct Retraction {
  foxglove_msgs::SceneUpdate scene, scene_ar, height, height_ar;
  visualization_msgs::MarkerArray markers;
  void merge(Retraction&& other);
};

// The runtime of one robot: its description, its input subscriptions, its path
// outputs and the state of what it shows on the shared outputs. A member is
// built from one profile and never changes; a new profile is a new member.
//
// Construction validates and loads (URDF, renderer) but touches no ROS
// resource. `activate` creates them, `retire` removes them. `tick` belongs to the
// scheduler thread; the rest to the control thread, which only touches a member
// the scheduler can no longer reach.
class Member {
 public:
  explicit Member(RobotProfile profile);
  ~Member();
  Member(const Member&) = delete;
  Member& operator=(const Member&) = delete;

  const RobotProfile& profile() const;
  const RateOverrides& overrides() const;
  void setOverrides(const RateOverrides& overrides);
  void activate(InputPool& pool);
  void tick(const TickContext& context, TickBatch& batch);
  Shown shown() const;
  void appendFixed(std::vector<geometry_msgs::TransformStamped>* output) const;
  // Deletions for what this robot put on the shared outputs; changes nothing.
  Retraction retraction(const ros::Time& now) const;
  // The shared outputs were shut down: forget what was sent on them.
  void resetOutputs();
  // Shut the robot's own resources down and return the deletions for the shared outputs.
  Retraction retire(const ros::Time& now);
  Json::Value counters() const;
  Json::Value peerStatus() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace xgc2_ros_visualizer

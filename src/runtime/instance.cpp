#include "instance.hpp"
#include "member.hpp"
#include "../relay/relay.hpp"
#include "../transport/demand.hpp"
#include "../transport/zenoh_input.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <stdexcept>
#include <foxglove_msgs/SceneUpdate.h>
#include <std_msgs/Empty.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/MarkerArray.h>

namespace xgc2_ros_visualizer {
namespace {
constexpr const char* kWorld = "world";

std::int64_t steadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool valid(const ros::Publisher& publisher) { return static_cast<bool>(publisher); }

// Which shared publishers the policy and the membership call for.
struct Need {
  bool tf{false}, statics{false}, scene{false}, markers{false}, scene_ready{false}, description_ready{false};
  bool operator==(const Need& o) const {
    return tf == o.tf && statics == o.statics && scene == o.scene && markers == o.markers &&
        scene_ready == o.scene_ready && description_ready == o.description_ready;
  }
};
struct Outputs {
  ros::Publisher tf, tf_static, standard_tf, scene, scene_ar, height, height_ar, markers, scene_ready, description_ready;
  std::array<ros::Publisher, 4> boundary;
  std::vector<std::shared_ptr<DemandToken>> demand;
};
using Members = std::map<std::string, std::shared_ptr<Member>>;

Need needOf(const ScenePolicy& policy, const Members& members) {
  const bool scene_robots = std::any_of(members.begin(), members.end(),
      [](const Members::value_type& entry) { return entry.second->profile().hasScene(); });
  Need need;
  need.tf = policy.publish_transforms;
  need.scene = policy.publish_scene;
  need.markers = policy.publish_markers && scene_robots;
  need.statics = policy.publish_transforms || !members.empty();
  need.scene_ready = policy.publish_scene || policy.publish_transforms || !members.empty();
  need.description_ready = !members.empty();
  return need;
}
template<class Message>
ros::Publisher advertiseDemand(ros::NodeHandle& node,const std::string& topic,int queue,bool latch,
                               Outputs& outputs,const std::shared_ptr<SharedDemand>& demand) {
  auto token=std::make_shared<DemandToken>(demand);outputs.demand.push_back(token);
  ros::AdvertiseOptions options;options.init<Message>(topic,queue);options.latch=latch;
  options.connect_cb=[token](const ros::SingleSubscriberPublisher& event){token->connect(event);};
  options.disconnect_cb=[token](const ros::SingleSubscriberPublisher& event){token->disconnect(event);};
  return node.advertise(options);
}
Outputs advertise(const Need& need,InputPool& pool,const std::shared_ptr<SharedDemand>& demand) {
  Outputs result;
  ros::NodeHandle node=pool.node("/xgc/visualizer-output-demand");
  if (need.tf) {
    result.tf = advertiseDemand<tf2_msgs::TFMessage>(node,"/xgc/tf",10,false,result,demand);
  }
  if (need.statics) {
    result.tf_static = node.advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);
    result.standard_tf = node.advertise<tf2_msgs::TFMessage>("/tf", 10, false);
  }
  if (need.scene) {
    result.scene = advertiseDemand<foxglove_msgs::SceneUpdate>(node,"/xgc/scene",1,true,result,demand);
    result.scene_ar = advertiseDemand<foxglove_msgs::SceneUpdate>(node,kIdentityArTopic,1,true,result,demand);
    result.height = advertiseDemand<foxglove_msgs::SceneUpdate>(node,kUavHeightProjectionTopic,1,true,result,demand);
    result.height_ar = advertiseDemand<foxglove_msgs::SceneUpdate>(node,kUavHeightProjectionArTopic,1,true,result,demand);
    const char* topics[] = {kWorldBoundaryTopic, kWorldBoundaryArTopic, kWorldBoundaryWallsTopic, kWorldBoundaryWallsArTopic};
    for (std::size_t i = 0; i < 4; ++i) result.boundary[i] = node.advertise<foxglove_msgs::SceneUpdate>(topics[i], 1, true);
  }
  if (need.markers) result.markers = advertiseDemand<visualization_msgs::MarkerArray>(node,"markers",1,false,result,demand);
  if (need.scene_ready) result.scene_ready = node.advertise<std_msgs::Empty>("/xgc/robot_scene/ready", 1, true);
  if (need.description_ready) result.description_ready = node.advertise<std_msgs::Empty>("/xgc/robot_descriptions/ready", 1, true);
  return result;
}
void publishRetraction(const Outputs& outputs, const Retraction& retraction) {
  if (valid(outputs.scene)) outputs.scene.publish(retraction.scene);
  if (valid(outputs.scene_ar)) outputs.scene_ar.publish(retraction.scene_ar);
  if (valid(outputs.height)) outputs.height.publish(retraction.height);
  if (valid(outputs.height_ar)) outputs.height_ar.publish(retraction.height_ar);
  if (valid(outputs.markers)) outputs.markers.publish(retraction.markers);
}
// Retract what the outputs show, then end them. Fixed boundary layers are
// deleted by identity; a latched topic is never cleared as a whole.
void closeOutputs(Outputs& outputs, const Retraction& retraction, const ros::Time& now) {
  for(auto& token:outputs.demand)token->close();
  outputs.demand.clear();
  publishRetraction(outputs, retraction);
  for (std::size_t i = 0; i < 4; ++i) if (valid(outputs.boundary[i]) && !now.isZero()) {
    foxglove_msgs::SceneUpdate update;
    update.deletions.push_back(i < 2 ? worldBoundaryDeletion(now) : worldWallsDeletion(now));
    outputs.boundary[i].publish(update);
  }
  for (auto* publisher : {&outputs.tf, &outputs.tf_static, &outputs.standard_tf, &outputs.scene,
                          &outputs.scene_ar, &outputs.height, &outputs.height_ar, &outputs.markers,
                          &outputs.scene_ready, &outputs.description_ready})
    publisher->shutdown();
  for (auto& publisher : outputs.boundary) publisher.shutdown();
}
Json::Value revisionJson(std::uint64_t desired, std::uint64_t applied) {
  Json::Value result(Json::objectValue);
  result["desiredRevision"] = Json::UInt64(desired);
  result["appliedRevision"] = Json::UInt64(applied);
  result["persistedRevision"] = Json::Value();
  return result;
}
} // namespace

class Instance::Impl {
 public:
  struct RelayRecord {
    RelayConfig config;
    std::unique_ptr<Relay> relay;
  };
  struct Revision {
    std::uint64_t desired{1}, applied{1};
    std::uint64_t generation{1};  // how often this robot's resources were built
  };

  explicit Impl(std::string name) : id(std::move(name)) {
    for (auto& row : counts) for (auto& value : row) value.store(0);
    // Nothing is published until the first apply asks for it.
    spec.scene.publish_transforms = spec.scene.publish_scene = false;
    scene = spec.scene;
  }
  ~Impl() { try { stop(); } catch (...) {} }

  // Make the actual state equal to `target`. Prepared members are built first, so
  // a profile that cannot be loaded changes nothing.
  void transition(const InstanceSpec& target, InputPool& pool) {
    const auto endpoint=remoteListen(target);
    // Bind once at configuration time, independently of ROS consumers. The
    // demand gates still declare only the keys with downstream consumers.
    auto next_listener=listener;
    if(endpoint!=listener_endpoint)
      next_listener=endpoint.empty()?nullptr:ZenohListener::open(endpoint);
    Members built;
    for (const auto& entry : target.robots) {
      const auto found = members.find(entry.first);
      if (found != members.end() && found->second->profile() == entry.second) continue;
      auto member = std::make_shared<Member>(entry.second);
      if (found != members.end()) member->setOverrides(found->second->overrides());
      built[entry.first] = std::move(member);
    }
    const ros::Time now = native_active ? ros::Time::now() : ros::Time();
    std::vector<std::shared_ptr<Member>> retired;
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (auto it = members.begin(); it != members.end();) {
        if (!target.robots.count(it->first) || built.count(it->first)) {
          retired.push_back(std::move(it->second));
          it = members.erase(it);
        } else {
          ++it;
        }
      }
      if (scene.boundary_json != target.scene.boundary_json || scene.boundary_mode != target.scene.boundary_mode) boundary_sent = false;
      scene = target.scene;
      history_max_rate_hz = target.history_max_rate_hz;
      history_visible = target.history_visible;
      prediction_max_rate_hz = target.prediction_max_rate_hz;
      static_dirty = true;
    }
    Retraction retraction;
    for (auto& member : retired) retraction.merge(member->retire(now));
    publishRetraction(outputs, retraction);
    if (native_active) for (auto& entry : built) entry.second->activate(pool, demand);
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (auto& entry : built) members[entry.first] = entry.second;
    }
    if (native_active) {
      reconcileOutputs(pool);
      applyRelays(target.scene.relays, pool);
    }
    listener=std::move(next_listener);
    listener_endpoint=endpoint;
  }

  // The shared publishers follow the policy and the membership. When what they
  // call for changes, they are retracted and advertised again as a whole; a
  // change of members that leaves it as it is touches no shared output.
  void reconcileOutputs(InputPool& pool) {
    const Need wanted = needOf(scene, members);
    if (wanted == need) return;
    const ros::Time now = ros::Time::now();
    Outputs stale;
    Retraction retraction;
    {
      std::lock_guard<std::mutex> lock(mutex);
      stale = outputs;
      outputs = Outputs();
      need = Need();
      for (auto& entry : members) {
        retraction.merge(entry.second->retraction(now));
        entry.second->resetOutputs();
      }
    }
    closeOutputs(stale, retraction, now);
    Outputs fresh = advertise(wanted,pool,demand);
    std::lock_guard<std::mutex> lock(mutex);
    outputs = fresh;
    need = wanted;
    static_dirty = true;
    boundary_sent = scene_ready_sent = description_ready_sent = false;
  }

  void applyRelays(const std::vector<RelayConfig>& wanted, InputPool& pool) {
    std::map<std::string, RelayConfig> by_source;
    for (const auto& relay : wanted) by_source.emplace(relay.source, relay);
    std::vector<std::unique_ptr<Relay>> stale;
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (auto it = relays.begin(); it != relays.end();) {
        const auto found = by_source.find(it->first);
        if (found == by_source.end() || !(found->second == it->second.config)) {
          stale.push_back(std::move(it->second.relay));
          it = relays.erase(it);
        } else {
          ++it;
        }
      }
    }
    for (auto& relay : stale) relay->stop();
    for (const auto& entry : by_source) {
      if (relays.count(entry.first)) continue;
      RelayRecord record{entry.second, makeRelay(pool, entry.second)};
      std::lock_guard<std::mutex> lock(mutex);
      relays.emplace(entry.first, std::move(record));
    }
  }

  void stop() {
    Members retired;
    std::map<std::string, RelayRecord> stopped;
    Outputs stale;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!active.exchange(false)) return;
      retired.swap(members);
      stopped.swap(relays);
      stale = outputs;
      outputs = Outputs();
      need = Need();
    }
    const ros::Time now = native_active ? ros::Time::now() : ros::Time();
    Retraction retraction;
    for (auto& entry : retired) retraction.merge(entry.second->retire(now));
    closeOutputs(stale, retraction, now);
    for (auto& entry : stopped) entry.second.relay->stop();
    listener.reset();
    listener_endpoint.clear();
  }

  std::shared_ptr<ZenohListener> listener;
  std::string listener_endpoint;
  std::string id;
  std::atomic<bool> active{true};
  // The actual, applied state; the control thread alone writes it.
  InstanceSpec spec;
  bool created{false};
  bool native_active{false}; // Published under mutex; only the control worker writes.
  std::uint64_t desired_revision{1}, applied_revision{1};
  std::int64_t applied_ns{steadyNs()};
  std::map<std::string, Revision> revisions;
  // Guarded by `mutex`, which the scheduler holds for a whole tick. The control
  // thread is the only writer and reads without it.
  std::mutex mutex;
  ScenePolicy scene;
  Need need;
  Outputs outputs;
  Members members;
  std::map<std::string, RelayRecord> relays;
  std::shared_ptr<SharedDemand> demand{std::make_shared<SharedDemand>()};
  std::array<RateGate, kChannelCount> gates;  // the global row: root TF, statics, boundary, readiness
  bool static_dirty{true}, boundary_sent{false}, scene_ready_sent{false}, description_ready_sent{false};
  double history_max_rate_hz{2};
  bool history_visible{true};
  double prediction_max_rate_hz{2};
  TickBatch batch;
  Counters counts;
};

Instance::Instance(std::string id) : impl_(new Impl(std::move(id))) {}
Instance::~Instance() = default;
const std::string& Instance::id() const { return impl_->id; }
const InstanceSpec& Instance::spec() const { return impl_->spec; }
bool Instance::hasRobot(const std::string& robot) const { return impl_->members.count(robot) != 0; }
std::uint64_t Instance::robotRevision(const std::string& robot) const { return impl_->revisions.at(robot).desired; }
RateOverrides Instance::robotRates(const std::string& robot) const { return impl_->members.at(robot)->overrides(); }
std::set<std::string> Instance::claims() const { return xgc2_ros_visualizer::claims(impl_->spec); }
void Instance::deactivate() { impl_->stop(); }
void Instance::activate(InputPool& pool) {
  auto& p = *impl_;
  if (p.native_active) return;
  for (auto& entry : p.members) entry.second->activate(pool,p.demand);
  p.reconcileOutputs(pool);
  p.applyRelays(p.spec.scene.relays, pool);
  std::lock_guard<std::mutex> lock(p.mutex);
  p.native_active = true;
}

Change Instance::apply(const InstanceSpec& next, InputPool& pool) {
  auto& p = *impl_;
  checkInstance(next);
  if (!p.active.load()) throw std::logic_error("instance is deactivated");
  const bool creation = !p.created;
  p.created = true;
  Change change = diffInstance(p.spec, next);
  if (change.empty()) return change;
  const InstanceSpec previous = p.spec;
  std::map<std::string, RateOverrides> saved;
  for (const auto& entry : p.members) saved[entry.first] = entry.second->overrides();
  try {
    p.transition(next, pool);
  } catch (...) {
    try {
      p.transition(previous, pool);
      std::lock_guard<std::mutex> lock(p.mutex);
      for (auto& entry : p.members) {
        const auto found = saved.find(entry.first);
        if (found != saved.end()) entry.second->setOverrides(found->second);
      }
    } catch (...) {
      // The previous state could not be restored either; the original error is the one to report.
    }
    throw;
  }
  p.spec = next;
  if (!creation) ++p.desired_revision;
  p.applied_revision = p.desired_revision;
  p.applied_ns = steadyNs();
  for (const auto& id : change.removed) p.revisions.erase(id);
  for (const auto& id : change.added) p.revisions[id] = Impl::Revision();
  for (const auto& id : change.rebuilt) {
    auto& revision = p.revisions[id];
    revision.applied = ++revision.desired;
    ++revision.generation;
  }
  return change;
}

bool Instance::setRobotRates(const std::string& robot, const RateOverrides& overrides) {
  auto& p = *impl_;
  const auto found = p.members.find(robot);
  if (found == p.members.end()) throw std::out_of_range("unknown robot");
  if (found->second->overrides() == overrides) return false;
  {
    std::lock_guard<std::mutex> lock(p.mutex);
    found->second->setOverrides(overrides);
  }
  auto& revision = p.revisions[robot];
  revision.applied = ++revision.desired;
  return true;
}

void Instance::tick(const Rates& table, const ros::Time& now) {
  auto& p = *impl_;
  std::lock_guard<std::mutex> lock(p.mutex);
  if (!p.active.load() || !p.native_active) return;
  const Outputs& o = p.outputs;
  TickContext context{table, p.scene.publish_transforms && valid(o.tf), p.scene.publish_scene && valid(o.scene),
                      p.scene.publish_markers && valid(o.markers), now, p.counts,
                      p.history_max_rate_hz, p.history_visible};
  TickBatch& b = p.batch;
  b.clear();
  for (auto& entry : p.members) entry.second->tick(context, b);
  auto counted = [&](RateKind kind, Channel channel) {
    ++p.counts[static_cast<std::size_t>(kind)][static_cast<std::size_t>(channel)];
  };
  if (valid(o.tf)) {
    if (!b.transforms.transforms.empty()) o.tf.publish(b.transforms);
    if (!b.joints.transforms.empty()) o.tf.publish(b.joints);
  }
  if (valid(o.markers) && !b.markers.markers.empty()) o.markers.publish(b.markers);
  if (valid(o.scene)) {
    if (!b.scene.entities.empty() || !b.scene.deletions.empty()) {
      // Robots that do not stream scene paths announce their label once; the
      // layer a late subscriber receives must still hold it.
      std::set<std::string> present;
      for (const auto& entity : b.scene.entities) present.insert(entity.id);
      for (const auto& entry : p.members) {
        const Shown shown = entry.second->shown();
        if (!shown.streams_scene_paths && shown.label && !present.count(shown.label->id)) b.scene.entities.push_back(*shown.label);
      }
      o.scene.publish(b.scene);
    }
    if (b.scene_ar_changed) {
      foxglove_msgs::SceneUpdate layer;
      for (const auto& entry : p.members) if (const auto* entity = entry.second->shown().ar_label) layer.entities.push_back(*entity);
      o.scene_ar.publish(layer);
    }
    if (b.height_changed) {
      foxglove_msgs::SceneUpdate layer = b.height;
      for (const auto& entry : p.members) if (const auto* entity = entry.second->shown().height) layer.entities.push_back(*entity);
      o.height.publish(layer);
      counted(RateKind::Fs150, Channel::HeightProjection);
    }
    if (b.height_ar_changed) {
      foxglove_msgs::SceneUpdate layer = b.height_ar;
      for (const auto& entry : p.members) if (const auto* entity = entry.second->shown().height_ar) layer.entities.push_back(*entity);
      o.height_ar.publish(layer);
      counted(RateKind::Fs150, Channel::HeightProjectionAr);
    }
  }
  std::array<bool, kChannelCount> global{};
  for (Channel channel : {Channel::TfStatic, Channel::WorldBoundary, Channel::Readiness})
    global[static_cast<std::size_t>(channel)] = p.gates[static_cast<std::size_t>(channel)].take(now, table.get(RateKind::Global, channel));
  if (valid(o.tf_static) && p.static_dirty && global[static_cast<std::size_t>(Channel::TfStatic)]) {
    tf2_msgs::TFMessage statics;
    statics.transforms.push_back(algorithmOverlayFrameAlias(kWorld, ros::Time(0)));
    statics.transforms.push_back(worldFixedFrameRoot(kWorld, ros::Time(0)));
    for (const auto& entry : p.members) entry.second->appendFixed(&statics.transforms);
    o.tf_static.publish(statics);
    p.static_dirty = false;
    counted(RateKind::Global, Channel::TfStatic);
  }
  if (valid(o.standard_tf) && !b.description_joints.transforms.empty()) o.standard_tf.publish(b.description_joints);
  if (valid(o.scene) && !p.boundary_sent && !now.isZero() && global[static_cast<std::size_t>(Channel::WorldBoundary)]) {
    const auto layers = worldBoundaryLayerMessages(p.scene.boundary, p.scene.boundary_mode, now, kWorld);
    o.boundary[0].publish(layers.ground);
    o.boundary[1].publish(layers.ground);
    o.boundary[2].publish(layers.walls);
    o.boundary[3].publish(layers.walls);
    p.boundary_sent = true;
    counted(RateKind::Global, Channel::WorldBoundary);
  }
  if (global[static_cast<std::size_t>(Channel::Readiness)]) {
    if (valid(o.scene_ready) && !p.scene_ready_sent) {
      o.scene_ready.publish(std_msgs::Empty());
      p.scene_ready_sent = true;
      counted(RateKind::Global, Channel::Readiness);
    }
    if (valid(o.description_ready) && !p.description_ready_sent) {
      o.description_ready.publish(std_msgs::Empty());
      p.description_ready_sent = true;
      counted(RateKind::Global, Channel::Readiness);
    }
  }
  const auto wall = std::chrono::steady_clock::now();
  for (auto& entry : p.relays)
    if (entry.second.relay->publish(table, wall, p.prediction_max_rate_hz, now))
      counted(entry.second.config.kind, entry.second.config.channel);
}

double Instance::maximumOverride() const {
  auto& p = *impl_;
  std::lock_guard<std::mutex> lock(p.mutex);
  double result = 0;
  for (const auto& entry : p.members) result = std::max(result, entry.second->overrides().maximum());
  return result;
}

Json::Value Instance::status() const {
  const auto& p = *impl_;
  Json::Value result(Json::objectValue);
  result["ok"] = true;
  result["id"] = p.id;
  result["ready"] = p.active.load();
  std::size_t scene_robots = 0;
  Json::Value robots(Json::arrayValue);
  for (const auto& entry : p.members) {
    const RobotProfile& profile = entry.second->profile();
    if (profile.hasScene()) ++scene_robots;
    Json::Value row(Json::objectValue);
    const auto& revision = p.revisions.at(entry.first);
    row["id"] = entry.first;
    row["kind"] = kindName(profile.kind);
    row["scene"] = profile.hasScene();
    row["configuration"] = revisionJson(revision.desired, revision.applied);
    row["generation"] = Json::UInt64(revision.generation);
    row["input"] = entry.second->peerStatus();
    robots.append(row);
  }
  result["robotCount"] = Json::UInt64(scene_robots);
  result["descriptionCount"] = Json::UInt64(p.members.size());
  result["relayCount"] = Json::UInt64(p.spec.scene.relays.size());
  result["configuration"] = revisionJson(p.desired_revision, p.applied_revision);
  result["configuration"]["state"] = "applied";
  result["configuration"]["appliedAtSteadyNs"] = Json::Int64(p.applied_ns);
  result["robots"] = robots;
  for (std::size_t k = 0; k < kKindCount; ++k) for (std::size_t c = 0; c < kChannelCount; ++c)
    if (applicable(static_cast<RateKind>(k), static_cast<Channel>(c)))
      result["publicationCounters"][kindName(static_cast<RateKind>(k))][channelName(static_cast<Channel>(c))] = Json::UInt64(p.counts[k][c].load());
  std::uint64_t relay_count = 0;
  for (const auto& entry : p.relays) relay_count += entry.second.relay->count();
  result["relayPublications"] = Json::UInt64(relay_count);
  return result;
}

Json::Value Instance::robotStatus(const std::string& robot, const Rates& table) const {
  const auto& p = *impl_;
  const Member& member = *p.members.at(robot);
  const RobotProfile& profile = member.profile();
  const auto& revision = p.revisions.at(robot);
  Json::Value result(Json::objectValue);
  result["ok"] = true;
  result["instanceId"] = p.id;
  result["id"] = robot;
  result["kind"] = kindName(profile.kind);
  result["scene"] = profile.hasScene();
  result["ready"] = p.active.load();
  result["configuration"] = revisionJson(revision.desired, revision.applied);
  result["generation"] = Json::UInt64(revision.generation);
  result["input"] = member.peerStatus();
  result["profile"] = profile.json();
  Json::Value effective(Json::objectValue);
  for (std::size_t c = 0; c < kChannelCount; ++c) {
    const auto channel = static_cast<Channel>(c);
    if (robotChannel(profile.kind, channel))
      effective[channelName(channel)] = member.overrides().get(channel, table.get(profile.kind, channel));
  }
  result["rates"]["overrides"] = member.overrides().json();
  result["rates"]["effective"] = effective;
  result["publicationCounters"] = member.counters();
  return result;
}
} // namespace xgc2_ros_visualizer

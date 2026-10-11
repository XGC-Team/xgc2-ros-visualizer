#include "member.hpp"
#include "descriptions.hpp"
#include <xgc2_ros_visualizer/source_history.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <render/robots/fs150_uav_visualizer.hpp>
#include <render/robots/scout_ugv_visualizer.hpp>
#include <render/robots/mecanum_ugv_visualizer.hpp>
#include <render/robots/robot_frames.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <nav_msgs/Path.h>

namespace xgc2_ros_visualizer {
namespace {
bool finite(const geometry_msgs::Pose& p) {
  return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&std::isfinite(p.position.z)&&
    std::isfinite(p.orientation.x)&&std::isfinite(p.orientation.y)&&std::isfinite(p.orientation.z)&&std::isfinite(p.orientation.w);
}
bool finite(const geometry_msgs::Twist& t) {
  return std::isfinite(t.linear.x)&&std::isfinite(t.linear.y)&&std::isfinite(t.linear.z)&&std::isfinite(t.angular.x)&&std::isfinite(t.angular.y)&&std::isfinite(t.angular.z);
}
geometry_msgs::Pose normalized(geometry_msgs::Pose p) {
  auto& q=p.orientation; const double n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  if (!std::isfinite(n)||n<1e-9) { q.x=q.y=q.z=0; q.w=1; }
  else { q.x/=n; q.y/=n; q.z/=n; q.w/=n; }
  return p;
}
double yaw(const geometry_msgs::Pose& p) {
  auto q=normalized(p).orientation; return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
bool fresh(const ros::Time& stamp,const ros::Time& now,double timeout) { return !stamp.isZero() && now-stamp<=ros::Duration(timeout); }
struct PoseValue {
  geometry_msgs::Pose pose;
  ros::Time stamp;
  // Fixed state stores the accepted frame category, rather than copying strings under its lock.
  std::uint8_t frame{0};
  bool available{false};
  CanonicalPoseSample sample() const {
    CanonicalPoseSample result; result.available=available; result.pose=pose; result.stamp=stamp;
    result.frame_id=frame==1?"world":frame==2?"map":"invalid"; return result;
  }
};
struct RobotState {
  PoseValue pose,ar;
  geometry_msgs::Twist cmd,twist;
  ros::Time cmd_stamp,twist_stamp,state_stamp,extended_stamp;
  bool cmd_available{false},twist_available{false},twist_world{false},state_available{false},extended_available{false},armed{false};
  std::uint8_t landed{0};
};
struct RobotInput {
  std::mutex mutex;
  RobotState state;
  SourceHistory path,ar_path;
};
foxglove_msgs::SceneEntityDeletion deletion(const std::string& id,const ros::Time& stamp) {
  foxglove_msgs::SceneEntityDeletion result; result.type=foxglove_msgs::SceneEntityDeletion::MATCHING_ID; result.id=id; result.timestamp=stamp; return result;
}
template<class Update> void append(std::vector<Update>* to, std::vector<Update>&& from) {
  to->insert(to->end(),std::make_move_iterator(from.begin()),std::make_move_iterator(from.end()));
}
} // namespace

bool RateGate::take(const ros::Time& now,double hz) {
  if (!initialized || now<last) { initialized=true;last=now;return true; }
  const double elapsed=(now-last).toSec(),period=1/hz;
  if (elapsed+1e-9<period) return false;
  last+=ros::Duration(std::floor((elapsed+1e-9)/period)*period); return true;
}

void TickBatch::clear() {
  transforms.transforms.clear();joints.transforms.clear();description_joints.transforms.clear();markers.markers.clear();
  scene.entities.clear();scene.deletions.clear();scene_ar.entities.clear();scene_ar.deletions.clear();
  height.entities.clear();height.deletions.clear();height_ar.entities.clear();height_ar.deletions.clear();
  scene_ar_changed=height_changed=height_ar_changed=false;
}

void Retraction::merge(Retraction&& other) {
  append(&scene.deletions,std::move(other.scene.deletions));
  append(&scene_ar.deletions,std::move(other.scene_ar.deletions));
  append(&height.deletions,std::move(other.height.deletions));
  append(&height_ar.deletions,std::move(other.height_ar.deletions));
  append(&markers.markers,std::move(other.markers.markers));
}

class Member::Impl {
 public:
  explicit Impl(RobotProfile value) : profile(std::move(value)), input(new RobotInput) {
    description.reset(new Description(profile));
    label_offsets.uav=label_offsets.scout=label_offsets.mecanum=profile.label_offset;
    if (!profile.hasScene()) return;
    const auto& a=profile.animation;
    Fs150UavVisualizer::Config u;u.frame_id=profile.frame_id;u.mesh_scale=profile.mesh_scale;
    ScoutUgvVisualizer::Config g;g.frame_id=profile.frame_id;g.mesh_scale=profile.mesh_scale;
    g.visual_wheel_radius=a.wheel_radius;g.visual_track_width=a.track_width;g.wheel_motion_deadband=a.wheel_deadband;g.max_visual_wheel_speed_rad_s=a.wheel_max_speed;
    MecanumUgvVisualizer::Config m;m.frame_id=profile.frame_id;m.mesh_scale=profile.mesh_scale;
    m.visual_wheel_radius=a.wheel_radius;m.visual_wheelbase_plus_track=a.wheelbase_plus_track;m.wheel_motion_deadband=a.wheel_deadband;m.max_visual_wheel_speed_rad_s=a.wheel_max_speed;
    // Extract the exact robot path metadata in cold configuration, with a renderer
    // of its own so the running animation state does not advance. Runtime points
    // come from the source-time ring.
    visualization_msgs::MarkerArray prototypes;
    const auto kind=profile.modelKind();
    if(kind==RobotModelKind::kFs150) {
      uav.reset(new Fs150UavVisualizer(u));Fs150UavVisualizer path_uav(u);
      for(unsigned sample=0;sample<2;++sample) {
        UavVisualState state;state.name=profile.scene_model;state.stamp=ros::Time(1+sample);state.pose.orientation.w=1;
        path_uav.append(state,&prototypes,nullptr,false,true,false);
      }
    } else if(kind==RobotModelKind::kScout) {
      scout.reset(new ScoutUgvVisualizer(g));ScoutUgvVisualizer path_scout(g);
      for(unsigned sample=0;sample<2;++sample) {
        UgvVisualState state;state.name=profile.scene_model;state.stamp=ros::Time(1+sample);state.pose.orientation.w=1;
        path_scout.append(state,&prototypes,nullptr,false,true,false);
      }
    } else {
      mecanum.reset(new MecanumUgvVisualizer(m));MecanumUgvVisualizer path_mecanum(m);
      for(unsigned sample=0;sample<2;++sample) {
        MecanumVisualState state;state.name=profile.scene_model;state.stamp=ros::Time(1+sample);state.pose.orientation.w=1;
        path_mecanum.append(state,&prototypes,nullptr,false,true,false);
      }
    }
    for(const auto& marker:prototypes.markers)if(marker.type==visualization_msgs::Marker::LINE_STRIP)path_marker=marker;
    path_marker.points.clear();path_marker.points.reserve(61);
    if (profile.publication.paths) {
      path_message.header.frame_id=profile.frame_id;path_message.poses.reserve(61);
      if (!profile.ar_path_topic.empty()) {ar_path_message.header.frame_id=profile.frame_id;ar_path_message.poses.reserve(61);}
    }
  }
  void counted(const TickContext& context,Channel channel) {
    ++counts[static_cast<std::size_t>(channel)];
    ++context.totals[static_cast<std::size_t>(profile.kind)][static_cast<std::size_t>(channel)];
  }
  void subscribe(InputPool& pool) {
    const auto& a=profile.animation;const std::string space=profile.rosNamespace();
    auto data=input;auto node=pool.node(space);
    const auto kind=profile.modelKind();
    const double timeout=a.pose_timeout;
    subscribers.push_back(node.subscribe<geometry_msgs::PoseStamped>(profile.pose_topic,1,[data,kind,timeout](const geometry_msgs::PoseStampedConstPtr& msg) {
      if (!finite(msg->pose)) return;
      PoseValue value;value.available=true;value.pose=normalized(msg->pose);value.stamp=msg->header.stamp;
      value.frame=isWorldFixedFrame(msg->header.frame_id)?1:(msg->header.frame_id=="map"||msg->header.frame_id=="/map")?2:0;
      const bool accepted=value.frame==1||(kind==RobotModelKind::kFs150&&value.frame==2);
      const bool record=accepted&&fresh(value.stamp,ros::Time::now(),timeout);
      const auto path_pose=slotHistoryPathPose(kind,value.pose);
      std::lock_guard<std::mutex> lock(data->mutex);data->state.pose=value;
      if(record)data->path.append(value.stamp,path_pose);
    },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay()));
    if (!profile.ar_pose_topic.empty()) {
      const auto offset=profile.world_offset;
      subscribers.push_back(node.subscribe<geometry_msgs::PoseStamped>(profile.ar_pose_topic,1,[data,offset](const geometry_msgs::PoseStampedConstPtr& msg) {
        if (!finite(msg->pose)) return;
        PoseValue value;value.available=true;value.frame=1;value.pose=applyExperimentWorldOffsetOnce(msg->pose,offset);value.stamp=msg->header.stamp;
        if (!finite(value.pose)) return;
        std::lock_guard<std::mutex> lock(data->mutex);data->state.ar=value;
        data->ar_path.append(value.stamp,value.pose);
      },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay()));
    }
    if (kind==RobotModelKind::kFs150) {
      subscribers.push_back(node.subscribe<mavros_msgs::State>(space+"/mavros/state",1,[data](const mavros_msgs::StateConstPtr& msg) {
        auto stamp=msg->header.stamp.isZero()?ros::Time::now():msg->header.stamp;
        std::lock_guard<std::mutex> lock(data->mutex);data->state.armed=msg->armed;data->state.state_stamp=stamp;data->state.state_available=true;
      }));
      subscribers.push_back(node.subscribe<mavros_msgs::ExtendedState>(space+"/mavros/extended_state",1,[data](const mavros_msgs::ExtendedStateConstPtr& msg) {
        const auto now=ros::Time::now();std::lock_guard<std::mutex> lock(data->mutex);
        data->state.landed=msg->landed_state;data->state.extended_stamp=now;data->state.extended_available=true;
      }));
    } else {
      subscribers.push_back(node.subscribe<geometry_msgs::Twist>(space+"/cmd_vel",1,[data](const geometry_msgs::TwistConstPtr& msg) {
        if(!finite(*msg)) return;
        const auto now=ros::Time::now();std::lock_guard<std::mutex> lock(data->mutex);
        data->state.cmd=*msg;data->state.cmd_stamp=now;data->state.cmd_available=true;
      }));
      subscribers.push_back(node.subscribe<geometry_msgs::TwistStamped>(space+"/twist",1,[data](const geometry_msgs::TwistStampedConstPtr& msg) {
        if(!finite(msg->twist)) return;
        const auto stamp=msg->header.stamp.isZero()?ros::Time::now():msg->header.stamp;
        const bool world=isWorldFixedFrame(msg->header.frame_id);std::lock_guard<std::mutex> lock(data->mutex);
        data->state.twist=msg->twist;data->state.twist_stamp=stamp;data->state.twist_world=world;data->state.twist_available=true;
      }));
    }
    if (profile.publication.paths) {
      path_pub=node.advertise<nav_msgs::Path>(space+"/"+profile.path_topic,1,true);
      if (!profile.ar_path_topic.empty()) ar_path_pub=node.advertise<nav_msgs::Path>(space+"/"+profile.ar_path_topic,1,true);
    }
  }
  void motion(const RobotState& sample,const geometry_msgs::Pose& pose,const ros::Time& now,double* forward,double* lateral,double* angular,bool* hint) const {
    *forward=*lateral=*angular=0;*hint=false;
    const double timeout=profile.animation.motion_timeout;
    const bool twist=sample.twist_available&&fresh(sample.twist_stamp,now,timeout);
    const bool cmd=sample.cmd_available&&fresh(sample.cmd_stamp,now,timeout);
    if (!twist&&!cmd) return;
    const auto& value=twist?sample.twist:sample.cmd;*hint=true;*angular=value.angular.z;
    if (twist&&sample.twist_world) { const double a=yaw(pose);*forward=value.linear.x*std::cos(a)+value.linear.y*std::sin(a);*lateral=-value.linear.x*std::sin(a)+value.linear.y*std::cos(a); }
    else { *forward=value.linear.x;*lateral=value.linear.y; }
  }
  void publishHistory(const TickContext& context,bool ar) {
    auto& revision=ar?ar_path_revision:path_revision;
    std::unique_lock<std::mutex> lock(input->mutex);
    auto& history=ar?input->ar_path:input->path;
    history.expire(context.now);
    if(revision==history.revision)return;
    const SourceHistory snapshot=history;
    lock.unlock();
    auto& message=ar?ar_path_message:path_message;
    message.poses.resize(snapshot.size);message.header.stamp=snapshot.stamp;
    for(std::size_t i=0;i<snapshot.size;++i) {
      auto& point=message.poses[i];point.header.frame_id=profile.frame_id;
      point.header.stamp=snapshot.at(i).stamp;point.pose=snapshot.at(i).pose;
    }
    (ar?ar_path_pub:path_pub).publish(message);revision=snapshot.revision;
    counted(context,ar?Channel::ArPath:Channel::Path);
  }
  void appendVisuals(TickBatch& batch,const RobotState& sample,const CanonicalWorldPose& pose,const ros::Time& now,bool mesh,bool path,bool label,bool joint) {
    batch.scratch.markers.clear();batch.scratch_tf.clear();const auto& a=profile.animation;
    auto* marker_output=(mesh||path||label)?&batch.scratch:nullptr;auto* tf_output=joint?&batch.scratch_tf:nullptr;
    const auto kind=profile.modelKind();
    if(kind==RobotModelKind::kFs150) {
      UavVisualState state;state.name=profile.scene_model;state.pose=pose.pose;state.stamp=now;
      state.rotors_active=sample.state_available&&sample.armed&&fresh(sample.state_stamp,now,a.state_timeout);
      state.rotor_speed_rad_s=a.rotor_airborne;
      if(sample.extended_available&&fresh(sample.extended_stamp,now,a.state_timeout)) {
        if(sample.landed==mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) state.rotor_speed_rad_s=a.rotor_ground;
        else if(sample.landed==mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF||sample.landed==mavros_msgs::ExtendedState::LANDED_STATE_LANDING) state.rotor_speed_rad_s=a.rotor_transition;
      }
      uav->append(state,marker_output,tf_output,mesh,false,label);
    } else {
      double forward,lateral,angular;bool hint;motion(sample,pose.pose,now,&forward,&lateral,&angular,&hint);
      if(kind==RobotModelKind::kMecanum) {
        MecanumVisualState state;state.name=profile.scene_model;state.pose=pose.pose;state.stamp=now;state.has_motion_hint=hint;state.forward_velocity_m_s=forward;state.lateral_velocity_m_s=lateral;state.yaw_rate_rad_s=angular;
        mecanum->append(state,marker_output,tf_output,mesh,false,label);
      } else {
        UgvVisualState state;state.name=profile.scene_model;state.pose=pose.pose;state.stamp=now;state.has_motion_hint=hint;state.forward_velocity_m_s=forward;state.yaw_rate_rad_s=angular;
        scout->append(state,marker_output,tf_output,mesh,false,label);
      }
    }
    if(marker_output) applyRobotMarkerLabel(marker_output,0,kind,profile.rosNamespace());
    if(path) {
      SourceHistory history;
      {
        std::lock_guard<std::mutex> lock(input->mutex);
        input->path.expire(now);history=input->path;
      }
      // Preserve robot marker IDs/style/frame and its two-point visibility threshold.
      if(history.size>=2) {
        path_marker.points.resize(history.size);path_marker.header.stamp=history.stamp;
        for(std::size_t i=0;i<history.size;++i)path_marker.points[i]=history.at(i).pose.position;
        batch.scratch.markers.push_back(path_marker);
      }
    }
    for(const auto& value:batch.scratch_tf) if(value.header.frame_id!=profile.frame_id) batch.joints.transforms.push_back(value);
  }
  void projectHeight(const TickContext& context,TickBatch& batch,const std::array<bool,kChannelCount>& due,const CanonicalPoseSample& canonical,const CanonicalPoseSample& ar,bool image) {
    if(profile.kind!=RateKind::Fs150||profile.height_projection_color.empty()) return;
    const auto channel=image?Channel::HeightProjectionAr:Channel::HeightProjection;
    if(!due[static_cast<std::size_t>(channel)]) return;
    bool& present=image?height_ar_present:height_present;
    ros::Time& stamp=image?height_ar_stamp:height_stamp;
    auto& entity=image?height_ar_entity:height_entity;
    auto pose=selectUavHeightProjectionWorldPose(image?HeightProjectionView::kVrpn:HeightProjectionView::kLocalPosition,canonical,ar,context.now,profile.animation.pose_timeout);
    if(pose.found) {
      if(!present||stamp!=pose.stamp) {
        entity=uavHeightProjectionEntity(profile.scene_model,pose.pose.position,pose.stamp,profile.frame_id,sceneColorFromHex(profile.height_projection_color));
        if(image) batch.height_ar_changed=true;else batch.height_changed=true;
      }
      present=true;stamp=pose.stamp;
    } else if(present) {
      (image?batch.height_ar:batch.height).deletions.push_back(uavHeightProjectionDeletion(profile.scene_model,context.now));present=false;
      if(image) batch.height_ar_changed=true;else batch.height_changed=true;
    }
  }
  void tickScene(const TickContext& context,TickBatch& batch,const std::array<bool,kChannelCount>& due) {
    const auto& now=context.now;const auto kind=profile.modelKind();const double timeout=profile.animation.pose_timeout;
    RobotState sample;{std::lock_guard<std::mutex> lock(input->mutex);sample=input->state;}
    const auto canonical=sample.pose.sample(),ar=sample.ar.sample();
    auto pose=selectSlotVisualizationWorldPose(kind,canonical,now,timeout);
    auto ar_pose=selectArIdentityWorldPose(kind,canonical,ar,now,timeout);
    const auto& publication=profile.publication;
    const bool transforms=context.transforms&&publication.transforms;
    const bool scene=context.scene&&publication.scene;
    const bool scene_paths=context.scene&&publication.scene_paths;
    const bool markers_out=context.markers&&publication.markers;
    auto is=[&](Channel c){return due[static_cast<std::size_t>(c)];};
    if(path_pub&&is(Channel::Path))publishHistory(context,false);
    if(ar_path_pub&&is(Channel::ArPath))publishHistory(context,true);
    if(transforms&&is(Channel::PoseTf)) {
      if(pose.found&&pose.stamp!=pose_tf_stamp) {
        const auto values=canonicalRobotPoseTransforms(kind,profile.scene_model,pose.pose,pose.stamp,profile.frame_id,label_offsets);
        batch.transforms.transforms.insert(batch.transforms.transforms.end(),values.begin(),values.end());pose_tf_stamp=pose.stamp;counted(context,Channel::PoseTf);
      }
      if(ar_pose.found&&ar_pose.stamp!=ar_tf_stamp) {
        batch.transforms.transforms.push_back(canonicalArIdentityLabelTransform(kind,profile.scene_model,ar_pose.pose,ar_pose.stamp,profile.frame_id,label_offsets));ar_tf_stamp=ar_pose.stamp;
      }
    }
    if(scene&&ar_pose.found&&!ar_label_sent&&is(Channel::ArIdentity)) {
      batch.scratch.markers.clear();batch.scratch.markers.push_back(identityLabelMarker(profile.scene_model,xgc2_ros_visualizer::robotFramePrefix(profile.scene_model)+"/label_ar",ar_pose.stamp));
      applyRobotMarkerLabel(&batch.scratch,0,kind,profile.rosNamespace());
      foxglove_msgs::SceneUpdate update;appendSceneEntityPart(kind,profile.id,SceneEntityPart::kArLabel,batch.scratch,0,ar_pose.stamp,profile.frame_id,profile.labelStyle(),&update);
      if(!update.entities.empty()) {ar_label=std::move(update.entities.front());ar_label_sent=true;batch.scene_ar_changed=true;counted(context,Channel::ArIdentity);}
    }
    if(scene){projectHeight(context,batch,due,canonical,ar,false);projectHeight(context,batch,due,canonical,ar,true);}
    if(!pose.found) return;
    const bool markers=markers_out&&is(Channel::Markers),joint=transforms&&is(Channel::JointTf);
    const bool label=scene&&is(Channel::Scene)&&(!label_sent||publication.scene_paths);
    const bool path=scene_paths&&is(Channel::ScenePath);
    if(!(markers||joint||label||path)) return;
    appendVisuals(batch,sample,pose,now,markers,markers||path,markers||label,joint);
    if(markers) {for(const auto& m:batch.scratch.markers)marker_ids.emplace(m.ns,m.id);batch.markers.markers.insert(batch.markers.markers.end(),batch.scratch.markers.begin(),batch.scratch.markers.end());counted(context,Channel::Markers);}
    if(joint) counted(context,Channel::JointTf);
    const auto style=profile.labelStyle();
    if(path) {appendSceneEntityPart(kind,profile.id,SceneEntityPart::kPath,batch.scratch,0,now,profile.frame_id,style,&batch.scene);counted(context,Channel::ScenePath);}
    if(label) {
      foxglove_msgs::SceneUpdate update;appendSceneEntityPart(kind,profile.id,SceneEntityPart::kLabel,batch.scratch,0,now,profile.frame_id,style,&update);
      if(!update.entities.empty()) {label_entity=std::move(update.entities.front());batch.scene.entities.push_back(label_entity);label_sent=true;counted(context,Channel::Scene);}
    }
  }
  RobotProfile profile;
  RateOverrides overrides;
  std::shared_ptr<RobotInput> input;
  std::unique_ptr<Description> description;
  std::unique_ptr<Fs150UavVisualizer> uav;
  std::unique_ptr<ScoutUgvVisualizer> scout;
  std::unique_ptr<MecanumUgvVisualizer> mecanum;
  SceneLabelOffsets label_offsets;
  std::vector<ros::Subscriber> subscribers;
  ros::Publisher path_pub,ar_path_pub;
  nav_msgs::Path path_message,ar_path_message;
  visualization_msgs::Marker path_marker;
  std::uint64_t path_revision{0},ar_path_revision{0};
  bool label_sent{false},ar_label_sent{false},height_present{false},height_ar_present{false};
  foxglove_msgs::SceneEntity label_entity,ar_label,height_entity,height_ar_entity;
  ros::Time pose_tf_stamp,ar_tf_stamp,height_stamp,height_ar_stamp;
  std::set<std::pair<std::string,int>> marker_ids;
  std::array<RateGate,kChannelCount> gates;
  std::array<std::atomic<std::uint64_t>,kChannelCount> counts;
  bool activated{false};
};

Member::Member(RobotProfile profile) : impl_(new Impl(std::move(profile))) {
  for(auto& value:impl_->counts)value.store(0);
}
Member::~Member() = default;
const RobotProfile& Member::profile() const { return impl_->profile; }
const RateOverrides& Member::overrides() const { return impl_->overrides; }
void Member::setOverrides(const RateOverrides& overrides) { impl_->overrides=overrides; }
void Member::appendFixed(std::vector<geometry_msgs::TransformStamped>* output) const { impl_->description->appendFixed(output); }
Shown Member::shown() const {
  const auto& p=*impl_;Shown result;
  result.streams_scene_paths=p.profile.publication.scene_paths;
  if(p.label_sent)result.label=&p.label_entity;
  if(p.ar_label_sent)result.ar_label=&p.ar_label;
  if(p.height_present)result.height=&p.height_entity;
  if(p.height_ar_present)result.height_ar=&p.height_ar_entity;
  return result;
}

void Member::activate(InputPool& pool) {
  auto& p=*impl_;if(p.activated) throw std::logic_error("member already activated");
  p.activated=true;
  try {
    p.description->activate(pool);
    if(p.profile.hasScene()) p.subscribe(pool);
  } catch(...) {retire(ros::Time());throw;}
}

void Member::tick(const TickContext& context,TickBatch& batch) {
  auto& p=*impl_;const auto& profile=p.profile;
  std::array<bool,kChannelCount> due;
  for(std::size_t c=0;c<kChannelCount;++c) {
    const auto channel=static_cast<Channel>(c);
    due[c]=robotChannel(profile.kind,channel)&&p.gates[c].take(context.now,p.overrides.get(channel,context.table.get(profile.kind,channel)));
  }
  if(profile.hasScene()) p.tickScene(context,batch,due);
  if(profile.description.state_publisher&&due[static_cast<std::size_t>(Channel::JointTf)]) {
    const auto before=batch.description_joints.transforms.size();
    p.description->appendJoint(&batch.description_joints.transforms);
    if(batch.description_joints.transforms.size()!=before)p.counted(context,Channel::JointTf);
  }
}

Retraction Member::retraction(const ros::Time& now) const {
  const auto& p=*impl_;const auto& profile=p.profile;Retraction result;
  if(!profile.hasScene()) return result;
  const auto kind=profile.modelKind();
  // Deletions name this robot's own entities; never clear a whole topic.
  if(!now.isZero()) {
    result.scene.deletions.push_back(deletion(sceneEntityPartID(kind,profile.id,SceneEntityPart::kLabel),now));
    result.scene.deletions.push_back(deletion(sceneEntityPartID(kind,profile.id,SceneEntityPart::kPath),now));
    result.scene_ar.deletions.push_back(deletion(sceneEntityPartID(kind,profile.id,SceneEntityPart::kArLabel),now));
    result.height.deletions.push_back(uavHeightProjectionDeletion(profile.scene_model,now));
    result.height_ar.deletions.push_back(uavHeightProjectionDeletion(profile.scene_model,now));
  }
  for(const auto& id:p.marker_ids){visualization_msgs::Marker m;m.ns=id.first;m.id=id.second;m.action=visualization_msgs::Marker::DELETE;result.markers.markers.push_back(m);}
  return result;
}

void Member::resetOutputs() {
  auto& p=*impl_;
  p.label_sent=p.ar_label_sent=p.height_present=p.height_ar_present=false;
  p.marker_ids.clear();
}

Retraction Member::retire(const ros::Time& now) {
  auto& p=*impl_;Retraction result=retraction(now);
  for(auto& subscriber:p.subscribers)subscriber.shutdown();
  for(auto* publisher:{&p.path_pub,&p.ar_path_pub}) if(*publisher) {
    nav_msgs::Path empty;empty.header.frame_id=p.profile.frame_id;empty.header.stamp=now;publisher->publish(empty);
  }
  p.path_pub.shutdown();p.ar_path_pub.shutdown();
  resetOutputs();
  p.description->stop();
  p.subscribers.clear();
  return result;
}

Json::Value Member::counters() const {
  Json::Value result(Json::objectValue);
  for(std::size_t c=0;c<kChannelCount;++c)
    if(robotChannel(impl_->profile.kind,static_cast<Channel>(c))) result[channelName(static_cast<Channel>(c))]=Json::UInt64(impl_->counts[c].load());
  return result;
}
Json::Value Member::peerStatus() const {
  const auto& p=*impl_;
  Json::Value result(Json::objectValue);
  if (!p.profile.hasScene()) { result["state"]="not required"; return result; }
  const auto publishers=p.subscribers.empty()?0:p.subscribers.front().getNumPublishers();
  result["topic"]=p.profile.pose_topic;
  result["publishers"]=publishers;
  result["state"]=publishers?"connected":"waiting for topic";
  return result;
}
} // namespace xgc2_ros_visualizer

#include "descriptions.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Transform.h>
#include <urdf/model.h>
namespace xgc2_ros_visualizer {
class Description::Impl {
 public:
  struct Joint {
    urdf::JointConstSharedPtr definition;
    double position{0};
  };
  struct Input {
    std::mutex mutex;
    sensor_msgs::JointStateConstPtr latest;
    std::uint64_t generation{0};
  };
  explicit Impl(const RobotProfile& configured)
      : name(configured.id),ros_namespace(configured.rosNamespace()),state_publisher(configured.description.state_publisher),
        joint_state_topic(configured.description.joint_state_topic),input(new Input) {
    parameter=ros_namespace+"/visual_robot_description";
    const auto root=ros::package::getPath(configured.description.package);
    if (root.empty()) throw std::invalid_argument("installed description package unavailable: "+configured.description.package);
    std::ifstream file(root+"/"+configured.description.file,std::ios::binary);
    if (!file) throw std::invalid_argument("installed description file unavailable: "+configured.description.file);
    file.seekg(0,std::ios::end); auto bytes=file.tellg(); file.seekg(0);
    if (bytes<=0 || bytes>4*1024*1024) throw std::invalid_argument("URDF must be nonempty and <=4 MiB");
    xml.resize(static_cast<std::size_t>(bytes)); file.read(&xml[0],bytes);
    urdf::Model model;
    if (!file || !model.initString(xml) || model.joints_.size()>2048) throw std::invalid_argument("invalid or oversized visual URDF");
    if (!state_publisher) return;
    // Preserve the original RSP tf_prefix=<roster name>, outside scene SDK frames.
    for (const auto& pair:model.joints_) {
      if (pair.second->type==urdf::Joint::FIXED) fixed.push_back(transform(*pair.second,0,ros::Time(0)));
      else if (pair.second->type==urdf::Joint::REVOLUTE || pair.second->type==urdf::Joint::CONTINUOUS || pair.second->type==urdf::Joint::PRISMATIC)
        joints.push_back({pair.second,0});
    }
  }
  geometry_msgs::TransformStamped transform(const urdf::Joint& joint,double position,const ros::Time& stamp) const {
    const auto& origin=joint.parent_to_joint_origin_transform;
    tf2::Transform value(tf2::Quaternion(origin.rotation.x,origin.rotation.y,origin.rotation.z,origin.rotation.w),tf2::Vector3(origin.position.x,origin.position.y,origin.position.z));
    tf2::Transform motion; motion.setIdentity();
    if (joint.type==urdf::Joint::REVOLUTE || joint.type==urdf::Joint::CONTINUOUS)
      motion.setRotation(tf2::Quaternion(tf2::Vector3(joint.axis.x,joint.axis.y,joint.axis.z),position));
    else if (joint.type==urdf::Joint::PRISMATIC) motion.setOrigin(tf2::Vector3(joint.axis.x,joint.axis.y,joint.axis.z)*position);
    value*=motion;
    geometry_msgs::TransformStamped result;
    result.header.frame_id=name+"/"+joint.parent_link_name; result.child_frame_id=name+"/"+joint.child_link_name; result.header.stamp=stamp;
    result.transform.translation.x=value.getOrigin().x(); result.transform.translation.y=value.getOrigin().y(); result.transform.translation.z=value.getOrigin().z();
    result.transform.rotation.x=value.getRotation().x(); result.transform.rotation.y=value.getRotation().y(); result.transform.rotation.z=value.getRotation().z(); result.transform.rotation.w=value.getRotation().w();
    return result;
  }
  std::string name,ros_namespace;
  bool state_publisher;
  std::string joint_state_topic;
  std::string parameter,xml;
  std::shared_ptr<Input> input;
  ros::Subscriber subscriber;
  ros::Publisher publisher;
  std::vector<geometry_msgs::TransformStamped> fixed;
  std::vector<Joint> joints;
  std::uint64_t applied_generation{0};
  ros::Time joint_stamp;
  bool parameters_set{false};
};
Description::Description(const RobotProfile& profile) : impl_(new Impl(profile)) {}
Description::~Description() { stop(); }
const std::string& Description::parameter() const { return impl_->parameter; }
bool Description::statePublisher() const { return impl_->state_publisher; }
void Description::activate(InputPool& pool) {
  ros::NodeHandle node; node.setParam(impl_->parameter,impl_->xml); impl_->parameters_set=true;
  // A description is a static snapshot. Latched delivery handles both startup
  // orders without making viewers poll the parameter server for new names.
  impl_->publisher=node.advertise<std_msgs::String>(impl_->parameter,1,true);
  std_msgs::String description; description.data=impl_->xml;
  impl_->publisher.publish(description);
  if (!statePublisher()) return;
  node.setParam(impl_->ros_namespace+"/robot_description",impl_->xml);
  auto input=impl_->input;
  const auto topic=impl_->ros_namespace+"/"+impl_->joint_state_topic;
  auto source=pool.node(topic);
  impl_->subscriber=source.subscribe<sensor_msgs::JointState>(topic,1,[input](const sensor_msgs::JointStateConstPtr& message) {
    if (message->name.size()!=message->position.size() || message->name.size()>2048) return;
    for (double value:message->position) if (!std::isfinite(value)) return;
    std::lock_guard<std::mutex> lock(input->mutex); input->latest=message; ++input->generation;
  },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
}
void Description::appendFixed(std::vector<geometry_msgs::TransformStamped>* output) const {
  output->insert(output->end(),impl_->fixed.begin(),impl_->fixed.end());
}
void Description::appendJoint(std::vector<geometry_msgs::TransformStamped>* output) {
  sensor_msgs::JointStateConstPtr message; std::uint64_t generation;
  { std::lock_guard<std::mutex> lock(impl_->input->mutex); message=impl_->input->latest; generation=impl_->input->generation; }
  if (!message || generation==impl_->applied_generation) return;
  for (auto& joint:impl_->joints) {
    const auto found=std::find(message->name.begin(),message->name.end(),joint.definition->name);
    if (found!=message->name.end()) joint.position=message->position[static_cast<std::size_t>(found-message->name.begin())];
  }
  for (auto& joint:impl_->joints) if (joint.definition->mimic) {
    const auto& mimic=*joint.definition->mimic;
    for (const auto& source:impl_->joints) if (source.definition->name==mimic.joint_name)
      joint.position=source.position*mimic.multiplier+mimic.offset;
  }
  impl_->joint_stamp=message->header.stamp.isZero()?ros::Time::now():message->header.stamp;
  for (const auto& joint:impl_->joints) output->push_back(impl_->transform(*joint.definition,joint.position,impl_->joint_stamp));
  impl_->applied_generation=generation;
}
void Description::stop() {
  impl_->subscriber.shutdown();
  impl_->publisher.shutdown();
  if (!impl_->parameters_set) return;
  ros::NodeHandle node;
  std::string current;
  if(node.getParam(impl_->parameter,current)&&current==impl_->xml)node.deleteParam(impl_->parameter);
  const auto standard=impl_->ros_namespace+"/robot_description";
  if(statePublisher()&&node.getParam(standard,current)&&current==impl_->xml)node.deleteParam(standard);
  impl_->parameters_set=false;
}
} // namespace xgc2_ros_visualizer

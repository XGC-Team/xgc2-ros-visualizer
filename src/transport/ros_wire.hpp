#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <ros/serialization.h>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace xgc2_ros_visualizer {
// These member messages contain fixed-size fields and ROS strings only. ROS's
// string decoder checks the remaining stream before allocating. Do not use this
// helper for array-bearing messages: their ROS decoder may resize from an
// untrusted count before checking the remaining bytes.
template<class T> struct SafeMemberWire : std::false_type {};
template<> struct SafeMemberWire<geometry_msgs::PoseStamped> : std::true_type {};
template<> struct SafeMemberWire<geometry_msgs::Twist> : std::true_type {};
template<> struct SafeMemberWire<geometry_msgs::TwistStamped> : std::true_type {};
template<> struct SafeMemberWire<mavros_msgs::State> : std::true_type {};
template<> struct SafeMemberWire<mavros_msgs::ExtendedState> : std::true_type {};

constexpr std::size_t kMaxMemberWireBytes = 65536;
template<class T> bool decodeMemberWire(const std::uint8_t* bytes, std::size_t size, T* out) {
  static_assert(SafeMemberWire<T>::value, "array-bearing ROS messages require a bounded decoder");
  if (!out || !bytes || !size || size > kMaxMemberWireBytes || size > std::numeric_limits<std::uint32_t>::max()) return false;
  try {
    ros::serialization::IStream stream(const_cast<std::uint8_t*>(bytes), static_cast<std::uint32_t>(size));
    T value;
    ros::serialization::deserialize(stream, value);
    if (stream.getLength() != 0) return false;
    *out = std::move(value);
    return true;
  } catch (const ros::serialization::StreamOverrunException&) {
    return false;
  }
}
} // namespace xgc2_ros_visualizer

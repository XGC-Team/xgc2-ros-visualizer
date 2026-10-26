#include "transport/ros_wire.hpp"
#include <gtest/gtest.h>
#include <ros/serialization.h>
#include <vector>
using namespace xgc2_ros_visualizer;

template<class T> std::vector<std::uint8_t> wire(const T& value) {
  std::vector<std::uint8_t> bytes(ros::serialization::serializationLength(value));
  ros::serialization::OStream stream(bytes.data(), bytes.size());
  ros::serialization::serialize(stream, value);
  return bytes;
}
TEST(MemberWire, RoundTripAndRejectTruncatedOrTrailingBytes) {
  geometry_msgs::PoseStamped original;
  original.header.stamp=ros::Time(123,456); original.header.frame_id="world";
  original.pose.position.x=2.5; original.pose.orientation.w=1;
  const auto bytes=wire(original);
  geometry_msgs::PoseStamped result;
  ASSERT_TRUE(decodeMemberWire(bytes.data(),bytes.size(),&result));
  EXPECT_EQ(original.header.stamp,result.header.stamp);
  EXPECT_EQ("world",result.header.frame_id);
  EXPECT_DOUBLE_EQ(2.5,result.pose.position.x);
  for(std::size_t size=0;size<bytes.size();++size)
    EXPECT_FALSE(decodeMemberWire(bytes.data(),size,&result)) << size;
  auto trailing=bytes; trailing.push_back(0);
  EXPECT_FALSE(decodeMemberWire(trailing.data(),trailing.size(),&result));
  EXPECT_DOUBLE_EQ(2.5,result.pose.position.x);
}
TEST(MemberWire, RejectMaliciousStringLengthAndOversizedPayload) {
  mavros_msgs::State state; auto bytes=wire(state);
  // The Header starts with seq, stamp and then frame_id length.
  ASSERT_GE(bytes.size(),16u);
  bytes[12]=bytes[13]=bytes[14]=bytes[15]=0xff;
  EXPECT_FALSE(decodeMemberWire(bytes.data(),bytes.size(),&state));
  bytes.resize(kMaxMemberWireBytes+1);
  EXPECT_FALSE(decodeMemberWire(bytes.data(),bytes.size(),&state));
}
TEST(MemberWire, AllMemberMessageTypesRoundTrip) {
  geometry_msgs::Twist twist; twist.angular.z=3;
  geometry_msgs::TwistStamped stamped; stamped.twist=twist;
  mavros_msgs::State state; state.armed=true; state.mode="OFFBOARD";
  mavros_msgs::ExtendedState extended; extended.landed_state=2;
  const auto t=wire(twist), s=wire(stamped), m=wire(state), e=wire(extended);
  geometry_msgs::Twist t2; geometry_msgs::TwistStamped s2; mavros_msgs::State m2; mavros_msgs::ExtendedState e2;
  EXPECT_TRUE(decodeMemberWire(t.data(),t.size(),&t2));
  EXPECT_TRUE(decodeMemberWire(s.data(),s.size(),&s2));
  EXPECT_TRUE(decodeMemberWire(m.data(),m.size(),&m2));
  EXPECT_TRUE(decodeMemberWire(e.data(),e.size(),&e2));
  EXPECT_DOUBLE_EQ(3,t2.angular.z); EXPECT_DOUBLE_EQ(3,s2.twist.angular.z);
  EXPECT_TRUE(m2.armed); EXPECT_EQ("OFFBOARD",m2.mode); EXPECT_EQ(2,e2.landed_state);
}

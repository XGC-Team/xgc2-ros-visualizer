#include "transport/display_wire.hpp"
#include <gtest/gtest.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
using namespace xgc2_ros_visualizer;
template<class T> std::vector<std::uint8_t> wire(const T& msg) {
  std::vector<std::uint8_t> out(ros::serialization::serializationLength(msg));
  ros::serialization::OStream stream(out.data(),out.size());
  ros::serialization::serialize(stream,msg); return out;
}
template<class T> void check(const char* type,const T& msg) {
  auto b=wire(msg); ASSERT_TRUE(validDisplayWire(type,b.data(),b.size())) << type;
  for(std::size_t i=0;i<b.size();++i) EXPECT_FALSE(validDisplayWire(type,b.data(),i)) << type << " " << i;
  b.push_back(0); EXPECT_FALSE(validDisplayWire(type,b.data(),b.size()));
}
TEST(DisplayWire, ValidatesEveryDeclaredMessageAndTruncation) {
  nav_msgs::Path path; path.header.frame_id="world"; path.poses.resize(2); path.poses[0].header.frame_id="map";
  geometry_msgs::PoseArray poses; poses.poses.resize(3);
  nav_msgs::OccupancyGrid grid; grid.data={1,-1,2};
  sensor_msgs::PointCloud2 cloud; cloud.fields.resize(2); cloud.fields[0].name="x"; cloud.data={1,2,3};
  visualization_msgs::Marker marker; marker.ns="test"; marker.points.resize(2); marker.colors.resize(2); marker.text="name"; marker.mesh_resource="mesh";
  visualization_msgs::MarkerArray markers; markers.markers={marker,marker};
  check("nav_msgs/Path",path); check("geometry_msgs/PoseArray",poses); check("nav_msgs/OccupancyGrid",grid);
  check("sensor_msgs/PointCloud2",cloud); check("visualization_msgs/Marker",marker); check("visualization_msgs/MarkerArray",markers);
}
TEST(DisplayWire, RejectsOverflowingArrayCountsBeforeAllocation) {
  nav_msgs::Path path; auto b=wire(path); ASSERT_GE(b.size(),4u);
  for(std::size_t i=b.size()-4;i<b.size();++i)b[i]=0xff;
  EXPECT_FALSE(validDisplayWire("nav_msgs/Path",b.data(),b.size()));
  visualization_msgs::MarkerArray markers; b=wire(markers); for(auto& v:b)v=0xff;
  EXPECT_FALSE(validDisplayWire("visualization_msgs/MarkerArray",b.data(),b.size()));
  b.resize(kMaxDisplayWireBytes+1); EXPECT_FALSE(validDisplayWire("nav_msgs/Path",b.data(),b.size()));
}

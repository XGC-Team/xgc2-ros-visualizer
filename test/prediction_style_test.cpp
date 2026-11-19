#include <xgc2_ros_visualizer/prediction_style.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {
using visualization_msgs::Marker;
using xgc2_ros_visualizer::makeFig11Prediction;

nav_msgs::Path example() {
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp.sec = 23;
  path.header.stamp.nsec = 81;
  geometry_msgs::PoseStamped sample;
  sample.header = path.header;
  path.poses.push_back(sample);
  sample.pose.position.x = 3;
  path.poses.push_back(sample);
  sample.pose.position.z = 7;
  path.poses.push_back(sample);
  return path;
}

std::uint32_t rgb(const Marker& marker) {
  return (std::lround(marker.color.r * 255) << 16) |
         (std::lround(marker.color.g * 255) << 8) | std::lround(marker.color.b * 255);
}
}  // namespace

TEST(Fig11Prediction, PreservesFrameStampsAndExactEndpointAndUsesAuthorGeometry) {
  const auto path = example();
  const auto overlay = makeFig11Prediction(path);
  const auto& m = overlay.markers;
  ASSERT_EQ(m.size(), 6u);
  const std::array<std::uint32_t, 6> colors{{0x0072b2, 0xe5b51b, 0xd55a5a, 0x00659e, 0xc99700, 0xb43c3c}};
  const std::array<std::size_t, 6> samples{{3, 2, 2, 1, 1, 1}};
  const std::array<int, 6> types{{Marker::LINE_STRIP, Marker::LINE_STRIP, Marker::LINE_STRIP,
                                  Marker::SPHERE_LIST, Marker::SPHERE_LIST, Marker::CUBE_LIST}};
  for (std::size_t i = 0; i < m.size(); ++i) {
    EXPECT_EQ(m[i].header, path.header);
    EXPECT_EQ(m[i].type, types[i]);
    EXPECT_EQ(m[i].action, Marker::ADD);
    EXPECT_EQ(rgb(m[i]), colors[i]);
    EXPECT_EQ(m[i].points.size(), samples[i]);
  }
  EXPECT_FLOAT_EQ(m[0].color.a, 0.96f);
  EXPECT_FLOAT_EQ(m[3].color.a, 0.99f);
  EXPECT_FLOAT_EQ(m[5].color.a, 1.0f);
  EXPECT_DOUBLE_EQ(m[0].scale.x, 0.042);
  EXPECT_DOUBLE_EQ(m[1].scale.x, 0.042 * 1.14);
  EXPECT_DOUBLE_EQ(m[2].scale.x, 0.042 * 1.06);
  EXPECT_DOUBLE_EQ(m[5].scale.x, 2.05 * 0.064);
  EXPECT_DOUBLE_EQ(m[0].points.back().z, 3.2);
  EXPECT_DOUBLE_EQ(m[1].points.back().z, 5.4);
  EXPECT_EQ(m[0].points[0], path.poses[0].pose.position);
  EXPECT_EQ(m[0].points[1], path.poses[1].pose.position);  // The original right angle.
  EXPECT_EQ(m[2].points.back(), path.poses.back().pose.position);
  EXPECT_EQ(m[0].points.back(), m[1].points.front());
  EXPECT_EQ(m[1].points.back(), m[2].points.front());
  EXPECT_EQ(m[5].points.front(), path.poses.back().pose.position);
}

TEST(Fig11Prediction, UsesExactVertexCrossingsWithoutInsertingExtraPoints) {
  auto path = example();
  path.poses[1].pose.position.x = 62;
  path.poses[2].pose.position.x = 62;
  path.poses[2].pose.position.y = 22;
  path.poses[2].pose.position.z = 0;
  auto final = path.poses.back();
  final.pose.position.z = 16;
  path.poses.push_back(final);
  const auto overlay = makeFig11Prediction(path);
  for (std::size_t i = 0; i < 3; ++i) {
    const auto& line = overlay.markers[i];
    ASSERT_EQ(line.points.size(), 2u);
    EXPECT_EQ(line.points.front(), path.poses[i].pose.position);
    EXPECT_EQ(line.points.back(), path.poses[i + 1].pose.position);
  }
  EXPECT_EQ(overlay.markers[5].points.front(), final.pose.position);
}

TEST(Fig11Prediction, InsertsBothCrossingsOnOneEdgeAndRetainsRepeatedSamples) {
  auto path = example();
  path.poses[1] = path.poses.front();
  path.poses.back() = path.poses.front();
  path.poses.back().pose.position.x = 100;
  const auto overlay = makeFig11Prediction(path);
  const auto& blue = overlay.markers[0].points;
  const auto& gold = overlay.markers[1].points;
  const auto& red = overlay.markers[2].points;
  ASSERT_EQ(blue.size(), 3u);
  ASSERT_EQ(gold.size(), 2u);
  ASSERT_EQ(red.size(), 2u);
  EXPECT_EQ(blue[0], path.poses[0].pose.position);
  EXPECT_EQ(blue[1], path.poses[1].pose.position);
  EXPECT_DOUBLE_EQ(blue.back().x, 62);
  EXPECT_DOUBLE_EQ(gold.back().x, 84);
  EXPECT_EQ(blue.back(), gold.front());
  EXPECT_EQ(gold.back(), red.front());
  EXPECT_EQ(red.back(), path.poses.back().pose.position);
}

TEST(Fig11Prediction, DeletesStaleMarkersForAbsentOrInvalidPathsAndKeepsOnlyShortPathTerminal) {
  auto path = example();
  path.poses[1].header.frame_id = "unrelated";
  for (const auto& m : makeFig11Prediction(path).markers) EXPECT_EQ(m.action, Marker::DELETE);
  path.poses[1].header.frame_id = "map";
  path.poses[1].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  for (const auto& m : makeFig11Prediction(path).markers) EXPECT_EQ(m.action, Marker::DELETE);
  path.poses.resize(1);
  for (const auto& m : makeFig11Prediction(path).markers) EXPECT_EQ(m.action, Marker::DELETE);
  path.poses.push_back(path.poses.front());
  path.poses.back().pose.position.x = 0.02;
  const auto short_path = makeFig11Prediction(path);
  for (std::size_t i = 0; i < 5; ++i) EXPECT_EQ(short_path.markers[i].action, Marker::DELETE);
  EXPECT_EQ(short_path.markers[5].action, Marker::ADD);
  EXPECT_EQ(short_path.markers[5].points.front(), path.poses.back().pose.position);
}

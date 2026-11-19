#include <xgc2_ros_visualizer/prediction_style.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace xgc2_ros_visualizer {
namespace {

using geometry_msgs::Point;
using visualization_msgs::Marker;

constexpr double kWidth = 0.042;
constexpr double kTerminalRadius = 0.064;
constexpr double kTrackEnd = 0.62;
constexpr double kAdjustEnd = 0.84;

struct Style {
  const char* name;
  int type;
  std::uint32_t rgb;
  float alpha;
  double size;
};

// Source: paper-dmpc-revision Fig. 11a and xgc2-headless-media
// make_horizon_markers/style_lock.yaml, dc2375c80.
const std::array<Style, 6> kStyles{{
    {"ht", Marker::LINE_STRIP, 0x0072b2, 0.96f, kWidth},
    {"ha", Marker::LINE_STRIP, 0xe5b51b, 0.96f, 1.14 * kWidth},
    {"hb", Marker::LINE_STRIP, 0xd55a5a, 0.96f, 1.06 * kWidth},
    {"j1", Marker::SPHERE_LIST, 0x00659e, 0.99f, 2 * 0.76 * kTerminalRadius},
    {"j2", Marker::SPHERE_LIST, 0xc99700, 0.99f, 2 * 0.82 * kTerminalRadius},
    {"stop", Marker::CUBE_LIST, 0xb43c3c, 1.0f, 2.05 * kTerminalRadius},
}};

bool cumulativeDistances(const nav_msgs::Path& path, std::vector<double>& cumulative) {
  if (path.header.frame_id.empty() || path.poses.size() < 2) return false;
  cumulative.push_back(0.0);
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto& pose = path.poses[i];
    const auto& p = pose.pose.position;
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
    if (i == 0) continue;
    const auto& prev = path.poses[i - 1].pose.position;
    const double length = std::hypot(std::hypot(p.x - prev.x, p.y - prev.y), p.z - prev.z);
    cumulative.push_back(cumulative.back() + length);
    if (!std::isfinite(cumulative.back())) return false;
  }
  return true;
}

// The crossing is strictly inside the edge. Exact vertex hits are copied from
// the input by copySection, so the stored coordinate is never interpolated.
Point edgePoint(const nav_msgs::Path& path, const std::vector<double>& cumulative,
                std::size_t next, double distance) {
  const double start_distance = cumulative[next - 1];
  const double weight = (distance - start_distance) / (cumulative[next] - start_distance);
  const auto& start = path.poses[next - 1].pose.position;
  const auto& end = path.poses[next].pose.position;
  Point p;
  p.x = start.x + weight * (end.x - start.x);
  p.y = start.y + weight * (end.y - start.y);
  p.z = start.z + weight * (end.z - start.z);
  return p;
}

void copySection(const nav_msgs::Path& path, const std::vector<double>& cumulative,
                 double begin, double end, Marker& line) {
  const auto first = std::lower_bound(cumulative.begin(), cumulative.end(), begin);
  const auto after = std::upper_bound(first, cumulative.end(), end);
  line.action = Marker::ADD;
  if (*first > begin)
    line.points.push_back(edgePoint(path, cumulative, first - cumulative.begin(), begin));
  for (auto vertex = first; vertex != after; ++vertex)
    line.points.push_back(path.poses[vertex - cumulative.begin()].pose.position);
  if (*(after - 1) < end)
    line.points.push_back(edgePoint(path, cumulative, after - cumulative.begin(), end));
}

}  // namespace

visualization_msgs::MarkerArray makeFig11Prediction(const nav_msgs::Path& path) {
  visualization_msgs::MarkerArray result;
  for (const auto& style : kStyles) {
    Marker marker;
    marker.header = path.header;
    marker.ns = std::string("tro_fig11_") + style.name;
    marker.type = style.type;
    marker.action = Marker::DELETE;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = style.size;
    marker.color.r = ((style.rgb >> 16) & 255) / 255.0f;
    marker.color.g = ((style.rgb >> 8) & 255) / 255.0f;
    marker.color.b = (style.rgb & 255) / 255.0f;
    marker.color.a = style.alpha;
    result.markers.push_back(std::move(marker));
  }

  std::vector<double> cumulative;
  cumulative.reserve(path.poses.size());
  if (!cumulativeDistances(path, cumulative)) return result;
  auto& stop = result.markers[5];
  stop.action = Marker::ADD;
  stop.points.push_back(path.poses.back().pose.position);
  const double length = cumulative.back();
  if (length < 0.06) return result;

  // Color the recorded polyline. Insert only the two band crossings, so the
  // display never shortcuts a corner or changes the original Path terminal.
  const std::array<double, 4> cuts{{0.0, kTrackEnd * length, kAdjustEnd * length, length}};
  for (std::size_t i = 0; i + 1 < cuts.size(); ++i)
    copySection(path, cumulative, cuts[i], cuts[i + 1], result.markers[i]);
  for (std::size_t i = 0; i < 2; ++i) {
    auto& joint = result.markers[3 + i];
    joint.action = Marker::ADD;
    joint.points.push_back(result.markers[i].points.back());
  }
  return result;
}

}  // namespace xgc2_ros_visualizer

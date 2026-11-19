#include <xgc2_ros_visualizer/prediction_style.hpp>

#include "prediction_style_data.hpp"

#include <json/json.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace xgc2_ros_visualizer {
namespace {

using geometry_msgs::Point;
using visualization_msgs::Marker;

struct DisplayStyle {
  std::array<double, 2> cuts;
  double minimumArcLength;
  visualization_msgs::MarkerArray template_markers;
};

const DisplayStyle& recordedPredictionStyle() {
  static const auto style = [] {
    Json::Value value;
    Json::Reader reader;
    if (!reader.parse(kRecordedPredictionStyle, value) ||
        value["schema"] != "xgc.recorded-prediction-style.v1")
      throw std::logic_error("invalid compiled recorded prediction style");
    DisplayStyle result;
    const auto& cuts = value["displayArcCuts"];
    result.cuts = {{cuts[0].asDouble(), cuts[1].asDouble()}};
    result.minimumArcLength = value["minimumArcLengthMeters"].asDouble();
    if (value["markers"].size() != 6 || !(0.0 < result.cuts[0] && result.cuts[0] < result.cuts[1] &&
                                         result.cuts[1] < 1.0 && result.minimumArcLength > 0.0))
      throw std::logic_error("invalid recorded prediction marker roles or arc cuts");
    result.template_markers.markers.reserve(6);
    for (const auto& style : value["markers"]) {
      Marker marker;
      marker.ns = std::string("tro_fig11_") + style["role"].asString();
      marker.type = style["type"].asInt();
      marker.action = Marker::DELETE;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = style["sizeMeters"].asDouble();
      const auto rgb = std::stoul(style["color"].asString().substr(1), nullptr, 16);
      marker.color.r = ((rgb >> 16) & 255) / 255.0f;
      marker.color.g = ((rgb >> 8) & 255) / 255.0f;
      marker.color.b = (rgb & 255) / 255.0f;
      marker.color.a = style["alpha"].asFloat();
      result.template_markers.markers.push_back(std::move(marker));
    }
    return result;
  }();
  return style;
}

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
  const auto& profile = recordedPredictionStyle();
  auto result = profile.template_markers;
  for (auto& marker : result.markers)
    marker.header = path.header;

  std::vector<double> cumulative;
  cumulative.reserve(path.poses.size());
  if (!cumulativeDistances(path, cumulative)) return result;
  auto& stop = result.markers[5];
  stop.action = Marker::ADD;
  stop.points.push_back(path.poses.back().pose.position);
  const double length = cumulative.back();
  if (length < profile.minimumArcLength) return result;

  // Color the recorded polyline. Insert only the two band crossings, so the
  // display never shortcuts a corner or changes the original Path terminal.
  const std::array<double, 4> cuts{{0.0, profile.cuts[0] * length, profile.cuts[1] * length, length}};
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

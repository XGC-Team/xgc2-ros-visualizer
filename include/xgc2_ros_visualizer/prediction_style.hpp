#pragma once

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

namespace xgc2_ros_visualizer {

// Fig. 11a display geometry for a single prediction topic. The stage cuts
// approximate the author's figure by arc length. They are not solver stages.
visualization_msgs::MarkerArray makeFig11Prediction(const nav_msgs::Path& path);

}  // namespace xgc2_ros_visualizer

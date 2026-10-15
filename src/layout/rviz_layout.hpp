#pragma once
#include <json/json.h>
#include <iosfwd>
namespace xgc2_ros_visualizer {
Json::Value prepareRvizLayout(const Json::Value &input);
Json::Value prepareRvizArtifact(const Json::Value &input);
void prepareRvizArtifact(std::istream &input, std::ostream &output);
}

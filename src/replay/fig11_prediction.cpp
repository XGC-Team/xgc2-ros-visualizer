#include <xgc2_ros_visualizer/config.hpp>
#include <xgc2_ros_visualizer/prediction_style.hpp>
#include "transport/display_wire.hpp"

#include <ros/ros.h>
#include <ros/serialization.h>
#include <rosgraph_msgs/Clock.h>

#include <cstdint>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using xgc2_ros_visualizer::makeFig11Prediction;

void convertWire() {
  using namespace xgc2_ros_visualizer;
  std::vector<std::uint8_t> bytes(kMaxDisplayWireBytes + 1);
  std::cin.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
  const auto size = static_cast<std::uint32_t>(std::cin.gcount());
  if (std::cin.bad() || !validDisplayWire("nav_msgs/Path", bytes.data(), size))
    throw std::invalid_argument("expected one bounded serialized ROS1 nav_msgs/Path");
  nav_msgs::Path path;
  ros::serialization::IStream input(bytes.data(), size);
  ros::serialization::deserialize(input, path);
  const auto markers = makeFig11Prediction(path);
  const auto output_size = ros::serialization::serializationLength(markers);
  std::vector<std::uint8_t> encoded(output_size);
  ros::serialization::OStream output(encoded.data(), output_size);
  ros::serialization::serialize(output, markers);
  std::cout.write(reinterpret_cast<const char*>(encoded.data()), encoded.size());
  if (!std::cout) throw std::runtime_error("cannot write serialized MarkerArray");
}

void replayTopics(int argc, char** argv) {
  using namespace xgc2_ros_visualizer;
  ros::init(argc, argv, "xgc_fig11_prediction", ros::init_options::AnonymousName);
  ros::NodeHandle node;
  std::set<std::string> used;
  struct Layer {
    ros::Publisher output;
    std_msgs::Header previous;
  };
  std::vector<Layer> layers;
  layers.reserve(argc - 2);
  std::vector<ros::Subscriber> subscriptions;
  for (int i = 2; i < argc; ++i) {
    const std::string source(argv[i]);
    const auto spec = relayConfig(source, "/xgc/display" + source, "nav_msgs/Path", RateKind::Global,
                                  0, RelayRole::Prediction, RelayStyle::Fig11);
    if (node.resolveName(spec.source) != spec.source || node.resolveName(spec.topic) != spec.topic ||
        !used.insert(spec.source).second)
      throw std::invalid_argument("repeat or remapping changed a Fig. 11 replay topic");
    layers.push_back({node.advertise<visualization_msgs::MarkerArray>(spec.topic, 1, true), {}});
    auto& layer = layers.back();
    subscriptions.push_back(node.subscribe<nav_msgs::Path>(spec.source, 1,
        [&layer](const nav_msgs::Path::ConstPtr& path) {
          if (ros::serialization::serializationLength(*path) > kMaxDisplayWireBytes) return;
          layer.output.publish(makeFig11Prediction(*path));
          layer.previous = path->header;
        }));
  }
  ros::Time previous_clock;
  const auto clock = node.subscribe<rosgraph_msgs::Clock>("/clock", 10,
      [&](const rosgraph_msgs::Clock::ConstPtr& now) {
        if (now->clock < previous_clock) {
          for (auto& layer : layers) {
            if (layer.previous.frame_id.empty() || layer.previous.stamp <= now->clock) continue;
            nav_msgs::Path empty;
            empty.header = layer.previous;
            empty.header.stamp = now->clock;
            layer.output.publish(makeFig11Prediction(empty));
            layer.previous = {};
          }
        }
        previous_clock = now->clock;
      });
  std::cout << "fig11-ready" << std::endl;
  ros::spin();
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--wire") {
      convertWire();
    } else if (argc > 2 && std::string(argv[1]) == "--ros") {
      replayTopics(argc, argv);
    } else {
      throw std::invalid_argument("usage: xgc2_fig11_prediction --wire | --ros /prediction/topic ...");
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}

#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <xgc2_ros_visualizer/config.hpp>
#include "../runtime/input_pool.hpp"
namespace xgc2_ros_visualizer {
class Relay {
 public:
  virtual ~Relay() = default;
  virtual bool publish(const Rates& rates, std::chrono::steady_clock::time_point now, double prediction_max_hz, const ros::Time& ros_now) = 0;
  virtual void stop() = 0;
  virtual std::uint64_t count() const = 0;
};
std::unique_ptr<Relay> makeRelay(InputPool& pool, const RelayConfig& spec);
} // namespace xgc2_ros_visualizer

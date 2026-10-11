#pragma once
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <json/json.h>
#include <ros/ros.h>
#include <xgc2_ros_visualizer/profile.hpp>
#include "input_pool.hpp"

namespace xgc2_ros_visualizer {

// One display scene on the ROS graph: a set of robots, what they share and the
// display relays. The desired state is replaced as a whole; only the robots
// whose profile differs are rebuilt.
//
// Locking: `tick` runs on the publication scheduler and holds the instance
// lock for its whole duration. Every other member function belongs to the
// control thread; it prepares outside the lock and takes the lock only to swap
// members and outputs, so the scheduler never waits for ROS or URDF work.
class Instance {
 public:
  explicit Instance(std::string id);
  ~Instance();
  const std::string& id() const;

  // Control thread.
  // Make the instance equal to `next`. Everything that can fail on its own (URDF,
  // renderer, validation) is prepared before the first resource changes; if a
  // resource still fails, the previous state is restored and the error rethrown.
  Change apply(const InstanceSpec& next, InputPool& pool);
  // The control worker activates the latest loaded state when ROS first binds.
  void activate(InputPool& pool);
  // Replaces the rate overrides of one robot; false if they are unchanged.
  bool setRobotRates(const std::string& robot, const RateOverrides& overrides);
  const InstanceSpec& spec() const;
  bool hasRobot(const std::string& robot) const;
  std::uint64_t robotRevision(const std::string& robot) const;
  RateOverrides robotRates(const std::string& robot) const;
  std::set<std::string> claims() const;
  Json::Value status() const;
  Json::Value robotStatus(const std::string& robot, const Rates& table) const;
  // Ends the instance: retracts every output and stops every input. Idempotent.
  void deactivate();

  // Scheduler thread.
  void tick(const Rates& table, const ros::Time& now);
  // The highest rate any robot of this instance asks for through its overrides.
  double maximumOverride() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace xgc2_ros_visualizer

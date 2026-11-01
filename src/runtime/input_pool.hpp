#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace xgc2_ros_visualizer {
class InputPool {
 public:
  explicit InputPool(std::size_t workers);
  ~InputPool();
  ros::NodeHandle node(const std::string& source) const;
  std::size_t size() const { return queues_.size(); }
  void stop();
  void rethrowFailure();
 private:
  class WakeQueue;
  void wakeWorkers();
  std::atomic<bool> stopping_{false};
  std::vector<std::unique_ptr<WakeQueue>> queues_;
  std::vector<std::thread> workers_;
  std::mutex failure_mutex_;
  std::exception_ptr failure_;
};
} // namespace xgc2_ros_visualizer

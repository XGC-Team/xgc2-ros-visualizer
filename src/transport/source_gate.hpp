#pragma once
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <utility>
namespace xgc2_ros_visualizer {
// One source resource per downstream demand period. The provider opens the ROS
// or Zenoh source and returns the activation resource; the last consumer
// releases it. Lifecycle and payload locks stay separate.
class SourceGate {
 public:
  using Open = std::function<std::shared_ptr<void>()>;
  explicit SourceGate(Open open): open_(std::move(open)) {}
  void connect(const ros::SingleSubscriberPublisher&) { acquire(); }
  void disconnect(const ros::SingleSubscriberPublisher&) { release(); }
  void acquire() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!active_)return;
    ++count_;
    if(!input_) {
      try {input_=open_();}
      catch(const std::exception& error) {ROS_ERROR_STREAM("display input: " << error.what());}
    }
  }
  void release() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Finish retracting before a concurrent reconnect may declare the source.
    if(count_>0 && --count_==0) input_.reset();
  }
  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_=false; count_=0; input_.reset();
  }
  std::shared_ptr<void> current() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return input_;
  }
 private:
  mutable std::mutex mutex_;
  Open open_;
  std::size_t count_{0};
  bool active_{true};
  std::shared_ptr<void> input_;
};
} // namespace xgc2_ros_visualizer

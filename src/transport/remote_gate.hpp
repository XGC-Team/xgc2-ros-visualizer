#pragma once
#include "zenoh_input.hpp"
#include <ros/ros.h>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
namespace xgc2_ros_visualizer {
// The output publisher's connect/disconnect callbacks are on the existing ROS
// input pool, not the scheduler. A remote key exists only while an output has
// subscribers. Stop disables callbacks before retracting the owned subscriber.
class RemoteGate {
 public:
  RemoteGate(std::string endpoint,std::string key,std::size_t max_bytes,ZenohInput::Callback callback)
    : endpoint_(std::move(endpoint)),key_(std::move(key)),max_bytes_(max_bytes),callback_(std::move(callback)) {}
  void connect(const ros::SingleSubscriberPublisher&) { acquire(); }
  void disconnect(const ros::SingleSubscriberPublisher&) { release(); }
  void acquire() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!active_)return;
    ++count_;
    if(!input_) {
      try {input_=ZenohInput::open(endpoint_,key_,max_bytes_,callback_);}
      catch(const std::exception& error) {ROS_ERROR_STREAM("remote display input: " << error.what());}
    }
  }
  void release() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Finish retracting before a concurrent reconnect may declare the key.
    if(count_>0 && --count_==0) input_.reset();
  }
  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_=false; count_=0; input_.reset();
  }
 private:
  std::mutex mutex_;
  std::string endpoint_,key_;
  std::size_t max_bytes_,count_{0};
  ZenohInput::Callback callback_;
  bool active_{true};
  std::unique_ptr<ZenohInput> input_;
};
} // namespace xgc2_ros_visualizer

#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
namespace xgc2_ros_visualizer {
// Holds the shared listening session without declaring any data subscription.
// Instances keep this lease while remote sources are configured, so peers can
// connect before a downstream ROS consumer appears.
class ZenohListener {
 public:
  static std::shared_ptr<ZenohListener> open(const std::string& endpoint);
  ~ZenohListener() = default;
  ZenohListener(const ZenohListener&)=delete;
  ZenohListener& operator=(const ZenohListener&)=delete;
 private:
  explicit ZenohListener(std::shared_ptr<void> session):session_(std::move(session)) {}
  std::shared_ptr<void> session_;
};
// Owns a single subscriber. Destruction retracts it and waits for SDK-owned
// callback state to be released; callbacks never capture a Member or Relay.
class ZenohInput {
 public:
  using Callback = std::function<void(const std::uint8_t*,std::size_t)>;
  static std::unique_ptr<ZenohInput> open(const std::string& endpoint,const std::string& key,
                                           std::size_t max_bytes, Callback callback);
  ~ZenohInput();
  ZenohInput(const ZenohInput&)=delete;
  ZenohInput& operator=(const ZenohInput&)=delete;
 private:
  struct Impl;
  explicit ZenohInput(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};
void validateZenohSource(const std::string& endpoint,const std::string& key);
} // namespace xgc2_ros_visualizer

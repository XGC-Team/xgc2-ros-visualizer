#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
namespace xgc2_ros_visualizer {
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

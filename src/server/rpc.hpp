#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <json/json.h>

namespace xgc2_ros_visualizer {

struct RpcReply {
  int status;
  Json::Value body;
};

using RpcHandler = std::function<RpcReply(const std::string& method,
                                          const std::string& path,
                                          const Json::Value& body)>;

// The describe document at this instant: the readiness envelope
// {service, api_version, instance_id, ready, facts}. It is built by the domain
// and may be called from the transport owner's thread at any time.
using DescribeProvider = std::function<Json::Value()>;

struct RpcOptions {
  // Fresh per process start; every bound call carries it.
  std::string instance_id;
};

// The XRPC http.v1 host of the visualizer. One fixed domain worker owns the
// handler: its handoff is bounded by the host admission limits and a retained
// reply keeps the endpoint lease until the business work really ended.
// `GET /v1/describe` is answered by the owner thread itself so that a call
// holding `wait_ready_ms` never occupies the domain worker; the owner loop
// re-evaluates the held calls on every iteration.
class RpcServer {
 public:
  RpcServer(std::string socket_path, RpcHandler handler, DescribeProvider describe,
            RpcOptions options, std::function<void()> quiesce_native = {},
            std::function<void()> activate_native = {});
  ~RpcServer();

  RpcServer(const RpcServer&) = delete;
  RpcServer& operator=(const RpcServer&) = delete;
  RpcServer(RpcServer&&) = delete;
  RpcServer& operator=(RpcServer&&) = delete;

  // Call once. Quiesce the domain worker and native owner before SDK drain
  // releases the endpoint lease. The native callback's captures outlive this host.
  // Return only after both are quiescent. In-progress
  // native work is never represented as rolled back by transport cancellation.
  void run(const std::atomic<bool>& stopping);
  void stop() noexcept;
  // Deliver the master's first binding to the same worker that owns control.
  void notifyNativeReady();
  const std::string& socket_path() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xgc2_ros_visualizer

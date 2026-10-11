#pragma once
#include <atomic>
#include <csignal>
#include <cstdint>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include "../runtime/instance.hpp"
namespace xgc2_ros_visualizer {
enum class ControlError { NotFound, Conflict, Unavailable };
class ControlFailure : public std::runtime_error {
 public:
  ControlFailure(ControlError code,const std::string& message):std::runtime_error(message),code(code) {}
  const ControlError code;
};
void validateInstanceId(const std::string& id);
void validateRobotId(const std::string& id);

class Server {
 public:
  Server(std::size_t workers,Rates rates,std::string world_clock,const volatile std::sig_atomic_t* signal_stop=nullptr);
  ~Server();
  // Readiness for the describe contract; safe from any thread.
  bool ready() const;
  Json::Value facts() const;
  void startMasterWatch(std::function<void()> on_bound);
  void activateRos(); // Domain worker: apply the latest loaded rows once.
  // Native control owner only; the fixed domain worker calls these directly.
  // They own validation, atomic application and publication fences.
  Json::Value status() const;
  Json::Value ratesStatus() const;
  Json::Value replaceRates(std::uint64_t expected_revision,const Json::Value& complete_rates);
  Json::Value instanceStatus(const std::string& id) const;
  Json::Value applyInstance(const std::string& id,const Json::Value& input);
  Json::Value removeInstance(const std::string& id);
  Json::Value robotStatus(const std::string& id,const std::string& robot) const;
  Json::Value applyRobot(const std::string& id,const std::string& robot,const Json::Value& request);
  Json::Value removeRobot(const std::string& id,const std::string& robot);
  Json::Value robotRates(const std::string& id,const std::string& robot) const;
  Json::Value applyRobotRates(const std::string& id,const std::string& robot,const Json::Value& request);
  const std::atomic<bool>& stopping() const { return stopping_; }
  void stop();
  void rethrowFailure();
 private:
  void publishLoop();
  void replaceSnapshot();
  void wakePublisher();
  void watchMaster();
  void fail(std::exception_ptr failure);
  void requireRunning() const;
  void requireMasterIfBound() const;
  Instance& instance(const std::string& id) const;
  void requireFreeClaims(const std::string& id,const InstanceSpec& spec) const;
  void countMembers();
  InputPool input_;
  class MasterErrors;
  std::unique_ptr<MasterErrors> master_errors_;
  std::string master_uri_,master_run_id_,world_clock_;
  mutable std::mutex master_state_mutex_;
  int master_pid_{0};
  std::atomic<bool> master_reachable_{false},ros_bound_{false};
  std::atomic<std::uint64_t> master_attempts_{0};
  std::unique_ptr<ros::NodeHandle> ros_runtime_;
  std::function<void()> on_master_bound_;
  bool native_active_{false}; // Domain worker only.
  std::map<std::string,std::shared_ptr<Instance>> instances_; // Control-loop owner only.
  std::shared_ptr<const std::vector<std::shared_ptr<Instance>>> snapshot_;
  std::shared_ptr<const Rates> rates_;
  std::uint64_t rates_revision_{1}; // Domain-worker owner; never persisted.
  std::int64_t rates_applied_steady_ns_{0};
  std::atomic<std::size_t> instance_count_{0},robot_count_{0};
  std::atomic<bool> stopping_{false};
  std::thread publisher_,master_watch_;
  std::mutex failure_mutex_;
  std::exception_ptr failure_;
  const volatile std::sig_atomic_t* signal_stop_;
  std::condition_variable wake_;
  std::mutex wait_mutex_;
  std::atomic<std::uint64_t> revision_{0};
};
} // namespace xgc2_ros_visualizer

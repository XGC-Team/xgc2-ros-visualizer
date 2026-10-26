#pragma once
#include "remote_gate.hpp"
#include <memory>
#include <map>
#include <mutex>
#include <vector>
namespace xgc2_ros_visualizer {
// The downstream ROS publisher event is the demand boundary. Closing a token
// revokes its entire contribution, so delayed disconnect events from an old
// publisher cannot affect a replacement publisher.
class DemandTarget {
 public:
  virtual ~DemandTarget()=default;
  virtual void adjust(int delta)=0;
};
class DemandToken {
 public:
  explicit DemandToken(std::shared_ptr<DemandTarget> target):target_(std::move(target)) {}
  void connect(const ros::SingleSubscriberPublisher&) {
    std::lock_guard<std::mutex> lock(mutex_);if(!active_)return;
    ++count_; target_->adjust(1);
  }
  void disconnect(const ros::SingleSubscriberPublisher&) {
    std::lock_guard<std::mutex> lock(mutex_);if(!active_ || !count_)return;
    --count_;target_->adjust(-1);
  }
  void close() {
    std::lock_guard<std::mutex> lock(mutex_);if(!active_)return;
    active_=false;if(count_)target_->adjust(-count_);count_=0;
  }
  ~DemandToken() { close(); }
 private:
  std::mutex mutex_;std::shared_ptr<DemandTarget> target_;int count_{0};bool active_{true};
};
class MemberDemand final : public std::enable_shared_from_this<MemberDemand> {
 public:
  void add(const std::string& source,std::shared_ptr<RemoteGate> gate) {
    std::lock_guard<std::mutex> lock(mutex_);
    gates_.push_back({source,std::move(gate),false});reconcile();
  }
  void path(bool ar,int delta) {
    std::lock_guard<std::mutex> lock(mutex_);(ar?ar_count_:pose_count_)+=delta;reconcile();
  }
  void shared(bool value) {
    std::lock_guard<std::mutex> lock(mutex_);shared_=value;reconcile();
  }
  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);stopped_=true;reconcile();
    for(auto& entry:gates_) entry.gate->stop();
    gates_.clear();
  }
 private:
  struct Entry {std::string source;std::shared_ptr<RemoteGate> gate;bool active;};
  void reconcile() {
    for(auto& entry:gates_) {
      const bool wanted=!stopped_ && (shared_ || (entry.source=="pose" && pose_count_>0) ||
                                       (entry.source=="arPose" && ar_count_>0));
      if(wanted==entry.active)continue;
      entry.active=wanted;
      if(wanted)entry.gate->acquire();else entry.gate->release();
    }
  }
  std::mutex mutex_;std::vector<Entry> gates_;
  int pose_count_{0},ar_count_{0};bool shared_{false},stopped_{false};
};
class PathDemand final : public DemandTarget {
 public:
  PathDemand(std::weak_ptr<MemberDemand> member,bool ar):member_(std::move(member)),ar_(ar) {}
  void adjust(int delta) override {if(auto member=member_.lock())member->path(ar_,delta);}
 private:
  std::weak_ptr<MemberDemand> member_;bool ar_;
};
class SharedDemand final : public DemandTarget {
 public:
  void add(const std::shared_ptr<MemberDemand>& member) {
    std::lock_guard<std::mutex> lock(mutex_);members_[member.get()]=member;member->shared(count_>0);
  }
  void remove(const std::shared_ptr<MemberDemand>& member) {
    std::lock_guard<std::mutex> lock(mutex_);members_.erase(member.get());
  }
  void adjust(int delta) override {
    std::lock_guard<std::mutex> lock(mutex_);const bool before=count_>0;count_+=delta;
    if(before==(count_>0))return;
    for(auto it=members_.begin();it!=members_.end();) {
      if(auto member=it->second.lock()) {member->shared(count_>0);++it;} else it=members_.erase(it);
    }
  }
 private:
  std::mutex mutex_;std::map<MemberDemand*,std::weak_ptr<MemberDemand>> members_;int count_{0};
};
} // namespace xgc2_ros_visualizer

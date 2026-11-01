#include "input_pool.hpp"
#include <condition_variable>
#include <stdexcept>
namespace xgc2_ros_visualizer {

// roscpp already calls addCallback when actual source work arrives. Use that
// owner notification to sleep without repeatedly timing out an empty queue.
class InputPool::WakeQueue final : public ros::CallbackQueue {
 public:
  void addCallback(const ros::CallbackInterfacePtr& callback,uint64_t removal_id=0) override {
    ros::CallbackQueue::addCallback(callback,removal_id);
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
    }
    wake_.notify_one();
  }
  bool wait(const std::atomic<bool>& stopping) {
    std::unique_lock<std::mutex> lock(wake_mutex_);
    wake_.wait(lock,[&]{return stopping.load() || !isEmpty();});
    return !stopping.load();
  }
  void interrupt() {
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
    }
    wake_.notify_all();
  }
 private:
  std::mutex wake_mutex_;
  std::condition_variable wake_;
};

InputPool::InputPool(std::size_t count) {
  if (count<1 || count>32) throw std::invalid_argument("callback_workers must be within 1..32");
  queues_.reserve(count); workers_.reserve(count);
  for (std::size_t i=0;i<count;++i) queues_.emplace_back(new WakeQueue);
  try { for (auto& queue:queues_) {
    auto* selected=queue.get();
    workers_.emplace_back([this,selected] {
      try { while (selected->wait(stopping_)) selected->callAvailable(ros::WallDuration(0)); }
      catch(...) {
        {std::lock_guard<std::mutex> lock(failure_mutex_);if(!failure_)failure_=std::current_exception();}
        stopping_.store(true);wakeWorkers();
      }
    });
  } } catch(...) {stop();throw;}
}
InputPool::~InputPool() { stop(); }
ros::NodeHandle InputPool::node(const std::string& source) const {
  // Stable FNV-1a sharding, independent of process/library hash randomization.
  std::uint64_t hash=14695981039346656037ULL;
  for (unsigned char value:source) { hash^=value; hash*=1099511628211ULL; }
  ros::NodeHandle result; result.setCallbackQueue(queues_[hash%queues_.size()].get()); return result;
}
void InputPool::wakeWorkers() {for (auto& queue:queues_) queue->interrupt();}
void InputPool::stop() {
  stopping_.store(true);
  for (auto& queue:queues_) queue->disable();
  wakeWorkers();
  for (auto& worker:workers_) if (worker.joinable()) worker.join();
  for (auto& queue:queues_) queue->clear();
}
void InputPool::rethrowFailure() {std::lock_guard<std::mutex> lock(failure_mutex_);if(failure_)std::rethrow_exception(failure_);}
} // namespace xgc2_ros_visualizer

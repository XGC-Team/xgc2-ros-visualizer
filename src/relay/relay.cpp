#include "relay.hpp"
#include "marker_coalesce.hpp"
#include "display_snapshot.hpp"
#include <xgc2_ros_visualizer/declared_wire.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdexcept>
#include <type_traits>
namespace xgc2_ros_visualizer {
namespace {
bool due(bool sent, const std::chrono::steady_clock::time_point& last, std::chrono::steady_clock::time_point now, double hz) {
  return !sent || std::chrono::duration<double>(now - last).count() >= 1.0 / hz;
}
template<class Message> class TypedRelay final : public Relay {
  using Wire=DeclaredWire<Message>;
  struct Latest {
    std::mutex mutex;
    typename Wire::ConstPtr message;
    std::uint64_t generation{0};
  };
 public:
  TypedRelay(InputPool& pool,const RelayConfig& spec) : spec_(spec),latest_(new Latest) {
    auto node=pool.node(spec.source);
    if (node.resolveName(spec.source)!=spec.source || node.resolveName(spec.topic)!=spec.topic)
      throw std::invalid_argument("remapping changed a canonical relay topic");
    ros::AdvertiseOptions options;
    options.template init<Wire>(spec.topic,1);
    options.has_header=false; options.latch=snapshot_type_;
    publisher_=node.advertise(options);
    auto latest=latest_;
    boost::function<void(const typename Wire::ConstPtr&)> callback=[latest](const typename Wire::ConstPtr& message) {
      std::lock_guard<std::mutex> lock(latest->mutex);
      latest->message=message; ++latest->generation;
    };
    subscriber_=node.subscribe<Wire>(spec.source,1,callback,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
    if (!publisher_ || !subscriber_) throw std::runtime_error("cannot activate relay: "+spec.source);
  }
  bool publish(const Rates& rates,std::chrono::steady_clock::time_point now,double prediction_max_hz,const ros::Time&) override {
    if (!due(sent_, last_, now, relayPublishHz(rates, spec_, prediction_max_hz))) return false;
    typename Wire::ConstPtr message; std::uint64_t generation;
    { std::lock_guard<std::mutex> lock(latest_->mutex); message=latest_->message; generation=latest_->generation; }
    if (!message || generation==published_generation_) return false;
    published_generation_=generation;
    if (snapshot_type_ && published_message_ &&
        sameWorldSnapshot(message->bytes, published_message_->bytes)) return false;
    publisher_.publish(message); // Same immutable source shared_ptr; no payload copy/replay.
    published_message_=message; last_=now; sent_=true; ++count_;return true;
  }
  void stop() override { subscriber_.shutdown(); publisher_.shutdown(); }
  std::uint64_t count() const override { return count_.load(); }
 private:
  RelayConfig spec_;
  static constexpr bool snapshot_type_ = std::is_same<Message,sensor_msgs::PointCloud2>::value ||
      std::is_same<Message,nav_msgs::OccupancyGrid>::value;
  typename Wire::ConstPtr published_message_;
  std::shared_ptr<Latest> latest_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::uint64_t published_generation_{0};
  std::atomic<std::uint64_t> count_{0};
  std::chrono::steady_clock::time_point last_{};
  bool sent_{false};
};
// Marker and MarkerArray cannot keep one latest message: a later ADD of another
// id would drop a DELETE. The callback only updates the bounded coalescer.
// MarkerArray sends one bounded batch per due period. A singular Marker sends
// one Marker per due period; the publisher queue is 1, so a burst would drop
// every op except the last. Unsent ops stay queued. No timer and no thread.
class MarkerRelay final : public Relay {
  struct Pending {
    std::mutex mutex;
    MarkerCoalescer coalescer;
  };
 public:
  MarkerRelay(InputPool& pool,const RelayConfig& spec) : spec_(spec),pending_(new Pending),array_(spec.message_type=="visualization_msgs/MarkerArray") {
    auto node=pool.node(spec.source);
    if (node.resolveName(spec.source)!=spec.source || node.resolveName(spec.topic)!=spec.topic)
      throw std::invalid_argument("remapping changed a canonical relay topic");
    auto pending=pending_;
    if (array_) {
      publisher_=node.advertise<visualization_msgs::MarkerArray>(spec.topic,1,false);
      boost::function<void(const visualization_msgs::MarkerArray::ConstPtr&)> callback=[pending](const visualization_msgs::MarkerArray::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(pending->mutex);
        pending->coalescer.ingest(*message, ros::Time::now());
      };
      subscriber_=node.subscribe<visualization_msgs::MarkerArray>(spec.source,64,callback,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
    } else {
      publisher_=node.advertise<visualization_msgs::Marker>(spec.topic,1,false);
      boost::function<void(const visualization_msgs::Marker::ConstPtr&)> callback=[pending](const visualization_msgs::Marker::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(pending->mutex);
        pending->coalescer.ingest(*message, ros::Time::now());
      };
      subscriber_=node.subscribe<visualization_msgs::Marker>(spec.source,64,callback,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
    }
    if (!publisher_ || !subscriber_) throw std::runtime_error("cannot activate relay: "+spec.source);
  }
  bool publish(const Rates& rates,std::chrono::steady_clock::time_point now,double prediction_max_hz,const ros::Time& ros_now) override {
    if (!due(sent_, last_, now, relayPublishHz(rates, spec_, prediction_max_hz))) return false;
    if (array_) {
      std::vector<visualization_msgs::Marker> markers;
      { std::lock_guard<std::mutex> lock(pending_->mutex); markers=pending_->coalescer.take(ros_now); }
      if (markers.empty()) return false;
      visualization_msgs::MarkerArray message;
      message.markers.swap(markers);
      publisher_.publish(message);
    } else {
      visualization_msgs::Marker marker;
      bool ready=false;
      { std::lock_guard<std::mutex> lock(pending_->mutex); ready=pending_->coalescer.takeOne(ros_now,&marker); }
      if (!ready) return false;
      publisher_.publish(marker);
    }
    last_=now; sent_=true; ++count_; return true;
  }
  void stop() override { subscriber_.shutdown(); publisher_.shutdown(); }
  std::uint64_t count() const override { return count_.load(); }
 private:
  RelayConfig spec_;
  std::shared_ptr<Pending> pending_;
  bool array_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::atomic<std::uint64_t> count_{0};
  std::chrono::steady_clock::time_point last_{};
  bool sent_{false};
};
}
std::unique_ptr<Relay> makeRelay(InputPool& pool,const RelayConfig& spec) {
  if (spec.message_type=="sensor_msgs/PointCloud2") return std::unique_ptr<Relay>(new TypedRelay<sensor_msgs::PointCloud2>(pool,spec));
  if (spec.message_type=="nav_msgs/OccupancyGrid") return std::unique_ptr<Relay>(new TypedRelay<nav_msgs::OccupancyGrid>(pool,spec));
  if (spec.message_type=="nav_msgs/Path") return std::unique_ptr<Relay>(new TypedRelay<nav_msgs::Path>(pool,spec));
  if (spec.message_type=="geometry_msgs/PoseArray") return std::unique_ptr<Relay>(new TypedRelay<geometry_msgs::PoseArray>(pool,spec));
  if (spec.message_type=="visualization_msgs/Marker") return std::unique_ptr<Relay>(new MarkerRelay(pool,spec));
  if (spec.message_type=="visualization_msgs/MarkerArray") return std::unique_ptr<Relay>(new MarkerRelay(pool,spec));
  throw std::invalid_argument("unsupported relay type");
}
} // namespace xgc2_ros_visualizer

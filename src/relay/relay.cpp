#include "relay.hpp"
#include "marker_coalesce.hpp"
#include "display_snapshot.hpp"
#include "../transport/source_gate.hpp"
#include "../transport/zenoh_input.hpp"
#include "../transport/display_wire.hpp"
#include <boost/make_shared.hpp>
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
  };
  struct Input {
    std::shared_ptr<Latest> latest;
    ros::Subscriber subscriber;
    std::shared_ptr<ZenohInput> zenoh;
  };
 public:
  TypedRelay(InputPool& pool,const RelayConfig& spec) : spec_(spec) {
    auto node=pool.node(spec.source);
    if (node.resolveName(spec.source)!=spec.source || node.resolveName(spec.topic)!=spec.topic)
      throw std::invalid_argument("remapping changed a canonical relay topic");
    const auto type=spec.message_type;
    gate_=std::make_shared<SourceGate>(SourceGate::Open([node,spec,type]() mutable -> std::shared_ptr<void> {
      auto input=std::make_shared<Input>();
      input->latest=std::make_shared<Latest>();
      if (!spec.zenoh_endpoint.empty()) {
        input->zenoh=std::shared_ptr<ZenohInput>(ZenohInput::open(spec.zenoh_endpoint,spec.zenoh_key,kMaxDisplayWireBytes,
            [latest=input->latest,type](const std::uint8_t* bytes,std::size_t size) {
              if (!validDisplayWire(type,bytes,size)) return;
              auto message=boost::make_shared<Wire>(); message->bytes.assign(bytes,bytes+size);
              std::lock_guard<std::mutex> lock(latest->mutex);
              latest->message=message;
            }));
      } else {
        auto latest=input->latest;
        boost::function<void(const typename Wire::ConstPtr&)> callback=[latest](const typename Wire::ConstPtr& message) {
          std::lock_guard<std::mutex> lock(latest->mutex);
          latest->message=message;
        };
        input->subscriber=node.subscribe<Wire>(spec.source,1,callback,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
        if (!input->subscriber) throw std::runtime_error("cannot open local display input: "+spec.source);
      }
      return input;
    }));
    ros::AdvertiseOptions options;
    options.template init<Wire>(spec.topic,1);
    options.has_header=false; options.latch=snapshot_type_;
    auto gate=gate_;
    boost::function<void(const ros::SingleSubscriberPublisher&)> connect=[gate](const ros::SingleSubscriberPublisher& event){gate->connect(event);};
    boost::function<void(const ros::SingleSubscriberPublisher&)> disconnect=[gate](const ros::SingleSubscriberPublisher& event){gate->disconnect(event);};
    options.connect_cb=connect;
    options.disconnect_cb=disconnect;
    publisher_=node.advertise(options);
    if (!publisher_) throw std::runtime_error("cannot activate relay: "+spec.source);
  }
  bool publish(const Rates& rates,std::chrono::steady_clock::time_point now,double prediction_max_hz,const ros::Time&) override {
    if (!due(sent_, last_, now, relayPublishHz(rates, spec_, prediction_max_hz))) return false;
    auto input=std::static_pointer_cast<Input>(gate_->current());
    if (!input) return false;
    typename Wire::ConstPtr message;
    { std::lock_guard<std::mutex> lock(input->latest->mutex); message.swap(input->latest->message); }
    if (!message) return false;
    if (snapshot_type_ && published_message_ &&
        sameWorldSnapshot(message->bytes, published_message_->bytes)) return false;
    publisher_.publish(message); // Same immutable source shared_ptr; no payload copy/replay.
    if (snapshot_type_) published_message_=message;
    last_=now; sent_=true; ++count_;return true;
  }
  void stop() override { gate_->stop(); publisher_.shutdown(); }
  std::uint64_t count() const override { return count_.load(); }
 private:
  RelayConfig spec_;
  static constexpr bool snapshot_type_ = std::is_same<Message,sensor_msgs::PointCloud2>::value ||
      std::is_same<Message,nav_msgs::OccupancyGrid>::value;
  typename Wire::ConstPtr published_message_;
  std::shared_ptr<SourceGate> gate_;
  ros::Publisher publisher_;
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
  struct Input {
    std::shared_ptr<Pending> pending;
    ros::Subscriber subscriber;
    std::shared_ptr<ZenohInput> zenoh;
  };
 public:
  MarkerRelay(InputPool& pool,const RelayConfig& spec) : spec_(spec),array_(spec.message_type=="visualization_msgs/MarkerArray") {
    auto node=pool.node(spec.source);
    if (node.resolveName(spec.source)!=spec.source || node.resolveName(spec.topic)!=spec.topic)
      throw std::invalid_argument("remapping changed a canonical relay topic");
    const auto type=spec.message_type;
    const bool array=array_;
    gate_=std::make_shared<SourceGate>(SourceGate::Open([node,spec,type,array]() mutable -> std::shared_ptr<void> {
      auto input=std::make_shared<Input>();
      input->pending=std::make_shared<Pending>();
      if (!spec.zenoh_endpoint.empty()) {
        input->zenoh=std::shared_ptr<ZenohInput>(ZenohInput::open(spec.zenoh_endpoint,spec.zenoh_key,kMaxDisplayWireBytes,
            [pending=input->pending,type](const std::uint8_t* bytes,std::size_t size) {
              if (!validDisplayWire(type,bytes,size)) return;
              try {
                ros::serialization::IStream stream(const_cast<std::uint8_t*>(bytes),static_cast<std::uint32_t>(size));
                if(type=="visualization_msgs/MarkerArray") {
                  visualization_msgs::MarkerArray message; ros::serialization::deserialize(stream,message);
                  std::lock_guard<std::mutex> lock(pending->mutex);pending->coalescer.ingest(message,ros::Time::now());
                } else {
                  visualization_msgs::Marker message; ros::serialization::deserialize(stream,message);
                  std::lock_guard<std::mutex> lock(pending->mutex);pending->coalescer.ingest(message,ros::Time::now());
                }
              } catch (const ros::serialization::StreamOverrunException&) {}
            }));
      } else if (array) {
        auto pending=input->pending;
        boost::function<void(const visualization_msgs::MarkerArray::ConstPtr&)> callback=[pending](const visualization_msgs::MarkerArray::ConstPtr& message) {
          std::lock_guard<std::mutex> lock(pending->mutex);
          pending->coalescer.ingest(*message, ros::Time::now());
        };
        input->subscriber=node.subscribe<visualization_msgs::MarkerArray>(spec.source,64,callback,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
        if (!input->subscriber) throw std::runtime_error("cannot open local display input: "+spec.source);
      } else {
        auto pending=input->pending;
        boost::function<void(const visualization_msgs::Marker::ConstPtr&)> callback=[pending](const visualization_msgs::Marker::ConstPtr& message) {
          std::lock_guard<std::mutex> lock(pending->mutex);
          pending->coalescer.ingest(*message, ros::Time::now());
        };
        input->subscriber=node.subscribe<visualization_msgs::Marker>(spec.source,64,callback,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
        if (!input->subscriber) throw std::runtime_error("cannot open local display input: "+spec.source);
      }
      return input;
    }));
    ros::AdvertiseOptions options;
    if (array_) options.init<visualization_msgs::MarkerArray>(spec.topic,1);
    else options.init<visualization_msgs::Marker>(spec.topic,1);
    auto gate=gate_;
    boost::function<void(const ros::SingleSubscriberPublisher&)> connect=[gate](const ros::SingleSubscriberPublisher& event){gate->connect(event);};
    boost::function<void(const ros::SingleSubscriberPublisher&)> disconnect=[gate](const ros::SingleSubscriberPublisher& event){gate->disconnect(event);};
    options.connect_cb=connect;
    options.disconnect_cb=disconnect;
    publisher_=node.advertise(options);
    if (!publisher_) throw std::runtime_error("cannot activate relay: "+spec.source);
  }
  bool publish(const Rates& rates,std::chrono::steady_clock::time_point now,double prediction_max_hz,const ros::Time& ros_now) override {
    if (!due(sent_, last_, now, relayPublishHz(rates, spec_, prediction_max_hz))) return false;
    auto input=std::static_pointer_cast<Input>(gate_->current());
    if (!input) return false;
    if (array_) {
      std::vector<visualization_msgs::Marker> markers;
      { std::lock_guard<std::mutex> lock(input->pending->mutex); markers=input->pending->coalescer.take(ros_now); }
      if (markers.empty()) return false;
      visualization_msgs::MarkerArray message;
      message.markers.swap(markers);
      publisher_.publish(message);
    } else {
      visualization_msgs::Marker marker;
      bool ready=false;
      { std::lock_guard<std::mutex> lock(input->pending->mutex); ready=input->pending->coalescer.takeOne(ros_now,&marker); }
      if (!ready) return false;
      publisher_.publish(marker);
    }
    last_=now; sent_=true; ++count_; return true;
  }
  void stop() override { gate_->stop(); publisher_.shutdown(); }
  std::uint64_t count() const override { return count_.load(); }
 private:
  RelayConfig spec_;
  std::shared_ptr<SourceGate> gate_;
  bool array_;
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

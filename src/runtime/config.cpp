#include <xgc2_ros_visualizer/config.hpp>
#include <algorithm>
#include <cmath>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace xgc2_ros_visualizer {
namespace {
constexpr const char* kinds[] = {"fs150", "scout", "mecanum", "global"};
constexpr const char* channels[] = {"pose_tf", "joint_tf", "markers", "scene", "scene_path", "path",
  "ar_path", "ar_identity", "height_projection", "height_projection_ar", "tf_root", "tf_static",
  "world_boundary", "readiness", "display_pointcloud", "display_grid", "display_path", "display_pose_array",
  "display_marker", "display_marker_array"};
static_assert(sizeof(channels) / sizeof(channels[0]) == kChannelCount, "channel names drifted from Channel");
void object(const Json::Value& v, const std::string& name) {
  if (!v.isObject()) throw std::invalid_argument(name + " must be an object");
}
double rate(const Json::Value& v, const std::string& name) {
  if (!v.isNumeric() || v.isBool() || !std::isfinite(v.asDouble()))
    throw std::invalid_argument(name + " must be finite numeric");
  const double hz = v.asDouble();
  if (hz < 0.1 || hz > 1000) throw std::invalid_argument("rate must be within 0.1..1000 Hz");
  return hz;
}
double ceiling(double hz, const char* name) {
  if (!std::isfinite(hz) || hz < 0.1 || hz > 100)
    throw std::invalid_argument(std::string(name) + " must be within 0.1..100 Hz");
  return hz;
}
std::size_t channelIndex(const std::string& name) {
  for (std::size_t c = 0; c < kChannelCount; ++c) if (name == channels[c]) return c;
  return kChannelCount;
}
bool canonicalTopic(const std::string& name) {
  static const std::regex pattern("^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$");
  return name.size() <= 499 && std::regex_match(name, pattern);
}
}
const char* kindName(RateKind kind) { return kinds[static_cast<std::size_t>(kind)]; }
const char* channelName(Channel channel) { return channels[static_cast<std::size_t>(channel)]; }
RateKind parseKind(const std::string& name) {
  for (std::size_t i=0; i<kKindCount; ++i) if (name == kinds[i]) return static_cast<RateKind>(i);
  throw std::invalid_argument("unknown robot kind: " + name);
}
RateKind rateKind(RobotModelKind kind) {
  if (kind == RobotModelKind::kFs150) return RateKind::Fs150;
  if (kind == RobotModelKind::kScout) return RateKind::Scout;
  if (kind == RobotModelKind::kMecanum) return RateKind::Mecanum;
  return RateKind::Global;
}
RobotModelKind modelKind(RateKind kind) {
  if (kind == RateKind::Fs150) return RobotModelKind::kFs150;
  if (kind == RateKind::Scout) return RobotModelKind::kScout;
  if (kind == RateKind::Mecanum) return RobotModelKind::kMecanum;
  return RobotModelKind::kNone;
}
bool applicable(RateKind kind, Channel channel) {
  if (channel >= Channel::DisplayPointcloud) return true;
  if (kind == RateKind::Global) return channel == Channel::JointTf || (channel >= Channel::TfRoot && channel <= Channel::Readiness);
  if (channel >= Channel::TfRoot) return false;
  if (channel == Channel::HeightProjection || channel == Channel::HeightProjectionAr)
    return kind == RateKind::Fs150;
  return true;
}
bool robotChannel(RateKind kind, Channel channel) {
  if (channel >= Channel::DisplayPointcloud || !applicable(kind, channel)) return false;
  return kind != RateKind::Global || channel == Channel::JointTf;
}
double Rates::get(RateKind kind, Channel channel) const {
  return values[static_cast<std::size_t>(kind)][static_cast<std::size_t>(channel)];
}
double Rates::maximum() const {
  double result=0.1;
  for (const auto& row : values) for (double value : row) result=std::max(result,value);
  return result;
}
Json::Value Rates::json() const {
  Json::Value result(Json::objectValue);
  for (std::size_t k=0; k<kKindCount; ++k) for (std::size_t c=0; c<kChannelCount; ++c)
    if (applicable(static_cast<RateKind>(k),static_cast<Channel>(c))) result[kinds[k]][channels[c]]=values[k][c];
  return result;
}
Rates defaultRates() {
  Rates result;
  for (std::size_t k=0; k<kKindCount; ++k) for (std::size_t c=0; c<kChannelCount; ++c) {
    auto channel=static_cast<Channel>(c);
    if (!applicable(static_cast<RateKind>(k),channel)) continue;
    double hz=10;
    if (channel==Channel::PoseTf) hz=120;
    if (channel==Channel::JointTf || channel==Channel::Markers || channel==Channel::TfRoot) hz=30;
    if (channel==Channel::TfStatic || channel==Channel::WorldBoundary || channel==Channel::Readiness) hz=1;
    result.values[k][c]=hz;
  }
  return result;
}
Rates parseRates(const Json::Value& value, bool complete) {
  object(value,"rates");
  Rates result=defaultRates();
  std::size_t entries=0;
  for (const auto& kind_name : value.getMemberNames()) {
    auto kind=parseKind(kind_name); object(value[kind_name],kind_name);
    for (const auto& channel_name : value[kind_name].getMemberNames()) {
      const auto c=channelIndex(channel_name);
      if (c==kChannelCount || !applicable(kind,static_cast<Channel>(c)))
        throw std::invalid_argument("non-applicable or unknown rate: " + kind_name + "/" + channel_name);
      result.values[static_cast<std::size_t>(kind)][c]=rate(value[kind_name][channel_name],channel_name); ++entries;
    }
  }
  if (complete) {
    std::size_t expected=0;
    for (std::size_t k=0;k<kKindCount;++k) for (std::size_t c=0;c<kChannelCount;++c)
      if (applicable(static_cast<RateKind>(k),static_cast<Channel>(c))) ++expected;
    if (entries!=expected) throw std::invalid_argument("runtime rates require the complete kind/channel table");
  }
  return result;
}
double RateOverrides::get(Channel channel, double inherited) const {
  const double value=values[static_cast<std::size_t>(channel)];
  return value>0?value:inherited;
}
double RateOverrides::maximum() const {
  double result=0;
  for (double value : values) result=std::max(result,value);
  return result;
}
bool RateOverrides::empty() const {
  return std::all_of(values.begin(),values.end(),[](double value){return value==0;});
}
Json::Value RateOverrides::json() const {
  Json::Value result(Json::objectValue);
  for (std::size_t c=0; c<kChannelCount; ++c) if (values[c]>0) result[channels[c]]=values[c];
  return result;
}
RateOverrides parseRateOverrides(RateKind kind, const Json::Value& value) {
  object(value,"rates");
  RateOverrides result;
  for (const auto& name : value.getMemberNames()) {
    const auto c=channelIndex(name);
    if (c==kChannelCount || !robotChannel(kind,static_cast<Channel>(c)))
      throw std::invalid_argument(std::string("a ")+kindName(kind)+" robot has no rate for "+name);
    result.values[c]=rate(value[name],name);
  }
  return result;
}
Json::Value parseJson(const std::string& text) {
  Json::CharReaderBuilder builder;
  builder["allowComments"]=false; builder["failIfExtra"]=true; builder["rejectDupKeys"]=true;
  Json::Value result; std::string errors; std::istringstream stream(text);
  if (!Json::parseFromStream(builder,stream,&result,&errors)) throw std::invalid_argument("invalid JSON: "+errors);
  return result;
}
std::string jsonText(const Json::Value& value) {
  Json::StreamWriterBuilder builder; builder["indentation"]=""; return Json::writeString(builder,value);
}
double relayPublishHz(const Rates& rates, const RelayConfig& spec, double prediction_max_hz) {
  double hz = rates.get(spec.kind, spec.channel);
  if (spec.max_rate_hz > 0) hz = std::min(hz, spec.max_rate_hz);
  if (spec.role == RelayRole::Prediction) hz = std::min(hz, prediction_max_hz > 0 ? prediction_max_hz : 2);
  return hz;
}
double historyPublishHz(double channel_hz, double history_max_hz) {
  return std::min(channel_hz, history_max_hz);
}
RelayConfig relayConfig(const std::string& source, const std::string& topic,
                      const std::string& message_type, RateKind kind,
                      double max_rate_hz, RelayRole role) {
  RelayConfig spec{source,topic,message_type,kind,Channel::DisplayPath,0,role};
  if (!canonicalTopic(spec.source) || spec.source=="/xgc/display" || spec.source.compare(0,13,"/xgc/display/")==0 ||
      spec.topic!="/xgc/display"+spec.source)
    throw std::invalid_argument("relay needs a canonical source and its display copy");
  if (max_rate_hz != 0) spec.max_rate_hz = ceiling(max_rate_hz, "maxRateHz");
  if (spec.message_type=="sensor_msgs/PointCloud2") spec.channel=Channel::DisplayPointcloud;
  else if (spec.message_type=="nav_msgs/OccupancyGrid") spec.channel=Channel::DisplayGrid;
  else if (spec.message_type=="nav_msgs/Path") spec.channel=Channel::DisplayPath;
  else if (spec.message_type=="geometry_msgs/PoseArray") spec.channel=Channel::DisplayPoseArray;
  else if (spec.message_type=="visualization_msgs/Marker") spec.channel=Channel::DisplayMarker;
  else if (spec.message_type=="visualization_msgs/MarkerArray") spec.channel=Channel::DisplayMarkerArray;
  else throw std::invalid_argument("relay requires PointCloud2, OccupancyGrid, Path, PoseArray, Marker or MarkerArray");
  return spec;
}
} // namespace xgc2_ros_visualizer

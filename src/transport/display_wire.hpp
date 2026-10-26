#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
namespace xgc2_ros_visualizer {
// Validate array lengths before ROS deserialization. All integer lengths in
// ROS1 serialization are little-endian uint32 and the entire wire payload is
// capped by the existing display relay budget.
constexpr std::size_t kMaxDisplayWireBytes = 1048576;
class WireCursor {
 public:
  WireCursor(const std::uint8_t* data, std::size_t size) : data_(data), remaining_(size) {}
  bool skip(std::size_t n) { if (n > remaining_) return false; data_ += n; remaining_ -= n; return true; }
  bool count(std::uint32_t* n) {
    if (remaining_ < 4) return false;
    *n=std::uint32_t(data_[0]) | (std::uint32_t(data_[1])<<8) |
       (std::uint32_t(data_[2])<<16) | (std::uint32_t(data_[3])<<24);
    return skip(4);
  }
  bool string() { std::uint32_t n; return count(&n) && skip(n); }
  bool header() { return skip(12) && string(); }
  bool array(std::size_t width) { std::uint32_t n; return count(&n) && n<=remaining_/width && skip(std::size_t(n)*width); }
  bool marker() {
    // Header, ns, id/type/action, pose, scale, color, duration, frame_locked,
    // points, colors, text, mesh resource, mesh_use_embedded_materials.
    return header() && string() && skip(12+56+24+16+8+1) && array(24) && array(16) && string() && string() && skip(1);
  }
  bool done() const { return remaining_ == 0; }
  std::size_t remaining() const { return remaining_; }
 private:
  const std::uint8_t* data_;
  std::size_t remaining_;
};
inline bool validDisplayWire(const std::string& type, const std::uint8_t* data, std::size_t size) {
  if (!data || !size || size>kMaxDisplayWireBytes) return false;
  WireCursor in(data,size);
  if (type=="nav_msgs/Path") {
    std::uint32_t count;
    if (!in.header() || !in.count(&count) || count>in.remaining()/72) return false;
    for (std::uint32_t i=0;i<count;++i) if (!in.header() || !in.skip(56)) return false;
  } else if (type=="geometry_msgs/PoseArray") {
    if (!in.header() || !in.array(56)) return false;
  } else if (type=="nav_msgs/OccupancyGrid") {
    if (!in.header() || !in.skip(8+4+4+4+56) || !in.array(1)) return false;
  } else if (type=="sensor_msgs/PointCloud2") {
    std::uint32_t count;
    if (!in.header() || !in.skip(8) || !in.count(&count) || count>in.remaining()/13) return false;
    for (std::uint32_t i=0;i<count;++i) if (!in.string() || !in.skip(9)) return false;
    if (!in.skip(1+4+4) || !in.array(1) || !in.skip(1)) return false;
  } else if (type=="visualization_msgs/Marker") {
    if (!in.marker()) return false;
  } else if (type=="visualization_msgs/MarkerArray") {
    std::uint32_t count;
    if (!in.count(&count) || count>in.remaining()/154) return false;
    for (std::uint32_t i=0;i<count;++i) if (!in.marker()) return false;
  } else return false;
  return in.done();
}
} // namespace xgc2_ros_visualizer

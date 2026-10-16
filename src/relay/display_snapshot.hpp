#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>

namespace xgc2_ros_visualizer {
// A world-frame map is a snapshot. Ignore only Header seq/stamp when its
// geometry, frame and metadata are identical. Other frames still need fresh
// stamps for time-dependent TF. The bytes actually published stay untouched.
inline bool sameWorldSnapshot(const std::vector<std::uint8_t>& a,
                              const std::vector<std::uint8_t>& b) {
  if (a.size() != b.size() || a.size() < 16) return false;
  const std::uint32_t n = std::uint32_t(a[12]) | (std::uint32_t(a[13]) << 8) |
      (std::uint32_t(a[14]) << 16) | (std::uint32_t(a[15]) << 24);
  const char* frame = n == 5 ? "world" : n == 6 ? "/world" : nullptr;
  if (!frame || a.size() < 16 + n ||
      !std::equal(a.begin() + 16, a.begin() + 16 + n, frame)) return false;
  return std::equal(a.begin() + 12, a.end(), b.begin() + 12);
}
} // namespace xgc2_ros_visualizer

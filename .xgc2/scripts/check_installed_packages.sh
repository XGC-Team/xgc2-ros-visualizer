#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
prefix=/opt/ros/noetic
package=ros-noetic-xgc2-ros-visualizer
expected="$(awk '/^version:/ {print $2; exit}' "$repo_root/.xgc2/product.yml")~focal"
test "$(dpkg-query -W -f='${db:Status-Status}' "$package")" = installed
test "$(dpkg-query -W -f='${Version}' "$package")" = "$expected"
test "$(rospack find xgc2_ros_visualizer)" = "$prefix/share/xgc2_ros_visualizer"
for path in lib/libxgc2_ros_visualizer_runtime.so lib/libxgc2_ros_visualizer_contract.so lib/pkgconfig/xgc2_ros_visualizer.pc include/xgc2_ros_visualizer/config.hpp include/xgc2_ros_visualizer/profile.hpp include/xgc2_ros_visualizer/declared_wire.hpp include/xgc2_ros_visualizer/scene_contract.hpp share/xgc2_ros_visualizer/cmake/xgc2_ros_visualizerConfig.cmake; do
  test -s "$prefix/$path"
done
binary="$prefix/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node"
test -x "$binary"
for path in "$binary" "$prefix/lib/libxgc2_ros_visualizer_runtime.so" "$prefix/lib/libxgc2_ros_visualizer_contract.so"; do
  linkage="$(ldd "$path")"
  [[ "$linkage" != *'not found'* ]]
  if printf '%s\n' "$linkage" | grep -Eq 'lib(fs150_uav_visualizer|scout_ugv_visualizer|mecanum_ugv_visualizer|robot_description_runtime|robot_path_runtime)\.so'; then
    echo "Retired robot visualization library remains linked: $path" >&2
    exit 1
  fi
done
consumer="$(mktemp -d)"
trap 'rm -rf "$consumer"' EXIT
cat > "$consumer/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.10)
project(visualizer_installed_consumer LANGUAGES CXX)
find_package(catkin REQUIRED COMPONENTS xgc2_ros_visualizer)
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_14)
target_include_directories(consumer PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(consumer PRIVATE ${catkin_LIBRARIES})
CMAKE
cat > "$consumer/main.cpp" <<'CPP'
#include <xgc2_ros_visualizer/config.hpp>
int main() {
  const auto rates=xgc2_ros_visualizer::defaultRates();
  return xgc2_ros_visualizer::parseRates(rates.json(),true).maximum()>0?0:1;
}
CPP
cmake -S "$consumer" -B "$consumer/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$consumer/build" -j1
"$consumer/build/consumer"
definition=/usr/share/xgc2/process-definitions/xgc2-ros-visualizer.json
test -s "$definition"
python3 "$repo_root/test/profile_probe.py" "$binary" "$definition"
python3 "$repo_root/test/installed_node_probe.py" "$binary"
python3 "$repo_root/test/start_order_probe.py" "$binary"
if [[ "${XGC2_RUN_LONG_TESTS:-0}" == 1 ]]; then
  python3 "$repo_root/test/start_order_probe.py" "$binary" --long
fi
python3 "$repo_root/test/check_description_resources.py"
echo 'Installed single-entry SDK, process definition, robot profiles, rates, persistent relay, URDF and owned cleanup checks passed'

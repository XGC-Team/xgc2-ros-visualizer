#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
install_root=""
output_dir=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root) install_root="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done
test -n "$install_root" && test -n "$output_dir"
version="$(awk '/^version:/ {print $2; exit}' "$repo_root/.xgc2/product.yml")"
test -n "$version"
version="${version}~focal"
arch="$(dpkg --print-architecture)"
case "$arch" in amd64|arm64) ;; *) exit 1 ;; esac
package=ros-noetic-xgc2-ros-visualizer
prefix=/opt/ros/noetic
stage="$install_root$prefix"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
pkg_root="$work/package"
mkdir -p "$pkg_root/DEBIAN" "$pkg_root/usr/share/doc/$package" "$output_dir" "$work/debian"
for path in share/xgc2_ros_visualizer include/xgc2_ros_visualizer lib/pkgconfig/xgc2_ros_visualizer.pc lib/libxgc2_ros_visualizer_runtime.so lib/libxgc2_ros_visualizer_contract.so lib/xgc2_ros_visualizer; do
  test -e "$stage/$path"
  mkdir -p "$pkg_root$prefix/$(dirname "$path")"
  cp -a "$stage/$path" "$pkg_root$prefix/$path"
done
# The process definition is read by the supervisor from the common directory.
definitions="$pkg_root/usr/share/xgc2/process-definitions"
mkdir -p "$definitions"
mv "$pkg_root$prefix/share/xgc2_ros_visualizer/process-definitions/xgc2-ros-visualizer.json" "$definitions/"
rmdir "$pkg_root$prefix/share/xgc2_ros_visualizer/process-definitions"
for path in include/xgc2_ros_visualizer/config.hpp include/xgc2_ros_visualizer/profile.hpp include/xgc2_ros_visualizer/declared_wire.hpp include/xgc2_ros_visualizer/scene_contract.hpp share/xgc2_ros_visualizer/cmake/xgc2_ros_visualizerConfig.cmake share/xgc2_ros_visualizer/package.xml; do
  test -s "$pkg_root$prefix/$path"
done
test -s "$definitions/xgc2-ros-visualizer.json"
test -x "$pkg_root$prefix/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node"
test -s "$pkg_root$prefix/lib/libxgc2_ros_visualizer_runtime.so"
test -s "$pkg_root$prefix/lib/libxgc2_ros_visualizer_contract.so"
cat > "$work/debian/control" <<EOF
Source: xgc2-ros-visualizer
Section: misc
Priority: optional
Maintainer: XGC Team <867768510@qq.com>

Package: $package
Architecture: any
Description: Single-process ROS1 visualization server
EOF
# ROS Noetic libraries have unversioned SONAMEs and dpkg-shlibdeps skips them.
# roscpp explicitly owns those runtime DSOs; derive system-library requirements
# from both ELF files. Message packages are required by the public SDK.
shlibs="$(cd "$work" && dpkg-shlibdeps -O -l"$stage/lib" -l"$prefix/lib" -e"$stage/lib/libxgc2_ros_visualizer_runtime.so" -e"$stage/lib/libxgc2_ros_visualizer_contract.so" -e"$stage/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node")"
case "$shlibs" in shlibs:Depends=*) shlibs="${shlibs#shlibs:Depends=}" ;; *) exit 1 ;; esac
test -n "$shlibs"
cat > "$pkg_root/DEBIAN/control" <<EOF
Package: $package
Version: $version
Section: misc
Priority: optional
Architecture: $arch
Maintainer: XGC Team <867768510@qq.com>
Depends: $shlibs, libxgc2-xrpc2 (>= 0.2.0-1~focal), ros-noetic-roscpp, ros-noetic-roslib, ros-noetic-urdf, ros-noetic-sensor-msgs, ros-noetic-nav-msgs, ros-noetic-geometry-msgs, ros-noetic-foxglove-msgs, ros-noetic-mavros-msgs, ros-noetic-std-msgs, ros-noetic-tf2-ros, ros-noetic-visualization-msgs, ros-noetic-xgc2-fs150-description (>= 0.1.0-11), ros-noetic-xgc2-scout-description (>= 0.4.10-16), ros-noetic-xgc2-mecanum-description (>= 0.1.0-10)
Description: XGC2 ROS1 visualization
 Run-owned visualization instances, in-process URDF transforms and persistent
 byte-preserving display relays. Gazebo rendering is not in this package.
EOF
cp "$repo_root/LICENSE" "$pkg_root/usr/share/doc/$package/copyright"
find "$pkg_root" -type d -exec chmod 0755 {} +
find "$pkg_root" -type f -exec chmod 0644 {} +
chmod 0755 "$pkg_root$prefix/lib/libxgc2_ros_visualizer_runtime.so" "$pkg_root$prefix/lib/libxgc2_ros_visualizer_contract.so" "$pkg_root$prefix/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node"
rm -f "$output_dir/${package}_"*.deb
dpkg-deb --root-owner-group --build "$pkg_root" "$output_dir/${package}_${version}_${arch}.deb"

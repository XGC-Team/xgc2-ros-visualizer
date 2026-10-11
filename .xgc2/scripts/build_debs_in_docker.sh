#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image=ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.8@sha256:fce2d76fddf4f6439bf0a188249b731650febdc163befc360bed186b269d252a
cxx="g++"
work_dir=""
output_dir=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) image="$2"; shift 2 ;;
    --cxx) cxx="$2"; shift 2 ;;
    --work-dir) work_dir="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done
test -n "$work_dir" && test -n "$output_dir"
if [[ -z "$cxx" ]]; then
  echo "--cxx must select a preprovisioned C++17 compiler in the controlled Noetic image" >&2
  exit 1
fi
install -d "$work_dir" "$output_dir"
work_dir="$(cd "$work_dir" && pwd)"
output_dir="$(cd "$output_dir" && pwd)"
host_uid="$(id -u)"
host_gid="$(id -g)"
# The build image does not carry published XGC2 packages. Install the released
# robot description assets and XRPC from production APT, then compile as the runner
# so the deb outputs stay writable. Installation stays inside the container.
docker run --rm -i --cpus 1 \
  -e HOME=/tmp -e ROS_HOME=/tmp/ros-home -e ROS_LOG_DIR=/tmp/ros-log -e ROS_IP=127.0.0.1 \
  -e XGC2_BUILD_CXX="$cxx" \
  -e XGC2_RUN_SOURCE_TESTS="${XGC2_RUN_SOURCE_TESTS:-1}" \
  -e HOST_UID="$host_uid" -e HOST_GID="$host_gid" \
  -v "$repo_root:/source:ro" -v "$work_dir:/work" -v "$output_dir:/out" \
  "$image" bash -s <<'EOF'
set -eo pipefail
echo "deb [trusted=yes arch=$(dpkg --print-architecture)] https://xgc2.apt.xiaokang.ink focal main" \
  >/etc/apt/sources.list.d/xgc2.list
apt-get update
apt-get install -y --no-install-recommends ros-noetic-xgc2-fs150-description ros-noetic-xgc2-scout-description ros-noetic-xgc2-mecanum-description libxgc2-xrpc-dev
chown "${HOST_UID}:${HOST_GID}" /work /out
python3 -c 'import os,sys; os.setgid(int(os.environ["HOST_GID"])); os.setuid(int(os.environ["HOST_UID"])); os.execvp("bash", ["bash", "-c", sys.argv[1]])' '
set -eo pipefail
unset ROS_HOSTNAME
source /opt/ros/noetic/setup.bash
set -u
command -v "$XGC2_BUILD_CXX" >/dev/null
cmake -S /source -B /work/build -DCMAKE_CXX_COMPILER="$XGC2_BUILD_CXX" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DCATKIN_ENABLE_TESTING=ON
cmake --build /work/build -j1
if [[ "$XGC2_RUN_SOURCE_TESTS" == 1 ]]; then
  cmake --build /work/build --target tests -j1
  cmake --build /work/build --target run_tests -j1
  catkin_test_results /work/build/test_results
fi
DESTDIR=/work/install cmake --install /work/build
/source/.xgc2/scripts/package_debs.sh --install-root /work/install --output-dir /out
'
EOF
docker run --rm -i --cpus 1 \
  -e HOME=/tmp -e ROS_HOME=/tmp/ros-home -e ROS_LOG_DIR=/tmp/ros-log -e ROS_IP=127.0.0.1 \
  -e XGC2_RUN_LONG_TESTS="${XGC2_RUN_LONG_TESTS:-0}" \
  -v "$repo_root:/source:ro" -v "$output_dir:/out:ro" \
  "$image" bash -s <<'EOF'
set -eo pipefail
unset ROS_HOSTNAME
echo "deb [trusted=yes arch=$(dpkg --print-architecture)] https://xgc2.apt.xiaokang.ink focal main" \
  >/etc/apt/sources.list.d/xgc2.list
apt-get update
debs=(/out/ros-noetic-xgc2-ros-visualizer_*.deb)
test "${#debs[@]}" -eq 1
apt-get install -y --no-install-recommends "${debs[0]}"
source /opt/ros/noetic/setup.bash
set -u
/source/.xgc2/scripts/check_installed_packages.sh
EOF

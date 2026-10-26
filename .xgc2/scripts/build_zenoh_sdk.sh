#!/usr/bin/env bash
# Build the Focal-compatible Zenoh C SDK once, outside deployment. The caller
# supplies a writable work directory and install prefix inside its build image.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source_commit=7b8478618861972b9f0115da75e694c9f0a2af5a
source_sha256=0ee86bbb03b0964cfe08e247fd117cad0e55bbe022b63f8f7188bee146536c64
zenoh_commit=29b3e63a311ab55f438280f3e71c6268b6f115cc
main_lock_sha=981a0ad256895aa52f41e6cbeecd34b0cbe1cda1c66350e78a5b192394c37123
opaque_lock_sha=d8fec3d78270f8304914c069435c123014b56bc5963d838c20e0d80be93e7183
work_dir=""; prefix=""; mode=build
while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir) work_dir="$2"; shift 2 ;;
    --install-prefix) prefix="$2"; shift 2 ;;
    --verify) mode=verify; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$prefix" = /* ]] || { echo 'install prefix must be absolute' >&2; exit 2; }
architecture="$(dpkg --print-architecture)"
case "$architecture" in amd64|arm64) ;; *) echo "unsupported architecture $architecture" >&2; exit 2 ;; esac
verify() {
  [[ -f "$prefix/.xgc2-zenoh-sdk" && -f "$prefix/SHA256SUMS" ]] || return 1
  [[ "$(cat "$prefix/.xgc2-zenoh-sdk")" == "zenoh-c=1.8.0 source=$source_commit zenoh=$zenoh_commit rust=1.93.0 arch=$architecture lock=$main_lock_sha opaque=$opaque_lock_sha" ]] || return 1
  local required
  for required in include/zenoh.h lib/libzenohc.a lib/cmake/zenohc/zenohcConfig.cmake lib/cmake/zenohc/zenohcConfigVersion.cmake; do
    [[ -s "$prefix/$required" ]] || return 1
    awk -v name="$required" '$2 == name { found=1 } END { exit !found }' "$prefix/SHA256SUMS" || return 1
  done
  (cd "$prefix" && sha256sum --quiet -c SHA256SUMS) || return 1
}
if [[ "$mode" == verify ]]; then verify; exit; fi
[[ "$work_dir" = /* ]] || { echo 'work directory must be absolute' >&2; exit 2; }
[[ "$(rustc --version)" == 'rustc 1.93.0 '* ]] || { echo 'Rust 1.93.0 is required' >&2; exit 1; }
[[ "$(cargo --version)" == 'cargo 1.93.0 '* ]] || { echo 'Cargo 1.93.0 is required' >&2; exit 1; }
mkdir -p "$work_dir" "$prefix"
work_dir="$(cd "$work_dir" && pwd)"
prefix="$(cd "$prefix" && pwd)"
if verify; then echo "using verified Zenoh SDK $prefix"; exit 0; fi
printf '%s  %s\n' "$main_lock_sha" "$repo_root/.xgc2/zenoh-c-1.8.0/Cargo.lock" "$opaque_lock_sha" "$repo_root/.xgc2/zenoh-c-1.8.0/opaque-Cargo.lock" | sha256sum --check --quiet
scratch="$(mktemp -d "$work_dir/zenoh-c.XXXXXXXX")"
trap 'rm -rf "$scratch"' EXIT
curl --fail --location --retry 3 --max-time 120 \
  "https://codeload.github.com/eclipse-zenoh/zenoh-c/tar.gz/$source_commit" -o "$scratch/source.tar.gz"
printf '%s  %s\n' "$source_sha256" "$scratch/source.tar.gz" | sha256sum --check --quiet
mkdir "$scratch/source"
tar -xzf "$scratch/source.tar.gz" -C "$scratch/source" --strip-components=1
# The 1.8.0 tag's manifest names the moving release branch while its shipped
# lock names old 1.7.2 main. Pin all three manifests to the 1.8.0 release
# commit, use the minimally updated locked dependencies, and prefetch the
# opaque-types helper's one additional locked crate before offline compilation.
python3 - "$scratch/source" "$zenoh_commit" <<'PY'
import pathlib, sys
root, commit = pathlib.Path(sys.argv[1]), sys.argv[2]
for name, expected in (("Cargo.toml", 5), ("Cargo.toml.in", 5), ("build-resources/opaque-types/Cargo.toml", 3)):
    path = root / name
    data = path.read_text()
    old = 'branch = "release/1.8.0"'
    if data.count(old) != expected:
        raise SystemExit("unexpected upstream Zenoh dependency declarations: " + name)
    path.write_text(data.replace(old, 'rev = "' + commit + '"'))
PY
cp "$repo_root/.xgc2/zenoh-c-1.8.0/Cargo.lock" "$scratch/source/Cargo.lock"
cp "$repo_root/.xgc2/zenoh-c-1.8.0/opaque-Cargo.lock" "$scratch/source/build-resources/opaque-types/Cargo.lock"
export CARGO_NET_GIT_FETCH_WITH_CLI=true
export CARGO_BUILD_JOBS="${XGC2_ZENOHC_JOBS:-1}"
[[ "$CARGO_BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || { echo 'XGC2_ZENOHC_JOBS must be positive' >&2; exit 2; }
cargo fetch --locked --manifest-path "$scratch/source/Cargo.toml"
cargo fetch --locked --manifest-path "$scratch/source/build-resources/opaque-types/Cargo.toml"
export CARGO_NET_OFFLINE=true
cmake -S "$scratch/source" -B "$scratch/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DZENOHC_BUILD_WITH_SHARED_MEMORY=TRUE \
  -DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE '-DZENOHC_CARGO_FLAGS=--locked;--offline'
cmake --build "$scratch/build" --parallel "$CARGO_BUILD_JOBS"
# The upstream build script reuses the main lock and resolves the opaque helper
# offline. Fail if that resolution differs from the reviewed helper lock.
printf '%s  %s\n' "$opaque_lock_sha" "$scratch/source/build-resources/opaque-types/Cargo.lock" | sha256sum --check --quiet
cmake --install "$scratch/build"
printf 'zenoh-c=1.8.0 source=%s zenoh=%s rust=1.93.0 arch=%s lock=%s opaque=%s\n' \
  "$source_commit" "$zenoh_commit" "$architecture" "$main_lock_sha" "$opaque_lock_sha" > "$prefix/.xgc2-zenoh-sdk"
(cd "$prefix" && find include lib/cmake/zenohc lib -maxdepth 2 -type f \( -name 'zenoh*.h' -o -name 'libzenohc.a' -o -name 'zenohcConfig*.cmake' \) -print | LC_ALL=C sort | xargs sha256sum > SHA256SUMS)
verify

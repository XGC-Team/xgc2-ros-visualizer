# ROS1 visualizer

`xgc2-ros-visualizer` provides one C++ ROS server, one executable
`xgc2_ros_visualizer_node`, and the canonical catkin package
`xgc2_ros_visualizer`. A server owns a fixed input pool (default 2, maximum 32
workers), one publication scheduler, and a bounded XRPC Unix HTTP control loop plus one fixed domain worker.
Robots, descriptions and relays are instance data. Adding robots does not create
application threads, helper processes or `robot_state_publisher` children.
Core starts it through its process definition, discovers the service with
`GET /v1/describe` and applies the display configuration of its Run through
RPC. Normal Stop terminates that Run's native server.
The socket is ready before the master, Foxglove bridge or robot topics exist.
One background master watcher uses jittered backoff; requested ROS subscribers
pick up late topics and publisher restarts through publisher events. The robot
roster comes from Core's session rows, and Core replays those rows after a
visualizer restart. The visualizer starts with an empty roster each time.

The visualization of an experiment is a set of robot profiles. A profile holds
everything needed to draw one robot (model, state source, frames, path, labels,
animation, publish policy). The desired configuration of an instance is
replaced as a whole or one robot at a time; the server diffs it against the
applied state and rebuilds only the robots whose profile changed. Rates have a
per-kind table as the default and per-robot overrides. Saved profiles belong to
Core configuration; this server keeps the applied state in memory only.

This product owns FS150, Scout and Mecanum geometry, wheel/rotor animation,
frame names and path styles together with ROS subscriptions, publication,
scheduling, URDF parameters and required fixed/joint TF. Robot rendering is
implemented privately in `src/render/robots`; there is no separate robot
visualization package or description-publisher process. Meshes and URDF assets
remain in the robot description products. Gazebo shadow rendering, algorithm
planning, map production and Viewer rendering remain their owning products.

ROS1 visualization is the single implementation authority. If future ROS2 or
other visualization publishers need the same behavior, extract the shared
calculations from this implementation into a ROS-independent library then.

## Startup and RPC

Source a ROS Noetic environment and select its master/IP normally. The
supervisor supplies allocated `ROS_HOME` and `ROS_LOG_DIR` locations for native
cache/log ownership and the socket path inside its private runtime directory:

```sh
rosrun xgc2_ros_visualizer xgc2_ros_visualizer_node \
  --socket-path "$XGC_RUNTIME_DIR/sockets/visualizer.sock" --world-clock simulation
curl --unix-socket "$XGC_RUNTIME_DIR/sockets/visualizer.sock" \
  -H 'X-Request-ID: discover-1' -H 'X-Xrpc-Timeout-Ms: 3000' \
  http://localhost/v1/describe
```

The process ships its definition for Core, `process-definitions/xgc2-ros-visualizer.json`
(installed to `/usr/share/xgc2/process-definitions`): the executable, its
parameters, the service it hosts, readiness by `describe` and `stop.graceMs: 5000`.
Core allocates the service socket named by `endpointParameter`; the socket parameter
is `fixedOnly`.
The service contract, with every route, the profile schema, revisions and the
lifecycle, is in [docs/visualization-v1.md](docs/visualization-v1.md).

| Request | Behavior |
| --- | --- |
| `GET /v1/describe[?wait_ready_ms=]` | Readiness envelope `{service, api_version, instance_id, ready, facts}`; the wait is answered without polling |
| `GET /v1/status` | Readiness, the rate table and every instance's status |
| `GET /v1/rates`, `PUT /v1/rates` | The per-kind default table; `{expectedRevision, rates}` replaces it atomically |
| `GET`/`PUT`/`DELETE /v1/instances/<id>` | Observe, apply a new desired configuration (only changed robots rebuild), or retract an instance |
| `GET`/`PUT`/`DELETE /v1/instances/<id>/robots/<robot>` | Observe, apply or remove one robot's profile |
| `GET`/`PUT /v1/instances/<id>/robots/<robot>/rates` | Observe or replace the rate overrides of one robot |

An instance PUT carries the full frozen public Robot array, the session context,
the panel settings and the display relays; the native product projects its own
visualization fields atomically and Core does not construct a second projected
roster. Empty rosters are valid. Configured scene instances, including
zero-robot worlds, publish `/xgc/robot_scene/ready`; this acknowledges
configuration, not fresh robot poses or scientific progress. Descriptions also
publish `/xgc/robot_descriptions/ready`.

```json
{"robots":[],"context":{"runMode":"simulation","worldClock":"simulation","worldBoundary":null},"settings":{},"displayRelays":[]}
```

Namespace, scene model and installed description metadata come from each
Robot's visualization configuration; the saved profile of a Robot is its
`visualization.profile`. The native FS150 AR projection selects the frozen
member's physical or simulation source, resolves its pose topic and applies
`context.localizationOffset` only to raw motion capture. Direct xsim
coordinates already include that offset. Numeric slot palette ordering and the
original Scout mocap scene-model binding are preserved. Unrelated panel fields
remain Viewer-owned.

Instances cannot share the existing global scene/frame outputs on the same ROS
graph. Independent relay-only instances can disable scene/transforms and use
disjoint display topics. Conflicts, duplicate models and duplicate path outputs
fail before anything changes. DELETE never publishes SceneDeletion
ALL or purges all ROS description parameters. A replaced foreign parameter value
is preserved during cleanup.

## Data and publication rates

Local ROS subscriptions remain active until their instance is deleted or the server
stops, regardless of Viewer connections. Remote display subscriptions follow
output demand as described below. ROS input queues are size 1. Callbacks
update fixed latest state and fixed source-time history under short locks;
serialization, publication, URDF parsing and network I/O occur outside those
locks. The rate table is an immutable publication snapshot; a robot's overrides apply
from the next tick. Rate changes, membership changes and Stop wake the single
scheduler immediately, including when all rates are 0.1 Hz.

| Kind/channel | Default ceiling | Output |
| --- | ---: | --- |
| FS150/Scout/Mecanum `pose_tf` | 120 Hz | `/xgc/tf` body/label/camera and AR anchor transforms, new source stamps only |
| Each kind `joint_tf`, `markers` | 30 Hz | `/xgc/tf` visual rotor/wheel TF; optional `/markers` geometry/path/labels |
| Each kind `scene`, `scene_path`, `path`, `ar_path`, `ar_identity` | 10 Hz | `/xgc/scene` labels/optional paths, namespace Path topics, `/xgc/scene_ar` |
| FS150 `height_projection`, `height_projection_ar` | 10 Hz | `/xgc/uav_height_projection` and `/xgc/uav_height_projection_ar` |
| Global `tf_root` | 30 Hz | Standard `/tf` world root |
| Global `tf_static`, `world_boundary`, `readiness` | 1 Hz | Latched fixed aliases/URDF TF, world boundary layers and ready events |
| Global `joint_tf` | 30 Hz | In-process URDF joint TF for descriptions without a matching concrete scene kind |
| All kinds `display_pointcloud`, `display_grid`, `display_path`, `display_pose_array` | 10 Hz | Original serialized data under `/xgc/display` |
| All kinds `display_marker`, `display_marker_array` | 10 Hz | Coalesced Marker / MarkerArray copies under `/xgc/display` |

All applicable ceilings accept finite 0.1–1000 Hz numbers. The table holds the
defaults of every robot of a kind; a robot can override the channels of its own
kind row (not the relay channels or the scene-wide `global` channels) through
`PUT /v1/instances/<id>/robots/<robot>/rates`. Static, readiness and
label outputs are event-driven with ceilings, rather than periodic replay.
Description joint TF uses the matching robot kind's `joint_tf` ceiling when
available; global `joint_tf` does not override the three concrete kinds.
World boundary topics remain `/xgc/world_boundary`, `/xgc/world_boundary/ar`,
`/xgc/world_boundary_walls` and `/xgc/world_boundary_walls/ar`.

3D FS150 reads its fused `/<namespace>/mavros/local_position/pose`; ground robots
read canonical `/<namespace>/pose`. AR reads the exact frozen source with its
world offset once. Source stamps, finite/frame/freshness checks, the Scout body
height, ground-path z=0, raw AR quaternion and the off/ground/walls semantics are
preserved. Rotor state reads MAVROS state/extended-state; ground wheel animation
reads canonical twist, with cmd_vel as the existing fallback. The server never
writes those scientific inputs or controller parameters.

Nav Path, Marker path and Scene path use the same source-stamp sampled 6-second,
10-Hz, maximum-61-point ring. Lowering an output ceiling does not lower history
sampling, including with accelerated simulation time. History expires using ROS
time and respects rollback/reset semantics. URDF XML is loaded from installed
package-relative files. Parameters and fixed/movable joint transforms are
produced in-process using the original description prefix; no RSP is forked.

A wire relay request is `{source,topic,messageType}` plus optional `maxRateHz`
(finite 0.1–100, JSON null or absent means no extra ceiling) and optional
`role` (`prediction`, `obstacle`, or `semantic`; null or absent means none).
Its kind comes from the frozen Robot roster. Source names are unique canonical
absolute names outside `/xgc/display`; output is exactly `/xgc/display` + source.
Maximum 128 per instance. PointCloud2, OccupancyGrid, Path and PoseArray are
copied byte for byte: the relay publishes the same immutable pointer and does
not decode or rewrite them. Marker and MarkerArray are coalesced by namespace
and id. A declaration above the channel's rate does not raise it. The effective
ceiling is the minimum of the channel rate, `maxRateHz` when set, and, for
`role=prediction` only, `settings.predictionMaxRateHz` (default 5, range
0.1–100). A wire `robotKind` or any other relay field is rejected.

The ceiling counts published messages, on the existing scheduler. MarkerArray
sends one bounded batch per due period. A singular Marker sends one Marker per
due period and keeps every unsent id, delete, and DELETEALL; the next id is
the next `(ns, id)` after the one just sent, wrapping, so a busy low id cannot
take every period. A positive Marker lifetime starts at relay ingestion, not
at `header.stamp`. `header.stamp` is copied through unchanged. Each new ADD,
including a refresh of the same stamp, starts that lifetime again. A delayed
send advertises only the time still left, never a longer duration than the
original. A lifetime of zero stays forever. An id that is already dead is deleted
if this relay published it, and dropped if it never did. Pending Marker payload
(points, colors, text, mesh resource) is bounded by the display-budget
`maxMessageBytes` contract, 1048576 bytes, and by 4096 ids. A key or byte
overflow deletes only that namespace. One global DELETEALL is reserved for an
unknown action or more than 32 DELETEALL barriers.

`settings.historyMaxRateHz` (default 10, range 0.1–100) and
`settings.historyVisible` (default true) limit Path, AR path, scene-path, and
the history `LINE_STRIP` on the markers topic. That line's ADD uses the path
ceiling, the minimum of the path channel rate and `historyMaxRateHz`. Meshes,
labels, and TF stay on their own channels and are not capped by this setting.
Hiding history publishes that line's DELETE once, plus the empty latched paths
and the scene-path deletion. The 6-second, 10 Hz, 61-point source ring is
unchanged. `settings.predictionMaxRateHz` is the prediction ceiling above.
Pose-axis settings are ignored here.

Membership, queues, latest-message retention and history point counts are
bounded. Full-state relay bytes are still the source message. Marker pending
bytes stop at `maxMessageBytes`. This is not a claim of zero allocations.

## Build, package and validation

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/path/to/prefix -DCMAKE_INSTALL_PREFIX=/path/to/prefix
cmake --build build -j2
cmake --build build --target tests -j2
cmake --build build --target run_tests -j2
catkin_test_results build/test_results
cmake --install build
```

The control host uses the installed `XgcXrpc` 0.2 `http` component (C++17, plain
limit structs; nothing is read from the environment). The whole product builds
as C++17 with the Focal compiler; the runtime and contract libraries stay C++14
for their consumers. The SDK is not copied into this product.

The native package is `ros-noetic-xgc2-ros-visualizer`, version 0.5.0-1. It installs
one executable, the runtime/contract libraries, public canonical headers and
the process definition. Consumers use
`find_package(catkin REQUIRED COMPONENTS xgc2_ros_visualizer)`.
The installed Robot SDK ABI dependency is at least 0.2.0-16.

`.xgc2/scripts/build_debs_in_docker.sh` requires an immutable Noetic build image,
and installs published SDK dependencies only inside independent containers. It runs
source tests, produces the Deb, then installs that Deb in a second container and
checks the catkin exports, linkage and actual executable. CI selects these resources through `XGC2_FOCAL_CXX_BUILD_IMAGE` and
`XGC2_FOCAL_CXX`; images are selected by digest. This build requires
network access to production APT for dependencies; it never uses an active Core
container as its builder.

Private-graph probes start their own master and run the actual executable:

* `test/profile_probe.py` covers this contract: describe and its wait, creating
  an instance, changing one robot's profile and checking, through the master's
  registry and the peer events of the fake robot sources, that exactly that
  robot's resources were rebuilt; saved profiles and panel settings reaching only
  the robots they change; scene switches that leave robot inputs alone; per-robot
  rate overrides, the per-kind table, revision CAS and rejected changes; adding and
  removing robots; output claims between instances; an unreachable and a replaced
  master.
* `test/marker_rate_probe.py` counts singular Marker messages under the ceiling,
  including distinct ids and a DELETE followed by an ADD. It does not treat a
  publish cycle as the Marker rate.
* `test/installed_node_probe.py` covers the resource behavior: zero-robot
  readiness, four-type exact-byte relays, atomic validation, 20/100 robot
  resource counts, source-time paths at accelerated clock and low rates,
  in-process URDF joint TF, foreign parameter preservation, bounded DELETE and
  low-rate SIGTERM, restart and SIGKILL recovery without any resurrected state.
* `test/start_order_probe.py` starts the service two seconds before its master,
  applies rows with no master/topics, starts and restarts a publisher process,
  and replays the identical PUT after a service restart. `--long` also measures
  60-second publisher and master outages: CPU below 1.2 seconds, bounded retries
  and log lines, stable thread/fd counts, recovery and master replacement.

```sh
python3 test/profile_probe.py /opt/ros/noetic/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node
python3 test/marker_rate_probe.py /opt/ros/noetic/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node
python3 test/installed_node_probe.py /opt/ros/noetic/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node
python3 test/start_order_probe.py /opt/ros/noetic/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node
python3 test/start_order_probe.py /opt/ros/noetic/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node --long
```

The CTest labels are `ros` and, for the outage test, `long`: fast cells may use
`(cd build && ctest -LE long)`; the amd64 installed-package CI cell sets
`XGC2_RUN_LONG_TESTS=1` and runs the outage probe.

Private probes are correctness/resource checks, not formal station, browser or
scientific-mission acceptance. Source CI and Deb preparation do not imply
production APT visibility or live station adoption; the central release
workflow owns publication.

## Explicit remote inputs

A configured remote input consumes the raw ROS1 serialization bytes on one
exact Zenoh key. There is no envelope, ROS graph discovery, local ROS publisher,
implicit remapping, or fallback to the local topic when the key is silent. The
sender must serialize the declared ROS1 type, connect to the visualizer's
`listen` TCP endpoint, and disable scouting. The visualizer listens on that
endpoint in Zenoh peer mode with multicast and gossip scouting disabled and no
outbound connect endpoints. All remote member inputs and display relays across
one visualizer process must use the same endpoint; they share one Zenoh
session. This prevents a per-topic listener or session. Changing a live endpoint
requires first removing its remote inputs. The address must be a local address
reachable by the sender; `tcp/127.0.0.1:17447` is only a same-host example.

The instance PUT `settings.robotProfiles` object accepts profile overlays keyed
by robot namespace without its leading slash. Each value overlays the generated
profile and then passes the existing strict profile parser; unknown robots,
robots without descriptions, non-object entries and invalid fields are rejected.
This settings overlay takes precedence over a Robot `visualization.profile`.
Per-input source entries not listed retain their existing local ROS subscription. An entry is
`{"listen":"tcp/<address>:<port>","key":"<exact-key>"}`. Wildcards and
non-TCP endpoints are rejected. The following example keeps AR pose local while
reading this member's local-position pose and MAVROS flight state remotely:

```json
{"robotProfiles":{"uav1":{"state":{"sources":{
  "pose":{"listen":"tcp/192.0.2.10:17447","key":"run/s1/uav1/local_pose"},
  "state":{"listen":"tcp/192.0.2.10:17447","key":"run/s1/uav1/state"},
  "extendedState":{"listen":"tcp/192.0.2.10:17447","key":"run/s1/uav1/extended_state"}
}}}}}
```

FS150 accepts `pose` and `arPose` (`geometry_msgs/PoseStamped`), `state`
(`mavros_msgs/State`) and `extendedState` (`mavros_msgs/ExtendedState`).
`arPose` is valid only when `state.arPoseTopic` is present. Scout/Mecanum accept
`pose` (`geometry_msgs/PoseStamped`), `twist` (`geometry_msgs/TwistStamped`)
and `cmdVel` (`geometry_msgs/Twist`). Existing source-time, world offset and
history callbacks are shared with local input. Replacing or deleting a profile
retracts its remote subscriptions. Remote member subscriptions are declared only while the member has a consumer
of its Path/AR Path or a shared scene, transform, height or marker output;
they are retracted when the last such consumer disconnects or the member is
removed. A Path-only consumer requests pose; an AR Path-only consumer requests
AR pose. A shared scene consumer requests all configured scene members. A
missing remote sample does not cause a ROS fallback.

For an already declared `displayRelays` source, the instance PUT settings can
select its remote source through `displayRelaySources`, keyed by the exact ROS
source topic. Unknown or undeclared sources and invalid entries are rejected.
For example, the settings below select a declared `nav_msgs/Path` prediction:

```json
{"displayRelaySources":{"/uav1/alg/prediction":
 {"listen":"tcp/192.0.2.10:17447","key":"run/s1/uav1/prediction"}}}
```

A direct instance PUT may alternatively add the same `{listen,key}` as `zenoh`
on its `displayRelays` record; `settings.displayRelaySources` takes precedence.
The `source`, output `topic` and `messageType` remain required relay fields.

The usual six display message types and existing rate/coalescing policies apply.
For a remote display relay, the Zenoh subscription exists only while the
corresponding `/xgc/display` publisher has a ROS subscriber. Sources are not
republished as raw ROS topics. Member messages are capped at 64 KiB and display
messages at 1 MiB; malformed, truncated, trailing or oversized payloads are
discarded before consumer callbacks and before any array-sized allocation.

The Focal build image contains Rust 1.93.0 but no Zenoh C SDK. The product's
builder automatically fetches the fixed Zenoh C 1.8.0 source commit, verifies
its archive SHA-256, pins the Zenoh 1.8.0 dependency commit, and builds the
checked-in Cargo locks using that toolchain. Downloads occur during SDK
preparation; compilation runs offline. The verified SDK is cached in
`<work-dir>/zenoh-sdk`; the product's CI and release jobs cache it per
architecture and SDK recipe. The optional `--zenoh-prefix` / `XGC2_ZENOHC_PREFIX`
is still available for a preprovisioned Focal-compatible 1.8.0 SDK. Zenoh is
statically linked into the visualizer runtime, so the installed package does not
need an SDK or separate Zenoh DSO. No SDK is compiled during robot deployment.
The upstream prebuilt GNU binaries require a newer glibc than Focal.

For local-swarm or another source-image build, this repository provides a
standalone cached SDK stage. Build it once for the target architecture:

```sh
docker build -f .xgc2/Dockerfile.zenoh-sdk --target zenoh-sdk \
  -t xgc2-visualizer-zenoh-sdk:1.8.0-focal .
```

The consuming Dockerfile may use the image as a stage and copy the build-only
prefix into its visualizer build stage:

```dockerfile
FROM xgc2-visualizer-zenoh-sdk:1.8.0-focal AS visualizer-zenoh-sdk
# In the existing Focal visualizer build stage:
COPY --from=visualizer-zenoh-sdk /opt/xgc2/zenoh-sdk/ /opt/xgc2/zenoh-sdk/
# Add /opt/xgc2/zenoh-sdk to the existing CMAKE_PREFIX_PATH before cmake.
```

Alternatively, a Dockerfile with this repository as a named BuildKit context
can copy `.xgc2/scripts/build_zenoh_sdk.sh` and `.xgc2/zenoh-c-1.8.0/` into
an isolated Focal build stage and run the script with `--work-dir` and
`--install-prefix`; its output is the same prefix. Reuse that stage/cache; do
not copy the SDK to the runtime image. The SDK recipe and its source/lock
identity are recorded in [the build manifest](.xgc2/zenoh-c-1.8.0/README.md).

The optional independent interop probe runs the actual onboarding sender against
the receiver on two private ROS masters. In a Noetic environment with Python
Zenoh 1.5.1 and the product sender available:

```sh
python3 test/telemetry_interop_probe.py /path/to/xgc2_ros_visualizer_node \
  /path/to/onboard-telemetry /path/to/python-site-packages
```

It starts and stops its own masters and processes; it does not use a running
station or experiment.

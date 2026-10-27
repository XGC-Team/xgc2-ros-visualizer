# Visualization v1

The service is `xgc2.visualization`, API version `1`, profile `http.v1`. It is a
Run-owned process: whoever starts it owns its lifetime, and it ends with its
Run. The process creates its control socket inside a private runtime directory
(owned by the effective user, mode 0700) that its owner names on the command
line; it never creates that directory.

```sh
xgc2_ros_visualizer_node --socket-path /run/xgc2/sockets/visualizer.sock \
    [--callback-workers 2] [--world-clock wall|simulation] [ROS remaps]
```

| Option | Meaning |
| --- | --- |
| `--socket-path` | Required. Absolute path of the Unix socket. A stale socket of a crashed owner is reclaimed; a live one is never taken over. |
| `--callback-workers` | Input pool size, 1-32, default 2. |
| `--world-clock` | `wall` (default) or `simulation`. Immutable: it selects the ROS data clock before ROS initializes. |
| ROS remaps | `__name`, `__ns`, `__master`, `__ip`, `__hostname`, `/use_sim_time:=...` and ordinary `from:=to` remaps. `__log` is rejected: ROS writes below the supervisor-allocated `ROS_LOG_DIR`. |

The environment must provide the absolute allocations `ROS_HOME` and
`ROS_LOG_DIR` and the ROS master (`ROS_MASTER_URI`, `ROS_IP`). Any other
option fails startup before a socket exists.

## Readiness

`GET /v1/describe`, with or without `wait_ready_ms`, is the unbound discovery
call (it needs no instance header) and returns the readiness envelope:

```json
{"service":"xgc2.visualization","api_version":"1","instance_id":"<32 hex>","ready":true,
 "facts":{"ros_master_uri":"http://127.0.0.1:11311","ros_master_run_id":"<uuid>","world_clock":"simulation",
          "callback_workers":2,"instances":1,"robots":3,"max_instances":64,"max_robots_per_instance":256}}
```

`instance_id` is fresh on every process start. Every request carries one
`X-Request-ID` and one `X-Xrpc-Timeout-Ms`; every non-discovery call
also carries the instance in `X-Xrpc-Instance-ID` and is rejected with 409 when
it differs, so a caller of a previous process can never reach the next one.
A supplied instance header is verified on discovery calls too.

`ready` means the process's socket is bound, its own state is loaded and it is
not stopping. It does not require a ROS master, a Foxglove bridge or any input
topic. `facts.ros_master_state` reports `waiting for master`, `connected` or
`unreachable`; `facts.reason` reports master absence or stopping separately.
`GET /v1/describe?wait_ready_ms=<0..30000>` is part of the
unbound discovery call, with no instance header required. It holds
the request without polling until `ready` is true, the wait elapses or the
call's own deadline is near, and then answers with the current document. At
most 16 calls are held; Stop answers them at once.

The process starts before the master and waits in one background watcher.
PUT loads the latest desired rows while it waits; the control worker activates
them once when the master appears. No ROS connection blocks socket startup or
the publication loop. Healthy master identity checks are two seconds apart;
failed checks use exponential backoff with full jitter (250 ms start, factor 2,
30 s cap), one attempt in flight and a 500 ms request bound. Failure logs report
the first failure and at most one summary per ten seconds, including suppressed
attempts. `facts.ros_master_connect_attempts` counts identity checks.

The process never rebinds to another master. It retains both the master's PID
and its run id (`/run_id`, when published by `roscore`/`roslaunch`); a changed
PID or run id ends the process with status 1, including for a bare `rosmaster`.
An unreachable master is peer status while the socket remains ready; native
configuration writes after binding return 503 while the master is unreachable.
Topics connect only when requested by display configuration, through normal
ROS publisher updates. A late topic or restarted publisher uses the same
subscription; robot status includes `input: {topic, publishers, state}`, where
`state` is `waiting for topic` or `connected` (`not required` for description
only robots). For a remote pose, `publishers` is absent and `connected` means
a source-stamped pose is within the configured freshness window; it is not a
Zenoh discovery or transport-liveness claim.

## Model

* **Instance**: one display scene on the ROS graph, named by the caller (a
  Run id: 1-128 letters, digits, `_`, `.`, `-`). At most 64 per process.
* **Robot**: one member of an instance, named by its ROS namespace without the
  slash (`uav1`). At most 256 per instance. A robot has exactly one **profile**
  and optional **rate overrides**.
* **Profile**: everything needed to draw one robot (below). It is a value; two
  profiles are equal when their resolved documents are.
* **Scene**: what the robots of an instance share: which optional outputs exist
  (`markers`, `transforms`, `scene`), the world boundary and its mode, and the
  display relays.
* **Rate table**: per kind and channel, the default rate of every robot of that
  kind. A robot can override channels of its own kind.

Nothing is persisted here. Saved profiles belong to Core configuration (the
Robot's visualization profile); this service holds the applied state in memory
and reports it honestly: `desiredRevision` is the revision the caller asked
for, `appliedRevision` the one in effect, `persistedRevision` is always `null`.
An accepted change is applied before the receipt is sent, so the two are equal
in every successful receipt; a rejected one changes neither.
Application here means the validated local configuration is loaded; input peers
can still be absent. The roster comes from Core's frozen session rows. After a
service restart the roster is empty and ready, revisions start fresh, and Core
replays the same instance PUT. The first replay creates revision 1 again; an
identical repeat in that process leaves its revision and generation unchanged.

## Routes

| Method and route | Body | Result |
| --- | --- | --- |
| `GET /v1/describe[?wait_ready_ms=]` | Empty | Readiness envelope |
| `GET /v1/status` | Empty | Server readiness, the rate table and the status of every instance |
| `GET /v1/rates` | Empty | Complete per-kind table and its revisions |
| `PUT /v1/rates` | `{expectedRevision, rates}` | Revision CAS and atomic replacement of the complete validated table |
| `GET /v1/instances/<id>` | Empty | Instance status: revisions, robots, counters |
| `PUT /v1/instances/<id>` | Frozen input `{robots, context, settings, displayRelays}` | Create the instance, or make it equal to this desired state: only robots whose profile differs are rebuilt |
| `DELETE /v1/instances/<id>` | Empty | Retract everything the instance published and stop its inputs |
| `GET /v1/instances/<id>/robots/<robot>` | Empty | Resolved profile, revisions, effective rates, counters |
| `PUT /v1/instances/<id>/robots/<robot>` | `{profile, expectedRevision?}` | Add the robot or replace its profile |
| `DELETE /v1/instances/<id>/robots/<robot>` | Empty | Remove one robot; removing an absent robot succeeds |
| `GET /v1/instances/<id>/robots/<robot>/rates` | Empty | Overrides and effective rates |
| `PUT /v1/instances/<id>/robots/<robot>/rates` | `{rates, expectedRevision?}` | Replace the robot's rate overrides |

Errors are `{"ok":false,"error":{"code","message"}}`: 400 `invalid_argument`
(nothing was changed), 404 `not_found`, 405, 409 `conflict` (stale revision,
output conflict, instance limit), 503 `unavailable` (stopping). Writes are never
replayed by the transport: a caller that lost a reply reads the instance or
robot back and compares `appliedRevision` and the resolved profile.

## The instance PUT

The body is the already frozen configuration, exactly `robots` (the public
Robot rows), `context` (the session context), `settings` (the panel entry) and
`displayRelays`. The service projects it into the desired state; Core passes the
facts through and calculates no topic table. Retired envelopes (`instanceId`,
`descriptions`, `worldBoundary`, a relay `robotKind`) are rejected.

A Robot becomes a member when `visualization.descriptionPackage` names an
installed description; a Robot without one is not displayed. `sceneClass`
(`fs150`, `scout`, `mecanum`) gives it a scene presence, an empty `sceneClass`
publishes its description only. Each field of the profile comes from, in
increasing precedence: the kind's defaults, the Robot facts (`visualization`,
`namespace`, localization sources, the run mode and simulator of `context`),
the panel `settings`, the Robot's saved profile `visualization.profile`, and `settings.robotProfiles` overlays keyed by namespace without slash, a
profile fragment with the sections below that is merged over the rest.

The panel `settings` the service reads: `robotProfiles` (strict per-robot profile overlays), `displayRelaySources` (strict declared-source overrides; see README), `markerColor`, `labelScaleInvariant`,
`labelFontSizeMeters`, `labelFontSizePixels`, `markerOpacity`, `uavLabelOffset`,
`scoutLabelOffset`, `mecanumLabelOffset`, `worldBoundaryMode` (`off`, `ground`,
`walls`), `uavHeightProjection`, the history palettes `uavPalette`,
`scoutPalette`, `mecanumPalette`, `historyMaxRateHz` (default 10, range
0.1–100), `historyVisible` (default true), `predictionMaxRateHz` (default 5,
range 0.1–100), and `publication`: `markers`, `transforms`,
`scene` (the scene's optional outputs), `scenePaths`, `paths` (the default of
each robot) and `groundScene` (`false` keeps Scout and Mecanum robots as
descriptions without a scene presence). `historyMaxRateHz` and `historyVisible`
limit path, AR path, scene-path, and the history `LINE_STRIP` ADD on the
markers topic. The line uses the path ceiling and does not repeat at the
markers channel rate. Meshes, labels, and TF keep their own rates. Hiding
history deletes that line once. The source history ring is unchanged. Pose-axis fields are viewer settings and are ignored.

`displayRelays` has at most 128 records. Required fields are `source`, `topic`,
and `messageType`. Optional `maxRateHz` and `role` may be null. Optional `zenoh` specifies a remote source as described in the README. `role` is
`prediction`, `obstacle`, or `semantic`. Any other field, including `robotKind`,
is rejected. PointCloud2, OccupancyGrid, Path, and PoseArray are copied byte
for byte. Marker and MarkerArray keep namespace/id add, delete, and DELETEALL
state. The publish rate is the minimum of the display channel, a set
`maxRateHz`, and `predictionMaxRateHz` when the role is `prediction`. A higher
declaration does not raise the channel rate. That rate is a message rate: one
MarkerArray batch or one singular Marker per period. Unsent Marker ids stay
queued and are drained in `(ns, id)` order, wrapping, so one busy id cannot
starve the others. A positive Marker lifetime starts when the relay ingests
the ADD and resets on every later ADD of that id, including one that repeats
the same `header.stamp`. `header.stamp` is forwarded unchanged. A delayed send
advertises only the time still left. Pending marker payload is capped at the
1 MiB `maxMessageBytes` display budget. Overflow of one namespace deletes that
namespace only.

The receipt is the instance status plus `created`, `unchanged` and `changes`:

```json
{"ok":true,"id":"run1","created":false,"unchanged":false,
 "changes":{"added":["uav5"],"rebuilt":["uav2"],"removed":[],"unchanged":["uav1","uav3"],
            "scene":false,"relaysAdded":0,"relaysRemoved":0},
 "configuration":{"desiredRevision":3,"appliedRevision":3,"persistedRevision":null,
                  "state":"applied","appliedAtSteadyNs":1234},
 "robots":[{"id":"uav1","kind":"fs150","scene":true,"generation":1,
            "configuration":{"desiredRevision":1,"appliedRevision":1,"persistedRevision":null}}]}
```

An identical PUT is a no-op: `unchanged: true`, no revision changes. The
instance revision is 1 at creation and advances by one for every applied change
of membership, profiles, scene or relays. A robot's revision is 1 when it is
added and advances for every change of its profile or of its rate overrides;
`generation` counts how often its resources were built, so it advances only
with the profile.

The instance PUT states the whole desired configuration. It therefore
replaces online adjustments made through the robot routes, and returns
profiles the caller changed to what the frozen input says. Rate overrides
belong to the robot id: they survive a rebuild and end with the robot.

## The robot profile

`PUT .../robots/<robot>` takes `{profile, expectedRevision?}`; `GET` returns the
resolved profile in the same schema, so a client reads it, changes it and puts
it back. `expectedRevision` is the robot's `desiredRevision` (0 expects that the
robot does not exist yet); a stale one fails with 409, an absent one makes the
write unconditional. The document is strict: unknown sections or fields, fields that do not
apply to the robot's kind and values out of range fail with 400 and change
nothing. Omitted fields take the kind's defaults, so a profile is the same
value whether it is written out or abbreviated. Required are `model.kind` and
`model.description.package` and `.file`.

| Section.field | Meaning | Default |
| --- | --- | --- |
| `model.kind` | `fs150`, `scout`, `mecanum` (renderer class and row of the rate table) or `global` (description only) | required |
| `model.scene` | Scene identity of the robot's entities and frames; empty for no scene presence | robot id (`global`: empty) |
| `model.meshScale` | Scale of the rendered meshes | 1 (`mecanum` 0.001) |
| `model.heightProjectionColor` | FS150 only. Lowercase `#rrggbb` of the height projection; empty disables it | empty |
| `model.description` | Installed URDF `{package, file, statePublisher, jointStateTopic}`. `statePublisher` makes the service publish its joint TF; a scene robot cannot also be a state publisher | `jointStateTopic` `joint_states` |
| `state.poseTopic` | The one pose that drives the scene robot | `/<ns>/mavros/local_position/pose` (FS150), `/<ns>/pose` (ground) |
| `state.arPoseTopic` | FS150 only. Mocap pose of the camera-pane identity and AR path; empty for none | empty |
| `state.worldOffset` | FS150 only. `[x,y,z]` added once to the AR pose | `[0,0,0]` |
| `state.sources` | Per-input exact Zenoh `{listen,key}` selections; unlisted inputs remain local ROS. See README for types and endpoint sharing. | empty |
| `frames.world` | The fixed frame (`world`) | `world` |
| `frames.labelOffset` | Height of the label anchor above the robot, -10..10 m | 0.55 (`scout` 0.65, `mecanum` 0.32) |
| `path.topic`, `path.arTopic` | Path topics below the robot namespace; `arTopic` exists with `arPoseTopic` only | `path`, `ar_path` |
| `labels` | `color` (lowercase `#rrggbb`), `scaleInvariant`, `fontSize` (meters, or pixels when scale invariant), `opacity` 0..1 | `#00a2ff`, `false`, 0.24 / 16, 1 |
| `animation` | Freshness windows and animation inputs of the kind: FS150 `poseTimeout`, `stateTimeout`, `rotorGround`, `rotorTransition`, `rotorAirborne`; Scout `poseTimeout`, `motionTimeout`, `wheelRadius`, `trackWidth`, `wheelDeadband`, `wheelMaxSpeed`; Mecanum the same with `wheelbasePlusTrack` for `trackWidth`; none for `global` | 0.5 s, 2 s, 25 / 90 / 60 rad/s, 0.08 m, 0.416 m, 0.02, 35 rad/s |
| `publication` | What the robot contributes: `markers`, `transforms`, `scene`, `scenePaths`, `paths` | `true`, `true`, `true`, `false`, `true` |

`publication` can only narrow what the scene publishes: the shared outputs
exist when the scene's `publication` switches them on (`markers` off by
default), and a robot decides whether it contributes to them. `paths` creates
the robot's own Path topics; `scenePaths` adds its history to the scene layer.

### What an apply does

Applying a profile that differs from the applied one rebuilds that robot and
nothing else. The old robot unsubscribes its inputs, deletes the scene
entities and markers it showed by identity, publishes an empty Path on and
withdraws its Path topics, and removes the URDF parameters it set (a parameter
that someone else replaced is left alone). The new robot loads its URDF and
renderer first, so a profile that cannot be built changes nothing, and then
creates its subscriptions, outputs and parameters. The resources of every other
robot, their subscriptions, publishers and rate phase, are untouched. A
resource failure after that point restores the previous state and fails the
call.

| Change | Effect |
| --- | --- |
| A robot's profile (any field) | That robot is rebuilt |
| Robot added or removed | Only that robot's resources are created or retracted |
| Rate table or a robot's rate overrides | Effective at the next scheduler tick, no rebuild |
| Scene switches (`markers`, `transforms`, `scene`) | The shared outputs are retracted and advertised again as the scene now calls for them; robot inputs are untouched |
| World boundary or its mode | The boundary layers are sent again |
| A display relay | That relay is replaced; the others keep running |

A robot's saved profile is the same as a profile written by hand: either way a
panel setting reaches exactly the robots whose resolved profile it changes (a
`scoutLabelOffset` rebuilds the Scouts only).

Output names are claimed per instance (`/xgc/tf`, `/xgc/scene` and the other
shared topics, `/markers`, `/tf_static`, each robot's Path topics and URDF
parameters, each relay's display topic). An apply that would take a name
another instance holds fails with 409 before anything changes. Instances can
share a process only when they publish disjoint outputs; an Experiment normally
has one.

## Rates

Rates are finite 0.1-1000 Hz values. The table has the four kind rows (`fs150`,
`scout`, `mecanum`, `global`) with their applicable channels (see the README).
`PUT /v1/rates` replaces the complete table under a positive integer
`expectedRevision`; an incomplete, invalid or stale candidate changes nothing.

A robot override replaces channels of the robot's own kind row for this robot
only: `PUT .../rates` with `{"rates":{"path":2}}` makes the path of this robot
2 Hz while every other robot follows the table. The body replaces the previous
overrides (`{}` clears them). Display-relay channels and the channels of the
scene (`tf_root`, `tf_static`, `world_boundary`, `readiness`) are not
overridable; a `global` robot can override `joint_tf` only. `tf_root` remains in
the rate table for compatibility but publishes no dynamic transform: the fixed
`world → xgc_origin` is owned only by latched `/tf_static`. `expectedRevision`
is the robot's `desiredRevision`; without it the write is unconditional.

The effective rate of a channel is the override if the robot has one, else the
table. A publication tick already running finishes under the rates it started
with; the next tick uses the replacement, and the scheduler wakes at once even
when every rate is 0.1 Hz.

## Completion

The receipt of a PUT is sent after the change is applied to the native
objects: the URDF is loaded, inputs are subscribed, outputs are advertised.
That does not prove a fresh pose, a viewer frame, a completed camera operation
or scientific progress; those success conditions stay with their consumers.

A caller that times out has an unknown outcome for that write and reads the
state back. Cancelling a call never rolls anything back.

## Lifecycle and resources

* **Stop** (`SIGTERM`/`SIGINT`) stops admission, answers held describe calls,
  finishes the domain worker, stops the scheduler and input pool, retracts the
  outputs of every instance by identity (a latched topic is never cleared as a
  whole), shuts ROS down and only then releases the socket. A slow native
  operation can outlive a caller deadline; releasing the lease never asserts
  that a cancelled operation ended.
* **Restart or crash** restores nothing: instances, profiles, overrides and the
  rate table are ephemeral, and reactivation is a new explicit PUT. The socket
  of a killed process is reclaimed by its successor (finite reachability proof,
  unchanged inode); a live endpoint or a non-socket path is never taken.
* **Threads**: one XRPC host (SDK default limits: 32 connections and calls,
  16 KiB headers, 1 MiB bodies), one domain worker with a bounded handoff, the
  fixed input pool, one publication scheduler and one master watcher. Robots
  create no threads and no helper processes.
* **Diagnostics**: warnings of the transport (rejected, expired or failed calls)
  go to stderr as bounded redacted records; the supervisor owns the log.

| Write class | Writer and location | Trigger and recovery |
| --- | --- | --- |
| Instance, profile and rate state | Process memory | Explicit calls; discarded at exit |
| Unix socket and lease | XRPC, in the owner's private runtime directory | Bind and stop; the lock persists, only the owned socket inode is removed |
| Robot descriptions and assets | Installed packages, read only | URDF loading; never rewritten |
| ROS parameters | This process, on the selected master | Robot rebuild/removal; only an unchanged owned value is deleted |
| ROS cache and logs | ROS libraries below `ROS_HOME`/`ROS_LOG_DIR` | Supervisor owns quota and rotation |

No database, durable save, browser store or filesystem fallback is part of
this service.

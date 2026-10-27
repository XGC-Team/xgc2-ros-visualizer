#pragma once

#include <array>
#include <cstddef>
#include <set>
#include <string>
#include <vector>

#include <foxglove_msgs/Color.h>
#include <foxglove_msgs/SceneEntityDeletion.h>
#include <foxglove_msgs/SceneUpdate.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace xgc2_ros_visualizer {

// Each concrete vehicle family owns a rendering path. Callers must provide the
// concrete lists so a future airframe cannot silently inherit the FS150 mesh.
enum class RobotModelKind { kNone, kFs150, kScout, kMecanum };

// SceneEntity updates replace the complete entity with the same ID. Keep the
// high-rate robot geometry and the lower-rate path in separate entities so a
// pose update cannot erase a path which was intentionally not retransmitted.
//
// The label is separate for a different reason. A trail is drawn where the
// message said it was; the label is anchored to a frame and drawn wherever that
// frame currently is, and one entity names exactly one frame.
//
// Robot geometry is not here at all. Every mesh this scene used to ship is a
// link of the robot's own URDF, which the viewer loads once from a parameter
// and places from transforms -- so shipping it again, in world coordinates, at
// the scene cadence, drew each vehicle twice: once smoothly from the
// description and once two-per-second on top of it.
enum class SceneEntityPart { kPath, kLabel, kArLabel };

struct SceneUpdateCadenceDecision {
    bool publish_label{false};
    bool publish_path{false};
};

struct SceneLabelStyle {
    double font_size{0.0};
    bool scale_invariant{false};
    foxglove_msgs::Color color;
};

constexpr const char* kUavHeightProjectionTopic = "/xgc/uav_height_projection";
constexpr const char* kUavHeightProjectionArTopic = "/xgc/uav_height_projection_ar";
constexpr const char* kIdentityArTopic = "/xgc/scene_ar";
constexpr const char* kWorldBoundaryTopic = "/xgc/world_boundary";
constexpr const char* kWorldBoundaryArTopic = "/xgc/world_boundary_ar";
constexpr const char* kWorldBoundaryEntityId = "world_boundary";
constexpr const char* kWorldBoundaryWallsTopic = "/xgc/world_boundary_walls";
constexpr const char* kWorldBoundaryWallsArTopic = "/xgc/world_boundary_walls_ar";
constexpr const char* kWorldBoundaryWallsEntityId = "world_boundary_walls";

enum class HeightProjectionView { kLocalPosition, kVrpn };

foxglove_msgs::Color sceneColorFromHex(const std::string& color);

// Geometry is authored in the fixed world frame; vehicle attitude never enters it.
// 3D and AR reuse this constructor; each view supplies its own already-final pose.
foxglove_msgs::SceneEntity uavHeightProjectionEntity(
    const std::string& scene_model, const geometry_msgs::Point& position,
    const ros::Time& stamp, const std::string& frame_id, const foxglove_msgs::Color& color);

foxglove_msgs::SceneEntityDeletion uavHeightProjectionDeletion(const std::string& scene_model,
                                                               const ros::Time& stamp);

struct WorldBoundaryDisplay {
    bool displayable{false};
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};
    double z_min{0.0};
    double z_max{0.0};
    double ground_z{0.0};
};

// Empty env is unconfigured. Canonical JSON with XY bounds and finite groundZ
// is displayable. groundZ is never taken from zMin.
WorldBoundaryDisplay parseWorldBoundaryDisplay(const std::string& json);

foxglove_msgs::SceneEntity worldBoundaryEntity(const WorldBoundaryDisplay& boundary, const ros::Time& stamp,
                                               const std::string& frame_id);
foxglove_msgs::SceneEntityDeletion worldBoundaryDeletion(const ros::Time& stamp);
foxglove_msgs::SceneUpdate worldBoundarySceneUpdate(const WorldBoundaryDisplay& boundary, const ros::Time& stamp,
                                                    const std::string& frame_id);

foxglove_msgs::SceneEntity worldWallsEntity(const WorldBoundaryDisplay& boundary, const ros::Time& stamp,
                                            const std::string& frame_id);
foxglove_msgs::SceneEntityDeletion worldWallsDeletion(const ros::Time& stamp);
foxglove_msgs::SceneUpdate worldWallsSceneUpdate(const WorldBoundaryDisplay& boundary, const ros::Time& stamp,
                                                 const std::string& frame_id);

// One display mode owns both fence layers. The layer that is off is a deletion,
// so a viewer cannot turn it back on from a latched geometry the publisher
// kept sending.
enum class WorldBoundaryDisplayMode { kOff, kGround, kWalls };

WorldBoundaryDisplayMode worldBoundaryDisplayModeFromString(const std::string& mode);

struct WorldBoundaryLayerMessages {
    foxglove_msgs::SceneUpdate ground;
    foxglove_msgs::SceneUpdate walls;
};

WorldBoundaryLayerMessages worldBoundaryLayerMessages(const WorldBoundaryDisplay& boundary,
                                                      WorldBoundaryDisplayMode mode, const ros::Time& stamp,
                                                      const std::string& frame_id);

// Experiment world offset is applied once to a VRPN sample. Callers must not
// add it again in geometry construction.
geometry_msgs::Point applyExperimentWorldOffsetOnce(geometry_msgs::Point position,
                                                    const std::array<double, 3>& offset);
geometry_msgs::Pose applyExperimentWorldOffsetOnce(geometry_msgs::Pose pose,
                                                    const std::array<double, 3>& offset);

SceneLabelStyle sceneLabelStyleFromMarkerColor(const std::string& marker_color,
                                               bool scale_invariant = false,
                                               double font_size = 0.24, double opacity = 1.0);

struct SceneLabelOffsets {
    double uav{0.55};
    double scout{0.65};
    double mecanum{0.32};
};

void validateSceneLabelOffsets(const SceneLabelOffsets& offsets);

// PublishCadence is one rate gate. It exists so a publisher can give each kind
// of fact its own cadence -- a pose, a path trail and a rotor animation are not
// equally urgent -- without every caller reimplementing drift-free gating.
class PublishCadence {
  public:
    explicit PublishCadence(double publish_rate);

    bool take(const ros::Time& now);

  private:
    double publish_rate_;
    bool initialized_{false};
    ros::Time last_stamp_;
};

class SceneUpdateCadence {
  public:
    SceneUpdateCadence(double label_publish_rate, double path_publish_rate);

    SceneUpdateCadenceDecision take(const ros::Time& now);

  private:
    bool takeGate(const ros::Time& now, double rate, bool* initialized, ros::Time* last_stamp);

    double label_publish_rate_;
    double path_publish_rate_;
    bool label_initialized_{false};
    bool path_initialized_{false};
    ros::Time last_label_stamp_;
    ros::Time last_path_stamp_;
};

std::set<std::string> parseModelNames(const std::string& csv);

bool modelListsAreDisjoint(const std::set<std::string>& fs150_models, const std::set<std::string>& scout_models,
                           const std::set<std::string>& mecanum_models);

RobotModelKind selectRobotModelKind(const std::string& model_name, const std::set<std::string>& configured_fs150_models,
                                    const std::set<std::string>& configured_scout_models,
                                    const std::set<std::string>& configured_mecanum_models, bool track_ugv);

std::string sceneEntityID(RobotModelKind kind, const std::string& model_name);

std::string sceneEntityPartID(RobotModelKind kind, const std::string& model_name, SceneEntityPart part);

// Experiment-slot viewer pose. PX4 deliberately follows MAVROS' fused local
// estimate so a bad vision/EKF alignment is visible before takeoff; ground
// robots follow the offset-corrected canonical slot pose.
std::string slotVisualizationPoseTopic(RobotModelKind kind, const std::string& ros_namespace);

// History `/<slot>/path` sample for the viewer. Scout/Mecanum force world z to
// 0 so mocap marker height does not float the trail; FS150 keeps fused z.
// Body TF is pinned the same way; canonical /pose is unchanged.
geometry_msgs::Pose slotHistoryPathPose(RobotModelKind kind, geometry_msgs::Pose world_pose);

// Convert one corrected canonical Robot pose into the only high-rate viewer
// facts: the body transform and its upright overhead label anchor. Meshes and
// label text stay static; rotor/wheel joints use their own bounded cadence.
// Scout / Mecanum body and label XY follow /pose; world Z is pinned to 0 so
// the mesh sits on the ground. UAV keeps /pose.z. Canonical /pose is unchanged.
std::vector<geometry_msgs::TransformStamped>
canonicalRobotPoseTransforms(RobotModelKind kind, const std::string& scene_model,
                             const geometry_msgs::Pose& pose, const ros::Time& stamp,
                             const std::string& frame_id, const SceneLabelOffsets& offsets = {});

// Upright image-pane label anchor. Pose must already be the AR identity sample
// (offset VRPN for FS150, canonical `/pose` for ground robots). Does not
// publish a second body; physical AR does not overlay URDF.
geometry_msgs::TransformStamped canonicalArIdentityLabelTransform(
    RobotModelKind kind, const std::string& scene_model, const geometry_msgs::Pose& pose,
    const ros::Time& stamp, const std::string& frame_id, const SceneLabelOffsets& offsets = {});

visualization_msgs::Marker identityLabelMarker(const std::string& scene_model, const std::string& label_frame,
                                                 const ros::Time& stamp);

// Replace only the operator-facing text generated for one Robot. The concrete
// robot kind owns the class word (FS150 -> UAV, Scout/Mecanum -> UGV), while
// the canonical lowercase /uavN or /ugvN namespace owns N. This also keeps a
// mixed-scene Scout in slot /uav7 visibly identified as UGV 7 without changing
// its ROS interface or its scene-model marker/frame identity.
void applyRobotMarkerLabel(visualization_msgs::MarkerArray* markers, std::size_t first_marker, RobotModelKind kind,
                           const std::string& ros_namespace);

void appendSceneEntity(RobotModelKind kind, const std::string& model_name,
                       const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                       const ros::Time& timestamp, const std::string& frame_id, const SceneLabelStyle& label_style,
                       foxglove_msgs::SceneUpdate* update);

void appendSceneEntityPart(RobotModelKind kind, const std::string& model_name, SceneEntityPart part,
                           const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                           const ros::Time& timestamp, const std::string& frame_id, const SceneLabelStyle& label_style,
                           foxglove_msgs::SceneUpdate* update);

// Body/path poses come only from the one viewer pose selected for the slot.
// Ground robots arrive in the product Fixed Frame (world ENU z-up). FS150 is
// deliberately different: its selected viewer pose is MAVROS' fused local
// estimate, whose ROS frame label is `map`; its numeric ENU coordinates are
// interpreted in the Experiment world so a bad FCU/vision alignment is visible
// before takeoff. There is no source priority or fallback.
bool isWorldFixedFrame(const std::string& frame_id);

struct CanonicalPoseSample {
    bool available{false};
    geometry_msgs::Pose pose;
    ros::Time stamp;
    std::string frame_id;
};

struct CanonicalWorldPose {
    bool found{false};
    geometry_msgs::Pose pose;
    ros::Time stamp;
    std::string frame_id;
};

// Accept the kind-specific single viewer pose, or nothing. timeout_sec is a
// freshness window on that one sample; it does not select a substitute source.
// FS150 accepts only MAVROS `map` (or world for a direct contract test/source);
// ground vehicles continue to require world.
CanonicalWorldPose selectSlotVisualizationWorldPose(RobotModelKind kind, const CanonicalPoseSample& pose,
                                                     const ros::Time& now, double timeout_sec);

// 3D uses fused local_position; AR uses the already-offset VRPN sample. Neither
// view substitutes the other source when that sample is missing or stale.
CanonicalWorldPose selectUavHeightProjectionWorldPose(HeightProjectionView view,
                                                    const CanonicalPoseSample& local_position,
                                                    const CanonicalPoseSample& vrpn_already_offset,
                                                    const ros::Time& now, double timeout_sec);

// Image-pane identity: FS150 uses already-offset VRPN; Scout/Mecanum use the
// same canonical `/pose` as 3D. Missing/stale samples are skipped, never
// replaced with fused local_position.
CanonicalWorldPose selectArIdentityWorldPose(RobotModelKind kind, const CanonicalPoseSample& canonical,
                                               const CanonicalPoseSample& vrpn_already_offset,
                                               const ros::Time& now, double timeout_sec);

// Identity child of the Fixed Frame. Latched on /tf_static so Lichtblick 3D can
// create `world` without plant /tf; RViz also consumes /tf_static. Displays
// consume the parent, not the child.
constexpr const char* kWorldFixedFrameRootChild = "xgc_origin";
geometry_msgs::TransformStamped worldFixedFrameRoot(const std::string& frame_id, const ros::Time& stamp);

// Algorithm overlays (predicted Path, leader Path) keep the message frame_id
// they declare, commonly `map`. The viewer Fixed Frame is `world`. This identity
// is product-owned on /tf_static so Lichtblick can pose those overlays without
// consuming plant /tf. It is not a robot body transform.
constexpr const char* kAlgorithmOverlayFrame = "map";
geometry_msgs::TransformStamped algorithmOverlayFrameAlias(const std::string& world_frame, const ros::Time& stamp);

// Runtime readiness is "the frozen roster is configured and this node can
// publish". Physical runs often have no VRPN, no camera, and only a subset of
// robots on the field; missing sibling poses must not block Lichtblick or fail
// the Experiment. Surplus poses (more than the frozen roster) remain invalid.
bool frozenVisualizationRosterReady(std::size_t tracked_models, std::size_t world_poses);

} // namespace xgc2_ros_visualizer

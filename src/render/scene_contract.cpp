#include "xgc2_ros_visualizer/scene_contract.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <foxglove_msgs/Color.h>
#include <foxglove_msgs/CubePrimitive.h>
#include <foxglove_msgs/LinePrimitive.h>
#include <foxglove_msgs/ModelPrimitive.h>
#include <foxglove_msgs/SceneEntity.h>
#include <foxglove_msgs/SceneEntityDeletion.h>
#include <foxglove_msgs/TextPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include "render/robots/path_history.hpp"
#include "render/robots/robot_frames.hpp"

namespace xgc2_ros_visualizer {
namespace {

std::string trim(const std::string& value) {
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return std::string();
    }
    return value.substr(first, value.find_last_not_of(whitespace) - first + 1);
}

geometry_msgs::Quaternion normalizedQuaternion(const geometry_msgs::Quaternion& source) {
    const double norm =
        std::sqrt(source.x * source.x + source.y * source.y + source.z * source.z + source.w * source.w);
    geometry_msgs::Quaternion result;
    if (!std::isfinite(norm) || norm < 1.0e-9) {
        result.w = 1.0;
        return result;
    }
    result.x = source.x / norm;
    result.y = source.y / norm;
    result.z = source.z / norm;
    result.w = source.w / norm;
    return result;
}

geometry_msgs::Pose copyPose(const geometry_msgs::Pose& source) {
    geometry_msgs::Pose result;
    result.position = source.position;
    result.orientation = normalizedQuaternion(source.orientation);
    return result;
}

// Scout display Z is Gazebo sitting height (ground_truth chassis z), not
// mocap marker height and not a hard chassis origin at 0. History Path
// still flattens separately.
double slotDisplayBodyZ(RobotModelKind kind, double pose_z) {
    if (kind == RobotModelKind::kScout) {
        return xgc2_ros_visualizer::scoutDisplayBodyZ();
    }
    if (kind == RobotModelKind::kMecanum) {
        return xgc2_ros_visualizer::mecanumDisplayBodyZ();
    }
    return pose_z;
}

foxglove_msgs::Color copyColor(const std_msgs::ColorRGBA& source) {
    foxglove_msgs::Color result;
    result.r = source.r;
    result.g = source.g;
    result.b = source.b;
    result.a = source.a;
    return result;
}

int hexDigit(char value) {
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    if (value >= 'a' && value <= 'f') {
        return value - 'a' + 10;
    }
    return -1;
}

double hexChannel(const std::string& value, std::size_t offset) {
    const int high = hexDigit(value[offset]);
    const int low = hexDigit(value[offset + 1U]);
    if (high < 0 || low < 0) {
        throw std::invalid_argument("marker color must use canonical lowercase #rrggbb syntax");
    }
    return static_cast<double>(high * 16 + low) / 255.0;
}

std::string normalizedFrameLabel(const std::string& frame_id) {
    std::string normalized;
    normalized.reserve(frame_id.size());
    for (unsigned char character : frame_id) {
        if (character == ' ' || character == '\t' || character == '\r' || character == '\n') {
            continue;
        }
        normalized.push_back(static_cast<char>(std::tolower(character)));
    }
    if (!normalized.empty() && normalized.front() == '/') {
        normalized.erase(normalized.begin());
    }
    const std::size_t slash = normalized.rfind('/');
    if (slash != std::string::npos && slash + 1 < normalized.size()) {
        normalized = normalized.substr(slash + 1);
    }
    return normalized;
}

bool isFreshCanonicalSample(const CanonicalPoseSample& source, const ros::Time& now, double timeout_sec) {
    if (!source.available || source.stamp.isZero()) {
        return false;
    }
    if (timeout_sec <= 0.0) {
        return true;
    }
    return now - source.stamp <= ros::Duration(timeout_sec);
}

bool canonicalROSIdentifier(const std::string& value) {
    static const std::regex pattern("^[A-Za-z_][A-Za-z0-9_]*$");
    return !value.empty() && value.size() <= 127 && std::regex_match(value, pattern);
}

} // namespace

foxglove_msgs::Color sceneColorFromHex(const std::string& marker_color) {
    if (marker_color.size() != 7U || marker_color.front() != '#') {
        throw std::invalid_argument("marker color must use canonical lowercase #rrggbb syntax");
    }
    foxglove_msgs::Color color;
    color.r = hexChannel(marker_color, 1U);
    color.g = hexChannel(marker_color, 3U);
    color.b = hexChannel(marker_color, 5U);
    color.a = 1.0;
    return color;
}

SceneLabelStyle sceneLabelStyleFromMarkerColor(const std::string& marker_color,
                                               bool scale_invariant, double font_size, double opacity) {
    const double minimum = scale_invariant ? 1.0 : 0.01;
    const double maximum = scale_invariant ? 256.0 : 10.0;
    if (!std::isfinite(font_size) || font_size < minimum || font_size > maximum ||
        !std::isfinite(opacity) || opacity < 0.0 || opacity > 1.0) {
        throw std::invalid_argument("invalid robot label font size or opacity");
    }
    SceneLabelStyle style;
    style.font_size = font_size;
    style.scale_invariant = scale_invariant;
    style.color = sceneColorFromHex(marker_color);
    style.color.a = opacity;
    return style;
}

foxglove_msgs::SceneEntity uavHeightProjectionEntity(
    const std::string& scene_model, const geometry_msgs::Point& position,
    const ros::Time& stamp, const std::string& frame_id, const foxglove_msgs::Color& color) {
    if (!canonicalROSIdentifier(scene_model) || !isWorldFixedFrame(frame_id) || stamp.isZero() ||
        !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
        throw std::invalid_argument("UAV height projection requires a finite world position and identity");
    }
    foxglove_msgs::SceneEntity entity;
    entity.id = scene_model + "/height_projection";
    entity.frame_id = frame_id;
    entity.timestamp = stamp;

    foxglove_msgs::LinePrimitive vertical;
    vertical.type = foxglove_msgs::LinePrimitive::LINE_LIST;
    vertical.pose.orientation.w = 1.0;
    vertical.thickness = 0.02;
    vertical.scale_invariant = false;
    vertical.color = color;
    vertical.points.push_back(position);
    geometry_msgs::Point ground = position;
    ground.z = 0.0;
    vertical.points.push_back(ground);
    entity.lines.push_back(std::move(vertical));

    foxglove_msgs::TriangleListPrimitive ring;
    ring.pose.orientation.w = 1.0;
    ring.color = color;
    constexpr std::size_t segments = 48;
    constexpr double outer_radius = 0.18;
    constexpr double inner_radius = 0.13;
    const double two_pi = 2.0 * std::acos(-1.0);
    ring.points.reserve(segments * 2);
    ring.indices.reserve(segments * 6);
    for (std::size_t i = 0; i < segments; ++i) {
        const double angle = two_pi * static_cast<double>(i) / segments;
        for (const double radius : {outer_radius, inner_radius}) {
            geometry_msgs::Point point = ground;
            point.x += radius * std::cos(angle);
            point.y += radius * std::sin(angle);
            ring.points.push_back(point);
        }
        const auto outer = static_cast<std::uint32_t>(2 * i);
        const auto next_outer = static_cast<std::uint32_t>(2 * ((i + 1) % segments));
        ring.indices.insert(ring.indices.end(), {outer, next_outer, outer + 1,
                                               outer + 1, next_outer, next_outer + 1});
    }
    entity.triangles.push_back(std::move(ring));
    return entity;
}

foxglove_msgs::SceneEntityDeletion uavHeightProjectionDeletion(const std::string& scene_model,
                                                               const ros::Time& stamp) {
    if (!canonicalROSIdentifier(scene_model) || stamp.isZero()) {
        throw std::invalid_argument("UAV height projection deletion requires identity and timestamp");
    }
    foxglove_msgs::SceneEntityDeletion deletion;
    deletion.timestamp = stamp;
    deletion.type = foxglove_msgs::SceneEntityDeletion::MATCHING_ID;
    deletion.id = scene_model + "/height_projection";
    return deletion;
}

namespace {

bool jsonNullMember(const std::string& text, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*null");
    return std::regex_search(text, pattern);
}

bool jsonNumberMember(const std::string& text, const std::string& key, double* value) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*([-+0-9.eE]+)");
    std::smatch match;
    if (!std::regex_search(text, match, pattern) || value == nullptr) {
        return false;
    }
    char* end = nullptr;
    const double parsed = std::strtod(match[1].str().c_str(), &end);
    if (end == nullptr || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

std::string jsonObjectMember(const std::string& text, const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    const auto key_pos = text.find(needle);
    if (key_pos == std::string::npos) {
        return std::string();
    }
    const auto colon = text.find(':', key_pos + needle.size());
    if (colon == std::string::npos) {
        throw std::invalid_argument("XGC2_WORLD_BOUNDARY is not canonical worldBoundary JSON");
    }
    std::size_t cursor = colon + 1;
    while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor])) != 0) {
        ++cursor;
    }
    if (cursor >= text.size()) {
        throw std::invalid_argument("XGC2_WORLD_BOUNDARY is not canonical worldBoundary JSON");
    }
    if (text.compare(cursor, 4, "null") == 0) {
        return "null";
    }
    if (text[cursor] != '{') {
        throw std::invalid_argument("XGC2_WORLD_BOUNDARY controlBounds must be an object or null");
    }
    int depth = 0;
    for (std::size_t end = cursor; end < text.size(); ++end) {
        if (text[end] == '{') {
            ++depth;
        } else if (text[end] == '}') {
            --depth;
            if (depth == 0) {
                return text.substr(cursor, end - cursor + 1);
            }
        }
    }
    throw std::invalid_argument("XGC2_WORLD_BOUNDARY controlBounds is truncated");
}

}  // namespace

WorldBoundaryDisplay parseWorldBoundaryDisplay(const std::string& json) {
    WorldBoundaryDisplay result;
    const std::string text = trim(json);
    if (text.empty()) {
        return result;
    }
    if (text.front() != '{' || text.back() != '}') {
        throw std::invalid_argument("XGC2_WORLD_BOUNDARY must be canonical Experiment worldBoundary JSON");
    }
    double schema_version = 0.0;
    if (jsonNumberMember(text, "schemaVersion", &schema_version) && schema_version != 1.0) {
        throw std::invalid_argument("XGC2_WORLD_BOUNDARY schemaVersion must be 1");
    }
    const std::string bounds_json = jsonObjectMember(text, "controlBounds");
    bool has_bounds = false;
    if (!bounds_json.empty() && bounds_json != "null") {
        if (!jsonNumberMember(bounds_json, "xMin", &result.x_min) || !jsonNumberMember(bounds_json, "xMax", &result.x_max) ||
            !jsonNumberMember(bounds_json, "yMin", &result.y_min) || !jsonNumberMember(bounds_json, "yMax", &result.y_max)) {
            throw std::invalid_argument("XGC2_WORLD_BOUNDARY controlBounds requires finite XY min/max");
        }
        double z_min = 0.0;
        double z_max = 0.0;
        if (!jsonNumberMember(bounds_json, "zMin", &z_min) || !jsonNumberMember(bounds_json, "zMax", &z_max) ||
            !(result.x_min < result.x_max) || !(result.y_min < result.y_max) || !(z_min < z_max)) {
            throw std::invalid_argument("XGC2_WORLD_BOUNDARY controlBounds requires min < max on each axis");
        }
        result.z_min = z_min;
        result.z_max = z_max;
        has_bounds = true;
    }
    double ground_z = 0.0;
    const bool has_ground = jsonNumberMember(text, "groundZ", &ground_z);
    if (!has_ground && !jsonNullMember(text, "groundZ") && text.find("\"groundZ\"") != std::string::npos) {
        throw std::invalid_argument("XGC2_WORLD_BOUNDARY groundZ must be a finite metre value or null");
    }
    result.ground_z = ground_z;
    result.displayable = has_bounds && has_ground;
    return result;
}

foxglove_msgs::SceneEntity worldBoundaryEntity(const WorldBoundaryDisplay& boundary, const ros::Time& stamp,
                                               const std::string& frame_id) {
    if (!boundary.displayable || stamp.isZero() || !isWorldFixedFrame(frame_id)) {
        throw std::invalid_argument("world fence display requires configured XY, groundZ, world frame, and timestamp");
    }
    foxglove_msgs::SceneEntity entity;
    entity.id = kWorldBoundaryEntityId;
    entity.frame_id = frame_id;
    entity.timestamp = stamp;

    foxglove_msgs::LinePrimitive loop;
    loop.type = foxglove_msgs::LinePrimitive::LINE_LOOP;
    loop.pose.orientation.w = 1.0;
    loop.thickness = 0.06;
    loop.scale_invariant = false;
    loop.color.r = 1.0;
    loop.color.g = 0.2;
    loop.color.b = 0.15;
    loop.color.a = 1.0;
    const double corners[4][2] = {
        {boundary.x_min, boundary.y_min},
        {boundary.x_max, boundary.y_min},
        {boundary.x_max, boundary.y_max},
        {boundary.x_min, boundary.y_max},
    };
    for (const auto& corner : corners) {
        geometry_msgs::Point point;
        point.x = corner[0];
        point.y = corner[1];
        point.z = boundary.ground_z;
        loop.points.push_back(point);
    }
    entity.lines.push_back(std::move(loop));
    return entity;
}

foxglove_msgs::SceneEntityDeletion worldBoundaryDeletion(const ros::Time& stamp) {
    if (stamp.isZero()) {
        throw std::invalid_argument("world fence deletion requires a timestamp");
    }
    foxglove_msgs::SceneEntityDeletion deletion;
    deletion.timestamp = stamp;
    deletion.type = foxglove_msgs::SceneEntityDeletion::MATCHING_ID;
    deletion.id = kWorldBoundaryEntityId;
    return deletion;
}

foxglove_msgs::SceneUpdate worldBoundarySceneUpdate(const WorldBoundaryDisplay& boundary, const ros::Time& stamp,
                                                    const std::string& frame_id) {
    foxglove_msgs::SceneUpdate update;
    if (boundary.displayable) {
        update.entities.push_back(worldBoundaryEntity(boundary, stamp, frame_id));
    } else {
        update.deletions.push_back(worldBoundaryDeletion(stamp));
    }
    return update;
}

foxglove_msgs::SceneEntity worldWallsEntity(const WorldBoundaryDisplay& boundary, const ros::Time& stamp,
                                            const std::string& frame_id) {
    if (!boundary.displayable || stamp.isZero() || !isWorldFixedFrame(frame_id)) {
        throw std::invalid_argument("world walls display requires configured bounds, world frame, and timestamp");
    }
    foxglove_msgs::SceneEntity entity;
    entity.id = kWorldBoundaryWallsEntityId;
    entity.frame_id = frame_id;
    entity.timestamp = stamp;

    // Clip the display at the ground plane without changing the control bounds.
    const double wall_bottom = std::max(boundary.z_min, boundary.ground_z);
    if (boundary.z_max <= wall_bottom) {
        return entity;  // Same-ID replacement clears any previously visible walls.
    }

    const double center_x = (boundary.x_min + boundary.x_max) / 2.0;
    const double center_y = (boundary.y_min + boundary.y_max) / 2.0;
    const double center_z = (wall_bottom + boundary.z_max) / 2.0;
    const double span_x = boundary.x_max - boundary.x_min;
    const double span_y = boundary.y_max - boundary.y_min;
    const double span_z = boundary.z_max - wall_bottom;
    constexpr double kWallThickness = 0.02;

    struct WallSpec {
        double center_x;
        double center_y;
        double size_x;
        double size_y;
    };
    const WallSpec specs[4] = {
        {boundary.x_min, center_y, kWallThickness, span_y},
        {boundary.x_max, center_y, kWallThickness, span_y},
        {center_x, boundary.y_min, span_x, kWallThickness},
        {center_x, boundary.y_max, span_x, kWallThickness},
    };
    for (const auto& spec : specs) {
        foxglove_msgs::CubePrimitive wall;
        wall.pose.orientation.w = 1.0;
        wall.pose.position.x = spec.center_x;
        wall.pose.position.y = spec.center_y;
        wall.pose.position.z = center_z;
        wall.size.x = spec.size_x;
        wall.size.y = spec.size_y;
        wall.size.z = span_z;
        wall.color.r = 1.0;
        wall.color.g = 0.1;
        wall.color.b = 0.1;
        wall.color.a = 0.25;
        entity.cubes.push_back(std::move(wall));
    }
    return entity;
}

foxglove_msgs::SceneEntityDeletion worldWallsDeletion(const ros::Time& stamp) {
    if (stamp.isZero()) {
        throw std::invalid_argument("world walls deletion requires a timestamp");
    }
    foxglove_msgs::SceneEntityDeletion deletion;
    deletion.timestamp = stamp;
    deletion.type = foxglove_msgs::SceneEntityDeletion::MATCHING_ID;
    deletion.id = kWorldBoundaryWallsEntityId;
    return deletion;
}

foxglove_msgs::SceneUpdate worldWallsSceneUpdate(const WorldBoundaryDisplay& boundary, const ros::Time& stamp,
                                                 const std::string& frame_id) {
    foxglove_msgs::SceneUpdate update;
    if (boundary.displayable) {
        update.entities.push_back(worldWallsEntity(boundary, stamp, frame_id));
    } else {
        update.deletions.push_back(worldWallsDeletion(stamp));
    }
    return update;
}

WorldBoundaryDisplayMode worldBoundaryDisplayModeFromString(const std::string& mode) {
    if (mode == "off") {
        return WorldBoundaryDisplayMode::kOff;
    }
    if (mode == "ground") {
        return WorldBoundaryDisplayMode::kGround;
    }
    if (mode == "walls") {
        return WorldBoundaryDisplayMode::kWalls;
    }
    throw std::invalid_argument("world boundary display mode must be off, ground, or walls");
}

WorldBoundaryLayerMessages worldBoundaryLayerMessages(const WorldBoundaryDisplay& boundary,
                                                      WorldBoundaryDisplayMode mode, const ros::Time& stamp,
                                                      const std::string& frame_id) {
    WorldBoundaryDisplay hidden;
    const bool show_ground = mode == WorldBoundaryDisplayMode::kGround && boundary.displayable;
    const bool show_walls = mode == WorldBoundaryDisplayMode::kWalls && boundary.displayable;
    WorldBoundaryLayerMessages messages;
    messages.ground = worldBoundarySceneUpdate(show_ground ? boundary : hidden, stamp, frame_id);
    messages.walls = worldWallsSceneUpdate(show_walls ? boundary : hidden, stamp, frame_id);
    return messages;
}

geometry_msgs::Point applyExperimentWorldOffsetOnce(geometry_msgs::Point position,
                                                    const std::array<double, 3>& offset) {
    position.x += offset[0];
    position.y += offset[1];
    position.z += offset[2];
    return position;
}

geometry_msgs::Pose applyExperimentWorldOffsetOnce(geometry_msgs::Pose pose,
                                                    const std::array<double, 3>& offset) {
    pose.position = applyExperimentWorldOffsetOnce(pose.position, offset);
    return pose;
}

void validateSceneLabelOffsets(const SceneLabelOffsets& offsets) {
    for (const double value : {offsets.uav, offsets.scout, offsets.mecanum}) {
        if (!std::isfinite(value) || value < -10.0 || value > 10.0) {
            throw std::invalid_argument("robot label offsets must be between -10 and 10 meters");
        }
    }
}

std::set<std::string> parseModelNames(const std::string& csv) {
    std::set<std::string> names;
    std::stringstream stream(csv);
    std::string item;
    while (std::getline(stream, item, ',')) {
        item = trim(item);
        if (!item.empty()) {
            names.insert(item);
        }
    }
    return names;
}

bool modelListsAreDisjoint(const std::set<std::string>& fs150_models, const std::set<std::string>& scout_models,
                           const std::set<std::string>& mecanum_models) {
    std::set<std::string> seen;
    for (const std::set<std::string>* models : {&fs150_models, &scout_models, &mecanum_models}) {
        for (const std::string& model : *models) {
            if (!seen.insert(model).second) {
                return false;
            }
        }
    }
    return true;
}

RobotModelKind selectRobotModelKind(const std::string& model_name, const std::set<std::string>& configured_fs150_models,
                                    const std::set<std::string>& configured_scout_models,
                                    const std::set<std::string>& configured_mecanum_models, bool track_ugv) {
    const bool configured_fs150 = configured_fs150_models.count(model_name) != 0U;
    const bool configured_scout = configured_scout_models.count(model_name) != 0U;
    const bool configured_mecanum = configured_mecanum_models.count(model_name) != 0U;
    if (static_cast<int>(configured_fs150) + static_cast<int>(configured_scout) + static_cast<int>(configured_mecanum) >
        1) {
        return RobotModelKind::kNone;
    }
    if (configured_fs150) {
        return RobotModelKind::kFs150;
    }
    if (configured_scout) {
        return track_ugv ? RobotModelKind::kScout : RobotModelKind::kNone;
    }
    if (configured_mecanum) {
        return track_ugv ? RobotModelKind::kMecanum : RobotModelKind::kNone;
    }
    return RobotModelKind::kNone;
}

std::string sceneEntityID(RobotModelKind kind, const std::string& model_name) {
    switch (kind) {
    case RobotModelKind::kFs150:
        return "xgc2/px4/" + model_name;
    case RobotModelKind::kScout:
        return "xgc2/scout/" + model_name;
    case RobotModelKind::kMecanum:
        return "xgc2/mecanum/" + model_name;
    case RobotModelKind::kNone:
        break;
    }
    throw std::invalid_argument("scene entity requires a concrete robot model kind");
}

std::string sceneEntityPartID(RobotModelKind kind, const std::string& model_name, SceneEntityPart part) {
    const std::string robot_id = sceneEntityID(kind, model_name);
    switch (part) {
    case SceneEntityPart::kPath:
        return robot_id + "/path";
    case SceneEntityPart::kArLabel:
        return robot_id + "/label_ar";
    case SceneEntityPart::kLabel:
        return robot_id + "/label";
    }
    throw std::invalid_argument("scene entity part is unknown");
}

std::string slotVisualizationPoseTopic(RobotModelKind kind, const std::string& ros_namespace) {
    if (ros_namespace.size() < 2U || ros_namespace.front() != '/' || ros_namespace.back() == '/') {
        throw std::invalid_argument("canonical pose namespace must be an absolute ROS namespace");
    }
    const std::string identity = ros_namespace.substr(1);
    if (!canonicalROSIdentifier(identity) || identity.find('/') != std::string::npos) {
        throw std::invalid_argument("canonical pose namespace must be /<slot>");
    }
    if (kind == RobotModelKind::kNone) {
        throw std::invalid_argument("visualization pose requires a concrete robot model kind");
    }
    return kind == RobotModelKind::kFs150
               ? ros_namespace + "/mavros/local_position/pose"
               : ros_namespace + "/pose";
}

geometry_msgs::Pose slotHistoryPathPose(RobotModelKind kind, geometry_msgs::Pose world_pose) {
    if (kind == RobotModelKind::kScout || kind == RobotModelKind::kMecanum) {
        return xgc2_ros_visualizer::flattenGroundVehicleHistoryPose(world_pose);
    }
    return world_pose;
}

std::vector<geometry_msgs::TransformStamped>
canonicalRobotPoseTransforms(RobotModelKind kind, const std::string& scene_model,
                             const geometry_msgs::Pose& pose, const ros::Time& stamp,
                             const std::string& frame_id, const SceneLabelOffsets& offsets) {
    validateSceneLabelOffsets(offsets);
    if (kind == RobotModelKind::kNone || !canonicalROSIdentifier(scene_model) || !isWorldFixedFrame(frame_id) ||
        stamp.isZero()) {
        throw std::invalid_argument("canonical Robot pose transform identity must be complete");
    }

    double label_height = 0.0;
    switch (kind) {
    case RobotModelKind::kFs150:
        label_height = offsets.uav;
        break;
    case RobotModelKind::kScout:
        label_height = offsets.scout;
        break;
    case RobotModelKind::kMecanum:
        label_height = offsets.mecanum;
        break;
    case RobotModelKind::kNone:
        throw std::invalid_argument("canonical Robot pose transform requires a concrete kind");
    }

    const double body_z = slotDisplayBodyZ(kind, pose.position.z);

    geometry_msgs::TransformStamped body;
    body.header.stamp = stamp;
    body.header.frame_id = frame_id;
    body.child_frame_id = xgc2_ros_visualizer::robotBodyFrame(scene_model);
    body.transform.translation.x = pose.position.x;
    body.transform.translation.y = pose.position.y;
    body.transform.translation.z = body_z;
    body.transform.rotation = normalizedQuaternion(pose.orientation);

    geometry_msgs::TransformStamped label;
    label.header = body.header;
    label.child_frame_id = xgc2_ros_visualizer::robotLabelFrame(scene_model);
    label.transform.translation.x = pose.position.x;
    label.transform.translation.y = pose.position.y;
    label.transform.translation.z = body_z + label_height;
    label.transform.rotation.w = 1.0;
    return {body, label};
}

double identityLabelHeight(RobotModelKind kind, const SceneLabelOffsets& offsets) {
    switch (kind) {
    case RobotModelKind::kFs150:
        return offsets.uav;
    case RobotModelKind::kScout:
        return offsets.scout;
    case RobotModelKind::kMecanum:
        return offsets.mecanum;
    case RobotModelKind::kNone:
        break;
    }
    throw std::invalid_argument("identity label height requires a concrete kind");
}

geometry_msgs::TransformStamped canonicalArIdentityLabelTransform(
    RobotModelKind kind, const std::string& scene_model, const geometry_msgs::Pose& pose, const ros::Time& stamp,
    const std::string& frame_id, const SceneLabelOffsets& offsets) {
    validateSceneLabelOffsets(offsets);
    if (kind == RobotModelKind::kNone || !canonicalROSIdentifier(scene_model) || !isWorldFixedFrame(frame_id) ||
        stamp.isZero()) {
        throw std::invalid_argument("AR identity label transform identity must be complete");
    }
    geometry_msgs::TransformStamped label;
    label.header.stamp = stamp;
    label.header.frame_id = frame_id;
    label.child_frame_id = xgc2_ros_visualizer::robotFramePrefix(scene_model) + "/label_ar";
    label.transform.translation.x = pose.position.x;
    label.transform.translation.y = pose.position.y;
    label.transform.translation.z =
        slotDisplayBodyZ(kind, pose.position.z) + identityLabelHeight(kind, offsets);
    label.transform.rotation.w = 1.0;
    return label;
}

visualization_msgs::Marker identityLabelMarker(const std::string& scene_model, const std::string& label_frame,
                                                 const ros::Time& stamp) {
    if (!canonicalROSIdentifier(scene_model) || label_frame.empty() || stamp.isZero()) {
        throw std::invalid_argument("identity label marker requires a scene model, frame, and stamp");
    }
    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = label_frame;
    marker.ns = scene_model + "_label";
    marker.id = 11;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.32;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.text = scene_model;
    return marker;
}

void applyRobotMarkerLabel(visualization_msgs::MarkerArray* markers, std::size_t first_marker, RobotModelKind kind,
                           const std::string& ros_namespace) {
    if (markers == nullptr || first_marker > markers->markers.size()) {
        throw std::invalid_argument("robot label markers and range must be valid");
    }

    static const std::regex namespace_pattern("^/[A-Za-z_]+([0-9]+)[A-Za-z_]*$");
    std::smatch namespace_match;
    if (!std::regex_match(ros_namespace, namespace_match, namespace_pattern)) {
        throw std::invalid_argument("robot label namespace must be a single-segment ROS namespace with a number");
    }

    const char* class_label = nullptr;
    switch (kind) {
    case RobotModelKind::kFs150:
        class_label = "UAV";
        break;
    case RobotModelKind::kScout:
    case RobotModelKind::kMecanum:
        class_label = "UGV";
        break;
    case RobotModelKind::kNone:
        throw std::invalid_argument("robot label requires a concrete robot model kind");
    }

    const std::string display_label = std::string(class_label) + " " + namespace_match.str(1);
    for (std::size_t index = first_marker; index < markers->markers.size(); ++index) {
        visualization_msgs::Marker& marker = markers->markers[index];
        if (marker.type == visualization_msgs::Marker::TEXT_VIEW_FACING) {
            marker.text = display_label;
        }
    }
}

SceneUpdateCadence::SceneUpdateCadence(double label_publish_rate, double path_publish_rate)
    : label_publish_rate_(std::max(1.0, label_publish_rate)), path_publish_rate_(std::max(0.1, path_publish_rate)) {}

namespace {

// One drift-free rate gate, shared by every cadence in this file so a pose, a
// path trail and a joint animation all advance the same way.
bool takeRateGate(const ros::Time& now, double rate, bool* initialized, ros::Time* last_stamp) {
    if (initialized == nullptr || last_stamp == nullptr) {
        throw std::invalid_argument("scene update cadence gate state must be valid");
    }
    if (!*initialized || now < *last_stamp) {
        *initialized = true;
        *last_stamp = now;
        return true;
    }

    const double elapsed = (now - *last_stamp).toSec();
    const double period = 1.0 / rate;
    if (elapsed + 1.0e-9 < period) {
        return false;
    }

    // Advance by whole periods instead of assigning `now`. This avoids
    // accumulating timer quantisation drift when the source timer frequency is
    // not an integer multiple of the requested SceneUpdate rate.
    const double elapsed_periods = std::floor((elapsed + 1.0e-9) / period);
    *last_stamp += ros::Duration(elapsed_periods * period);
    return true;
}

} // namespace

bool SceneUpdateCadence::takeGate(const ros::Time& now, double rate, bool* initialized, ros::Time* last_stamp) {
    return takeRateGate(now, rate, initialized, last_stamp);
}

PublishCadence::PublishCadence(double publish_rate) : publish_rate_(publish_rate > 0.0 ? publish_rate : 1.0) {}

bool PublishCadence::take(const ros::Time& now) {
    return takeRateGate(now, publish_rate_, &initialized_, &last_stamp_);
}

SceneUpdateCadenceDecision SceneUpdateCadence::take(const ros::Time& now) {
    SceneUpdateCadenceDecision decision;
    decision.publish_label = takeGate(now, label_publish_rate_, &label_initialized_, &last_label_stamp_);
    decision.publish_path = takeGate(now, path_publish_rate_, &path_initialized_, &last_path_stamp_);
    return decision;
}

namespace {

bool markerBelongsToPart(const visualization_msgs::Marker& marker, SceneEntityPart part) {
    static const std::string path_suffix = "_actual_path";
    const bool is_path = marker.type == visualization_msgs::Marker::LINE_STRIP &&
                         marker.ns.size() >= path_suffix.size() &&
                         marker.ns.compare(marker.ns.size() - path_suffix.size(), path_suffix.size(), path_suffix) == 0;
    const bool is_label = marker.type == visualization_msgs::Marker::TEXT_VIEW_FACING;
    return part == SceneEntityPart::kPath ? is_path : is_label;
}

// A label is anchored rather than positioned: its entity names the robot's own
// label frame and asks the viewer to follow it. Every other part is drawn in
// world coordinates exactly where the message placed it.
bool partFollowsItsFrame(const SceneEntityPart* part) {
    return part != nullptr && (*part == SceneEntityPart::kLabel || *part == SceneEntityPart::kArLabel);
}

void appendSceneEntityImpl(RobotModelKind kind, const std::string& entity_id,
                           const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                           const ros::Time& timestamp, const std::string& frame_id, const SceneEntityPart* part,
                           const SceneLabelStyle& label_style, foxglove_msgs::SceneUpdate* update) {
    if (update == nullptr || first_marker > markers.markers.size()) {
        throw std::invalid_argument("scene entity output and marker range must be valid");
    }
    if (kind == RobotModelKind::kNone) {
        throw std::invalid_argument("scene entity requires a concrete robot model kind");
    }
    foxglove_msgs::SceneEntity entity;
    entity.timestamp = timestamp;
    entity.frame_id = frame_id;
    entity.id = entity_id;
    entity.lifetime = ros::Duration(0.0);
    entity.frame_locked = false;
    if (partFollowsItsFrame(part)) {
        // Take the frame from the marker itself: the visualizer that built it
        // is the one that knows which anchor it belongs to, and a caller
        // passing the world frame for every part cannot know that.
        for (std::size_t index = first_marker; index < markers.markers.size(); ++index) {
            if (markerBelongsToPart(markers.markers[index], *part)) {
                entity.frame_id = markers.markers[index].header.frame_id;
                entity.frame_locked = true;
                break;
            }
        }
    }

    for (std::size_t index = first_marker; index < markers.markers.size(); ++index) {
        const visualization_msgs::Marker& marker = markers.markers[index];
        if (part != nullptr && !markerBelongsToPart(marker, *part)) {
            continue;
        }
        switch (marker.type) {
        case visualization_msgs::Marker::MESH_RESOURCE: {
            foxglove_msgs::ModelPrimitive primitive;
            primitive.pose = copyPose(marker.pose);
            primitive.scale = marker.scale;
            primitive.color = copyColor(marker.color);
            primitive.override_color = !marker.mesh_use_embedded_materials;
            primitive.url = marker.mesh_resource;
            entity.models.push_back(std::move(primitive));
            break;
        }
        case visualization_msgs::Marker::LINE_STRIP: {
            foxglove_msgs::LinePrimitive primitive;
            primitive.type = foxglove_msgs::LinePrimitive::LINE_STRIP;
            primitive.pose = copyPose(marker.pose);
            primitive.thickness = marker.scale.x;
            primitive.scale_invariant = false;
            primitive.points = marker.points;
            primitive.color = copyColor(marker.color);
            entity.lines.push_back(std::move(primitive));
            break;
        }
        case visualization_msgs::Marker::TEXT_VIEW_FACING: {
            foxglove_msgs::TextPrimitive primitive;
            primitive.pose = copyPose(marker.pose);
            primitive.billboard = true;
            primitive.font_size = label_style.font_size;
            primitive.scale_invariant = label_style.scale_invariant;
            primitive.color = label_style.color;
            primitive.text = marker.text;
            entity.texts.push_back(std::move(primitive));
            break;
        }
        default:
            break;
        }
    }

    if (!entity.models.empty() || !entity.lines.empty() || !entity.texts.empty()) {
        update->entities.push_back(std::move(entity));
    }
}

} // namespace

void appendSceneEntity(RobotModelKind kind, const std::string& model_name,
                       const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                       const ros::Time& timestamp, const std::string& frame_id, const SceneLabelStyle& label_style,
                       foxglove_msgs::SceneUpdate* update) {
    appendSceneEntityImpl(kind, sceneEntityID(kind, model_name), markers, first_marker, timestamp, frame_id, nullptr,
                          label_style, update);
}

void appendSceneEntityPart(RobotModelKind kind, const std::string& model_name, SceneEntityPart part,
                           const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                           const ros::Time& timestamp, const std::string& frame_id, const SceneLabelStyle& label_style,
                           foxglove_msgs::SceneUpdate* update) {
    appendSceneEntityImpl(kind, sceneEntityPartID(kind, model_name, part), markers, first_marker, timestamp, frame_id,
                          &part, label_style, update);
}

bool isWorldFixedFrame(const std::string& frame_id) {
    return normalizedFrameLabel(frame_id) == "world";
}

CanonicalWorldPose selectSlotVisualizationWorldPose(RobotModelKind kind, const CanonicalPoseSample& pose,
                                                     const ros::Time& now, double timeout_sec) {
    CanonicalWorldPose selected;
    const bool fs150_fused_local_frame =
        kind == RobotModelKind::kFs150 && (pose.frame_id == "map" || pose.frame_id == "/map");
    if (kind == RobotModelKind::kNone || !isFreshCanonicalSample(pose, now, timeout_sec) ||
        (!isWorldFixedFrame(pose.frame_id) && !fs150_fused_local_frame)) {
        return selected;
    }
    selected.found = true;
    selected.pose = pose.pose;
    selected.stamp = pose.stamp;
    selected.frame_id = "world";
    return selected;
}

CanonicalWorldPose selectUavHeightProjectionWorldPose(HeightProjectionView view,
                                                       const CanonicalPoseSample& local_position,
                                                       const CanonicalPoseSample& vrpn_already_offset,
                                                       const ros::Time& now, double timeout_sec) {
    switch (view) {
        case HeightProjectionView::kLocalPosition:
            return selectSlotVisualizationWorldPose(RobotModelKind::kFs150, local_position, now, timeout_sec);
        case HeightProjectionView::kVrpn:
            return selectSlotVisualizationWorldPose(RobotModelKind::kScout, vrpn_already_offset, now, timeout_sec);
        default:
            return CanonicalWorldPose{};
    }
}

CanonicalWorldPose selectArIdentityWorldPose(RobotModelKind kind, const CanonicalPoseSample& canonical,
                                               const CanonicalPoseSample& vrpn_already_offset, const ros::Time& now,
                                               double timeout_sec) {
    if (kind == RobotModelKind::kFs150) {
        return selectUavHeightProjectionWorldPose(HeightProjectionView::kVrpn, canonical, vrpn_already_offset, now,
                                                     timeout_sec);
    }
    return selectSlotVisualizationWorldPose(kind, canonical, now, timeout_sec);
}

geometry_msgs::TransformStamped worldFixedFrameRoot(const std::string& frame_id, const ros::Time& stamp) {
    geometry_msgs::TransformStamped origin;
    origin.header.stamp = stamp;
    origin.header.frame_id = frame_id;
    origin.child_frame_id = kWorldFixedFrameRootChild;
    origin.transform.rotation.w = 1.0;
    return origin;
}

geometry_msgs::TransformStamped algorithmOverlayFrameAlias(const std::string& world_frame, const ros::Time& stamp) {
    if (!isWorldFixedFrame(world_frame)) {
        throw std::invalid_argument("algorithm overlay alias parent must be the world Fixed Frame");
    }
    geometry_msgs::TransformStamped alias;
    alias.header.stamp = stamp;
    alias.header.frame_id = "world";
    alias.child_frame_id = kAlgorithmOverlayFrame;
    alias.transform.rotation.w = 1.0;
    return alias;
}

bool frozenVisualizationRosterReady(std::size_t tracked_models, std::size_t world_poses) {
    return tracked_models > 0U && world_poses <= tracked_models;
}

} // namespace xgc2_ros_visualizer

#include "xgc2_ros_visualizer/scene_contract.hpp"
#include "render/robots/path_history.hpp"
#include "render/robots/robot_frames.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

#include <foxglove_msgs/LinePrimitive.h>
#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <ros/serialization.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace xgc2_ros_visualizer {
namespace {

SceneLabelStyle blackLabelStyle() {
    return sceneLabelStyleFromMarkerColor("#000000");
}

double displayBodyZ(RobotModelKind kind, double pose_z) {
    if (kind == RobotModelKind::kScout) {
        return xgc2_ros_visualizer::scoutDisplayBodyZ();
    }
    if (kind == RobotModelKind::kMecanum) {
        return xgc2_ros_visualizer::mecanumDisplayBodyZ();
    }
    return pose_z;
}

TEST(SceneContract, MarkerColorIsStrictAndProducesOpaqueTextWithUnifiedFontSize) {
    const SceneLabelStyle black = blackLabelStyle();
    EXPECT_DOUBLE_EQ(black.font_size, 0.24);
    EXPECT_DOUBLE_EQ(black.color.r, 0.0);
    EXPECT_DOUBLE_EQ(black.color.g, 0.0);
    EXPECT_DOUBLE_EQ(black.color.b, 0.0);
    EXPECT_DOUBLE_EQ(black.color.a, 1.0);

    const SceneLabelStyle custom = sceneLabelStyleFromMarkerColor("#1a2b3c");
    EXPECT_NEAR(custom.color.r, 26.0 / 255.0, 1.0e-12);
    EXPECT_NEAR(custom.color.g, 43.0 / 255.0, 1.0e-12);
    EXPECT_NEAR(custom.color.b, 60.0 / 255.0, 1.0e-12);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("000000"), std::invalid_argument);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("#FFFFFF"), std::invalid_argument);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("#00000000"), std::invalid_argument);
}

TEST(SceneContract, FS150ScoutAndMecanumLabelsUseOneSceneStyle) {
    const RobotModelKind kinds[] = {
        RobotModelKind::kFs150,
        RobotModelKind::kScout,
        RobotModelKind::kMecanum,
    };
    const char* names[] = {"uav1", "ugv1", "mecanum1"};
    const SceneLabelStyle style = sceneLabelStyleFromMarkerColor("#102030");

    for (std::size_t index = 0U; index < 3U; ++index) {
        visualization_msgs::Marker label;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.pose.orientation.w = 1.0;
        label.scale.z = index == 2U ? 0.18 : 0.32;
        label.color.r = 1.0;
        label.color.g = index == 2U ? 0.85 : 1.0;
        label.color.b = index == 2U ? 0.25 : 1.0;
        label.color.a = 1.0;
        label.text = names[index];
        visualization_msgs::MarkerArray markers;
        markers.markers.push_back(label);

        foxglove_msgs::SceneUpdate update;
        appendSceneEntity(kinds[index], names[index], markers, 0U, ros::Time(42, 0), "world", style, &update);
        ASSERT_EQ(update.entities.size(), 1U);
        ASSERT_EQ(update.entities[0].texts.size(), 1U);
        const foxglove_msgs::TextPrimitive& text = update.entities[0].texts[0];
        EXPECT_DOUBLE_EQ(text.font_size, 0.24);
        EXPECT_DOUBLE_EQ(text.color.r, 16.0 / 255.0);
        EXPECT_DOUBLE_EQ(text.color.g, 32.0 / 255.0);
        EXPECT_DOUBLE_EQ(text.color.b, 48.0 / 255.0);
        EXPECT_DOUBLE_EQ(text.color.a, 1.0);
        EXPECT_TRUE(text.billboard);
        EXPECT_FALSE(text.scale_invariant);
    }
}

TEST(SceneContract, ConfiguredLabelStyleAndOffsetsPreserveUprightAnchors) {
    const RobotModelKind kinds[] = {RobotModelKind::kFs150, RobotModelKind::kScout, RobotModelKind::kMecanum};
    const char* names[] = {"uav1", "ugv1", "ugv2"};
    const SceneLabelOffsets offsets{1.2, 0.0, -0.4};
    const double heights[] = {1.2, 0.0, -0.4};
    geometry_msgs::Pose pose;
    pose.position.x = 3.0;
    pose.position.y = -2.0;
    pose.position.z = 4.0;
    pose.orientation.x = std::sin(0.7);
    pose.orientation.w = std::cos(0.7);
    for (const bool fixed : {false, true}) {
        const auto style = sceneLabelStyleFromMarkerColor("#123456", fixed, fixed ? 28.0 : 0.4, 0.35);
        for (std::size_t i = 0; i < 3; ++i) {
            const auto transforms = canonicalRobotPoseTransforms(kinds[i], names[i], pose, ros::Time(42), "world", offsets);
            ASSERT_EQ(transforms.size(), 2U);
            const double body_z = displayBodyZ(kinds[i], pose.position.z);
            EXPECT_DOUBLE_EQ(transforms[0].transform.translation.z, body_z);
            EXPECT_DOUBLE_EQ(transforms[0].transform.rotation.x, pose.orientation.x);
            const auto& anchor = transforms[1];
            EXPECT_EQ(anchor.header.frame_id, "world");
            EXPECT_EQ(anchor.child_frame_id, "xgc/robots/" + std::string(names[i]) + "/label");
            EXPECT_DOUBLE_EQ(anchor.transform.translation.x, 3.0);
            EXPECT_DOUBLE_EQ(anchor.transform.translation.y, -2.0);
            EXPECT_DOUBLE_EQ(anchor.transform.translation.z, body_z + heights[i]);
            EXPECT_DOUBLE_EQ(anchor.transform.rotation.x, 0.0);
            EXPECT_DOUBLE_EQ(anchor.transform.rotation.y, 0.0);
            EXPECT_DOUBLE_EQ(anchor.transform.rotation.z, 0.0);
            EXPECT_DOUBLE_EQ(anchor.transform.rotation.w, 1.0);
            visualization_msgs::MarkerArray markers;
            visualization_msgs::Marker label;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.header.frame_id = anchor.child_frame_id;
            label.pose.orientation.w = 1.0;
            label.text = "identity";
            markers.markers.push_back(label);
            foxglove_msgs::SceneUpdate update;
            appendSceneEntityPart(kinds[i], names[i], SceneEntityPart::kLabel, markers, 0U, ros::Time(42), "world", style, &update);
            ASSERT_EQ(update.entities.size(), 1U);
            EXPECT_EQ(update.entities[0].frame_id, anchor.child_frame_id);
            ASSERT_EQ(update.entities[0].texts.size(), 1U);
            const auto& text = update.entities[0].texts[0];
            EXPECT_EQ(text.text, "identity");
            EXPECT_EQ(text.scale_invariant, fixed);
            EXPECT_DOUBLE_EQ(text.font_size, fixed ? 28.0 : 0.4);
            EXPECT_DOUBLE_EQ(text.color.a, 0.35);
            EXPECT_DOUBLE_EQ(text.pose.position.x, 0.0);
            EXPECT_DOUBLE_EQ(text.pose.position.y, 0.0);
            EXPECT_DOUBLE_EQ(text.pose.position.z, 0.0);
        }
    }
}

TEST(SceneContract, LabelStyleRejectsInvalidUnitsAndAllowsZeroOpacity) {
    EXPECT_DOUBLE_EQ(sceneLabelStyleFromMarkerColor("#ffbf00", true, 16.0, 0.0).color.a, 0.0);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("#ffbf00", true, 0.24, 1.0), std::invalid_argument);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("#ffbf00", false, 16.0, 1.0), std::invalid_argument);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("#ffbf00", false, 0.24, -0.1), std::invalid_argument);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("#ffbf00", false, 0.24, 1.1), std::invalid_argument);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("#ffbf00", false, std::numeric_limits<double>::quiet_NaN(), 1.0), std::invalid_argument);
    const SceneLabelOffsets invalid{0.55, 0.65, std::numeric_limits<double>::infinity()};
    EXPECT_THROW(validateSceneLabelOffsets(invalid), std::invalid_argument);
}

TEST(SceneContract, ImmutableListsSelectOnlyConfiguredModels) {
    const std::set<std::string> fs150s{"uav1", "uav2"};
    const std::set<std::string> scouts{"ugv1"};
    const std::set<std::string> mecanums{"ugv2"};

    EXPECT_EQ(selectRobotModelKind("uav1", fs150s, scouts, mecanums, true), RobotModelKind::kFs150);
    EXPECT_EQ(selectRobotModelKind("ugv1", fs150s, scouts, mecanums, true), RobotModelKind::kScout);
    EXPECT_EQ(selectRobotModelKind("ugv2", fs150s, scouts, mecanums, true), RobotModelKind::kMecanum);
    EXPECT_EQ(selectRobotModelKind("uav9", fs150s, scouts, mecanums, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("ugv9", fs150s, scouts, mecanums, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("ground_plane", fs150s, scouts, mecanums, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("uav1", fs150s, scouts, mecanums, false), RobotModelKind::kFs150);
    EXPECT_EQ(selectRobotModelKind("ugv1", fs150s, scouts, mecanums, false), RobotModelKind::kNone);
}

TEST(SceneContract, ModelListsMustBeDisjoint) {
    EXPECT_TRUE(modelListsAreDisjoint({"uav1"}, {"ugv1"}, {"ugv2"}));
    EXPECT_FALSE(modelListsAreDisjoint({"shared"}, {"ugv1"}, {"shared"}));
}

TEST(SceneContract, RobotKindOwnsUppercaseMarkerClassAndCanonicalNamespaceOwnsNumber) {
    struct Case {
        RobotModelKind kind;
        const char* ros_namespace;
        const char* expected;
    };
    const Case cases[] = {
        {RobotModelKind::kFs150, "/uav1", "UAV 1"},
        {RobotModelKind::kFs150, "/SQ01s", "UAV 01"},
        {RobotModelKind::kScout, "/ugv2", "UGV 2"},
        {RobotModelKind::kScout, "/scout1", "UGV 1"},
        {RobotModelKind::kMecanum, "/ugv3", "UGV 3"},
        // Mixed fleets may bind a Scout scene model to a /uavN slot. Robot
        // category, not the lowercase interface prefix, still owns UGV.
        {RobotModelKind::kScout, "/uav7", "UGV 7"},
    };

    for (const Case& test_case : cases) {
        visualization_msgs::Marker untouched;
        untouched.type = visualization_msgs::Marker::LINE_STRIP;
        untouched.text = "unchanged";
        visualization_msgs::Marker label;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.text = "lowercase-interface-name";
        visualization_msgs::MarkerArray markers;
        markers.markers = {untouched, label};

        applyRobotMarkerLabel(&markers, 1U, test_case.kind, test_case.ros_namespace);

        EXPECT_EQ(markers.markers[0].text, "unchanged");
        EXPECT_EQ(markers.markers[1].text, test_case.expected);
    }

    visualization_msgs::MarkerArray markers;
    EXPECT_THROW(applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kNone, "/uav1"), std::invalid_argument);
    EXPECT_THROW(applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kFs150, "uav1"), std::invalid_argument);
    EXPECT_THROW(applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kScout, "/scout"), std::invalid_argument);
    EXPECT_THROW(applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kFs150, "/bad_slot"), std::invalid_argument);
    EXPECT_THROW(applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kFs150, "/a/b"), std::invalid_argument);
}

TEST(SceneContract, CanonicalPosePublishesOnlyBodyAndUprightLabelTransforms) {
    geometry_msgs::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = -2.0;
    pose.position.z = 3.0;
    pose.orientation.z = std::sqrt(0.5);
    pose.orientation.w = std::sqrt(0.5);
    const ros::Time stamp(12, 34);

    struct Case {
        RobotModelKind kind;
        const char* name;
        double label_height;
    };
    const Case cases[] = {
        {RobotModelKind::kFs150, "uav1", 0.55},
        {RobotModelKind::kScout, "ugv1", 0.65},
        {RobotModelKind::kMecanum, "ugv2", 0.32},
    };
    for (const Case& test_case : cases) {
        const auto transforms = canonicalRobotPoseTransforms(test_case.kind, test_case.name, pose, stamp, "world");
        ASSERT_EQ(transforms.size(), 2U);
        EXPECT_EQ(transforms[0].child_frame_id, "xgc/robots/" + std::string(test_case.name) + "/base_link");
        EXPECT_EQ(transforms[1].child_frame_id, "xgc/robots/" + std::string(test_case.name) + "/label");
        EXPECT_EQ(transforms[0].header.stamp, stamp);
        EXPECT_DOUBLE_EQ(transforms[0].transform.translation.x, pose.position.x);
        EXPECT_NEAR(transforms[0].transform.rotation.z, std::sqrt(0.5), 1.0e-12);
        const double body_z = displayBodyZ(test_case.kind, pose.position.z);
        EXPECT_DOUBLE_EQ(transforms[0].transform.translation.z, body_z);
        EXPECT_DOUBLE_EQ(transforms[1].transform.translation.z, body_z + test_case.label_height);
        EXPECT_DOUBLE_EQ(transforms[1].transform.rotation.w, 1.0);
    }
    EXPECT_THROW(canonicalRobotPoseTransforms(RobotModelKind::kNone, "uav1", pose, stamp, "world"),
                 std::invalid_argument);
    EXPECT_THROW(canonicalRobotPoseTransforms(RobotModelKind::kFs150, "uav1", pose, ros::Time(), "world"),
                 std::invalid_argument);
}

TEST(SceneContract, GroundVehicleBodyTransformSitsWheelsOnTheXYPlane) {
    geometry_msgs::Pose pose;
    pose.position.x = -6.01;
    pose.position.y = -6.00;
    pose.position.z = 0.504;
    pose.orientation.w = 1.0;
    const ros::Time stamp(17, 0);
    const SceneLabelOffsets offsets{};

    const auto scout = canonicalRobotPoseTransforms(RobotModelKind::kScout, "ugv5", pose, stamp, "world", offsets);
    ASSERT_EQ(scout.size(), 2U);
    EXPECT_DOUBLE_EQ(scout[0].transform.translation.x, pose.position.x);
    EXPECT_DOUBLE_EQ(scout[0].transform.translation.y, pose.position.y);
    EXPECT_DOUBLE_EQ(scout[0].transform.translation.z, xgc2_ros_visualizer::kScoutGazeboChassisZ);
    EXPECT_NEAR(scout[0].transform.translation.z,
                xgc2_ros_visualizer::kScoutVisualWheelRadius - xgc2_ros_visualizer::kScoutVisualWheelAxleZ,
                0.002);
    EXPECT_DOUBLE_EQ(scout[1].transform.translation.z, xgc2_ros_visualizer::scoutDisplayBodyZ() + 0.65);

    const auto mecanum =
        canonicalRobotPoseTransforms(RobotModelKind::kMecanum, "ugv2", pose, stamp, "world", offsets);
    ASSERT_EQ(mecanum.size(), 2U);
    EXPECT_DOUBLE_EQ(mecanum[0].transform.translation.z, xgc2_ros_visualizer::mecanumDisplayBodyZ());
    EXPECT_DOUBLE_EQ(mecanum[1].transform.translation.z, 0.32);

    const auto uav = canonicalRobotPoseTransforms(RobotModelKind::kFs150, "uav1", pose, stamp, "world", offsets);
    ASSERT_EQ(uav.size(), 2U);
    EXPECT_DOUBLE_EQ(uav[0].transform.translation.z, 0.504);
    EXPECT_DOUBLE_EQ(uav[1].transform.translation.z, 0.504 + 0.55);

    const auto ar = canonicalArIdentityLabelTransform(RobotModelKind::kScout, "ugv5", pose, stamp, "world", offsets);
    EXPECT_DOUBLE_EQ(ar.transform.translation.z, xgc2_ros_visualizer::scoutDisplayBodyZ() + 0.65);
}

TEST(SceneContract, ScoutMarkersBecomeScoutSceneEntity) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://scout_description/meshes/base_link.dae";
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);

    foxglove_msgs::SceneUpdate update;
    appendSceneEntity(RobotModelKind::kScout, "ugv1", markers, 0U, ros::Time(42, 0), "world", blackLabelStyle(),
                      &update);

    ASSERT_EQ(update.entities.size(), 1U);
    EXPECT_EQ(update.entities[0].id, "xgc2/scout/ugv1");
    EXPECT_EQ(update.entities[0].frame_id, "world");
    ASSERT_EQ(update.entities[0].models.size(), 1U);
    EXPECT_EQ(update.entities[0].models[0].url, marker.mesh_resource);
}

TEST(SceneContract, MecanumMarkersBecomeMecanumSceneEntity) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://mecanum_description/meshes/nexus_base_link.STL";
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);

    foxglove_msgs::SceneUpdate update;
    appendSceneEntity(RobotModelKind::kMecanum, "ugv1", markers, 0U, ros::Time(42, 0), "world", blackLabelStyle(),
                      &update);

    ASSERT_EQ(update.entities.size(), 1U);
    EXPECT_EQ(update.entities[0].id, "xgc2/mecanum/ugv1");
    ASSERT_EQ(update.entities[0].models.size(), 1U);
    EXPECT_EQ(update.entities[0].models[0].url, marker.mesh_resource);
}

TEST(SceneContract, RobotAndPathUsePersistentIndependentEntityIDs) {
    visualization_msgs::Marker mesh;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.mesh_resource = "package://fs150_description/meshes/iris.stl";
    mesh.pose.orientation.w = 1.0;
    mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0;

    visualization_msgs::Marker path;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.ns = "uav1_actual_path";
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.02;
    for (int index = 0; index < 151; ++index) {
        geometry_msgs::Point point;
        point.x = static_cast<double>(index) * 0.1;
        path.points.push_back(point);
    }

    visualization_msgs::Marker label;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.header.frame_id = "uav1/label";
    label.pose.orientation.w = 1.0;
    label.text = "UAV 1";
    label.scale.z = 0.32;

    visualization_msgs::MarkerArray markers;
    markers.markers = {mesh, path, label};

    // Geometry is not a scene entity any more. Every mesh here is a link of the
    // robot's own URDF, which the viewer loads once and places from transforms,
    // so a part that carried it would draw each vehicle a second time.
    foxglove_msgs::SceneUpdate path_only;
    appendSceneEntityPart(RobotModelKind::kFs150, "uav1", SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0),
                          "world", blackLabelStyle(), &path_only);
    for (const foxglove_msgs::SceneEntity& entity : path_only.entities) {
        EXPECT_TRUE(entity.models.empty()) << "a scene part shipped robot geometry the URDF already draws";
    }

    foxglove_msgs::SceneUpdate path_update;
    appendSceneEntityPart(RobotModelKind::kFs150, "uav1", SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0),
                          "world", blackLabelStyle(), &path_update);
    ASSERT_EQ(path_update.entities.size(), 1U);
    EXPECT_EQ(path_update.entities[0].id, "xgc2/px4/uav1/path");
    EXPECT_EQ(path_update.entities[0].lines.size(), 1U);
    EXPECT_TRUE(path_update.entities[0].models.empty());
    EXPECT_TRUE(path_update.entities[0].texts.empty());
    EXPECT_TRUE(path_update.entities[0].lifetime.isZero());

    // A label follows its anchor instead of being redrawn: the entity names the
    // robot's own label frame and asks the viewer to re-resolve it. That is the
    // whole reason a label transmitted twice a second can move thirty times a
    // second, so both halves are asserted here.
    foxglove_msgs::SceneUpdate label_update;
    appendSceneEntityPart(RobotModelKind::kFs150, "uav1", SceneEntityPart::kLabel, markers, 0U, ros::Time(42, 0),
                          "world", blackLabelStyle(), &label_update);
    ASSERT_EQ(label_update.entities.size(), 1U);
    EXPECT_EQ(label_update.entities[0].id, "xgc2/px4/uav1/label");
    EXPECT_EQ(label_update.entities[0].frame_id, "uav1/label");
    EXPECT_TRUE(label_update.entities[0].frame_locked);
    EXPECT_EQ(label_update.entities[0].texts.size(), 1U);
    EXPECT_TRUE(label_update.entities[0].models.empty());
    EXPECT_TRUE(label_update.entities[0].lines.empty());

    foxglove_msgs::SceneUpdate legacy_full_update;
    appendSceneEntity(RobotModelKind::kFs150, "uav1", markers, 0U, ros::Time(42, 0), "world", blackLabelStyle(),
                      &legacy_full_update);
    EXPECT_LT(ros::serialization::serializationLength(label_update),
              ros::serialization::serializationLength(legacy_full_update));
}

TEST(SceneContract, MixedSlotIdentityChangesPresentationButPreservesSceneModelAnchors) {
    visualization_msgs::Marker mesh;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.mesh_resource = "package://scout_description/meshes/base_link.dae";
    mesh.pose.orientation.w = 1.0;
    mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0;

    visualization_msgs::Marker path;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.ns = "ugv1_actual_path";
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.02;
    path.points.resize(2);
    path.points[1].x = 1.0;

    visualization_msgs::Marker label;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.header.frame_id = "ugv1/label";
    label.pose.orientation.w = 1.0;
    label.text = "ugv1";
    label.scale.z = 0.32;

    visualization_msgs::MarkerArray markers;
    markers.markers = {mesh, path, label};
    applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kScout, "/uav7");

    // Re-labeling is presentation-only. The scene-model marker namespace and
    // anchor frame remain ugv1 so pose/TF lookup cannot drift to the slot name.
    EXPECT_EQ(markers.markers[0].mesh_resource, mesh.mesh_resource);
    EXPECT_EQ(markers.markers[1].ns, "ugv1_actual_path");
    EXPECT_EQ(markers.markers[2].header.frame_id, "ugv1/label");
    EXPECT_EQ(markers.markers[2].text, "UGV 7");

    foxglove_msgs::SceneUpdate label_update;
    appendSceneEntityPart(RobotModelKind::kScout, "uav7", SceneEntityPart::kLabel, markers, 0U,
                          ros::Time(42, 0), "world", blackLabelStyle(), &label_update);
    ASSERT_EQ(label_update.entities.size(), 1U);
    EXPECT_EQ(label_update.entities[0].id, "xgc2/scout/uav7/label");
    EXPECT_EQ(label_update.entities[0].frame_id, "ugv1/label");
    ASSERT_EQ(label_update.entities[0].texts.size(), 1U);
    EXPECT_EQ(label_update.entities[0].texts[0].text, "UGV 7");
    EXPECT_TRUE(label_update.entities[0].models.empty());

    foxglove_msgs::SceneUpdate path_update;
    appendSceneEntityPart(RobotModelKind::kScout, "uav7", SceneEntityPart::kPath, markers, 0U,
                          ros::Time(42, 0), "world", blackLabelStyle(), &path_update);
    ASSERT_EQ(path_update.entities.size(), 1U);
    EXPECT_EQ(path_update.entities[0].id, "xgc2/scout/uav7/path");
    EXPECT_EQ(path_update.entities[0].frame_id, "world");
    ASSERT_EQ(path_update.entities[0].lines.size(), 1U);
    EXPECT_TRUE(path_update.entities[0].models.empty());
}

TEST(SceneContract, SceneCadenceSeparatesRobotAndPathUpdatesWithoutDrift) {
    SceneUpdateCadence cadence(10.0, 2.0);

    SceneUpdateCadenceDecision decision = cadence.take(ros::Time(10, 0));
    EXPECT_TRUE(decision.publish_label);
    EXPECT_TRUE(decision.publish_path);

    decision = cadence.take(ros::Time(10, 50000000));
    EXPECT_FALSE(decision.publish_label);
    EXPECT_FALSE(decision.publish_path);

    decision = cadence.take(ros::Time(10, 100000000));
    EXPECT_TRUE(decision.publish_label);
    EXPECT_FALSE(decision.publish_path);

    decision = cadence.take(ros::Time(10, 500000000));
    EXPECT_TRUE(decision.publish_label);
    EXPECT_TRUE(decision.publish_path);

    // A simulated-clock reset immediately republishes a complete pair so the
    // consumer cannot remain stuck with stale entities from the old epoch.
    decision = cadence.take(ros::Time(1, 0));
    EXPECT_TRUE(decision.publish_label);
    EXPECT_TRUE(decision.publish_path);
}

TEST(SceneContract, MixedFleetSceneSerializationBudgetIsBounded) {
    foxglove_msgs::SceneUpdate legacy_updates;
    foxglove_msgs::SceneUpdate label_updates;
    foxglove_msgs::SceneUpdate path_updates;

    for (int robot_index = 1; robot_index <= 10; ++robot_index) {
        const bool is_uav = robot_index <= 6;
        const RobotModelKind kind = is_uav ? RobotModelKind::kFs150 : RobotModelKind::kScout;
        const std::string name = is_uav ? "uav" + std::to_string(robot_index) : "ugv" + std::to_string(robot_index - 6);
        const int model_count = is_uav ? 5 : 6;
        visualization_msgs::MarkerArray markers;
        for (int model_index = 0; model_index < model_count; ++model_index) {
            visualization_msgs::Marker mesh;
            mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
            mesh.mesh_resource = is_uav ? "package://fs150_description/meshes/iris_prop_ccw.dae"
                                        : "package://scout_description/meshes/wheel.dae";
            mesh.pose.orientation.w = 1.0;
            mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0;
            markers.markers.push_back(mesh);
        }

        visualization_msgs::Marker path;
        path.type = visualization_msgs::Marker::LINE_STRIP;
        path.ns = name + "_actual_path";
        path.pose.orientation.w = 1.0;
        path.scale.x = 0.02;
        for (int point_index = 0; point_index < 151; ++point_index) {
            geometry_msgs::Point point;
            point.x = static_cast<double>(point_index) * 0.1;
            point.y = static_cast<double>(robot_index);
            path.points.push_back(point);
        }
        markers.markers.push_back(path);

        visualization_msgs::Marker label;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.pose.orientation.w = 1.0;
        label.text = name;
        label.scale.z = 0.32;
        markers.markers.push_back(label);

        appendSceneEntity(kind, name, markers, 0U, ros::Time(42, 0), "world", blackLabelStyle(), &legacy_updates);
        appendSceneEntityPart(kind, name, SceneEntityPart::kLabel, markers, 0U, ros::Time(42, 0), "world",
                              blackLabelStyle(), &label_updates);
        appendSceneEntityPart(kind, name, SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0), "world",
                              blackLabelStyle(), &path_updates);
    }

    ASSERT_EQ(legacy_updates.entities.size(), 10U);
    ASSERT_EQ(label_updates.entities.size(), 10U);
    ASSERT_EQ(path_updates.entities.size(), 10U);
    EXPECT_EQ(label_updates.entities.front().id, "xgc2/px4/uav1/label");
    EXPECT_EQ(label_updates.entities.back().id, "xgc2/scout/ugv4/label");
    EXPECT_EQ(path_updates.entities.front().id, "xgc2/px4/uav1/path");
    EXPECT_EQ(path_updates.entities.back().id, "xgc2/scout/ugv4/path");

    const std::size_t legacy_bytes_per_second =
        ros::serialization::serializationLength(legacy_updates) * static_cast<std::size_t>(30);
    const std::size_t bounded_bytes_per_second =
        ros::serialization::serializationLength(label_updates) * static_cast<std::size_t>(10) +
        ros::serialization::serializationLength(path_updates) * static_cast<std::size_t>(2);
    RecordProperty("legacy_bytes_per_second", static_cast<int>(legacy_bytes_per_second));
    RecordProperty("bounded_bytes_per_second", static_cast<int>(bounded_bytes_per_second));
    EXPECT_LT(bounded_bytes_per_second * static_cast<std::size_t>(4), legacy_bytes_per_second);
}

TEST(SceneContract, EntityIDsRemainKindScoped) {
    EXPECT_EQ(sceneEntityID(RobotModelKind::kFs150, "uav1"), "xgc2/px4/uav1");
    EXPECT_EQ(sceneEntityID(RobotModelKind::kScout, "ugv1"), "xgc2/scout/ugv1");
    EXPECT_EQ(sceneEntityID(RobotModelKind::kMecanum, "ugv1"), "xgc2/mecanum/ugv1");
    EXPECT_EQ(sceneEntityPartID(RobotModelKind::kFs150, "uav1", SceneEntityPart::kPath), "xgc2/px4/uav1/path");
    EXPECT_EQ(sceneEntityPartID(RobotModelKind::kFs150, "uav1", SceneEntityPart::kLabel), "xgc2/px4/uav1/label");
    EXPECT_EQ(sceneEntityPartID(RobotModelKind::kFs150, "uav1", SceneEntityPart::kArLabel), "xgc2/px4/uav1/label_ar");
    EXPECT_EQ(sceneEntityPartID(RobotModelKind::kScout, "ugv1", SceneEntityPart::kPath), "xgc2/scout/ugv1/path");
    EXPECT_EQ(sceneEntityPartID(RobotModelKind::kMecanum, "ugv1", SceneEntityPart::kPath), "xgc2/mecanum/ugv1/path");
    EXPECT_THROW(sceneEntityID(RobotModelKind::kNone, "unknown"), std::invalid_argument);
}

TEST(SceneContract, HistoricalPathNamespaceIsSharedAcrossKinds) {
    const RobotModelKind kinds[] = {RobotModelKind::kFs150, RobotModelKind::kScout, RobotModelKind::kMecanum};
    const char* names[] = {"uav1", "ugv1", "ugv2"};
    const char* ids[] = {"xgc2/px4/uav1/path", "xgc2/scout/ugv1/path", "xgc2/mecanum/ugv2/path"};
    for (std::size_t index = 0U; index < 3U; ++index) {
        visualization_msgs::Marker path;
        path.type = visualization_msgs::Marker::LINE_STRIP;
        path.ns = std::string(names[index]) + "_actual_path";
        path.pose.position.z = 0.18;
        path.pose.orientation.w = 1.0;
        path.scale.x = 0.02;
        geometry_msgs::Point point;
        point.z = 0.18;
        path.points.push_back(point);
        visualization_msgs::MarkerArray markers;
        markers.markers.push_back(path);
        foxglove_msgs::SceneUpdate update;
        appendSceneEntityPart(kinds[index], names[index], SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0),
                              "world", blackLabelStyle(), &update);
        ASSERT_EQ(update.entities.size(), 1U);
        EXPECT_EQ(update.entities[0].id, ids[index]);
        EXPECT_EQ(update.entities[0].frame_id, "world");
        ASSERT_EQ(update.entities[0].lines.size(), 1U);
        EXPECT_DOUBLE_EQ(update.entities[0].lines[0].pose.position.z, 0.18);
        EXPECT_DOUBLE_EQ(update.entities[0].lines[0].points[0].z, 0.18);
    }
}

TEST(CanonicalWorldPose, AcceptsOnlyTheKindSpecificSlotViewerSample) {
    const ros::Time now(10, 0);
    CanonicalPoseSample pose;
    pose.available = true;
    pose.pose.position.x = 1.25;
    pose.pose.position.z = 0.16;
    pose.pose.orientation.w = 1.0;
    pose.stamp = now;
    pose.frame_id = "world";

    const CanonicalWorldPose selected =
        selectSlotVisualizationWorldPose(RobotModelKind::kScout, pose, now, 0.5);
    ASSERT_TRUE(selected.found);
    EXPECT_EQ(selected.stamp, now);
    EXPECT_EQ(selected.frame_id, "world");
    EXPECT_DOUBLE_EQ(selected.pose.position.z, 0.16);

    CanonicalPoseSample map_pose = pose;
    map_pose.frame_id = "map";
    const CanonicalWorldPose fs150_map =
        selectSlotVisualizationWorldPose(RobotModelKind::kFs150, map_pose, now, 0.5);
    ASSERT_TRUE(fs150_map.found);
    EXPECT_EQ(fs150_map.frame_id, "world");
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kScout, map_pose, now, 0.5).found);
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kMecanum, map_pose, now, 0.5).found);

    CanonicalPoseSample namespaced_map = pose;
    namespaced_map.frame_id = "uav1/map";
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, namespaced_map, now, 0.5).found);

    CanonicalPoseSample odom_pose = pose;
    odom_pose.frame_id = "odom";
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, odom_pose, now, 0.5).found);

    CanonicalPoseSample mavros_local = pose;
    mavros_local.frame_id = "uav1/local_origin";
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, mavros_local, now, 0.5).found);

    CanonicalPoseSample missing;
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, missing, now, 0.5).found);

    CanonicalPoseSample zero_stamp = pose;
    zero_stamp.stamp = ros::Time();
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, zero_stamp, now, 0.5).found);

    CanonicalPoseSample stale = pose;
    stale.stamp = ros::Time(9, 0);
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, stale, now, 0.5).found);
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kNone, pose, now, 0.5).found);

    EXPECT_FALSE(isWorldFixedFrame("map"));
    EXPECT_FALSE(isWorldFixedFrame("odom"));
    EXPECT_TRUE(isWorldFixedFrame("/world"));
    EXPECT_EQ(slotVisualizationPoseTopic(RobotModelKind::kFs150, "/uav7"),
              "/uav7/mavros/local_position/pose");
    EXPECT_EQ(slotVisualizationPoseTopic(RobotModelKind::kScout, "/ugv1"), "/ugv1/pose");
    EXPECT_EQ(slotVisualizationPoseTopic(RobotModelKind::kMecanum, "/ugv2"), "/ugv2/pose");
    EXPECT_THROW(slotVisualizationPoseTopic(RobotModelKind::kNone, "/uav7"), std::invalid_argument);
    EXPECT_THROW(slotVisualizationPoseTopic(RobotModelKind::kFs150, "uav7"), std::invalid_argument);
    EXPECT_THROW(slotVisualizationPoseTopic(RobotModelKind::kScout, "/uav7/extra"), std::invalid_argument);

    geometry_msgs::Pose elevated;
    elevated.position.x = 2.0;
    elevated.position.z = 0.41;
    elevated.orientation.w = 1.0;
    const geometry_msgs::Pose scout_path = slotHistoryPathPose(RobotModelKind::kScout, elevated);
    EXPECT_DOUBLE_EQ(scout_path.position.x, 2.0);
    EXPECT_DOUBLE_EQ(scout_path.position.z, 0.0);
    const geometry_msgs::Pose mecanum_path = slotHistoryPathPose(RobotModelKind::kMecanum, elevated);
    EXPECT_DOUBLE_EQ(mecanum_path.position.z, 0.0);
    const geometry_msgs::Pose uav_path = slotHistoryPathPose(RobotModelKind::kFs150, elevated);
    EXPECT_DOUBLE_EQ(uav_path.position.z, 0.41);
}

TEST(WorldFixedFrameRoot, AdvertisesParentOnTfWithoutMovingDisplays) {
    const ros::Time now(10, 0);
    const geometry_msgs::TransformStamped root = worldFixedFrameRoot("world", now);
    EXPECT_EQ(root.header.frame_id, "world");
    EXPECT_EQ(root.child_frame_id, kWorldFixedFrameRootChild);
    EXPECT_EQ(root.header.stamp, now);
    EXPECT_DOUBLE_EQ(root.transform.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(root.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(root.transform.translation.z, 0.0);
    EXPECT_DOUBLE_EQ(root.transform.rotation.w, 1.0);
    const geometry_msgs::TransformStamped latched = worldFixedFrameRoot("world", ros::Time(0));
    EXPECT_TRUE(latched.header.stamp.isZero());
    EXPECT_EQ(latched.child_frame_id, kWorldFixedFrameRootChild);
}

TEST(AlgorithmOverlayFrameAlias, IdentityWorldToMapIsNotARobotBody) {
    const ros::Time now(10, 0);
    const geometry_msgs::TransformStamped alias = algorithmOverlayFrameAlias("world", now);
    EXPECT_EQ(alias.header.frame_id, "world");
    EXPECT_EQ(alias.child_frame_id, kAlgorithmOverlayFrame);
    EXPECT_EQ(alias.header.stamp, now);
    EXPECT_DOUBLE_EQ(alias.transform.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(alias.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(alias.transform.translation.z, 0.0);
    EXPECT_DOUBLE_EQ(alias.transform.rotation.w, 1.0);
    EXPECT_FALSE(isWorldFixedFrame(kAlgorithmOverlayFrame));
    EXPECT_THROW(algorithmOverlayFrameAlias("map", now), std::invalid_argument);
    EXPECT_THROW(algorithmOverlayFrameAlias("odom", now), std::invalid_argument);
}

TEST(FrozenVisualizationRosterReady, AllowsMissingSiblingPoses) {
    EXPECT_FALSE(frozenVisualizationRosterReady(0U, 0U));
    EXPECT_TRUE(frozenVisualizationRosterReady(4U, 0U));
    EXPECT_TRUE(frozenVisualizationRosterReady(4U, 1U));
    EXPECT_TRUE(frozenVisualizationRosterReady(4U, 3U));
    EXPECT_TRUE(frozenVisualizationRosterReady(4U, 4U));
    EXPECT_FALSE(frozenVisualizationRosterReady(4U, 5U));
}

TEST(UavHeightProjection, UsesWorldVerticalAndHollowGroundRingAtEveryHeight) {
    geometry_msgs::Point position;
    position.x = 4.5;
    position.y = -2.0;
    const auto color = sceneColorFromHex("#123abc");
    for (const double height : {3.0, 0.0, -0.5}) {
        position.z = height;
        const auto entity = uavHeightProjectionEntity("uav5", position, ros::Time(42, 0), "world", color);
        EXPECT_EQ(entity.id, "uav5/height_projection");
        EXPECT_EQ(entity.frame_id, "world");
        EXPECT_EQ(entity.timestamp, ros::Time(42, 0));
        EXPECT_TRUE(entity.texts.empty());
        EXPECT_TRUE(entity.models.empty());
        ASSERT_EQ(entity.lines.size(), 1U);
        const auto& vertical = entity.lines[0];
        EXPECT_EQ(vertical.type, foxglove_msgs::LinePrimitive::LINE_LIST);
        ASSERT_EQ(vertical.points.size(), 2U);
        EXPECT_DOUBLE_EQ(vertical.points[0].z, height);
        EXPECT_DOUBLE_EQ(vertical.points[1].z, 0.0);
        for (const auto& point : vertical.points) {
            EXPECT_DOUBLE_EQ(point.x, position.x);
            EXPECT_DOUBLE_EQ(point.y, position.y);
        }
        ASSERT_EQ(entity.triangles.size(), 1U);
        const auto& ring = entity.triangles[0];
        ASSERT_EQ(ring.points.size(), 96U);
        ASSERT_EQ(ring.indices.size(), 288U);
        for (std::size_t i = 0; i < ring.points.size(); ++i) {
            const auto& point = ring.points[i];
            EXPECT_NEAR(std::hypot(point.x - position.x, point.y - position.y),
                        i % 2 == 0 ? 0.18 : 0.13, 1e-12);
            EXPECT_DOUBLE_EQ(point.z, 0.0);
        }
        double area = 0.0;
        for (std::size_t i = 0; i < ring.indices.size(); i += 3) {
            const auto& a = ring.points.at(ring.indices[i]);
            const auto& b = ring.points.at(ring.indices[i + 1]);
            const auto& c = ring.points.at(ring.indices[i + 2]);
            const double twice_area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            EXPECT_GT(twice_area, 0.0);
            area += twice_area / 2.0;
            EXPECT_GT(std::hypot((a.x + b.x + c.x) / 3.0 - position.x,
                                 (a.y + b.y + c.y) / 3.0 - position.y), 0.12);
        }
        EXPECT_NEAR(area, std::acos(-1.0) * (0.18 * 0.18 - 0.13 * 0.13), 0.0002);
        EXPECT_DOUBLE_EQ(ring.pose.orientation.w, 1.0);
        EXPECT_DOUBLE_EQ(ring.color.r, color.r);
        EXPECT_DOUBLE_EQ(ring.color.a, 1.0);
        for (const auto& line : entity.lines) {
            EXPECT_DOUBLE_EQ(line.thickness, 0.02);
            EXPECT_FALSE(line.scale_invariant);
            EXPECT_DOUBLE_EQ(line.pose.orientation.w, 1.0);
            EXPECT_DOUBLE_EQ(line.pose.position.z, 0.0);
            EXPECT_DOUBLE_EQ(line.color.r, 0x12 / 255.0);
            EXPECT_DOUBLE_EQ(line.color.g, 0x3a / 255.0);
            EXPECT_DOUBLE_EQ(line.color.b, 0xbc / 255.0);
            EXPECT_DOUBLE_EQ(line.color.a, 1.0);
        }
    }
    EXPECT_THROW(uavHeightProjectionEntity("uav5", position, ros::Time(42, 0), "uav5/base_link", color), std::invalid_argument);
    position.z = std::numeric_limits<double>::infinity();
    EXPECT_THROW(uavHeightProjectionEntity("uav5", position, ros::Time(42, 0), "world", color), std::invalid_argument);
}

TEST(UavHeightProjection, SplitsLocalPositionAndOffsetVrpnWithoutFallback) {
    CanonicalPoseSample local;
    local.available = true;
    local.pose.position.x = 1.0;
    local.pose.position.y = 2.0;
    local.pose.position.z = 3.0;
    local.pose.orientation.w = 1.0;
    local.stamp = ros::Time(10, 0);
    local.frame_id = "map";

    CanonicalPoseSample vrpn_raw;
    vrpn_raw.available = true;
    vrpn_raw.pose.position.x = 10.0;
    vrpn_raw.pose.position.y = 20.0;
    vrpn_raw.pose.position.z = 4.0;
    vrpn_raw.pose.orientation.w = 1.0;
    vrpn_raw.stamp = ros::Time(10, 0);
    vrpn_raw.frame_id = "world";
    const std::array<double, 3> offset{{5.0, 6.0, 7.0}};
    CanonicalPoseSample vrpn_offset = vrpn_raw;
    vrpn_offset.pose = applyExperimentWorldOffsetOnce(vrpn_raw.pose, offset);
    EXPECT_DOUBLE_EQ(vrpn_offset.pose.position.x, 15.0);
    EXPECT_DOUBLE_EQ(vrpn_offset.pose.position.y, 26.0);
    EXPECT_DOUBLE_EQ(vrpn_offset.pose.position.z, 11.0);
    const geometry_msgs::Pose twice = applyExperimentWorldOffsetOnce(vrpn_offset.pose, offset);
    EXPECT_DOUBLE_EQ(twice.position.x, 20.0);

    const ros::Time now(10, 200000000);
    const CanonicalWorldPose three_d = selectUavHeightProjectionWorldPose(
        HeightProjectionView::kLocalPosition, local, vrpn_offset, now, 0.5);
    const CanonicalWorldPose ar = selectUavHeightProjectionWorldPose(
        HeightProjectionView::kVrpn, local, vrpn_offset, now, 0.5);
    ASSERT_TRUE(three_d.found);
    ASSERT_TRUE(ar.found);
    EXPECT_DOUBLE_EQ(three_d.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(three_d.pose.position.z, 3.0);
    EXPECT_DOUBLE_EQ(ar.pose.position.x, 15.0);
    EXPECT_DOUBLE_EQ(ar.pose.position.z, 11.0);

    CanonicalPoseSample stale_local = local;
    stale_local.stamp = ros::Time(9, 0);
    EXPECT_FALSE(selectUavHeightProjectionWorldPose(HeightProjectionView::kLocalPosition, stale_local, vrpn_offset,
                                                    now, 0.5)
                     .found);
    EXPECT_TRUE(selectUavHeightProjectionWorldPose(HeightProjectionView::kVrpn, stale_local, vrpn_offset, now, 0.5)
                    .found);

    CanonicalPoseSample missing_vrpn;
    EXPECT_TRUE(selectUavHeightProjectionWorldPose(HeightProjectionView::kLocalPosition, local, missing_vrpn, now, 0.5)
                    .found);
    EXPECT_FALSE(selectUavHeightProjectionWorldPose(HeightProjectionView::kVrpn, local, missing_vrpn, now, 0.5).found);

    const auto color = sceneColorFromHex("#123abc");
    const auto three_d_entity =
        uavHeightProjectionEntity("uav5", three_d.pose.position, three_d.stamp, "world", color);
    const auto ar_entity = uavHeightProjectionEntity("uav5", ar.pose.position, ar.stamp, "world", color);
    EXPECT_EQ(three_d_entity.id, ar_entity.id);
    EXPECT_EQ(three_d_entity.id, "uav5/height_projection");
    ASSERT_EQ(three_d_entity.lines[0].points.size(), 2U);
    EXPECT_DOUBLE_EQ(three_d_entity.lines[0].points[0].x, 1.0);
    EXPECT_DOUBLE_EQ(ar_entity.lines[0].points[0].x, 15.0);
    EXPECT_DOUBLE_EQ(three_d_entity.lines[0].thickness, 0.02);
    EXPECT_NEAR(std::hypot(ar_entity.triangles[0].points[0].x - 15.0, ar_entity.triangles[0].points[0].y - 26.0),
                0.18, 1e-12);

    const auto deletion = uavHeightProjectionDeletion("uav5", ros::Time(11, 0));
    EXPECT_EQ(deletion.type, foxglove_msgs::SceneEntityDeletion::MATCHING_ID);
    EXPECT_EQ(deletion.id, "uav5/height_projection");
}

TEST(UavHeightProjection, TopicsAreDistinctPublishers) {
    EXPECT_STREQ(kUavHeightProjectionTopic, "/xgc/uav_height_projection");
    EXPECT_STREQ(kUavHeightProjectionArTopic, "/xgc/uav_height_projection_ar");
    EXPECT_STRNE(kUavHeightProjectionTopic, kUavHeightProjectionArTopic);
}

TEST(WorldBoundary, EmptyEnvAndNullObjectAreNotDisplayable) {
    EXPECT_FALSE(parseWorldBoundaryDisplay("").displayable);
    EXPECT_FALSE(parseWorldBoundaryDisplay("  ").displayable);
    const auto unconfigured = parseWorldBoundaryDisplay(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":null,"groundZ":null})");
    EXPECT_FALSE(unconfigured.displayable);
    const auto xy_only = parseWorldBoundaryDisplay(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-12,"xMax":12,"yMin":-7,"yMax":7,"zMin":-1,"zMax":2},"groundZ":null})");
    EXPECT_FALSE(xy_only.displayable);
}

TEST(WorldBoundary, DisplayableLoopUsesGroundZNotZMin) {
    const auto boundary = parseWorldBoundaryDisplay(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-12,"xMax":12,"yMin":-7,"yMax":7,"zMin":-1,"zMax":2},"groundZ":0.18})");
    ASSERT_TRUE(boundary.displayable);
    EXPECT_DOUBLE_EQ(boundary.x_min, -12.0);
    EXPECT_DOUBLE_EQ(boundary.x_max, 12.0);
    EXPECT_DOUBLE_EQ(boundary.y_min, -7.0);
    EXPECT_DOUBLE_EQ(boundary.y_max, 7.0);
    EXPECT_DOUBLE_EQ(boundary.ground_z, 0.18);
    EXPECT_NE(boundary.ground_z, -1.0);
    const auto entity = worldBoundaryEntity(boundary, ros::Time(1, 0), "world");
    EXPECT_EQ(entity.id, kWorldBoundaryEntityId);
    EXPECT_EQ(entity.frame_id, "world");
    ASSERT_EQ(entity.lines.size(), 1U);
    EXPECT_EQ(entity.lines[0].type, foxglove_msgs::LinePrimitive::LINE_LOOP);
    EXPECT_DOUBLE_EQ(entity.lines[0].thickness, 0.06);
    EXPECT_DOUBLE_EQ(entity.lines[0].color.r, 1.0);
    EXPECT_DOUBLE_EQ(entity.lines[0].color.g, 0.2);
    EXPECT_DOUBLE_EQ(entity.lines[0].color.b, 0.15);
    EXPECT_DOUBLE_EQ(entity.lines[0].color.a, 1.0);
    ASSERT_EQ(entity.lines[0].points.size(), 4U);
    EXPECT_DOUBLE_EQ(entity.lines[0].points[0].x, -12.0);
    EXPECT_DOUBLE_EQ(entity.lines[0].points[0].y, -7.0);
    EXPECT_DOUBLE_EQ(entity.lines[0].points[0].z, 0.18);
    EXPECT_DOUBLE_EQ(entity.lines[0].points[2].x, 12.0);
    EXPECT_DOUBLE_EQ(entity.lines[0].points[2].y, 7.0);
    EXPECT_DOUBLE_EQ(entity.lines[0].points[2].z, 0.18);
    const auto hidden = worldBoundarySceneUpdate(WorldBoundaryDisplay{}, ros::Time(1, 0), "world");
    ASSERT_EQ(hidden.deletions.size(), 1U);
    EXPECT_EQ(hidden.deletions[0].id, kWorldBoundaryEntityId);
    EXPECT_TRUE(hidden.entities.empty());
}

TEST(WorldBoundary, DisplayModePublishesOnlyTheSelectedLayer) {
    const auto boundary = parseWorldBoundaryDisplay(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-12,"xMax":12,"yMin":-7,"yMax":7,"zMin":-1,"zMax":2},"groundZ":0.18})");
    const auto off = worldBoundaryLayerMessages(
        boundary, worldBoundaryDisplayModeFromString("off"), ros::Time(1, 0), "world");
    EXPECT_TRUE(off.ground.entities.empty());
    EXPECT_EQ(off.ground.deletions.size(), 1U);
    EXPECT_TRUE(off.walls.entities.empty());
    EXPECT_EQ(off.walls.deletions.size(), 1U);

    const auto ground = worldBoundaryLayerMessages(
        boundary, WorldBoundaryDisplayMode::kGround, ros::Time(1, 0), "world");
    EXPECT_EQ(ground.ground.entities.size(), 1U);
    EXPECT_TRUE(ground.ground.deletions.empty());
    EXPECT_TRUE(ground.walls.entities.empty());
    EXPECT_EQ(ground.walls.deletions.size(), 1U);

    const auto walls = worldBoundaryLayerMessages(
        boundary, WorldBoundaryDisplayMode::kWalls, ros::Time(1, 0), "world");
    EXPECT_TRUE(walls.ground.entities.empty());
    EXPECT_EQ(walls.ground.deletions.size(), 1U);
    EXPECT_EQ(walls.walls.entities.size(), 1U);
    EXPECT_TRUE(walls.walls.deletions.empty());
    EXPECT_THROW(worldBoundaryDisplayModeFromString("ceiling"), std::invalid_argument);
}

TEST(WorldBoundary, TopicsAreDistinctFromScene) {
    EXPECT_STREQ(kWorldBoundaryTopic, "/xgc/world_boundary");
    EXPECT_STREQ(kWorldBoundaryArTopic, "/xgc/world_boundary_ar");
    EXPECT_STRNE(kWorldBoundaryTopic, kIdentityArTopic);
    EXPECT_STRNE(kWorldBoundaryTopic, "/xgc/scene");
    EXPECT_STRNE(kWorldBoundaryTopic, kWorldBoundaryArTopic);
}

TEST(WorldBoundaryWalls, DisplayableBoundsBuildFourTranslucentRedWalls) {
    const auto boundary = parseWorldBoundaryDisplay(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-12,"xMax":12,"yMin":-7,"yMax":7,"zMin":-1,"zMax":2},"groundZ":0.18})");
    ASSERT_TRUE(boundary.displayable);
    EXPECT_DOUBLE_EQ(boundary.z_min, -1.0);
    EXPECT_DOUBLE_EQ(boundary.z_max, 2.0);
    const auto entity = worldWallsEntity(boundary, ros::Time(1, 0), "world");
    EXPECT_EQ(entity.id, kWorldBoundaryWallsEntityId);
    EXPECT_EQ(entity.frame_id, "world");
    ASSERT_EQ(entity.cubes.size(), 4U);

    EXPECT_DOUBLE_EQ(entity.cubes[0].pose.position.x, -12.0);
    EXPECT_DOUBLE_EQ(entity.cubes[0].pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(entity.cubes[0].pose.position.z, 1.09);
    EXPECT_DOUBLE_EQ(entity.cubes[0].size.x, 0.02);
    EXPECT_DOUBLE_EQ(entity.cubes[0].size.y, 14.0);
    EXPECT_DOUBLE_EQ(entity.cubes[0].size.z, 1.82);

    EXPECT_DOUBLE_EQ(entity.cubes[1].pose.position.x, 12.0);
    EXPECT_DOUBLE_EQ(entity.cubes[1].pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(entity.cubes[1].pose.position.z, 1.09);
    EXPECT_DOUBLE_EQ(entity.cubes[1].size.x, 0.02);
    EXPECT_DOUBLE_EQ(entity.cubes[1].size.y, 14.0);
    EXPECT_DOUBLE_EQ(entity.cubes[1].size.z, 1.82);

    EXPECT_DOUBLE_EQ(entity.cubes[2].pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(entity.cubes[2].pose.position.y, -7.0);
    EXPECT_DOUBLE_EQ(entity.cubes[2].pose.position.z, 1.09);
    EXPECT_DOUBLE_EQ(entity.cubes[2].size.x, 24.0);
    EXPECT_DOUBLE_EQ(entity.cubes[2].size.y, 0.02);
    EXPECT_DOUBLE_EQ(entity.cubes[2].size.z, 1.82);

    EXPECT_DOUBLE_EQ(entity.cubes[3].pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(entity.cubes[3].pose.position.y, 7.0);
    EXPECT_DOUBLE_EQ(entity.cubes[3].pose.position.z, 1.09);
    EXPECT_DOUBLE_EQ(entity.cubes[3].size.x, 24.0);
    EXPECT_DOUBLE_EQ(entity.cubes[3].size.y, 0.02);
    EXPECT_DOUBLE_EQ(entity.cubes[3].size.z, 1.82);

    for (const auto& wall : entity.cubes) {
        EXPECT_DOUBLE_EQ(wall.pose.orientation.w, 1.0);
        EXPECT_DOUBLE_EQ(wall.color.r, 1.0);
        EXPECT_DOUBLE_EQ(wall.color.g, 0.1);
        EXPECT_DOUBLE_EQ(wall.color.b, 0.1);
        EXPECT_DOUBLE_EQ(wall.color.a, 0.25);
    }
}

TEST(WorldBoundaryWalls, ScoutWallsClipBelowGroundWithoutChangingControlBounds) {
    const auto boundary = parseWorldBoundaryDisplay(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-12,"xMax":12,"yMin":-7,"yMax":7,"zMin":-1,"zMax":1},"groundZ":0})");
    const auto entity = worldWallsEntity(boundary, ros::Time(1, 0), "world");
    ASSERT_EQ(entity.cubes.size(), 4U);
    for (const auto& wall : entity.cubes) {
        EXPECT_DOUBLE_EQ(wall.pose.position.z - wall.size.z / 2.0, 0.0);
        EXPECT_DOUBLE_EQ(wall.pose.position.z + wall.size.z / 2.0, 1.0);
    }
    EXPECT_DOUBLE_EQ(boundary.z_min, -1.0);
    EXPECT_DOUBLE_EQ(boundary.z_max, 1.0);
    EXPECT_DOUBLE_EQ(boundary.ground_z, 0.0);
}

TEST(WorldBoundaryWalls, PositiveLowerBoundStaysAboveGround) {
    const auto boundary = parseWorldBoundaryDisplay(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-6.9,"xMax":6.9,"yMin":-4.4,"yMax":4.4,"zMin":0.2,"zMax":2.8},"groundZ":0})");
    const auto entity = worldWallsEntity(boundary, ros::Time(1, 0), "world");
    ASSERT_EQ(entity.cubes.size(), 4U);
    for (const auto& wall : entity.cubes) {
        EXPECT_NEAR(wall.pose.position.z - wall.size.z / 2.0, 0.2, 1.0e-12);
        EXPECT_NEAR(wall.pose.position.z + wall.size.z / 2.0, 2.8, 1.0e-12);
    }
}

TEST(WorldBoundaryWalls, RaisedGroundClipsAtItsWorldHeight) {
    const auto boundary = parseWorldBoundaryDisplay(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-12,"xMax":12,"yMin":-7,"yMax":7,"zMin":-1,"zMax":2},"groundZ":0.75})");
    const auto entity = worldWallsEntity(boundary, ros::Time(1, 0), "world");
    ASSERT_EQ(entity.cubes.size(), 4U);
    for (const auto& wall : entity.cubes) {
        EXPECT_DOUBLE_EQ(wall.pose.position.z - wall.size.z / 2.0, 0.75);
        EXPECT_DOUBLE_EQ(wall.pose.position.z + wall.size.z / 2.0, 2.0);
    }
}

TEST(WorldBoundaryWalls, GroundAtOrAboveCeilingReplacesWallsWithEmptyEntity) {
    auto boundary = parseWorldBoundaryDisplay(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-12,"xMax":12,"yMin":-7,"yMax":7,"zMin":-1,"zMax":1},"groundZ":0})");
    const auto visible = worldWallsSceneUpdate(boundary, ros::Time(1, 0), "world");
    ASSERT_EQ(visible.entities.size(), 1U);
    ASSERT_EQ(visible.entities[0].cubes.size(), 4U);
    for (const double ground_z : {1.0, 2.0}) {
        boundary.ground_z = ground_z;
        const auto hidden = worldWallsSceneUpdate(boundary, ros::Time(2, 0), "world");
        ASSERT_EQ(hidden.entities.size(), 1U);
        EXPECT_EQ(hidden.entities[0].id, visible.entities[0].id);
        EXPECT_EQ(hidden.entities[0].frame_id, "world");
        EXPECT_EQ(hidden.entities[0].timestamp, ros::Time(2, 0));
        EXPECT_TRUE(hidden.entities[0].cubes.empty());
        EXPECT_TRUE(hidden.deletions.empty());
    }
}

TEST(WorldBoundaryWalls, EmptyEnvDeletesWallsEntity) {
    const auto hidden = worldWallsSceneUpdate(parseWorldBoundaryDisplay(""), ros::Time(1, 0), "world");
    ASSERT_EQ(hidden.deletions.size(), 1U);
    EXPECT_EQ(hidden.deletions[0].type, foxglove_msgs::SceneEntityDeletion::MATCHING_ID);
    EXPECT_EQ(hidden.deletions[0].id, kWorldBoundaryWallsEntityId);
    EXPECT_TRUE(hidden.entities.empty());
}

TEST(WorldBoundaryWalls, TopicsAreDistinctFromBoundaryAndScene) {
    EXPECT_STREQ(kWorldBoundaryWallsTopic, "/xgc/world_boundary_walls");
    EXPECT_STREQ(kWorldBoundaryWallsArTopic, "/xgc/world_boundary_walls_ar");
    EXPECT_STRNE(kWorldBoundaryWallsTopic, kWorldBoundaryWallsArTopic);
    EXPECT_STRNE(kWorldBoundaryWallsTopic, kWorldBoundaryTopic);
    EXPECT_STRNE(kWorldBoundaryWallsTopic, kWorldBoundaryArTopic);
    EXPECT_STRNE(kWorldBoundaryWallsArTopic, kWorldBoundaryArTopic);
    EXPECT_STRNE(kWorldBoundaryWallsTopic, kIdentityArTopic);
    EXPECT_STRNE(kWorldBoundaryWallsTopic, "/xgc/scene");
}

TEST(ArIdentity, ImagePaneUsesOffsetVrpnAndIgnoresFusedLocal) {
    CanonicalPoseSample local;
    local.available = true;
    local.pose.position.x = 1.0;
    local.pose.position.y = 2.0;
    local.pose.position.z = 3.0;
    local.pose.orientation.w = 1.0;
    local.stamp = ros::Time(10, 0);
    local.frame_id = "map";

    CanonicalPoseSample vrpn;
    vrpn.available = true;
    vrpn.pose.position.x = 15.0;
    vrpn.pose.position.y = 26.0;
    vrpn.pose.position.z = 11.0;
    vrpn.pose.orientation.w = 1.0;
    vrpn.stamp = ros::Time(10, 0);
    vrpn.frame_id = "world";

    const ros::Time now(10, 200000000);
    const CanonicalWorldPose uav_ar =
        selectArIdentityWorldPose(RobotModelKind::kFs150, local, vrpn, now, 0.5);
    ASSERT_TRUE(uav_ar.found);
    EXPECT_DOUBLE_EQ(uav_ar.pose.position.x, 15.0);
    EXPECT_DOUBLE_EQ(uav_ar.pose.position.z, 11.0);

    CanonicalPoseSample missing_vrpn;
    EXPECT_FALSE(selectArIdentityWorldPose(RobotModelKind::kFs150, local, missing_vrpn, now, 0.5).found);

    CanonicalPoseSample ugv;
    ugv.available = true;
    ugv.pose.position.x = 4.0;
    ugv.pose.position.y = 5.0;
    ugv.pose.position.z = 0.2;
    ugv.pose.orientation.w = 1.0;
    ugv.stamp = ros::Time(10, 0);
    ugv.frame_id = "world";
    const CanonicalWorldPose scout_ar =
        selectArIdentityWorldPose(RobotModelKind::kScout, ugv, missing_vrpn, now, 0.5);
    ASSERT_TRUE(scout_ar.found);
    EXPECT_DOUBLE_EQ(scout_ar.pose.position.x, 4.0);
}

TEST(ArIdentity, LabelArFrameIsUprightSeparateFromFusedLabel) {
    using xgc2_ros_visualizer::robotFramePrefix;
    using xgc2_ros_visualizer::robotLabelFrame;
    const std::string label_ar = robotFramePrefix("uav1") + "/label_ar";
    EXPECT_NE(robotLabelFrame("uav1"), label_ar);

    geometry_msgs::Pose pose;
    pose.position.x = 4.0;
    pose.position.y = -2.5;
    pose.position.z = 12.0;
    pose.orientation.z = 0.5;
    pose.orientation.w = 0.5;
    const SceneLabelOffsets offsets{0.55, 0.65, 0.32};
    const auto three_d = canonicalRobotPoseTransforms(RobotModelKind::kFs150, "uav1", pose, ros::Time(7, 0), "world",
                                                      offsets);
    const auto ar = canonicalArIdentityLabelTransform(RobotModelKind::kFs150, "uav1", pose, ros::Time(7, 0),
                                                          "world", offsets);
    ASSERT_EQ(three_d.size(), 2U);
    EXPECT_EQ(three_d[1].child_frame_id, robotLabelFrame("uav1"));
    EXPECT_EQ(ar.child_frame_id, label_ar);
    EXPECT_EQ(ar.header.frame_id, "world");
    EXPECT_DOUBLE_EQ(ar.transform.translation.z, 12.55);
    EXPECT_DOUBLE_EQ(ar.transform.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(ar.transform.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(ar.transform.rotation.z, 0.0);
    EXPECT_DOUBLE_EQ(ar.transform.rotation.w, 1.0);

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(identityLabelMarker("uav1", label_ar, ros::Time(7, 0)));
    applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kFs150, "/uav1");
    foxglove_msgs::SceneUpdate update;
    appendSceneEntityPart(RobotModelKind::kFs150, "uav1", SceneEntityPart::kArLabel, markers, 0U, ros::Time(7, 0),
                           "world", blackLabelStyle(), &update);
    ASSERT_EQ(update.entities.size(), 1U);
    EXPECT_EQ(update.entities[0].id, "xgc2/px4/uav1/label_ar");
    EXPECT_EQ(update.entities[0].frame_id, label_ar);
    EXPECT_TRUE(update.entities[0].frame_locked);
    ASSERT_EQ(update.entities[0].texts.size(), 1U);
    EXPECT_EQ(update.entities[0].texts[0].text, "UAV 1");
}

TEST(ArIdentity, SceneArTopicIsNotTheFusedScene) {
    EXPECT_STREQ(kIdentityArTopic, "/xgc/scene_ar");
    EXPECT_STRNE(kIdentityArTopic, "/xgc/scene");
}

} // namespace
} // namespace xgc2_ros_visualizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#include <xgc2_ros_visualizer/instance_input.hpp>
#include <xgc2_ros_visualizer/profile.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
using namespace xgc2_ros_visualizer;

namespace {
Json::Value minimal(const std::string& kind,const std::string& package="fs150_description") {
  Json::Value document;
  document["model"]["kind"]=kind;
  document["model"]["description"]["package"]=package;
  document["model"]["description"]["file"]="urdf/robot.urdf";
  return document;
}
// A projected display Robot row: exactly the five fields the display workflow
// publishes. Source parameters stay in the frozen Session context rows.
Json::Value fullRobot(const std::string& name,const std::string& kind="px4_multirotor",const std::string& scene="fs150") {
  Json::Value result;result["slotId"]=name;result["namespace"]="/"+name;result["kind"]=kind;result["hybridSource"]="simulation";
  auto& visual=result["visualization"];visual["sceneClass"]=scene;visual["descriptionPackage"]="fs150_description";
  visual["descriptionFile"]="urdf/fs150_visual.urdf";visual["robotStatePublisher"]=false;
  visual["jointStateTopic"]="joint_states";visual["pathTopic"]="path";
  return result;
}
Json::Value ground(const std::string& name,const std::string& kind="scout_mini",const std::string& scene="scout") {
  auto result=fullRobot(name,kind,scene);
  result["visualization"]["descriptionPackage"]=scene=="mecanum"?"mecanum_description":"scout_description";
  result["visualization"]["descriptionFile"]="urdf/robot.urdf";
  return result;
}
Json::Value frozenRow(const std::string& name,const std::string& profile_id="px4-multirotor.xsim.ros",
                      const std::string& localization="") {
  Json::Value result;result["slotId"]=name;result["namespace"]="/"+name;result["source"]="simulation";
  result["profile"]["id"]=profile_id;
  if(!localization.empty())result["profile"]["parameters"]["localization_pose_topic"]=localization;
  return result;
}
void addRobot(Json::Value& value,const Json::Value& robot,const std::string& profile_id="px4-multirotor.xsim.ros",
              const std::string& localization="") {
  value["robots"].append(robot);
  const auto name=robot["slotId"].asString();
  value["context"]["robots"].append(frozenRow(name,profile_id,
      localization.empty()?"/frozen/"+name+"/pose":localization));
}
Json::Value envelope() {
  return parseJson("{\"robots\":[],\"context\":{\"runMode\":\"simulation\",\"worldBoundary\":null,\"scene\":{\"simulator\":\"xsim\"},\"localizationOffset\":{\"x\":1,\"y\":2,\"z\":3},\"robots\":[]},\"settings\":{},\"displayRelays\":[]}");
}
Json::Value fleet() {
  auto value=envelope();
  addRobot(value,fullRobot("uav1"));
  addRobot(value,fullRobot("uav2"));
  addRobot(value,ground("ugv3"),"scout-mini.xsim.ros");
  addRobot(value,ground("ugv4","mecanum_ugv","mecanum"),"mecanum-ugv.xsim.ros");
  return value;
}
// Sanitized actual HIL specimen from the failed hybrid Run: the projected
// eight-Robot roster plus the frozen Session source rows and explicit remote
// arPose settings. Unrelated Session, camera and device data were removed.
Json::Value hilInput() {
  std::ifstream input(std::string(PROFILE_TESTDATA)+"/hil-frozen-input.json");
  if(!input.good())throw std::runtime_error("missing HIL frozen input fixture");
  std::ostringstream contents;contents<<input.rdbuf();
  return parseJson(contents.str());
}
}

TEST(RobotProfile, KindDefaultsAreResolvedAndTheDocumentRoundTrips) {
  const auto uav=parseRobotProfile("uav1",minimal("fs150"));
  EXPECT_EQ(uav.scene_model,"uav1");EXPECT_TRUE(uav.hasScene());
  EXPECT_EQ(uav.pose_topic,"/uav1/mavros/local_position/pose");
  EXPECT_DOUBLE_EQ(uav.label_offset,0.55);EXPECT_DOUBLE_EQ(uav.mesh_scale,1.0);
  EXPECT_EQ(uav.path_topic,"path");EXPECT_TRUE(uav.ar_pose_topic.empty());EXPECT_TRUE(uav.ar_path_topic.empty());
  EXPECT_TRUE(uav.publication.transforms);EXPECT_TRUE(uav.publication.scene);EXPECT_TRUE(uav.publication.paths);
  EXPECT_TRUE(uav.publication.markers);EXPECT_FALSE(uav.publication.scene_paths);
  EXPECT_DOUBLE_EQ(uav.animation.rotor_airborne,60);EXPECT_DOUBLE_EQ(uav.animation.state_timeout,2.0);
  const auto scout=parseRobotProfile("ugv3",minimal("scout","scout_description"));
  EXPECT_EQ(scout.pose_topic,"/ugv3/pose");EXPECT_DOUBLE_EQ(scout.label_offset,0.65);
  EXPECT_DOUBLE_EQ(scout.animation.wheel_radius,0.08);EXPECT_DOUBLE_EQ(scout.animation.track_width,0.416);
  const auto mecanum=parseRobotProfile("ugv4",minimal("mecanum","mecanum_description"));
  EXPECT_DOUBLE_EQ(mecanum.label_offset,0.32);EXPECT_DOUBLE_EQ(mecanum.mesh_scale,0.001);
  EXPECT_DOUBLE_EQ(mecanum.animation.wheel_radius,0.05);EXPECT_DOUBLE_EQ(mecanum.animation.wheelbase_plus_track,0.30);
  const auto plain=parseRobotProfile("tool_arm",minimal("global","scout_description"));
  EXPECT_FALSE(plain.hasScene());EXPECT_TRUE(plain.scene_model.empty());EXPECT_TRUE(plain.pose_topic.empty());
  for(const auto& profile:{uav,scout,mecanum,plain}) {
    const auto dense=profile.json();
    EXPECT_EQ(parseRobotProfile(profile.id,dense),profile) << profile.id;
    EXPECT_EQ(parseRobotProfile(profile.id,dense).json(),dense) << profile.id;
  }
  // Only the animation inputs of the kind exist.
  EXPECT_TRUE(uav.json()["animation"].isMember("rotorGround"));EXPECT_FALSE(uav.json()["animation"].isMember("wheelRadius"));
  EXPECT_TRUE(scout.json()["animation"].isMember("trackWidth"));EXPECT_FALSE(scout.json()["animation"].isMember("wheelbasePlusTrack"));
  EXPECT_TRUE(mecanum.json()["animation"].isMember("wheelbasePlusTrack"));EXPECT_TRUE(plain.json()["animation"].empty());
}
TEST(RobotProfile, EverySectionOverridesItsDefaultsAndStaysStrict) {
  auto document=minimal("fs150");
  document["model"]["scene"]="ghost";document["model"]["meshScale"]=2;document["model"]["heightProjectionColor"]="#f2003c";
  document["state"]["poseTopic"]="/frozen/uav1/pose";document["state"]["arPoseTopic"]="/vrpn_client_node/uav1/pose";
  document["state"]["worldOffset"]=parseJson("[1,2,3]");
  document["frames"]["labelOffset"]=1.25;document["path"]["topic"]="track";document["path"]["arTopic"]="ar_track";
  document["labels"]["color"]="#ff8800";document["labels"]["scaleInvariant"]=true;document["labels"]["fontSize"]=20;document["labels"]["opacity"]=0.5;
  document["animation"]["rotorGround"]=10;document["animation"]["poseTimeout"]=1.5;
  document["publication"]["markers"]=false;document["publication"]["scenePaths"]=true;document["publication"]["paths"]=false;
  const auto profile=parseRobotProfile("uav1",document);
  EXPECT_EQ(profile.scene_model,"ghost");EXPECT_DOUBLE_EQ(profile.mesh_scale,2);
  EXPECT_EQ(profile.height_projection_color,"#f2003c");EXPECT_EQ(profile.pose_topic,"/frozen/uav1/pose");
  EXPECT_EQ(profile.ar_pose_topic,"/vrpn_client_node/uav1/pose");EXPECT_DOUBLE_EQ(profile.world_offset[2],3);
  EXPECT_DOUBLE_EQ(profile.label_offset,1.25);EXPECT_EQ(profile.path_topic,"track");EXPECT_EQ(profile.ar_path_topic,"ar_track");
  EXPECT_EQ(profile.labels.color,"#ff8800");EXPECT_TRUE(profile.labels.scale_invariant);EXPECT_DOUBLE_EQ(profile.labels.font_size,20);
  EXPECT_DOUBLE_EQ(profile.animation.rotor_ground,10);EXPECT_DOUBLE_EQ(profile.animation.pose_timeout,1.5);
  EXPECT_FALSE(profile.publication.markers);EXPECT_TRUE(profile.publication.scene_paths);EXPECT_FALSE(profile.publication.paths);
  EXPECT_EQ(parseRobotProfile("uav1",profile.json()),profile);
  // A path topic is not a ROS name unless it is a canonical relative one.
  document["path"]["topic"]="/absolute";EXPECT_THROW(parseRobotProfile("uav1",document),std::invalid_argument);
}
TEST(RobotProfile, RejectsWhatDoesNotBelongToTheRobot) {
  auto check=[](Json::Value document,const std::string& id="uav1") {EXPECT_THROW(parseRobotProfile(id,document),std::invalid_argument) << jsonText(document);};
  check(Json::Value(Json::arrayValue));
  auto document=minimal("fs150");document["extra"]=1;check(document);
  document=minimal("fs150");document["model"]["unknown"]=1;check(document);
  document=minimal("fs150");document["frames"]["unknown"]=1;check(document);
  document=minimal("fs150");document["model"].removeMember("kind");check(document);
  document=minimal("hovercraft");check(document);
  document=minimal("fs150");document["model"].removeMember("description");check(document);
  document=minimal("fs150");document["model"]["description"]["package"]="Not-Canonical";check(document);
  document=minimal("fs150");document["model"]["description"]["file"]="../outside.urdf";check(document);
  document=minimal("fs150");document["model"]["description"]["file"]="urdf/robot.xacro";check(document);
  document=minimal("fs150");document["model"]["description"]["jointStateTopic"]="/absolute";check(document);
  // Animation inputs and sources of another kind.
  document=minimal("scout","scout_description");document["animation"]["rotorGround"]=10;check(document,"ugv3");
  document=minimal("fs150");document["animation"]["wheelRadius"]=0.1;check(document);
  document=minimal("global");document["animation"]["poseTimeout"]=1;check(document,"tool");
  document=minimal("scout","scout_description");document["state"]["arPoseTopic"]="/vrpn/ugv3/pose";check(document,"ugv3");
  document=minimal("scout","scout_description");document["state"]["worldOffset"]=parseJson("[1,0,0]");check(document,"ugv3");
  document=minimal("scout","scout_description");document["model"]["heightProjectionColor"]="#f2003c";check(document,"ugv3");
  document=minimal("global");document["model"]["scene"]="uav1";check(document,"tool");
  // A scene robot is a canonical slot with a plain description.
  check(minimal("fs150"),"bad_slot");
  document=minimal("fs150");document["model"]["description"]["statePublisher"]=true;check(document);
  // Value ranges and types.
  document=minimal("fs150");document["labels"]["fontSize"]=11;check(document);
  document=minimal("fs150");document["labels"]["opacity"]=1.5;check(document);
  document=minimal("fs150");document["labels"]["color"]="blue";check(document);
  document=minimal("fs150");document["labels"]["color"]="#FF8800";check(document);
  document=minimal("fs150");document["frames"]["labelOffset"]=11;check(document);
  document=minimal("fs150");document["frames"]["world"]="map";check(document);
  document=minimal("fs150");document["animation"]["poseTimeout"]=-1;check(document);
  document=minimal("fs150");document["publication"]["scene"]="yes";check(document);
  document=minimal("fs150");document["model"]["heightProjectionColor"]="#F2003C";check(document);
  document=minimal("fs150");document["model"]["meshScale"]=0;check(document);
  document=minimal("fs150");document["state"]["worldOffset"]=parseJson("[1,2]");check(document);
  document=minimal("fs150");document["state"]["arPoseTopic"]="/vrpn/uav1/pose";document["path"]["arTopic"]="path";check(document);
  document=minimal("fs150");document["state"]["arPoseTopic"]="/vrpn/uav1/pose";document["path"]["arTopic"]="";check(document);
  document=minimal("fs150");document["path"]["arTopic"]="ar_path";check(document);
  EXPECT_THROW(parseRobotProfile("not/an/identifier",minimal("fs150")),std::invalid_argument);
}
TEST(RobotProfile, RemoteArPoseIsAnExplicitSourceWithoutALocalTopic) {
  auto document=minimal("fs150");
  document["state"]["sources"]["arPose"]["listen"]="tcp/127.0.0.1:17447";
  document["state"]["sources"]["arPose"]["key"]="run/uav1/arPose";
  const auto profile=parseRobotProfile("uav1",document);
  EXPECT_TRUE(profile.ar_pose_topic.empty());
  EXPECT_EQ(profile.ar_path_topic,"ar_path");
  EXPECT_EQ(profile.remote_sources.at("arPose").key,"run/uav1/arPose");
  EXPECT_EQ(parseRobotProfile("uav1",profile.json()),profile);
  document["state"]["sources"]["arPose"]["key"]="run/*";
  EXPECT_THROW(parseRobotProfile("uav1",document),std::invalid_argument);
  document["state"]["sources"]["arPose"]["key"]="run/uav1/arPose";
  document["state"]["arPoseTopic"]="/local/uav1/pose";
  EXPECT_NO_THROW(parseRobotProfile("uav1",document));
}
TEST(RobotProfile, EqualityIsOverTheResolvedValues) {
  auto sparse=minimal("fs150");
  auto dense=parseRobotProfile("uav1",sparse).json();
  EXPECT_EQ(parseRobotProfile("uav1",sparse),parseRobotProfile("uav1",dense));
  sparse["labels"]["color"]="#00a2ff";  // the default, spelled out
  EXPECT_EQ(parseRobotProfile("uav1",sparse),parseRobotProfile("uav1",dense));
  sparse["labels"]["color"]="#ff0000";
  EXPECT_NE(parseRobotProfile("uav1",sparse),parseRobotProfile("uav1",dense));
  EXPECT_NE(parseRobotProfile("uav1",dense),parseRobotProfile("uav2",dense));
}

TEST(InstanceInput, FrozenModeSourceOffsetAndNumericPaletteParity) {
  auto value=envelope();
  addRobot(value,fullRobot("uav10"),"px4-multirotor.xsim.ros","/frozen/simulation/uav10");
  addRobot(value,fullRobot("uav2"),"px4-multirotor.xsim.ros","/frozen/simulation/uav2");
  value["settings"]["uavPalette"]=parseJson("[\"#ABCDEF\",\"#123456\"]");
  for(const auto& mode:{"simulation","physical","hybrid"}) {
    value["context"]["runMode"]=mode;const auto spec=projectInstanceInput(value);
    ASSERT_EQ(spec.robots.size(),2U);
    const auto& first=spec.robots.at("uav10");const auto& second=spec.robots.at("uav2");
    EXPECT_EQ(first.height_projection_color,"#123456");EXPECT_EQ(second.height_projection_color,"#abcdef");
    EXPECT_EQ(second.ar_pose_topic,"/frozen/simulation/uav2");
    EXPECT_DOUBLE_EQ(second.world_offset[2],std::string(mode)=="physical"?3:0);
    EXPECT_EQ(second.ar_path_topic,"ar_path");
  }
  value["robots"][0].removeMember("hybridSource");EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  value["robots"][0]["hybridSource"]="physical";
  auto physical=projectInstanceInput(value);EXPECT_EQ(physical.robots.at("uav10").ar_pose_topic,"/frozen/simulation/uav10");
  EXPECT_DOUBLE_EQ(physical.robots.at("uav10").world_offset[2],3);
  value["context"]["runMode"]="simulation";value["context"]["scene"]["simulator"]="gazebo";
  value["context"]["robots"][0]["profile"]["parameters"]["mocap_rigid_body"]="uav10";
  value["context"]["robots"][1]["profile"]["parameters"]["mocap_rigid_body"]="uav2";
  auto gazebo=projectInstanceInput(value);EXPECT_EQ(gazebo.robots.at("uav10").ar_pose_topic,"/vrpn_client_node/uav10/pose");
  EXPECT_DOUBLE_EQ(gazebo.robots.at("uav10").world_offset[2],0);
  value["context"]["runMode"]="physical";value["context"]["localizationOffset"]["z"]="invalid";
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
}
TEST(InstanceInput, NoneSimulatorUsesTheFrozenDirectPoseForBothTopics) {
  auto value=envelope();
  auto robot=fullRobot("uav1");
  addRobot(value,robot,"px4-multirotor.xsim.ros","/frozen/simulation/uav1");
  value["context"]["scene"]["simulator"]="none";
  const auto direct=projectInstanceInput(value);
  const auto& admitted=direct.robots.at("uav1");
  EXPECT_EQ(admitted.ar_pose_topic,"/frozen/simulation/uav1");
  EXPECT_EQ(admitted.pose_topic,"/frozen/simulation/uav1");
  EXPECT_EQ(admitted.ar_path_topic,"ar_path");
  EXPECT_EQ(admitted.path_topic,"path");
  EXPECT_DOUBLE_EQ(admitted.world_offset[2],0);
  auto missing=value;missing["context"]["robots"][0]["profile"]["parameters"].removeMember("localization_pose_topic");
  EXPECT_THROW(projectInstanceInput(missing),std::invalid_argument);
  auto numbered=fullRobot("SQ01s");
  value["robots"][0]=numbered;
  value["context"]["robots"][0]=frozenRow("SQ01s","px4-multirotor.xsim.ros","/plant/SQ01s/pose");
  const auto plant=projectInstanceInput(value);
  EXPECT_EQ(plant.robots.at("SQ01s").pose_topic,"/plant/SQ01s/pose");
  EXPECT_EQ(plant.robots.at("SQ01s").ar_pose_topic,"/plant/SQ01s/pose");
  value["robots"][0]=robot;
  value["context"]["robots"][0]=frozenRow("uav1","px4-multirotor.xsim.ros","/frozen/simulation/uav1");
  value["context"]["scene"]["simulator"]="xsim";
  const auto xsim=projectInstanceInput(value);
  EXPECT_EQ(xsim.robots.at("uav1").ar_pose_topic,"/frozen/simulation/uav1");
  EXPECT_EQ(xsim.robots.at("uav1").pose_topic,"/uav1/mavros/local_position/pose");
  value["context"]["scene"]["simulator"]="other";
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
}
TEST(InstanceInput, HeightOffScoutBindingAndDescriptionOnlyArePreserved) {
  auto value=envelope();value["settings"]["uavHeightProjection"]=false;
  addRobot(value,fullRobot("uav1"),"px4-multirotor.xsim.ros","/frozen/uav1/pose");
  auto a=ground("ugv2");addRobot(value,a,"scout-mini.xsim.ros");
  auto b=ground("uav3");b["visualization"]["sceneModel"]="rigid";addRobot(value,b,"scout-mini.xsim.ros");
  auto plain=fullRobot("description_only","other","");plain["visualization"]["descriptionPackage"]="scout_description";
  addRobot(value,plain,"scout-mini.xsim.ros");
  auto spec=projectInstanceInput(value);
  ASSERT_EQ(spec.robots.size(),4U);
  // A ground robot of the ugv slots is named by its slot; a Scout in another slot by its explicit sceneModel.
  EXPECT_EQ(spec.robots.at("uav3").scene_model,"rigid");EXPECT_EQ(spec.robots.at("ugv2").scene_model,"ugv2");
  EXPECT_TRUE(spec.robots.at("uav1").height_projection_color.empty());
  const auto& description_only=spec.robots.at("description_only");
  EXPECT_FALSE(description_only.hasScene());EXPECT_EQ(description_only.kind,RateKind::Global);
}
TEST(InstanceInput, FrozenHilRemoteArSourceWinsWithoutLocalOrCameraFallback) {
  auto value=hilInput();
  value["context"]["localizationOffset"]["x"]=2.0;
  ASSERT_FALSE(value["context"].isMember("cameras"));  // no camera presence requirement
  const auto spec=projectInstanceInput(value);
  ASSERT_EQ(spec.robots.size(),8U);
  const auto& hil=spec.robots.at("uav3");
  EXPECT_TRUE(hil.ar_pose_topic.empty());
  EXPECT_EQ(hil.ar_path_topic,"ar_path");
  EXPECT_EQ(hil.remote_sources.at("arPose").key,
            "xgc/observations/04f8f436-3bb3-53bb-a036-eecd8eea292b/px4-03/measurement");
  EXPECT_DOUBLE_EQ(hil.world_offset[0],0); // the remote producer owns its transform
  EXPECT_EQ(parseRobotProfile("uav3",hil.json()),hil);
  EXPECT_EQ(spec.robots.at("uav1").remote_sources.at("arPose").key,
            "xgc/observations/04f8f436-3bb3-53bb-a036-eecd8eea292b/px4-01/measurement");
  // A remote source wins even if an obsolete/local topic is authored beside it.
  auto both=value;
  both["settings"]["robotProfiles"]["uav3"]["state"]["arPoseTopic"]="/local/invented/pose";
  const auto remote_spec=projectInstanceInput(both);
  const auto& remote_wins=remote_spec.robots.at("uav3");
  EXPECT_TRUE(remote_wins.ar_pose_topic.empty());
  EXPECT_EQ(remote_wins.remote_sources.at("arPose").key,hil.remote_sources.at("arPose").key);
  // The explicitly authored transform is applied once, never replaced by the
  // frozen Session localization offset.
  auto explicit_offset=value;
  explicit_offset["settings"]["robotProfiles"]["uav3"]["state"]["worldOffset"]=parseJson("[0.5,0,0]");
  EXPECT_DOUBLE_EQ(projectInstanceInput(explicit_offset).robots.at("uav3").world_offset[0],0.5);
  value["settings"]["robotProfiles"]["uav3"]["state"]["sources"]["arPose"]["key"]="run/*";
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument); // never fall back to ROS
}
TEST(InstanceInput, FrozenLocalAndSimulationSourcesKeepTheirOwnProvenance) {
  const auto captured=hilInput();
  auto physical=captured;
  physical["robots"]=Json::Value(Json::arrayValue);
  physical["robots"].append(captured["robots"][2]);
  physical["settings"]=Json::Value(Json::objectValue);
  physical["context"]["robots"]=Json::Value(Json::arrayValue);
  physical["context"]["robots"].append(captured["context"]["robots"][2]);
  physical["context"]["localizationOffset"]["x"]=2.0;
  auto real=projectInstanceInput(physical).robots.at("uav3");
  EXPECT_EQ(real.ar_pose_topic,"/vrpn_client_node_physical/uav3/pose");
  EXPECT_DOUBLE_EQ(real.world_offset[0],2.0);
  EXPECT_TRUE(real.remote_sources.empty());
  physical["context"]["robots"][0]["profile"]["parameters"].removeMember("localization_pose_topic");
  EXPECT_EQ(projectInstanceInput(physical).robots.at("uav3").ar_pose_topic,
            "/vrpn_client_node_physical/uav3/pose"); // explicit frozen root and tracker
  physical["context"]["robots"][0]["profile"]["parameters"].removeMember("mocap_source_root");
  EXPECT_THROW(projectInstanceInput(physical),std::invalid_argument); // no invented identity
  physical["settings"]["robotProfiles"]["uav3"]["state"]["arPoseTopic"]="/explicit/mocap/pose";
  EXPECT_EQ(projectInstanceInput(physical).robots.at("uav3").ar_pose_topic,"/explicit/mocap/pose");
  auto simulation=captured;
  simulation["robots"]=Json::Value(Json::arrayValue);
  simulation["robots"].append(captured["robots"][0]);
  simulation["settings"]=Json::Value(Json::objectValue);
  simulation["context"]["robots"]=Json::Value(Json::arrayValue);
  simulation["context"]["robots"].append(captured["context"]["robots"][0]);
  simulation["context"]["localizationOffset"]["x"]=2.0;
  const auto sim=projectInstanceInput(simulation).robots.at("uav1");
  EXPECT_EQ(sim.ar_pose_topic,"/uav1/pose");EXPECT_DOUBLE_EQ(sim.world_offset[0],0);
  simulation["context"]["scene"]["simulator"]="gazebo";
  simulation["context"]["robots"][0]["profile"]["parameters"]["mocap_rigid_body"]="gazebo_uav1";
  const auto gazebo=projectInstanceInput(simulation).robots.at("uav1");
  EXPECT_EQ(gazebo.ar_pose_topic,"/vrpn_client_node_simulation/gazebo_uav1/pose");
  EXPECT_DOUBLE_EQ(gazebo.world_offset[0],0);
  auto missing=simulation;missing["context"]["robots"][0]["profile"]["parameters"].removeMember("mocap_rigid_body");
  EXPECT_THROW(projectInstanceInput(missing),std::invalid_argument);
  auto no_slot=simulation;no_slot["context"]["robots"]=Json::Value(Json::arrayValue);
  EXPECT_THROW(projectInstanceInput(no_slot),std::invalid_argument);
  auto legacy=simulation;legacy["robots"][0]["px4"]["mocapRigidBodyName"]="uav1";
  EXPECT_THROW(projectInstanceInput(legacy),std::invalid_argument); // obsolete shape is explicit
}
TEST(InstanceInput, RobotsWithoutAnInstalledDescriptionAreNotMembers) {
  auto value=envelope();auto bare=fullRobot("uav1");bare["visualization"]=Json::Value();addRobot(value,bare);
  addRobot(value,fullRobot("uav2"));
  const auto spec=projectInstanceInput(value);
  EXPECT_EQ(spec.robots.size(),1U);EXPECT_EQ(spec.robots.count("uav1"),0U);
  auto invalid=envelope();addRobot(invalid,fullRobot("uav1"));invalid["robots"][0]["visualization"]["descriptionPackage"]="";
  EXPECT_THROW(projectInstanceInput(invalid),std::invalid_argument);
}
TEST(InstanceInput, RelayKindComesFromTheRobotThatOwnsTheSource) {
  auto value=envelope();addRobot(value,fullRobot("uav1"));
  value["displayRelays"]=parseJson("[{\"source\":\"/uav1/path\",\"topic\":\"/xgc/display/uav1/path\",\"messageType\":\"nav_msgs/Path\"},{\"source\":\"/map\",\"topic\":\"/xgc/display/map\",\"messageType\":\"nav_msgs/OccupancyGrid\"}]");
  const auto result=projectInstanceInput(value);
  ASSERT_EQ(result.scene.relays.size(),2U);
  EXPECT_EQ(result.scene.relays[0].kind,RateKind::Fs150);EXPECT_EQ(result.scene.relays[1].kind,RateKind::Global);
  value["displayRelays"][0]["maxRateHz"]=3;
  auto rated=projectInstanceInput(value);
  EXPECT_DOUBLE_EQ(rated.scene.relays[0].max_rate_hz,3);EXPECT_EQ(rated.scene.relays[0].role,RelayRole::None);
  value["displayRelays"][0]["maxRateHz"]=Json::nullValue;value["displayRelays"][0]["role"]=Json::nullValue;
  auto missing=projectInstanceInput(value);
  EXPECT_DOUBLE_EQ(missing.scene.relays[0].max_rate_hz,0);EXPECT_EQ(missing.scene.relays[0].role,RelayRole::None);
  value["displayRelays"][0]["role"]="prediction";
  EXPECT_EQ(projectInstanceInput(value).scene.relays[0].role,RelayRole::Prediction);
  value["displayRelays"][0]["role"]="obstacle";value["displayRelays"][0]["maxRateHz"]=8;
  EXPECT_EQ(projectInstanceInput(value).scene.relays[0].role,RelayRole::Obstacle);
  value["displayRelays"][0]["role"]="semantic";
  EXPECT_EQ(projectInstanceInput(value).scene.relays[0].role,RelayRole::Semantic);
  value["displayRelays"][0]["role"]="unknown";EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  value["displayRelays"][0]["role"]="prediction";value["displayRelays"][0]["maxRateHz"]=0;
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  value["displayRelays"][0]["maxRateHz"]=101;EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  value["displayRelays"][0].removeMember("maxRateHz");value["displayRelays"][0].removeMember("role");
  value["displayRelays"][0]["robotKind"]="scout";EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  value["displayRelays"][0].removeMember("robotKind");
  value["displayRelays"].append(value["displayRelays"][0]);EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  EXPECT_NO_THROW(projectInstanceInput(envelope()));
}
TEST(InstanceInput, RejectsRetiredEnvelopeAndValidatesPublicationControls) {
  auto value=envelope();value["instanceId"]="old";
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);value.removeMember("instanceId");
  value["descriptions"]=Json::Value(Json::arrayValue);
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);value.removeMember("descriptions");
  value["settings"]["publication"]["scene"]=false;
  value["settings"]["publication"]["transforms"]=false;
  auto spec=projectInstanceInput(value);
  EXPECT_FALSE(spec.scene.publish_scene);EXPECT_FALSE(spec.scene.publish_transforms);EXPECT_FALSE(spec.scene.publish_markers);
  value["settings"]["publication"]["scene"]="false";
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  value["settings"]["publication"]["scene"]=false;
  value["settings"]["publication"]["alias"]=false;
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
}
TEST(InstanceInput, HistoryAndPredictionCeilingsStayOutOfTheRobotProfile) {
  auto value=fleet();
  const auto defaults=projectInstanceInput(value);
  EXPECT_DOUBLE_EQ(defaults.history_max_rate_hz,2);EXPECT_TRUE(defaults.history_visible);
  EXPECT_DOUBLE_EQ(defaults.prediction_max_rate_hz,2);
  EXPECT_TRUE(diffInstance(defaults,projectInstanceInput(fleet())).empty());
  value["settings"]["historyMaxRateHz"]=2;value["settings"]["historyVisible"]=false;value["settings"]["predictionMaxRateHz"]=4;
  value["settings"]["historyShowPoseAxes"]=true;value["settings"]["predictionShowPoseAxes"]=false;
  const auto tuned=projectInstanceInput(value);
  EXPECT_DOUBLE_EQ(tuned.history_max_rate_hz,2);EXPECT_FALSE(tuned.history_visible);EXPECT_DOUBLE_EQ(tuned.prediction_max_rate_hz,4);
  const auto change=diffInstance(defaults,tuned);
  EXPECT_TRUE(change.settings);EXPECT_TRUE(change.rebuilt.empty());EXPECT_FALSE(change.scene);EXPECT_EQ(change.relays_added,0U);
  value["settings"]["historyMaxRateHz"]=0;EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  value["settings"]["historyMaxRateHz"]=10;value["settings"]["predictionMaxRateHz"]=100.1;
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  value["settings"]["predictionMaxRateHz"]=5;value["settings"]["historyVisible"]="yes";
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
}
TEST(InstanceInput, GroundSceneSwitchKeepsDescriptionsAndKinds) {
  auto value=fleet();value["settings"]["publication"]["groundScene"]=false;
  const auto spec=projectInstanceInput(value);
  EXPECT_TRUE(spec.robots.at("uav1").hasScene());
  EXPECT_FALSE(spec.robots.at("ugv3").hasScene());EXPECT_EQ(spec.robots.at("ugv3").kind,RateKind::Scout);
  EXPECT_FALSE(spec.robots.at("ugv4").hasScene());EXPECT_EQ(spec.robots.at("ugv4").kind,RateKind::Mecanum);
}
TEST(InstanceInput, InvalidSceneLabelNamespaceFailsBeforeActivation) {
  auto value=envelope();value["robots"].append(fullRobot("bad slot"));
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  EXPECT_THROW(projectInstanceInput(parseJson("{\"robots\":[{\"namespace\":\"/a/b\",\"kind\":\"x\"}],\"context\":{\"runMode\":\"simulation\"},\"settings\":{},\"displayRelays\":[]}")),std::invalid_argument);
}
TEST(InstanceInput, PanelStyleReachesEveryRobotAndSavedProfilesWin) {
  auto value=fleet();
  value["settings"]["markerColor"]="#112233";value["settings"]["labelFontSizeMeters"]=0.5;value["settings"]["markerOpacity"]=0.75;
  value["settings"]["uavLabelOffset"]=0.8;value["settings"]["scoutLabelOffset"]=0.9;
  value["settings"]["publication"]["scenePaths"]=true;value["settings"]["publication"]["markers"]=true;
  auto spec=projectInstanceInput(value);
  for(const auto& entry:spec.robots) {
    EXPECT_EQ(entry.second.labels.color,"#112233");EXPECT_DOUBLE_EQ(entry.second.labels.font_size,0.5);
    EXPECT_DOUBLE_EQ(entry.second.labels.opacity,0.75);EXPECT_TRUE(entry.second.publication.scene_paths);
  }
  EXPECT_DOUBLE_EQ(spec.robots.at("uav1").label_offset,0.8);EXPECT_DOUBLE_EQ(spec.robots.at("ugv3").label_offset,0.9);
  EXPECT_DOUBLE_EQ(spec.robots.at("ugv4").label_offset,0.32);
  EXPECT_TRUE(spec.scene.publish_markers);
  // The saved profile of one robot is applied over the panel and the Robot facts.
  value["robots"][1]["visualization"]["profile"]["labels"]["color"]="#ff0000";
  value["robots"][1]["visualization"]["profile"]["animation"]["rotorGround"]=5;
  value["robots"][1]["visualization"]["profile"]["publication"]["paths"]=false;
  auto saved=projectInstanceInput(value);
  EXPECT_EQ(saved.robots.at("uav2").labels.color,"#ff0000");EXPECT_DOUBLE_EQ(saved.robots.at("uav2").animation.rotor_ground,5);
  EXPECT_FALSE(saved.robots.at("uav2").publication.paths);
  EXPECT_EQ(saved.robots.at("uav1").labels.color,"#112233");
  value["robots"][1]["visualization"]["profile"]["labels"]["fontSize"]=99;
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
  auto bad=fleet();bad["settings"]["labelFontSizePixels"]=0;EXPECT_THROW(projectInstanceInput(bad),std::invalid_argument);
  bad=fleet();bad["settings"]["uavLabelOffset"]=11;EXPECT_THROW(projectInstanceInput(bad),std::invalid_argument);
}

TEST(InstanceDiff, OnlyTheRobotsWhoseProfileDiffersAreRebuilt) {
  const auto base=projectInstanceInput(fleet());
  EXPECT_TRUE(diffInstance(base,projectInstanceInput(fleet())).empty());
  EXPECT_EQ(diffInstance(base,projectInstanceInput(fleet())).unchanged.size(),4U);
  // One saved profile changes.
  auto value=fleet();value["robots"][1]["visualization"]["profile"]["labels"]["color"]="#ff0000";
  auto change=diffInstance(base,projectInstanceInput(value));
  EXPECT_EQ(change.rebuilt,std::vector<std::string>{"uav2"});EXPECT_TRUE(change.added.empty());EXPECT_TRUE(change.removed.empty());
  EXPECT_EQ(change.unchanged.size(),3U);EXPECT_FALSE(change.scene);
  // A panel setting reaches the robots it applies to, and only those.
  value=fleet();value["settings"]["scoutLabelOffset"]=0.7;
  change=diffInstance(base,projectInstanceInput(value));
  EXPECT_EQ(change.rebuilt,std::vector<std::string>{"ugv3"});
  value=fleet();value["settings"]["markerColor"]="#ff0000";
  EXPECT_EQ(diffInstance(base,projectInstanceInput(value)).rebuilt.size(),4U);
  // Membership.
  value=fleet();addRobot(value,fullRobot("uav5"));
  change=diffInstance(base,projectInstanceInput(value));
  EXPECT_EQ(change.added,std::vector<std::string>{"uav5"});EXPECT_TRUE(change.rebuilt.empty());
  value=fleet();value["robots"].resize(2);
  change=diffInstance(base,projectInstanceInput(value));
  EXPECT_EQ(change.removed,(std::vector<std::string>{"ugv3","ugv4"}));EXPECT_EQ(change.unchanged.size(),2U);
  // The same Robot with a new AR source (physical to simulation) is a new profile.
  value=fleet();value["context"]["runMode"]="physical";
  EXPECT_EQ(diffInstance(base,projectInstanceInput(value)).rebuilt,(std::vector<std::string>{"uav1","uav2"}));
}
TEST(InstanceDiff, SceneSwitchesBoundaryAndRelaysAreNotRobotRebuilds) {
  const auto base=projectInstanceInput(fleet());
  auto value=fleet();value["settings"]["publication"]["markers"]=true;
  // The scene-wide switch changes the shared outputs; the robots follow their own publication flags.
  auto change=diffInstance(base,projectInstanceInput(value));
  EXPECT_TRUE(change.scene);
  value=fleet();value["context"]["worldBoundary"]=parseJson("{\"schemaVersion\":1,\"controlBounds\":{\"xMin\":-1,\"xMax\":1,\"yMin\":-1,\"yMax\":1,\"zMin\":0,\"zMax\":2},\"groundZ\":0}");
  change=diffInstance(base,projectInstanceInput(value));
  EXPECT_TRUE(change.scene);EXPECT_TRUE(change.rebuilt.empty());
  value=fleet();value["settings"]["worldBoundaryMode"]="ground";
  EXPECT_TRUE(diffInstance(base,projectInstanceInput(value)).scene);
  value=fleet();value["displayRelays"]=parseJson("[{\"source\":\"/map\",\"topic\":\"/xgc/display/map\",\"messageType\":\"nav_msgs/OccupancyGrid\"}]");
  const auto with_relay=projectInstanceInput(value);
  change=diffInstance(base,with_relay);
  EXPECT_FALSE(change.scene);EXPECT_EQ(change.relays_added,1U);EXPECT_EQ(change.relays_removed,0U);EXPECT_TRUE(change.rebuilt.empty());
  value["displayRelays"][0]["messageType"]="nav_msgs/Path";value["displayRelays"][0]["source"]="/map";
  value["displayRelays"][0]["messageType"]="geometry_msgs/PoseArray";
  change=diffInstance(with_relay,projectInstanceInput(value));
  EXPECT_EQ(change.relays_added,1U);EXPECT_EQ(change.relays_removed,1U);
  change=diffInstance(with_relay,base);
  EXPECT_EQ(change.relays_removed,1U);EXPECT_EQ(change.relays_added,0U);
}
TEST(InstanceDiff, ChangeReportNamesEveryOutcome) {
  const auto base=projectInstanceInput(fleet());
  auto value=fleet();value["robots"][0]["visualization"]["profile"]["labels"]["opacity"]=0.5;addRobot(value,fullRobot("uav5"));
  const auto report=diffInstance(base,projectInstanceInput(value)).json();
  EXPECT_EQ(report["rebuilt"][0].asString(),"uav1");EXPECT_EQ(report["added"][0].asString(),"uav5");
  EXPECT_TRUE(report["removed"].empty());EXPECT_EQ(report["unchanged"].size(),3U);EXPECT_FALSE(report["scene"].asBool());
}

TEST(InstanceRules, ClaimsAndConsistencyChecks) {
  auto spec=projectInstanceInput(fleet());
  const auto claimed=claims(spec);
  for(const char* claim:{"scene-frame-tree","scene-global-topics","topic:/tf_static","topic:/uav1/path","topic:/uav1/ar_path","topic:/uav2/path","topic:/ugv3/path","param:/uav1/visual_robot_description","topic:/uav1/visual_robot_description"})
    EXPECT_TRUE(claimed.count(claim)) << claim;
  EXPECT_FALSE(claimed.count("topic:/markers"));
  spec.scene.publish_markers=true;EXPECT_TRUE(claims(spec).count("topic:/markers"));
  spec.robots.at("uav1").publication.paths=false;EXPECT_FALSE(claims(spec).count("topic:/uav1/path"));
  // Relay-only instances claim nothing of the scene.
  auto relay_only=envelope();relay_only["settings"]["publication"]["scene"]=false;relay_only["settings"]["publication"]["transforms"]=false;
  relay_only["displayRelays"]=parseJson("[{\"source\":\"/map\",\"topic\":\"/xgc/display/map\",\"messageType\":\"nav_msgs/OccupancyGrid\"}]");
  EXPECT_EQ(claims(projectInstanceInput(relay_only)),std::set<std::string>{"topic:/xgc/display/map"});
  // Two robots cannot show one scene model, or publish one path topic.
  auto duplicate=projectInstanceInput(fleet());duplicate.robots.at("uav2")=duplicate.robots.at("uav1");duplicate.robots.at("uav2").id="uav2";
  EXPECT_THROW(checkInstance(duplicate),std::invalid_argument);
  InstanceSpec collision;
  collision.robots["uav1"]=parseRobotProfile("uav1",minimal("fs150"));
  collision.robots["uav2"]=parseRobotProfile("uav2",minimal("fs150"));
  EXPECT_NO_THROW(checkInstance(collision));
  InstanceSpec many;
  for(int i=0;i<=static_cast<int>(kMaxRobots);++i) many.robots["uav"+std::to_string(i)]=parseRobotProfile("uav"+std::to_string(i),minimal("fs150"));
  EXPECT_THROW(checkInstance(many),std::invalid_argument);
  InstanceSpec relays;
  for(int i=0;i<static_cast<int>(kMaxRelays);++i)
    relays.scene.relays.push_back(relayConfig("/cloud_"+std::to_string(i),"/xgc/display/cloud_"+std::to_string(i),"sensor_msgs/PointCloud2",RateKind::Global));
  EXPECT_NO_THROW(checkInstance(relays));
  relays.scene.relays.push_back(relayConfig("/cloud_extra","/xgc/display/cloud_extra","sensor_msgs/PointCloud2",RateKind::Global));
  EXPECT_THROW(checkInstance(relays),std::invalid_argument);
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

TEST(RobotProfile, ExplicitZenohSourcesRoundTripAndRebuildOnlyTheirRobot) {
  auto document=minimal("fs150");
  document["state"]["arPoseTopic"]="/vrpn_client_node/uav1/pose";
  document["state"]["sources"]["pose"]["listen"]="tcp/127.0.0.1:17447";
  document["state"]["sources"]["pose"]["key"]="run/uav1/local_pose";
  const auto original=parseRobotProfile("uav1",document);
  EXPECT_EQ(original.remote_sources.at("pose").key,"run/uav1/local_pose");
  EXPECT_EQ(parseRobotProfile("uav1",original.json()),original);
  InstanceSpec before,after; before.robots.emplace("uav1",original); after=before;
  document["state"]["sources"]["pose"]["key"]="run/uav1/new_pose";
  after.robots["uav1"]=parseRobotProfile("uav1",document);
  EXPECT_EQ(diffInstance(before,after).rebuilt,std::vector<std::string>{"uav1"});
  document["state"]["sources"]["pose"]["key"]="run/**";
  EXPECT_THROW(parseRobotProfile("uav1",document),std::invalid_argument);
  document["state"]["sources"]["pose"]["key"]="run/uav1/pose";
  document["state"]["sources"]["pose"]["listen"]="udp/127.0.0.1:17447";
  EXPECT_THROW(parseRobotProfile("uav1",document),std::invalid_argument);
  document["state"]["sources"]["pose"]["listen"]="tcp/127.0.0.1:17447";
  document["state"]["sources"]["twist"]=document["state"]["sources"]["pose"];
  EXPECT_THROW(parseRobotProfile("uav1",document),std::invalid_argument);
}

TEST(InstanceInput, RemoteDisplaySourceIsExplicitAndChangesRelayIdentity) {
  auto value=fleet();
  auto relay=parseJson("{\"source\":\"/uav1/alg/prediction\",\"topic\":\"/xgc/display/uav1/alg/prediction\",\"messageType\":\"nav_msgs/Path\",\"role\":\"prediction\",\"zenoh\":{\"listen\":\"tcp/127.0.0.1:17447\",\"key\":\"run/uav1/prediction\"}}");
  value["displayRelays"].append(relay);
  auto before=projectInstanceInput(value);
  EXPECT_EQ(before.scene.relays.at(0).zenoh_key,"run/uav1/prediction");
  value["displayRelays"][0]["zenoh"]["key"]="run/uav1/other";
  const auto change=diffInstance(before,projectInstanceInput(value));
  EXPECT_EQ(change.relays_added,1u); EXPECT_EQ(change.relays_removed,1u);
  value["displayRelays"][0]["zenoh"]["key"]="run/*";
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
}

TEST(InstanceInput, EveryRemoteInputSharesOneListener) {
  auto value=fleet();
  auto remote=parseJson("{\"listen\":\"tcp/192.0.2.10:17447\",\"key\":\"run/uav1/pose\"}");
  value["robots"][0]["visualization"]["profile"]["state"]["sources"]["pose"]=remote;
  value["robots"][1]["visualization"]["profile"]["state"]["sources"]["pose"]=remote;
  auto spec=projectInstanceInput(value);
  EXPECT_EQ(remoteListen(spec),"tcp/192.0.2.10:17447");
  value["robots"][1]["visualization"]["profile"]["state"]["sources"]["pose"]["listen"]="tcp/192.0.2.10:17448";
  EXPECT_THROW(projectInstanceInput(value),std::invalid_argument);
}

TEST(InstanceInput, SettingsRobotProfilesOverrideExactlyTheNamedFrozenRobot) {
  auto value=fleet();
  value["settings"]["robotProfiles"]["uav1"]["state"]["sources"]["pose"]=parseJson("{\"listen\":\"tcp/192.0.2.10:17447\",\"key\":\"run/uav1/pose\"}");
  const auto spec=projectInstanceInput(value);
  EXPECT_EQ(spec.robots.at("uav1").remote_sources.at("pose").key,"run/uav1/pose");
  EXPECT_TRUE(spec.robots.at("uav2").remote_sources.empty());
  auto changed=value;changed["settings"]["robotProfiles"]["uav1"]["state"]["sources"]["pose"]["key"]="run/uav1/other";
  EXPECT_EQ(diffInstance(spec,projectInstanceInput(changed)).rebuilt,std::vector<std::string>{"uav1"});
  changed=value;changed["settings"]["robotProfiles"]["/uav1"]=Json::Value(Json::objectValue);
  EXPECT_THROW(projectInstanceInput(changed),std::invalid_argument);
  changed=value;changed["settings"]["robotProfiles"]["absent"]=Json::Value(Json::objectValue);
  EXPECT_THROW(projectInstanceInput(changed),std::invalid_argument);
  changed=value;changed["settings"]["robotProfiles"]["uav1"]="bad";
  EXPECT_THROW(projectInstanceInput(changed),std::invalid_argument);
  changed=value;changed["settings"]["robotProfiles"]["uav1"]["bogus"]=1;
  EXPECT_THROW(projectInstanceInput(changed),std::invalid_argument);
  changed=value;changed["robots"][0]["visualization"].removeMember("descriptionPackage");
  EXPECT_THROW(projectInstanceInput(changed),std::invalid_argument);
}

TEST(InstanceInput, SettingsSelectsDeclaredDisplayRelaySources) {
  auto value=fleet();
  value["displayRelays"].append(parseJson("{\"source\":\"/uav1/alg/prediction\",\"topic\":\"/xgc/display/uav1/alg/prediction\",\"messageType\":\"nav_msgs/Path\"}"));
  value["settings"]["displayRelaySources"]["/uav1/alg/prediction"]=parseJson("{\"listen\":\"tcp/192.0.2.10:17447\",\"key\":\"run/uav1/path\"}");
  const auto spec=projectInstanceInput(value);
  EXPECT_EQ(spec.scene.relays.at(0).zenoh_key,"run/uav1/path");
  auto bad=value;bad["settings"]["displayRelaySources"]["/missing"]=bad["settings"]["displayRelaySources"]["/uav1/alg/prediction"];
  EXPECT_THROW(projectInstanceInput(bad),std::invalid_argument);
  bad=value;bad["settings"]["displayRelaySources"]["/uav1/alg/prediction"]["key"]="run/*";
  EXPECT_THROW(projectInstanceInput(bad),std::invalid_argument);
}

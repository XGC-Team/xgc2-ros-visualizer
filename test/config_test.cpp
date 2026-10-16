#include <xgc2_ros_visualizer/config.hpp>
#include <xgc2_ros_visualizer/declared_wire.hpp>
#include <xgc2_ros_visualizer/source_history.hpp>
#include "relay/marker_coalesce.hpp"
#include "relay/display_snapshot.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <gtest/gtest.h>
#include <set>
#include <stdexcept>
using namespace xgc2_ros_visualizer;
TEST(DisplaySnapshot, IgnoresOnlySequenceAndTimeOfAnUnchangedWorldMap) {
  sensor_msgs::PointCloud2 cloud; cloud.header.frame_id="world"; cloud.width=1;
  cloud.data={1,2,3};
  auto bytes=[](const sensor_msgs::PointCloud2& value) {
    std::vector<std::uint8_t> out(ros::serialization::serializationLength(value));
    ros::serialization::OStream stream(out.data(),out.size()); ros::serialization::serialize(stream,value);
    return out;
  };
  const auto original=bytes(cloud);
  cloud.header.seq=12; cloud.header.stamp=ros::Time(5,123);
  EXPECT_TRUE(sameWorldSnapshot(original,bytes(cloud)));
  cloud.data[0]=4; EXPECT_FALSE(sameWorldSnapshot(original,bytes(cloud)));
  cloud.data[0]=1; cloud.width=2; EXPECT_FALSE(sameWorldSnapshot(original,bytes(cloud)));
  cloud.width=1; cloud.header.frame_id="other";
  const auto moving=bytes(cloud); cloud.header.stamp=ros::Time(6,123);
  EXPECT_FALSE(sameWorldSnapshot(moving,bytes(cloud)));
  EXPECT_FALSE(sameWorldSnapshot(original,moving));
  EXPECT_FALSE(sameWorldSnapshot({},{}));
}
TEST(Rates, CompleteRoundTripAndKindIsolation) {
  auto original=defaultRates();auto json=original.json();json["scout"]["path"]=2.5;
  auto changed=parseRates(json,true);
  EXPECT_DOUBLE_EQ(changed.get(RateKind::Scout,Channel::Path),2.5);
  EXPECT_DOUBLE_EQ(changed.get(RateKind::Fs150,Channel::Path),10);
  EXPECT_DOUBLE_EQ(changed.get(RateKind::Mecanum,Channel::Path),10);
  EXPECT_DOUBLE_EQ(changed.get(RateKind::Global,Channel::JointTf),30);
  EXPECT_FALSE(applicable(RateKind::Global,Channel::PoseTf));
  EXPECT_EQ(changed.json(),json);
}
TEST(Rates, StartupOverlayAndStrictAtomicCandidateValidation) {
  auto empty=Json::Value(Json::objectValue);
  EXPECT_EQ(parseRates(empty,false).json(),defaultRates().json());
  EXPECT_THROW(parseRates(empty,true),std::invalid_argument);
  for(const auto& text:{"{\"scout\":{\"height_projection\":10}}","{\"global\":{\"pose_tf\":10}}","{\"fs150\":{\"path\":true}}","{\"fs150\":{\"path\":0}}","{\"fs150\":{\"path\":1001}}","{\"fs150\":{\"unknown\":10}}"})
    EXPECT_THROW(parseRates(parseJson(text),false),std::invalid_argument);
  auto original=defaultRates();auto bad=original.json();bad["scout"]["path"]=-1;
  EXPECT_THROW(parseRates(bad,true),std::invalid_argument);
  EXPECT_EQ(original.json(),defaultRates().json());
  EXPECT_THROW(parseJson("{\"x\":1,\"x\":2}"),std::invalid_argument);
}
TEST(RateOverrides, FollowTheKindTableUntilARobotOverridesAChannel) {
  const auto overrides=parseRateOverrides(RateKind::Fs150,parseJson("{\"path\":5,\"markers\":20}"));
  EXPECT_FALSE(overrides.empty());
  EXPECT_DOUBLE_EQ(overrides.get(Channel::Path,10),5);
  EXPECT_DOUBLE_EQ(overrides.get(Channel::Markers,30),20);
  EXPECT_DOUBLE_EQ(overrides.get(Channel::Scene,10),10);
  EXPECT_DOUBLE_EQ(overrides.maximum(),20);
  EXPECT_EQ(overrides.json(),parseJson("{\"path\":5.0,\"markers\":20.0}"));
  EXPECT_TRUE(RateOverrides().empty());
  EXPECT_NE(overrides,RateOverrides());
  EXPECT_EQ(parseRateOverrides(RateKind::Scout,Json::Value(Json::objectValue)),RateOverrides());
}
TEST(RateOverrides, OnlyTheChannelsOfTheRobotKindWithinTheRateRange) {
  // Height projection belongs to FS150 only, the root TF to the scene, relays to the instance.
  EXPECT_THROW(parseRateOverrides(RateKind::Scout,parseJson("{\"height_projection\":10}")),std::invalid_argument);
  EXPECT_THROW(parseRateOverrides(RateKind::Fs150,parseJson("{\"tf_root\":10}")),std::invalid_argument);
  EXPECT_THROW(parseRateOverrides(RateKind::Fs150,parseJson("{\"display_path\":10}")),std::invalid_argument);
  EXPECT_THROW(parseRateOverrides(RateKind::Fs150,parseJson("{\"display_marker\":10}")),std::invalid_argument);
  EXPECT_THROW(parseRateOverrides(RateKind::Global,parseJson("{\"display_marker_array\":10}")),std::invalid_argument);
  EXPECT_THROW(parseRateOverrides(RateKind::Global,parseJson("{\"pose_tf\":10}")),std::invalid_argument);
  EXPECT_NO_THROW(parseRateOverrides(RateKind::Global,parseJson("{\"joint_tf\":10}")));
  for(const auto& text:{"{\"path\":0}","{\"path\":1001}","{\"path\":\"fast\"}","{\"path\":true}","{\"unknown\":5}"})
    EXPECT_THROW(parseRateOverrides(RateKind::Fs150,parseJson(text)),std::invalid_argument) << text;
  EXPECT_THROW(parseRateOverrides(RateKind::Fs150,parseJson("[1]")),std::invalid_argument);
}
TEST(RelayConfig, KindRowOwnsTheRateAndDisplayTypesAreAccepted) {
  auto spec=relayConfig("/ugv8/path","/xgc/display/ugv8/path","nav_msgs/Path",RateKind::Scout);
  EXPECT_EQ(spec.kind,RateKind::Scout);EXPECT_EQ(spec.channel,Channel::DisplayPath);
  EXPECT_EQ(spec.max_rate_hz,0);EXPECT_EQ(spec.role,RelayRole::None);
  EXPECT_EQ(relayConfig("/map","/xgc/display/map","nav_msgs/OccupancyGrid",RateKind::Global).channel,Channel::DisplayGrid);
  EXPECT_EQ(relayConfig("/c","/xgc/display/c","sensor_msgs/PointCloud2",RateKind::Global).channel,Channel::DisplayPointcloud);
  EXPECT_EQ(relayConfig("/p","/xgc/display/p","geometry_msgs/PoseArray",RateKind::Global).channel,Channel::DisplayPoseArray);
  auto marks=relayConfig("/marks","/xgc/display/marks","visualization_msgs/MarkerArray",RateKind::Scout,2,RelayRole::Prediction);
  EXPECT_EQ(marks.channel,Channel::DisplayMarkerArray);EXPECT_DOUBLE_EQ(marks.max_rate_hz,2);EXPECT_EQ(marks.role,RelayRole::Prediction);
  EXPECT_EQ(relayConfig("/one","/xgc/display/one","visualization_msgs/Marker",RateKind::Global).channel,Channel::DisplayMarker);
  EXPECT_THROW(relayConfig("/ugv8/path","/xgc/display/ugv8/path","std_msgs/String",RateKind::Scout),std::invalid_argument);
  EXPECT_THROW(relayConfig("/ugv8/path","/xgc/display/ugv8/path","nav_msgs/Path",RateKind::Scout,0.05),std::invalid_argument);
  EXPECT_THROW(relayConfig("/ugv8/path","/xgc/display/ugv8/path","nav_msgs/Path",RateKind::Scout,101),std::invalid_argument);
  EXPECT_THROW(relayConfig("/ugv8/path","/elsewhere/ugv8/path","nav_msgs/Path",RateKind::Scout),std::invalid_argument);
  EXPECT_THROW(relayConfig("/xgc/display/ugv8","/xgc/display/xgc/display/ugv8","nav_msgs/Path",RateKind::Scout),std::invalid_argument);
  EXPECT_THROW(relayConfig("ugv8/path","/xgc/displayugv8/path","nav_msgs/Path",RateKind::Scout),std::invalid_argument);
}
TEST(RelayCeiling, DeclarationIsACeilingAndPredictionUsesThePanelCap) {
  const auto table=defaultRates();
  auto path=relayConfig("/planner/path","/xgc/display/planner/path","nav_msgs/Path",RateKind::Global,100,RelayRole::None);
  EXPECT_DOUBLE_EQ(relayPublishHz(table,path,5),10);
  path.max_rate_hz=2;
  EXPECT_DOUBLE_EQ(relayPublishHz(table,path,5),2);
  path.max_rate_hz=0;path.role=RelayRole::Prediction;
  EXPECT_DOUBLE_EQ(relayPublishHz(table,path,5),5);
  path.max_rate_hz=2;
  EXPECT_DOUBLE_EQ(relayPublishHz(table,path,5),2);
  path.max_rate_hz=100;
  EXPECT_DOUBLE_EQ(relayPublishHz(table,path,5),5);
  auto cloud=relayConfig("/map/voxels","/xgc/display/map/voxels","sensor_msgs/PointCloud2",RateKind::Global,8,RelayRole::Obstacle);
  EXPECT_DOUBLE_EQ(relayPublishHz(table,cloud,2),8);
  cloud.role=RelayRole::Semantic;cloud.max_rate_hz=0;
  EXPECT_DOUBLE_EQ(relayPublishHz(table,cloud,2),10);
  auto lowered=table;lowered.values[static_cast<std::size_t>(RateKind::Global)][static_cast<std::size_t>(Channel::DisplayPointcloud)]=4;
  cloud.max_rate_hz=100;
  EXPECT_DOUBLE_EQ(relayPublishHz(lowered,cloud,2),4);
  EXPECT_DOUBLE_EQ(historyPublishHz(20,10),10);
  EXPECT_DOUBLE_EQ(historyPublishHz(2,10),2);
  EXPECT_DOUBLE_EQ(historyPublishHz(5,10),5);
}
visualization_msgs::Marker marker(const std::string& ns,int id,uint8_t action) {
  visualization_msgs::Marker message;message.ns=ns;message.id=id;message.action=action;message.pose.position.x=id;return message;
}
TEST(MarkerCoalesce, DifferentIdsAndDeletesKeepOrderWithoutGhosts) {
  MarkerCoalescer pending;
  pending.ingest(marker("probe",1,visualization_msgs::Marker::ADD));
  pending.ingest(marker("probe",2,visualization_msgs::Marker::ADD));
  auto sent=pending.take();
  ASSERT_EQ(sent.size(),2U);
  EXPECT_EQ(sent[0].id,1);EXPECT_EQ(sent[1].id,2);
  pending.ingest(marker("probe",1,visualization_msgs::Marker::DELETE));
  pending.ingest(marker("probe",3,visualization_msgs::Marker::ADD));
  sent=pending.take();
  ASSERT_EQ(sent.size(),2U);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::DELETE);EXPECT_EQ(sent[0].id,1);
  EXPECT_EQ(sent[1].action,visualization_msgs::Marker::ADD);EXPECT_EQ(sent[1].id,3);
  pending.ingest(marker("probe",9,visualization_msgs::Marker::ADD));
  pending.ingest(marker("probe",9,visualization_msgs::Marker::DELETE));
  EXPECT_TRUE(pending.take().empty());
  pending.ingest(marker("probe",3,visualization_msgs::Marker::ADD));
  pending.ingest(marker("probe",3,visualization_msgs::Marker::DELETE));
  sent=pending.take();
  ASSERT_EQ(sent.size(),1U);EXPECT_EQ(sent[0].action,visualization_msgs::Marker::DELETE);EXPECT_EQ(sent[0].id,3);
  visualization_msgs::MarkerArray batch;
  batch.markers.push_back(marker("",0,visualization_msgs::Marker::DELETEALL));
  batch.markers.push_back(marker("probe",4,visualization_msgs::Marker::ADD));
  pending.ingest(batch);
  sent=pending.take();
  ASSERT_EQ(sent.size(),2U);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::DELETEALL);EXPECT_TRUE(sent[0].ns.empty());
  EXPECT_EQ(sent[1].id,4);
  pending.ingest(marker("keep",1,visualization_msgs::Marker::ADD));
  pending.ingest(marker("drop",2,visualization_msgs::Marker::ADD));
  visualization_msgs::Marker wipe=marker("drop",0,visualization_msgs::Marker::DELETEALL);
  pending.ingest(wipe);
  sent=pending.take();
  ASSERT_EQ(sent.size(),2U);
  EXPECT_EQ(sent[0].id,1);EXPECT_EQ(sent[0].ns,"keep");
  EXPECT_EQ(sent[1].action,visualization_msgs::Marker::DELETEALL);EXPECT_EQ(sent[1].ns,"drop");
}
TEST(MarkerCoalesce, PublishedIdsStayBoundedWithoutClearingAnotherNamespace) {
  MarkerCoalescer pending;
  pending.ingest(marker("keep",1,visualization_msgs::Marker::ADD));
  ASSERT_EQ(pending.take().size(),1U);
  std::vector<visualization_msgs::Marker> last;
  for(int id=0;id<static_cast<int>(MarkerCoalescer::kMaxKeys);++id) {
    pending.ingest(marker("probe",id,visualization_msgs::Marker::ADD));
    last=pending.take();
  }
  ASSERT_EQ(last.size(),1U);
  EXPECT_EQ(last[0].action,visualization_msgs::Marker::DELETEALL);
  EXPECT_EQ(last[0].ns,"probe");
  EXPECT_FALSE(pending.overflowed());
  EXPECT_EQ(pending.publishedCount(),1U);
  pending.ingest(marker("keep",1,visualization_msgs::Marker::DELETE));
  const auto sent=pending.take();
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::DELETE);
  EXPECT_EQ(sent[0].ns,"keep");
}
TEST(MarkerCoalesce, KeyOverflowWipesOnlyTheNamespaceThatDidNotFit) {
  MarkerCoalescer pending;
  pending.ingest(marker("keep",1,visualization_msgs::Marker::ADD));
  for(int id=0;id<static_cast<int>(MarkerCoalescer::kMaxKeys)+1;++id) pending.ingest(marker("probe",id,visualization_msgs::Marker::ADD));
  const auto sent=pending.take();
  ASSERT_EQ(sent.size(),2U);
  EXPECT_EQ(sent[0].ns,"keep");
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::ADD);
  EXPECT_EQ(sent[1].action,visualization_msgs::Marker::DELETEALL);
  EXPECT_EQ(sent[1].ns,"probe");
  EXPECT_FALSE(pending.overflowed());
  EXPECT_EQ(pending.keyCount(),0U);
  EXPECT_EQ(pending.publishedCount(),1U);
}
TEST(MarkerCoalesce, PayloadBytesUseMaxMessageBytesAndSpareOtherNamespaces) {
  MarkerCoalescer pending;
  pending.ingest(marker("keep",1,visualization_msgs::Marker::ADD));
  visualization_msgs::Marker cloud=marker("cloud",7,visualization_msgs::Marker::ADD);
  cloud.points.resize(MarkerCoalescer::kMaxMessageBytes/24+1);
  pending.ingest(cloud);
  EXPECT_LT(pending.payloadBytes(),static_cast<std::size_t>(MarkerCoalescer::kMaxMessageBytes));
  auto sent=pending.take();
  ASSERT_EQ(sent.size(),2U);
  EXPECT_EQ(sent[0].ns,"keep");
  EXPECT_EQ(sent[1].action,visualization_msgs::Marker::DELETEALL);
  EXPECT_EQ(sent[1].ns,"cloud");
  EXPECT_TRUE(sent[1].points.empty());
  visualization_msgs::Marker words=marker("words",1,visualization_msgs::Marker::ADD);
  words.text.assign(MarkerCoalescer::kMaxMessageBytes+1,'x');
  pending.ingest(words);
  sent=pending.take();
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::DELETEALL);
  EXPECT_EQ(sent[0].ns,"words");
  EXPECT_EQ(pending.publishedCount(),1U);
}
TEST(MarkerCoalesce, UnknownActionIsTheExplicitGlobalWipe) {
  MarkerCoalescer pending;
  pending.ingest(marker("keep",1,visualization_msgs::Marker::ADD));
  pending.ingest(marker("other",2,1));
  EXPECT_TRUE(pending.overflowed());
  const auto sent=pending.take();
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::DELETEALL);
  EXPECT_TRUE(sent[0].ns.empty());
  EXPECT_EQ(pending.publishedCount(),0U);
  pending.ingest(marker("keep",1,visualization_msgs::Marker::DELETE));
  EXPECT_TRUE(pending.take().empty());
}
TEST(MarkerCoalesce, TooManyBarriersIsTheExplicitGlobalWipe) {
  MarkerCoalescer pending;
  pending.ingest(marker("keep",1,visualization_msgs::Marker::ADD));
  ASSERT_EQ(pending.take().size(),1U);
  for(int i=0;i<=static_cast<int>(MarkerCoalescer::kMaxBarriers);++i) {
    const std::string ns="ns"+std::to_string(i);
    pending.ingest(marker(ns,1,visualization_msgs::Marker::ADD));
    pending.ingest(marker(ns,0,visualization_msgs::Marker::DELETEALL));
  }
  EXPECT_TRUE(pending.overflowed());
  const auto sent=pending.take();
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::DELETEALL);
  EXPECT_TRUE(sent[0].ns.empty());
  EXPECT_EQ(pending.publishedCount(),0U);
}
visualization_msgs::Marker timed(const std::string& ns,int id,const ros::Time& stamp,double seconds) {
  visualization_msgs::Marker message=marker(ns,id,visualization_msgs::Marker::ADD);
  message.header.stamp=stamp;
  message.lifetime=ros::Duration(seconds);
  return message;
}
TEST(MarkerCoalesce, LifetimeAnchorsAtIngestAndSameStampRenews) {
  const ros::Time ingest(1000,0);
  const ros::Time old_stamp(100,0);
  MarkerCoalescer pending;
  auto forever=timed("ns",1,old_stamp,0);
  forever.lifetime=ros::Duration(0);
  pending.ingest(forever,ingest);
  auto sent=pending.take(ingest+ros::Duration(5));
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::ADD);
  EXPECT_EQ(sent[0].lifetime,ros::Duration(0));
  EXPECT_EQ(sent[0].header.stamp,old_stamp);
  // An ancient geometry stamp is not the lifetime clock. Ingest is fresh.
  pending.ingest(timed("ns",2,old_stamp,1.0),ingest);
  sent=pending.take(ingest+ros::Duration(0.25));
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].id,2);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::ADD);
  EXPECT_EQ(sent[0].header.stamp,old_stamp);
  EXPECT_NEAR(sent[0].lifetime.toSec(),0.75,1e-6);
  EXPECT_NE(sent[0].lifetime,ros::Duration(1.0));
  EXPECT_TRUE(pending.take(ingest+ros::Duration(0.5)).empty());
  // The same stamp on a later ADD starts a full lifetime at that ingest.
  const ros::Time again=ingest+ros::Duration(0.9);
  pending.ingest(timed("ns",2,old_stamp,1.0),again);
  sent=pending.take(again);
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].id,2);
  EXPECT_EQ(sent[0].header.stamp,old_stamp);
  EXPECT_NEAR(sent[0].lifetime.toSec(),1.0,1e-6);
  const ros::Time third=again+ros::Duration(0.2);
  pending.ingest(timed("ns",2,old_stamp,1.0),third);
  sent=pending.take(third+ros::Duration(0.3));
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].header.stamp,old_stamp);
  EXPECT_NEAR(sent[0].lifetime.toSec(),0.7,1e-6);
  EXPECT_TRUE(pending.take(third+ros::Duration(0.9)).empty());
  sent=pending.take(third+ros::Duration(1.0));
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::DELETE);
  EXPECT_EQ(sent[0].id,2);
  EXPECT_EQ(pending.publishedCount(),1U);
  auto other=marker("ns",4,visualization_msgs::Marker::ADD);
  other.header.stamp=old_stamp;
  pending.ingest(other,third+ros::Duration(1.2));
  sent=pending.take(third+ros::Duration(1.2));
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].id,4);
  EXPECT_EQ(sent[0].action,visualization_msgs::Marker::ADD);
  EXPECT_EQ(sent[0].header.stamp,old_stamp);
  EXPECT_EQ(pending.publishedCount(),2U);
  visualization_msgs::Marker unstamped=marker("ns",6,visualization_msgs::Marker::ADD);
  unstamped.lifetime=ros::Duration(1);
  pending.ingest(unstamped,ingest);
  sent=pending.take(ingest+ros::Duration(0.25));
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].id,6);
  EXPECT_TRUE(sent[0].header.stamp.isZero());
  EXPECT_NEAR(sent[0].lifetime.toSec(),0.75,1e-6);
  MarkerCoalescer dropped;
  dropped.ingest(timed("gone",8,old_stamp,0.05),ingest);
  EXPECT_TRUE(dropped.take(ingest+ros::Duration(0.05)).empty());
  EXPECT_EQ(dropped.publishedCount(),0U);
  dropped.ingest(timed("gone",8,old_stamp,0.2),ingest);
  sent=dropped.take(ingest+ros::Duration(0.05));
  ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].header.stamp,old_stamp);
  EXPECT_NEAR(sent[0].lifetime.toSec(),0.15,1e-6);
  MarkerCoalescer one;
  one.ingest(timed("ns",1,old_stamp,1.0),ingest);
  one.ingest(timed("ns",2,old_stamp,1.0),ingest);
  visualization_msgs::Marker single;
  ASSERT_TRUE(one.takeOne(ingest+ros::Duration(0.25),&single));
  EXPECT_EQ(single.id,1);
  EXPECT_EQ(single.header.stamp,old_stamp);
  EXPECT_NEAR(single.lifetime.toSec(),0.75,1e-6);
  one.ingest(timed("ns",1,old_stamp,1.0),ingest+ros::Duration(0.4));
  ASSERT_TRUE(one.takeOne(ingest+ros::Duration(0.4),&single));
  EXPECT_EQ(single.id,2);
  EXPECT_EQ(single.header.stamp,old_stamp);
  EXPECT_NEAR(single.lifetime.toSec(),0.6,1e-6);
  ASSERT_TRUE(one.takeOne(ingest+ros::Duration(0.4),&single));
  EXPECT_EQ(single.id,1);
  EXPECT_EQ(single.header.stamp,old_stamp);
  EXPECT_NEAR(single.lifetime.toSec(),1.0,1e-6);
}
TEST(MarkerCoalesce, SingularDrainEmitsOneAndDoesNotStarveHighIds) {
  MarkerCoalescer pending;
  pending.ingest(marker("ns",1,visualization_msgs::Marker::ADD));
  pending.ingest(marker("ns",2,visualization_msgs::Marker::ADD));
  pending.ingest(marker("ns",3,visualization_msgs::Marker::ADD));
  visualization_msgs::Marker sent;
  ASSERT_TRUE(pending.takeOne(ros::Time(),&sent));
  EXPECT_EQ(sent.id,1);
  EXPECT_EQ(sent.action,visualization_msgs::Marker::ADD);
  EXPECT_EQ(pending.publishedCount(),1U);
  EXPECT_EQ(pending.keyCount(),2U);
  pending.ingest(marker("ns",1,visualization_msgs::Marker::ADD));
  ASSERT_TRUE(pending.takeOne(ros::Time(),&sent));
  EXPECT_EQ(sent.id,2);
  ASSERT_TRUE(pending.takeOne(ros::Time(),&sent));
  EXPECT_EQ(sent.id,3);
  ASSERT_TRUE(pending.takeOne(ros::Time(),&sent));
  EXPECT_EQ(sent.id,1);
  EXPECT_EQ(pending.publishedCount(),3U);
  pending.ingest(marker("ns",1,visualization_msgs::Marker::DELETE));
  pending.ingest(marker("ns",4,visualization_msgs::Marker::ADD));
  std::set<int> deletes,adds;
  for(int i=0;i<2;++i) {
    ASSERT_TRUE(pending.takeOne(ros::Time(),&sent));
    if(sent.action==visualization_msgs::Marker::DELETE) deletes.insert(sent.id);
    if(sent.action==visualization_msgs::Marker::ADD) adds.insert(sent.id);
  }
  EXPECT_EQ(deletes,std::set<int>({1}));
  EXPECT_EQ(adds,std::set<int>({4}));
  pending.ingest(marker("keep",1,visualization_msgs::Marker::ADD));
  pending.ingest(marker("drop",2,visualization_msgs::Marker::ADD));
  pending.ingest(marker("drop",0,visualization_msgs::Marker::DELETEALL));
  pending.ingest(marker("drop",4,visualization_msgs::Marker::ADD));
  ASSERT_TRUE(pending.takeOne(ros::Time(),&sent));
  EXPECT_EQ(sent.ns,"keep");
  EXPECT_EQ(sent.action,visualization_msgs::Marker::ADD);
  ASSERT_TRUE(pending.takeOne(ros::Time(),&sent));
  EXPECT_EQ(sent.action,visualization_msgs::Marker::DELETEALL);
  EXPECT_EQ(sent.ns,"drop");
  ASSERT_TRUE(pending.takeOne(ros::Time(),&sent));
  EXPECT_EQ(sent.ns,"drop");
  EXPECT_EQ(sent.id,4);
  EXPECT_EQ(sent.action,visualization_msgs::Marker::ADD);
  EXPECT_FALSE(pending.takeOne(ros::Time(),&sent));
}
template<class T> void exactWire() {
  DeclaredWire<T> input;input.bytes={0xff,0x00,0x7f,0x80,0x01,0x00,0x00,0x00,0xfe,0xcd};
  std::vector<std::uint8_t> output(input.bytes.size());ros::serialization::OStream writer(output.data(),output.size());ros::serialization::serialize(writer,input);
  EXPECT_EQ(input.bytes,output);DeclaredWire<T> received;ros::serialization::IStream reader(output.data(),output.size());ros::serialization::deserialize(reader,received);EXPECT_EQ(input.bytes,received.bytes);
  EXPECT_STREQ(ros::message_traits::DataType<T>::value(),ros::message_traits::DataType<DeclaredWire<T>>::value());
  EXPECT_STREQ(ros::message_traits::MD5Sum<T>::value(),ros::message_traits::MD5Sum<DeclaredWire<T>>::value());
  EXPECT_FALSE(ros::message_traits::HasHeader<DeclaredWire<T>>::value);
}
TEST(RelayWire, AllFourTypesPreserveSerializedPayloadAndHeaderSequence) {
  exactWire<sensor_msgs::PointCloud2>();exactWire<nav_msgs::OccupancyGrid>();exactWire<nav_msgs::Path>();exactWire<geometry_msgs::PoseArray>();
}
TEST(SourceHistory, SamplingCapacityExpiryAndClockRollback) {
  SourceHistory history;
  geometry_msgs::Pose pose;
  pose.orientation.w=1;
  for(int i=0;i<70;++i) {
    pose.position.x=i;
    ASSERT_TRUE(history.append(ros::Time(100+i/10,(i%10)*100000000),pose));
    EXPECT_LE(history.size,61U);
  }
  ASSERT_EQ(history.size,61U);
  EXPECT_EQ(history.at(0).pose.position.x,9);
  EXPECT_EQ(history.at(60).pose.position.x,69);
  EXPECT_FALSE(history.append(ros::Time(),pose));
  EXPECT_FALSE(history.append(ros::Time(106,950000000),pose));
  EXPECT_FALSE(history.append(ros::Time(106,800000000),pose));
  ASSERT_TRUE(history.append(ros::Time(30),pose));
  ASSERT_EQ(history.size,1U);
  EXPECT_EQ(history.stamp,ros::Time(30));
  ASSERT_TRUE(history.expire(ros::Time(37)));
  EXPECT_EQ(history.size,0U);
  EXPECT_EQ(history.stamp,ros::Time(37));
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

// Compile the actual member in this test so a ROS-free fixture can deliver
// samples to its existing input owner and exercise tick, render and retire.
#include "../src/runtime/member.cpp"
#include "../src/runtime/instance.hpp"
#include <xgc2_ros_visualizer/instance_input.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <sstream>

namespace xgc2_ros_visualizer {

struct MemberTestAccess {
  static std::shared_ptr<RobotInput> input(Member& member) { return member.impl_->input; }
  static void pose(const std::shared_ptr<RobotInput>& input,const ros::Time& stamp=ros::Time(94)) {
    std::lock_guard<std::mutex> lock(input->mutex);
    input->state.pose.available=true;
    input->state.pose.frame=2; // FS150 local map frame
    input->state.pose.stamp=stamp;
    input->state.pose.pose.orientation.w=1;
  }
  static void flight(const std::shared_ptr<RobotInput>& input,bool available,bool armed,const ros::Time& stamp) {
    std::lock_guard<std::mutex> lock(input->mutex);
    input->state.state_available=available;
    input->state.armed=armed;
    input->state.state_stamp=stamp;
  }
  static void landed(const std::shared_ptr<RobotInput>& input,bool available,std::uint8_t state,const ros::Time& receipt) {
    std::lock_guard<std::mutex> lock(input->mutex);
    input->state.extended_available=available;
    input->state.landed=state;
    input->state.extended_stamp=receipt;
  }
};

namespace {

RobotProfile profile(const std::string& id) {
  RobotProfile value;
  value.id=id;value.kind=RateKind::Fs150;value.scene_model=id;
  value.description.package="fs150_description";
  value.description.file="urdf/fs150_visual.urdf";
  value.publication.paths=false;
  return value;
}

RobotProfile frozenProfile(const std::string& id) {
  std::ifstream input(std::string(PROFILE_TESTDATA)+"/hil-frozen-input.json");
  if(!input.good()) throw std::runtime_error("missing HIL frozen input fixture");
  std::ostringstream contents;contents<<input.rdbuf();
  return projectInstanceInput(parseJson(contents.str())).robots.at(id);
}

class MemberRotor : public ::testing::Test {
 protected:
  MemberRotor() : rates(defaultRates()) {
    ros::Time::init();
    for(auto& row:totals) for(auto& count:row) count.store(0);
  }
  double tick(Member& member,const ros::Time& now) {
    TickBatch batch;
    TickContext context{rates,true,true,true,now,totals,2.0,true};
    member.tick(context,batch);
    for(const auto& transform:batch.joints.transforms) {
      if(transform.child_frame_id=="xgc/robots/"+member.profile().scene_model+"/rotor_0")
        return transform.transform.rotation.z;
    }
    ADD_FAILURE() << "rotor transform missing";
    return 0;
  }
  Rates rates;
  Counters totals;
};

TEST_F(MemberRotor, PhysicalArmedFactAdvancesWithSourceSixSecondsBehind) {
  Member member(frozenProfile("uav3"));
  const auto input=MemberTestAccess::input(member);
  MemberTestAccess::pose(input);
  MemberTestAccess::flight(input,true,true,ros::Time(94));
  const auto before=tick(member,ros::Time(100));
  const auto after=tick(member,ros::Time(100,100000000));
  EXPECT_NE(before,after);
}

TEST_F(MemberRotor, AlignedArmedFactAdvancesAndDisarmedOrUnknownDoesNot) {
  Member member(frozenProfile("uav2"));
  const auto input=MemberTestAccess::input(member);
  MemberTestAccess::pose(input);
  MemberTestAccess::flight(input,false,true,ros::Time(100));
  const auto unknown=tick(member,ros::Time(100));
  EXPECT_DOUBLE_EQ(unknown,tick(member,ros::Time(100,100000000)));
  MemberTestAccess::flight(input,true,true,ros::Time(100));
  const auto armed=tick(member,ros::Time(100,200000000));
  EXPECT_NE(unknown,armed);
  MemberTestAccess::flight(input,true,false,ros::Time(100));
  EXPECT_DOUBLE_EQ(armed,tick(member,ros::Time(100,300000000)));
  MemberTestAccess::flight(input,false,true,ros::Time(100));
  EXPECT_DOUBLE_EQ(armed,tick(member,ros::Time(100,400000000)));
}

TEST_F(MemberRotor, GroundAndAirSpeedUseRetainedLandedFactAfterReceiptWindow) {
  const auto selected=frozenProfile("uav3");
  auto phase=[&](bool available,std::uint8_t landed) {
    Member member(selected);
    const auto input=MemberTestAccess::input(member);
    MemberTestAccess::pose(input);
    MemberTestAccess::flight(input,true,true,ros::Time(94));
    // This field records local receipt time, not the ExtendedState header.
    MemberTestAccess::landed(input,available,landed,ros::Time(94));
    tick(member,ros::Time(100));
    return tick(member,ros::Time(100,100000000));
  };
  const auto ground=phase(true,mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND);
  const auto air=phase(true,mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR);
  const auto takeoff=phase(true,mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF);
  const auto landing=phase(true,mavros_msgs::ExtendedState::LANDED_STATE_LANDING);
  EXPECT_NEAR(ground,std::sin(selected.animation.rotor_ground*0.1/2),1e-8);
  EXPECT_NEAR(air,std::sin(selected.animation.rotor_airborne*0.1/2),1e-8);
  EXPECT_NEAR(takeoff,std::sin(std::fmod(selected.animation.rotor_transition*0.1,2*M_PI)/2),1e-8);
  EXPECT_NEAR(landing,takeoff,1e-8);
  EXPECT_NE(ground,air);
  // Without an extended state, preserve the existing default speed tier.
  EXPECT_NEAR(air,phase(false,mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND),1e-8);
}

TEST_F(MemberRotor, RemotePoseStatusReportsReceiptNotConnectionOrSourceClockAge) {
  Member member(frozenProfile("uav3"));
  const auto input=MemberTestAccess::input(member);
  EXPECT_EQ("no sample received",member.peerStatus()["state"].asString());
  MemberTestAccess::pose(input,ros::Time::now()-ros::Duration(6));
  EXPECT_EQ("sample received",member.peerStatus()["state"].asString());
  // A retained sample is not proof of a currently connected publisher; this
  // member has no activated subscriber in the fixture.
  EXPECT_NE("connected",member.peerStatus()["state"].asString());
  MemberTestAccess::pose(input,ros::Time::now());
  EXPECT_EQ("sample received",member.peerStatus()["state"].asString());
}

TEST_F(MemberRotor, RetiredMemberRetractsAndReplacementHasNoOldFlightOrPhase) {
  auto member=std::make_shared<Member>(profile("uav3"));
  const auto old_input=MemberTestAccess::input(*member);
  MemberTestAccess::pose(old_input);
  MemberTestAccess::flight(old_input,true,true,ros::Time(100));
  tick(*member,ros::Time(100));
  EXPECT_NE(0.0,tick(*member,ros::Time(100,100000000)));
  ASSERT_NE(nullptr,member->shown().label);
  const auto retraction=member->retire(ros::Time(101));
  EXPECT_FALSE(retraction.markers.markers.empty());
  for(const auto& marker:retraction.markers.markers)
    EXPECT_EQ(visualization_msgs::Marker::DELETE,marker.action);
  EXPECT_FALSE(retraction.scene.deletions.empty());
  EXPECT_EQ(nullptr,member->shown().label);
  std::weak_ptr<Member> retired=member;
  member.reset();
  EXPECT_TRUE(retired.expired());

  // Even a late write to the old callback's retained input cannot enter the
  // replacement member or carry its old animation phase across the lifetime.
  MemberTestAccess::flight(old_input,true,true,ros::Time(102));
  Member replacement(profile("uav3"));
  const auto replacement_input=MemberTestAccess::input(replacement);
  MemberTestAccess::pose(replacement_input);
  EXPECT_NE(old_input,replacement_input);
  const auto first=tick(replacement,ros::Time(102));
  EXPECT_DOUBLE_EQ(0.0,first);
  EXPECT_DOUBLE_EQ(first,tick(replacement,ros::Time(102,100000000)));
}

TEST_F(MemberRotor, InstanceRemovesAndRecreatesTheMember) {
  InputPool pool(1);
  Instance instance("rotor-test");
  InstanceSpec wanted;
  wanted.scene.publish_scene=false;wanted.scene.publish_transforms=false;
  wanted.robots.emplace("uav3",profile("uav3"));
  EXPECT_EQ((std::vector<std::string>{"uav3"}),instance.apply(wanted,pool).added);
  EXPECT_TRUE(instance.hasRobot("uav3"));
  InstanceSpec empty=wanted;empty.robots.clear();
  EXPECT_EQ((std::vector<std::string>{"uav3"}),instance.apply(empty,pool).removed);
  EXPECT_FALSE(instance.hasRobot("uav3"));
  EXPECT_EQ((std::vector<std::string>{"uav3"}),instance.apply(wanted,pool).added);
  EXPECT_TRUE(instance.hasRobot("uav3"));
  instance.deactivate();
}

} // namespace
} // namespace xgc2_ros_visualizer

int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}

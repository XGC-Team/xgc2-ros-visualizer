#include "transport/zenoh_input.hpp"
#include <gtest/gtest.h>
#include <zenoh.h>
#include <atomic>
#include <chrono>
#include <thread>
using namespace xgc2_ros_visualizer;
TEST(ZenohInput, ExactKeyDirectPeerLimitAndRetraction) {
  const char* config="{mode:'peer',listen:{endpoints:[]},connect:{endpoints:['tcp/127.0.0.1:17569']},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}";
  z_owned_config_t c; ASSERT_EQ(0,zc_config_from_str(&c,config));
  z_owned_session_t session; ASSERT_EQ(0,z_open(&session,z_move(c),nullptr));
  z_view_keyexpr_t key; ASSERT_EQ(0,z_view_keyexpr_from_str(&key,"test/private/pose"));
  z_owned_publisher_t publisher; ASSERT_EQ(0,z_declare_publisher(z_loan(session),&publisher,z_loan(key),nullptr));
  std::atomic<int> delivered{0};
  auto input=ZenohInput::open("tcp/127.0.0.1:17569","test/private/pose",4,[&](const std::uint8_t* data,std::size_t size) {
    if(size==3 && data[0]==1 && data[1]==2 && data[2]==3)++delivered;
  });
  auto put=[&](std::size_t size) {
    const std::uint8_t value[]={1,2,3,4,5};z_owned_bytes_t bytes;
    EXPECT_EQ(0,z_bytes_copy_from_buf(&bytes,value,size));
    EXPECT_EQ(0,z_publisher_put(z_loan(publisher),z_move(bytes),nullptr));
  };
  for(int i=0;i<100 && !delivered.load();++i) { put(3);std::this_thread::sleep_for(std::chrono::milliseconds(30)); }
  ASSERT_GT(delivered.load(),0);
  const auto before=delivered.load();
  for(int i=0;i<10;++i)put(5);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(before,delivered.load());
  input.reset();
  const auto after=delivered.load();
  for(int i=0;i<10;++i)put(3);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(after,delivered.load());
  z_drop(z_move(publisher));z_drop(z_move(session));
}
TEST(ZenohInput, OnlyExplicitEndpointsAndExactKeys) {
  EXPECT_NO_THROW(validateZenohSource("tcp/192.0.2.1:7447","xgc/run/robot1/pose"));
  EXPECT_THROW(validateZenohSource("udp/192.0.2.1:7447","xgc/run/robot1/pose"),std::invalid_argument);
  EXPECT_THROW(validateZenohSource("tcp/192.0.2.1:0","xgc/run/robot1/pose"),std::invalid_argument);
  EXPECT_THROW(validateZenohSource("tcp/192.0.2.1:7447","xgc/**"),std::invalid_argument);
}

TEST(ZenohInput, ListenerLeaseIsSharedAndClosesAfterLastOwner) {
  const std::string endpoint="tcp/127.0.0.1:17570";
  auto first=ZenohListener::open(endpoint);
  auto second=ZenohListener::open(endpoint);
  z_owned_config_t c;
  ASSERT_EQ(0,zc_config_from_str(&c,"{mode:'client',connect:{endpoints:['tcp/127.0.0.1:17570']},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"));
  z_owned_session_t session;
  ASSERT_EQ(0,z_open(&session,z_move(c),nullptr));
  z_view_keyexpr_t key;
  ASSERT_EQ(0,z_view_keyexpr_from_str(&key,"test/private/no-consumer"));
  z_owned_publisher_t publisher;
  ASSERT_EQ(0,z_declare_publisher(z_loan(session),&publisher,z_loan(key),nullptr));
  z_matching_status_t status{};
  ASSERT_EQ(0,z_publisher_get_matching_status(z_loan(publisher),&status));
  EXPECT_FALSE(status.matching);
  first.reset();
  auto subscriber=ZenohInput::open(endpoint,"test/private/no-consumer",64,[](const std::uint8_t*,std::size_t){});
  for(int i=0;i<100;++i) {
    ASSERT_EQ(0,z_publisher_get_matching_status(z_loan(publisher),&status));
    if(status.matching) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_TRUE(status.matching);
  subscriber.reset();
  second.reset();
  z_drop(z_move(publisher));
  z_drop(z_move(session));
  // Rebinding requires the last lease and subscriber to have released the socket.
  EXPECT_NO_THROW(ZenohListener::open(endpoint));
}

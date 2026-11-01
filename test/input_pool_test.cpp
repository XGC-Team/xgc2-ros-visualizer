#include "runtime/input_pool.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <dirent.h>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace xgc2_ros_visualizer {
namespace {
std::set<std::string> tasks() {
  std::set<std::string> result;
  DIR* directory=opendir("/proc/self/task");
  if(!directory) throw std::runtime_error("cannot inspect test process tasks");
  while(auto* entry=readdir(directory)) {
    const std::string id(entry->d_name);
    if(!id.empty() && std::all_of(id.begin(),id.end(),[](char c){return c>='0' && c<='9';})) result.insert(id);
  }
  closedir(directory);
  return result;
}
unsigned long switches(const std::string& id) {
  std::ifstream input("/proc/self/task/"+id+"/status");
  std::string key;
  while(input>>key) {
    if(key=="voluntary_ctxt_switches:") {unsigned long value=0;input>>value;return value;}
    std::string rest;std::getline(input,rest);
  }
  throw std::runtime_error("cannot inspect callback worker status");
}
template<class P> bool waitFor(P condition) {
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(std::chrono::steady_clock::now()<until) {
    if(condition())return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return condition();
}

TEST(InputPool, RealWorkersSleepIdleAndReceiveLatchedAndLaterMessages) {
  ros::NodeHandle base;
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const auto before=tasks();
  InputPool pool(3);
  const auto after=tasks();
  std::vector<std::string> workers;
  std::set_difference(after.begin(),after.end(),before.begin(),before.end(),std::back_inserter(workers));
  ASSERT_EQ(3u,workers.size());
  std::vector<unsigned long> starts;
  for(const auto& id:workers)starts.push_back(switches(id));
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  unsigned long idle_wakes=0;
  for(std::size_t i=0;i<workers.size();++i)idle_wakes+=switches(workers[i])-starts[i];
  std::cout<<"{\"phase\":\"idle\",\"workers\":"<<workers.size()
           <<",\"voluntary_switches\":"<<idle_wakes<<"}"<<std::endl;
  EXPECT_LE(idle_wakes,9ul);

  auto publisher=base.advertise<std_msgs::String>("/input_pool_test/latched",1,true);
  std_msgs::String message;message.data="latched-before-subscribe";
  publisher.publish(message);
  std::mutex mutex;std::vector<std::string> received;
  auto input=pool.node("/input_pool_test/latched");
  auto subscriber=input.subscribe<std_msgs::String>("/input_pool_test/latched",4,
      [&](const std_msgs::String::ConstPtr& item){std::lock_guard<std::mutex> lock(mutex);received.push_back(item->data);});
  ASSERT_TRUE(waitFor([&]{std::lock_guard<std::mutex> lock(mutex);return !received.empty();}));
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ("latched-before-subscribe",received.front());
  }
  message.data="later-event";publisher.publish(message);
  ASSERT_TRUE(waitFor([&]{std::lock_guard<std::mutex> lock(mutex);return received.size()>=2;}));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ("later-event",received.back());
  }
  subscriber.shutdown();
  auto stopped=std::async(std::launch::async,[&]{pool.stop();});
  ASSERT_EQ(std::future_status::ready,stopped.wait_for(std::chrono::seconds(2)));
  stopped.get();
}

TEST(InputPool, StopJoinsTheActualRunningCallback) {
  ros::NodeHandle base;
  InputPool pool(2);
  auto input=pool.node("/input_pool_test/blocking");
  std::mutex mutex;std::condition_variable changed;bool entered=false,released=false;
  auto subscriber=input.subscribe<std_msgs::String>("/input_pool_test/blocking",1,
      [&](const std_msgs::String::ConstPtr&){
        std::unique_lock<std::mutex> lock(mutex);entered=true;changed.notify_all();
        changed.wait(lock,[&]{return released;});
      });
  auto publisher=base.advertise<std_msgs::String>("/input_pool_test/blocking",1,false);
  ASSERT_TRUE(waitFor([&]{return publisher.getNumSubscribers()==1;}));
  std_msgs::String value;value.data="block";publisher.publish(value);
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(changed.wait_for(lock,std::chrono::seconds(3),[&]{return entered;}));
  }
  auto stopped=std::async(std::launch::async,[&]{pool.stop();});
  EXPECT_EQ(std::future_status::timeout,stopped.wait_for(std::chrono::milliseconds(40)));
  {
    std::lock_guard<std::mutex> lock(mutex);released=true;changed.notify_all();
  }
  ASSERT_EQ(std::future_status::ready,stopped.wait_for(std::chrono::seconds(2)));
  stopped.get();
  subscriber.shutdown();
}

TEST(InputPool, CallbackFailureStopsAndJoinsAllQueues) {
  ros::NodeHandle base;
  const auto before=tasks();
  InputPool pool(3);
  const auto after=tasks();
  std::vector<std::string> workers;
  std::set_difference(after.begin(),after.end(),before.begin(),before.end(),std::back_inserter(workers));
  ASSERT_EQ(3u,workers.size());
  auto input=pool.node("/input_pool_test/failure");
  auto subscriber=input.subscribe<std_msgs::String>("/input_pool_test/failure",1,
      [&](const std_msgs::String::ConstPtr&){throw std::runtime_error("callback failure");});
  auto publisher=base.advertise<std_msgs::String>("/input_pool_test/failure",1,false);
  ASSERT_TRUE(waitFor([&]{return publisher.getNumSubscribers()==1;}));
  std_msgs::String value;value.data="fail";publisher.publish(value);
  ASSERT_TRUE(waitFor([&]{try{pool.rethrowFailure();return false;}catch(const std::runtime_error&){return true;}}));
  ASSERT_TRUE(waitFor([&]{const auto remaining=tasks();return std::none_of(workers.begin(),workers.end(),
      [&](const std::string& id){return remaining.count(id)!=0;});}));
  auto stopped=std::async(std::launch::async,[&]{pool.stop();});
  ASSERT_EQ(std::future_status::ready,stopped.wait_for(std::chrono::seconds(2)));
  stopped.get();
  subscriber.shutdown();
}
}
}
int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  ros::init(argc,argv,"input_pool_test",ros::init_options::AnonymousName);
  const auto result=RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}

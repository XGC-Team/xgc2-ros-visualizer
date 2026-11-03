#include "transport/source_gate.hpp"
#include <gtest/gtest.h>
#include <memory>
using namespace xgc2_ros_visualizer;
namespace {
struct Resource {
  Resource(int value,int* destroyed):value(value),destroyed(destroyed) {}
  ~Resource() { ++*destroyed; }
  int value;
  int* destroyed;
};
TEST(SourceGate, OneActivationPerDemandPeriodAndLastReleaseDestroysIt) {
  int opens=0,destroyed=0;
  SourceGate gate([&]() -> std::shared_ptr<void> {
    return std::shared_ptr<Resource>(new Resource(++opens,&destroyed));
  });
  EXPECT_EQ(nullptr,gate.current());
  gate.acquire();
  auto first=gate.current();
  ASSERT_TRUE(first);
  EXPECT_EQ(1,opens);
  EXPECT_EQ(1,std::static_pointer_cast<Resource>(first)->value);
  gate.acquire();
  EXPECT_EQ(1,opens);
  EXPECT_EQ(first,gate.current());
  auto held=gate.current();
  first.reset();
  gate.release();
  gate.release();
  EXPECT_EQ(nullptr,gate.current());
  EXPECT_EQ(0,destroyed);   // an in-flight callback/publish still owns this activation
  held.reset();
  EXPECT_EQ(1,destroyed);
  gate.acquire();
  auto second=gate.current();
  ASSERT_TRUE(second);
  EXPECT_EQ(2,opens);
  EXPECT_EQ(2,std::static_pointer_cast<Resource>(second)->value);
  second.reset();
  gate.stop();
  EXPECT_EQ(nullptr,gate.current());
  EXPECT_EQ(2,destroyed);
  gate.acquire();
  EXPECT_EQ(2,opens);       // stop rejects late demand
}
TEST(SourceGate, StopDropsCountAndActivation) {
  int opens=0,destroyed=0;
  SourceGate gate([&]() -> std::shared_ptr<void> {
    return std::shared_ptr<Resource>(new Resource(++opens,&destroyed));
  });
  gate.acquire();
  gate.acquire();
  gate.stop();
  EXPECT_EQ(nullptr,gate.current());
  EXPECT_EQ(1,destroyed);
  gate.release();
  gate.release();
  EXPECT_EQ(nullptr,gate.current());
  EXPECT_EQ(1,destroyed);
}
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

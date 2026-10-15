#include "rviz_layout.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace {
Json::Value input() {
  std::ifstream stream(std::string(RVIZ_TESTDATA) + "/rviz_layout_input.json");
  Json::Value value;
  stream >> value;
  return value;
}
} // namespace

TEST(RVizLayout, PreservesFrozenMixedRobotLayout) {
  const auto result = xgc2_ros_visualizer::prepareRvizLayout(input());
  std::ifstream expected(std::string(RVIZ_TESTDATA) +
                         "/rviz_layout_mixed.golden.rviz");
  ASSERT_TRUE(expected.good());
  EXPECT_EQ(result["config"].asString(),
            std::string(std::istreambuf_iterator<char>(expected), {}));
  EXPECT_EQ(result["fixedFrame"].asString(), "world");
  EXPECT_EQ(result["tfTopic"].asString(), "/xgc/tf");
  EXPECT_EQ(result["worldClock"].asString(), "simulation");
}

TEST(RVizLayout, RejectsDuplicateMembershipAndMixedTransformTrees) {
  auto value = input();
  value["robots"][1]["namespace"] = value["robots"][0]["namespace"];
  EXPECT_THROW(xgc2_ros_visualizer::prepareRvizLayout(value),
               std::invalid_argument);
  value = input();
  value["robots"][1]["visualization"]["sceneClass"] = "";
  EXPECT_THROW(xgc2_ros_visualizer::prepareRvizLayout(value),
               std::invalid_argument);
}

TEST(RVizLayout, ClockBelongsToNativeRunMode) {
  auto value = input();
  value["context"]["runMode"] = "hybrid";
  EXPECT_EQ(xgc2_ros_visualizer::prepareRvizLayout(value)["worldClock"].asString(), "wall");
  value["context"]["runMode"] = "physical";
  EXPECT_EQ(xgc2_ros_visualizer::prepareRvizLayout(value)["worldClock"].asString(), "wall");
}

class RVizArtifact : public ::testing::Test {
protected:
  std::string root;
  void SetUp() override {
    char pattern[] = "/tmp/xgc-rviz-artifact-XXXXXX";
    const auto directory = ::mkdtemp(pattern);
    ASSERT_NE(directory, nullptr);
    root = directory;
  }
  void TearDown() override {
    ::unlink((root + "/run/rviz/layout.rviz").c_str());
    ::rmdir((root + "/run/rviz").c_str());
    ::rmdir((root + "/run").c_str());
    ::unlink((root + "/occupied").c_str());
    ::unlink((root + "/link").c_str());
    ::rmdir(root.c_str());
  }
};

TEST_F(RVizArtifact, WritesThePreparedLayoutAndReturnsItsPath) {
  auto value = input();
  value["parameters"]["outputDirectory"] = root + "/run/rviz";
  const auto expected = xgc2_ros_visualizer::prepareRvizLayout(value);
  const auto result = xgc2_ros_visualizer::prepareRvizArtifact(value);
  EXPECT_FALSE(result.isMember("config"));
  EXPECT_EQ(result["configPath"].asString(), root + "/run/rviz/layout.rviz");
  EXPECT_EQ(result["tfTopic"], expected["tfTopic"]);
  EXPECT_EQ(result["worldClock"], expected["worldClock"]);
  std::ifstream artifact(result["configPath"].asString());
  ASSERT_TRUE(artifact.good());
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(artifact), {}),
            expected["config"].asString());
  struct stat status {};
  ASSERT_EQ(::stat(result["configPath"].asCString(), &status), 0);
  EXPECT_EQ(status.st_mode & 0777, 0600);
  ASSERT_EQ(::stat((root + "/run/rviz").c_str(), &status), 0);
  EXPECT_EQ(status.st_mode & 0777, 0700);
}

TEST_F(RVizArtifact, RequiresAnExplicitOwnedDirectory) {
  auto value = input();
  EXPECT_THROW(xgc2_ros_visualizer::prepareRvizArtifact(value), std::invalid_argument);
  for (const auto &directory : {"", "relative", "/", "/tmp/../unowned"}) {
    value["parameters"]["outputDirectory"] = directory;
    EXPECT_THROW(xgc2_ros_visualizer::prepareRvizArtifact(value), std::invalid_argument);
  }
  std::ofstream(root + "/occupied") << "existing file";
  value["parameters"]["outputDirectory"] = root + "/occupied";
  EXPECT_THROW(xgc2_ros_visualizer::prepareRvizArtifact(value), std::invalid_argument);
  ASSERT_EQ(::symlink(root.c_str(), (root + "/link").c_str()), 0);
  value["parameters"]["outputDirectory"] = root + "/link";
  EXPECT_THROW(xgc2_ros_visualizer::prepareRvizArtifact(value), std::invalid_argument);
}

TEST_F(RVizArtifact, InvalidRosterDoesNotCreateAnArtifactDirectory) {
  auto value = input();
  value["parameters"]["outputDirectory"] = root + "/run/rviz";
  value["robots"] = Json::arrayValue;
  EXPECT_THROW(xgc2_ros_visualizer::prepareRvizArtifact(value), std::invalid_argument);
  struct stat status {};
  EXPECT_NE(::stat((root + "/run").c_str(), &status), 0);
}

TEST_F(RVizArtifact, CLIEmitsTheMaterializedPathAndRejectsMalformedInput) {
  auto value = input();
  value["parameters"]["outputDirectory"] = root + "/run/rviz";
  Json::StreamWriterBuilder writer;
  const auto bytes = Json::writeString(writer, value);
  std::istringstream request(bytes);
  std::ostringstream response;
  xgc2_ros_visualizer::prepareRvizArtifact(request, response);
  Json::Value result;
  std::istringstream emitted(response.str());
  emitted >> result;
  EXPECT_EQ(result["configPath"].asString(), root + "/run/rviz/layout.rviz");
  EXPECT_FALSE(result.isMember("config"));
  for (const auto &invalid : {bytes + "{}", std::string("{\"parameters\":{},\"parameters\":{}}"),
                              std::string(8 * 1024 * 1024 + 1, ' ')}) {
    std::istringstream rejected(invalid);
    std::ostringstream empty;
    EXPECT_THROW(xgc2_ros_visualizer::prepareRvizArtifact(rejected, empty), std::invalid_argument);
    EXPECT_TRUE(empty.str().empty());
  }
}

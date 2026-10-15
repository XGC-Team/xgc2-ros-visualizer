#include "rviz_layout.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
namespace xgc2_ros_visualizer {
namespace {
void require(bool ok, const char *message) {
  if (!ok)
    throw std::invalid_argument(message);
}
std::string number(double value) {
  std::ostringstream s;
  s << std::setprecision(15) << value;
  return s.str();
}
std::string format(std::string value,
                   const std::vector<std::string> &arguments) {
  std::size_t at = 0;
  for (const auto &argument : arguments) {
    at = value.find("%s", at);
    require(at != std::string::npos, "invalid RViz template");
    value.replace(at, 2, argument);
    at += argument.size();
  }
  return value;
}
const char *template0 = R"RVIZ(Panels:
  - Class: rviz/Displays
    Name: Displays
    Property Tree Widget:
      Expanded: ~
      Splitter Ratio: 0.5
Preferences:
  PromptSaveOnExit: false
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 160
      Enabled: true
      Name: Grid
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz/TF
      Enabled: true
      Frame Timeout: 15
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree: {}
      Update Interval: 0
      Value: true
)RVIZ";
const char *template1 = R"RVIZ(    - Alpha: 1
      Class: rviz/RobotModel
      Collision Enabled: false
      Enabled: true
      Name: %s RobotModel
      Robot Description: %s
      TF Prefix: %s
      Update Interval: 0
      Visual Enabled: true
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz/Path
      Color: %s
      Enabled: true
      Head Diameter: 0.3
      Head Length: 0.2
      Length: 0.3
      Line Style: Lines
      Line Width: %s
      Name: %s Path
      Pose Style: None
      Queue Size: 10
      Topic: %s
      Value: true
)RVIZ";
const char *template2 = R"RVIZ(    - Class: rviz/Image
      Enabled: true
      Image Topic: %s
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Camera Image
      Normalize Range: true
      Queue Size: 2
      Transport Hint: raw
      Unreliable: false
      Value: true
)RVIZ";
const char *template3 = R"RVIZ(  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: %s
    Frame Rate: 30
  Name: xgc-rviz:%s
  Tools:
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: %s
      Focal Point: {X: %s, Y: %s, Z: %s}
      Name: Current View
      Target Frame: <Fixed Frame>
    Saved: ~
)RVIZ";
} // namespace
Json::Value prepareRvizLayout(const Json::Value &input) {
  require(input.isObject() && input["parameters"].isObject() &&
              input["context"].isObject() && input["robots"].isArray(),
          "explicit parameters/context/robots required");
  const auto &settings = input["parameters"];
  const auto &context = input["context"];
  const auto frame = settings["fixedFrame"].asString(),
             mode = context["runMode"].asString();
  const std::regex identifier("^[A-Za-z_][A-Za-z0-9_]*$"),
      topic("^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$");
  require(std::regex_match(frame, identifier), "invalid fixed frame");
  require(mode == "simulation" || mode == "physical" || mode == "hybrid",
          "invalid frozen runMode");
  const std::string clock = mode == "simulation" ? "simulation" : "wall";
  require(!input["robots"].empty() && input["robots"].size() <= 256,
          "RViz requires 1..256 robots");
  std::vector<Json::Value> robots;
  for (const auto &r : input["robots"])
    robots.push_back(r);
  std::sort(robots.begin(), robots.end(),
            [](const Json::Value &a, const Json::Value &b) {
              return a["namespace"].asString() < b["namespace"].asString();
            });
  std::set<std::string> seen;
  std::string tf, names, config = template0;
  double low[3]{}, high[3]{}, distance = 12;
  for (const auto &r : robots) {
    const auto space = r["namespace"].asString(),
               name = space.substr(space.empty() ? 0 : 1);
    const auto &v = r["visualization"];
    require(std::regex_match(name, identifier) && space == "/" + name &&
                seen.insert(name).second,
            "invalid or repeated frozen robot namespace");
    require(v.isObject() && v["visuals"].isArray() &&
                v["descriptionPackage"].isString() &&
                !v["descriptionPackage"].asString().empty(),
            "RViz requires an installed robot description");
    const auto path = space + "/" + v["pathTopic"].asString();
    require(std::regex_match(path, topic),
            "RViz requires an explicit path topic");
    const auto robot_tf =
        v.get("sceneClass", "").asString().empty() ? "/tf" : "/xgc/tf";
    require(tf.empty() || tf == robot_tf,
            "RViz cannot mix independent transform trees");
    tf = robot_tf;
    auto model = v.get("sceneModel", name).asString();
    if (model.empty())
      model = name;
    if (r["kind"] == "scout_mini" && r["scout"].isObject()) {
      const auto mocap = r["scout"].get("mocapRigidBodyName", "").asString();
      model =
          (!mocap.empty() && mocap != name && name.compare(0, 3, "ugv") != 0)
              ? mocap
              : name;
    }
    require(std::regex_match(model, identifier), "invalid native scene model");
    const auto prefix = v.get("sceneClass", "").asString().empty()
                            ? model
                            : "xgc/robots/" + model;
    const auto kind = r["kind"].asString();
    const std::string color =
        kind == "scout_mini"
            ? "203; 171; 1"
            : kind == "mecanum_ugv" ? "40; 143; 140" : "242; 0; 60";
    const double width = v.get("pathLineWidthMeters", .03).asDouble();
    require(std::isfinite(width) && width >= .001 && width <= 100,
            "invalid path width");
    config += format(template1, {name, space + "/visual_robot_description",
                                 prefix, color, number(width), name, path});
    if (!names.empty())
      names += ",";
    names += name;
    const auto &pose = r["initialPose"];
    require(pose.isObject(), "initial pose required");
    for (int axis = 0; axis < 3; ++axis) {
      const auto &value = pose[axis == 0 ? "x" : axis == 1 ? "y" : "z"];
      require(value.isNumeric() && !value.isBool() &&
                  std::isfinite(value.asDouble()),
              "finite initial pose required");
      const double x = value.asDouble();
      if (seen.size() == 1)
        low[axis] = high[axis] = x;
      else {
        low[axis] = std::min(low[axis], x);
        high[axis] = std::max(high[axis], x);
      }
    }
    const double hint = v.get("initialCameraDistanceMeters", 12).asDouble();
    require(std::isfinite(hint) && hint >= .1 && hint <= 100000,
            "invalid camera distance");
    distance = std::max(distance, hint);
  }
  const auto camera = settings.get("cameraImageTopic", "").asString();
  if (!camera.empty()) {
    require(std::regex_match(camera, topic), "invalid camera image topic");
    config += format(template2, {camera});
  }
  double span = 0;
  for (int axis = 0; axis < 3; ++axis)
    span = std::max(span, high[axis] - low[axis]);
  distance = std::max(distance, span * 1.75 + 2);
  config +=
      format(template3,
             {frame, names, number(distance), number((low[0] + high[0]) / 2),
              number((low[1] + high[1]) / 2), number((low[2] + high[2]) / 2)});
  Json::Value output;
  output["config"] = config;
  output["fixedFrame"] = frame;
  output["tfTopic"] = tf;
  output["runMode"] = mode;
  output["worldClock"] = clock;
  return output;
}

Json::Value prepareRvizArtifact(const Json::Value &input) {
  auto output = prepareRvizLayout(input);
  const auto &value = input["parameters"]["outputDirectory"];
  require(value.isString(), "RViz outputDirectory is required");
  const auto directory = value.asString();
  require(!directory.empty() && directory.front() == '/' &&
              directory.find('\0') == std::string::npos,
          "RViz outputDirectory must be an absolute path");
  std::string current;
  std::istringstream components(directory);
  std::string component;
  while (std::getline(components, component, '/')) {
    if (component.empty())
      continue;
    require(component != "." && component != "..",
            "RViz outputDirectory must be normalized");
    current += "/" + component;
    if (::mkdir(current.c_str(), 0700) != 0 && errno != EEXIST)
      throw std::runtime_error("create RViz directory: " +
                               std::string(std::strerror(errno)));
    struct stat status {};
    if (::lstat(current.c_str(), &status) != 0 || !S_ISDIR(status.st_mode))
      throw std::invalid_argument("RViz outputDirectory requires real directories");
  }
  require(!current.empty(), "RViz outputDirectory must name an artifact directory");
  const auto path = current + "/layout.rviz";
  std::string temporary = current + "/.layout-XXXXXX";
  std::vector<char> name(temporary.begin(), temporary.end());
  name.push_back('\0');
  int fd = ::mkstemp(name.data());
  if (fd < 0)
    throw std::runtime_error("create RViz artifact: " +
                             std::string(std::strerror(errno)));
  try {
    const auto config = output["config"].asString();
    std::size_t offset = 0;
    while (offset < config.size()) {
      const auto count = ::write(fd, config.data() + offset, config.size() - offset);
      if (count < 0 && errno == EINTR)
        continue;
      if (count <= 0)
        throw std::runtime_error("write RViz artifact: " +
                                 std::string(std::strerror(errno)));
      offset += static_cast<std::size_t>(count);
    }
    const int closed = ::close(fd);
    fd = -1;
    if (closed != 0 || ::rename(name.data(), path.c_str()) != 0)
      throw std::runtime_error("publish RViz artifact: " +
                               std::string(std::strerror(errno)));
  } catch (...) {
    if (fd >= 0)
      ::close(fd);
    ::unlink(name.data());
    throw;
  }
  output.removeMember("config");
  output["configPath"] = path;
  return output;
}

void prepareRvizArtifact(std::istream &input, std::ostream &output) {
  std::string bytes;
  char buffer[65536];
  while (input.read(buffer, sizeof(buffer)) || input.gcount()) {
    if (bytes.size() + static_cast<std::size_t>(input.gcount()) > 8 * 1024 * 1024)
      throw std::invalid_argument("RViz preparation input exceeds 8 MiB");
    bytes.append(buffer, static_cast<std::size_t>(input.gcount()));
  }
  require(input.eof(), "unable to read RViz preparation input");
  Json::CharReaderBuilder reader;
  reader["allowComments"] = false;
  reader["failIfExtra"] = true;
  reader["rejectDupKeys"] = true;
  Json::Value request;
  std::string error;
  std::istringstream json(bytes);
  if (!Json::parseFromStream(reader, json, &request, &error))
    throw std::invalid_argument("invalid RViz preparation JSON: " + error);
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  output << Json::writeString(writer, prepareRvizArtifact(request));
}
} // namespace xgc2_ros_visualizer

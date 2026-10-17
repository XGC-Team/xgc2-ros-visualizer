#include <xgc2_ros_visualizer/instance_input.hpp>
#include <algorithm>
#include <cerrno>
#include <climits>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>
namespace xgc2_ros_visualizer {
namespace {
std::string text(const Json::Value& value,const std::string& name) {
  if(!value.isString())throw std::invalid_argument(name+" must be a string");
  return value.asString();
}
std::string optionalText(const Json::Value& value,const std::string& name) {return value.isNull()?"":text(value,name);}
bool flag(const Json::Value& value,bool fallback,const std::string& name) {
  if(value.isNull())return fallback;
  if(!value.isBool())throw std::invalid_argument(name+" must be boolean");
  return value.asBool();
}
double number(const Json::Value& value,double fallback,const std::string& name) {
  if(value.isNull())return fallback;
  if(value.isBool()||!value.isNumeric()||!std::isfinite(value.asDouble()))throw std::invalid_argument(name+" must be finite numeric");
  return value.asDouble();
}
double ceilingHz(const Json::Value& value,double fallback,const std::string& name) {
  if(value.isNull())return fallback;
  const double hz=number(value,fallback,name);
  if(hz<0.1||hz>100)throw std::invalid_argument(name+" must be within 0.1..100 Hz");
  return hz;
}
RelayRole relayRole(const Json::Value& value) {
  if(value.isNull())return RelayRole::None;
  if(!value.isString())throw std::invalid_argument("relay role must be a string");
  const std::string role=value.asString();
  if(role=="prediction")return RelayRole::Prediction;
  if(role=="obstacle")return RelayRole::Obstacle;
  if(role=="semantic")return RelayRole::Semantic;
  throw std::invalid_argument("relay role must be prediction, obstacle or semantic");
}
std::string trim(std::string value) {
  const auto first=value.find_first_not_of(" \t\r\n");if(first==std::string::npos)return "";
  return value.substr(first,value.find_last_not_of(" \t\r\n")-first+1);
}
// Objects merge member by member; every other value replaces.
void overlay(Json::Value& target,const Json::Value& source) {
  if(!source.isObject()||!target.isObject()) {target=source;return;}
  for(const auto& name:source.getMemberNames()) {
    if(target.isMember(name)&&target[name].isObject()&&source[name].isObject())overlay(target[name],source[name]);
    else target[name]=source[name];
  }
}
void arSource(Json::Value& profile, const Json::Value& robot, const Json::Value& context) {
  const auto mode = text(context["runMode"], "context.runMode");
  const auto source = mode == "hybrid" ? text(robot["hybridSource"], "Robot hybridSource") : mode;
  if (source != "physical" && source != "simulation")
    throw std::invalid_argument("AR requires an explicit physical or simulation source");
  const auto space = robot["namespace"].asString();
  const bool rotor = robot["profileId"] == "px4.mocap-rotor.ros1.v1";
  const auto simulator = source == "simulation" && !rotor
      ? text(context["scene"]["simulator"], "context.scene.simulator") : "";
  if (source == "simulation" && !rotor && simulator != "xsim" && simulator != "gazebo" && simulator != "none")
    throw std::invalid_argument("AR requires an explicit supported simulator");
  const bool direct = simulator == "xsim" || simulator == "none";
  profile["path"]["arTopic"] = "ar_path";
  if (direct) {
    const auto authored = optionalText(robot["simulationPoseTopic"], "simulationPoseTopic");
    profile["state"]["arPoseTopic"] = authored.empty() ? space + "/pose" : authored;
    if (simulator == "none") profile["state"]["poseTopic"] = profile["state"]["arPoseTopic"];
    return;
  }
  const auto body = source == "physical" || rotor
      ? text(robot["px4"]["mocapRigidBodyName"], "px4.mocapRigidBodyName") : space.substr(1);
  if (body.empty() || body.find('/') != std::string::npos)
    throw std::invalid_argument("AR requires a single rigid body name");
  profile["state"]["arPoseTopic"] = "/vrpn_client_node" +
      (mode == "hybrid" ? "_" + source : "") + "/" + body + "/pose";
  if (source == "physical" || rotor) {
    const auto& offset = context["localizationOffset"];
    profile["state"]["worldOffset"] = Json::Value(Json::arrayValue);
    for (int i = 0; i < 3; ++i) {
      const auto& v = offset[i == 0 ? "x" : i == 1 ? "y" : "z"];
      if (!v.isNumeric() || v.isBool() || !std::isfinite(v.asDouble()))
        throw std::invalid_argument("AR localization offset must be finite");
      profile["state"]["worldOffset"].append(v);
    }
  }
}
long slotNumber(const std::string& name) {
  std::size_t first=name.size();while(first>0&&std::isdigit(static_cast<unsigned char>(name[first-1])))--first;
  if(first==name.size())return 0;
  errno=0;char* end=nullptr;const long value=std::strtol(name.c_str()+first,&end,10);
  return errno||value>INT_MAX?0:value;
}
std::string heightColor(const Json::Value& settings,const Json::Value& robot,const std::vector<Json::Value>& ordered) {
  const auto kind=text(robot["kind"],"Robot kind");const char* key=kind=="scout_mini"?"scoutPalette":kind=="mecanum_ugv"?"mecanumPalette":"uavPalette";
  std::vector<std::string> palette;
  if(settings.isMember(key)) {
    const auto& colors=settings[key];if(!colors.isArray()||colors.empty()||colors.size()>32)throw std::invalid_argument(std::string(key)+" requires 1..32 colors");
    static const std::regex color("^#[0-9a-fA-F]{6}$");
    for(const auto& entry:colors) {auto value=text(entry,key);if(!std::regex_match(value,color))throw std::invalid_argument("invalid history palette color");std::transform(value.begin(),value.end(),value.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});palette.push_back(value);}
  } else if(kind=="px4_multirotor") palette={"#f2003c","#ff7043","#ab47bc","#ec407a","#7e57c2","#ef5350","#ffa726","#d4e157"};
  else if(kind=="scout_mini")palette={"#cbab01","#8bc34a","#ffca28","#66bb6a","#c0ca33","#26a69a","#d4a373","#a1887f"};
  else if(kind=="mecanum_ugv")palette={"#288f8c","#29b6f6","#5c6bc0","#26c6da","#42a5f5","#7e57c2","#80cbc4","#90caf9"};
  if(palette.empty())return kind=="scout_mini"?"#cbab01":kind=="mecanum_ugv"?"#288f8c":"#f2003c";
  std::vector<std::string> peers;
  for(const auto& candidate:ordered)if(candidate["kind"]==robot["kind"])peers.push_back(candidate["namespace"].asString().substr(1));
  std::sort(peers.begin(),peers.end(),[](const std::string& a,const std::string& b){auto x=slotNumber(a),y=slotNumber(b);return x==y?a<b:x<y;});
  const auto name=robot["namespace"].asString().substr(1);auto found=std::find(peers.begin(),peers.end(),name);
  return palette[static_cast<std::size_t>(found-peers.begin())%palette.size()];
}
// The panel's publication controls: product data, independent of the frozen
// Viewer style and context fields. markers, transforms and scene are the shared
// outputs of the scene; scenePaths and paths default each robot.
struct PanelPublication {
  bool markers{false}, transforms{true}, scene{true}, scene_paths{false}, paths{true}, ground_scene{true};
};
PanelPublication panelPublication(const Json::Value& panel) {
  PanelPublication result;
  if(!panel.isMember("publication"))return result;
  const auto& publication=panel["publication"];
  if(!publication.isObject())throw std::invalid_argument("settings.publication must be an object");
  const std::map<std::string,bool*> fields{{"markers",&result.markers},{"transforms",&result.transforms},{"scene",&result.scene},{"scenePaths",&result.scene_paths},{"paths",&result.paths},{"groundScene",&result.ground_scene}};
  for(const auto& field:publication.getMemberNames()) {
    const auto target=fields.find(field);
    if(target==fields.end()||!publication[field].isBool())throw std::invalid_argument("unknown or nonboolean publication control");
    *target->second=publication[field].asBool();
  }
  return result;
}
}
InstanceSpec projectInstanceInput(const Json::Value& value) {
  if(!value.isObject())throw std::invalid_argument("instance input must be an object");
  const std::set<std::string> fields{"robots","context","settings","displayRelays"};auto keys=value.getMemberNames();
  if(std::set<std::string>(keys.begin(),keys.end())!=fields)throw std::invalid_argument("instance input requires exactly robots/context/settings/displayRelays");
  static const std::regex ns("^/[A-Za-z_][A-Za-z0-9_]{0,126}$");
  const auto& context=value["context"];const auto& panel=value["settings"];
  if(!context.isObject()||!panel.isObject()||!value["robots"].isArray()||value["robots"].size()>kMaxRobots)throw std::invalid_argument("instance context/settings/robots types are invalid");
  const auto mode=text(context["runMode"],"context.runMode");
  if(mode!="simulation"&&mode!="physical"&&mode!="hybrid")throw std::invalid_argument("invalid frozen runMode");

  InstanceSpec spec;
  const auto publication=panelPublication(panel);
  spec.scene.publish_markers=publication.markers;spec.scene.publish_transforms=publication.transforms;spec.scene.publish_scene=publication.scene;
  if(!context["worldBoundary"].isNull()&&!context["worldBoundary"].isObject())throw std::invalid_argument("worldBoundary must be object or null");
  spec.scene.boundary_json=context["worldBoundary"].isNull()?"":jsonText(context["worldBoundary"]);
  spec.scene.boundary=parseWorldBoundaryDisplay(spec.scene.boundary_json);
  spec.scene.boundary_mode=worldBoundaryDisplayModeFromString(optionalText(panel["worldBoundaryMode"],"worldBoundaryMode").empty()?"walls":panel["worldBoundaryMode"].asString());

  // Label style and offsets of the panel. Both size variants must be valid even
  // though one of them is used, so a bad panel fails before any robot does.
  std::string color=optionalText(panel["markerColor"],"markerColor");if(color.empty())color="#00a2ff";
  const bool invariant=flag(panel["labelScaleInvariant"],false,"labelScaleInvariant");
  const double meters=number(panel["labelFontSizeMeters"],.24,"labelFontSizeMeters"),pixels=number(panel["labelFontSizePixels"],16,"labelFontSizePixels");
  const double opacity=number(panel["markerOpacity"],1,"markerOpacity");
  sceneLabelStyleFromMarkerColor(color,false,meters,opacity);
  sceneLabelStyleFromMarkerColor(color,true,pixels,opacity);
  SceneLabelOffsets offsets;
  offsets.uav=number(panel["uavLabelOffset"],offsets.uav,"uavLabelOffset");
  offsets.scout=number(panel["scoutLabelOffset"],offsets.scout,"scoutLabelOffset");
  offsets.mecanum=number(panel["mecanumLabelOffset"],offsets.mecanum,"mecanumLabelOffset");
  validateSceneLabelOffsets(offsets);
  const bool height_projection=flag(panel["uavHeightProjection"],true,"uavHeightProjection");
  spec.history_max_rate_hz=ceilingHz(panel["historyMaxRateHz"],2,"historyMaxRateHz");
  spec.history_visible=flag(panel["historyVisible"],true,"historyVisible");
  spec.prediction_max_rate_hz=ceilingHz(panel["predictionMaxRateHz"],2,"predictionMaxRateHz");

  std::vector<Json::Value> ordered;std::set<std::string> namespaces;
  for(const auto& robot:value["robots"]) {
    if(!robot.isObject())throw std::invalid_argument("frozen Robot must be an object");
    const auto name=text(robot["namespace"],"Robot namespace");text(robot["kind"],"Robot kind");
    if(!std::regex_match(name,ns)||!namespaces.insert(name).second)throw std::invalid_argument("invalid or repeated frozen Robot namespace");
    ordered.push_back(robot);
  }
  std::sort(ordered.begin(),ordered.end(),[](const Json::Value& a,const Json::Value& b){return a["namespace"].asString()<b["namespace"].asString();});
  std::map<std::string,std::string> classes;
  for(const auto& robot:ordered) {
    const auto space=robot["namespace"].asString(),name=space.substr(1);const auto& visual=robot["visualization"];
    if(!visual.isNull()&&!visual.isObject())throw std::invalid_argument("Robot visualization must be an object");
    const auto scene_class=optionalText(visual["sceneClass"],"sceneClass");
    if(!scene_class.empty()&&scene_class!="fs150"&&scene_class!="scout"&&scene_class!="mecanum")throw std::invalid_argument("unsupported concrete sceneClass");
    classes[space]=scene_class.empty()?"global":scene_class;
    const auto package=optionalText(visual["descriptionPackage"],"descriptionPackage");
    if(package.empty()) {if(!scene_class.empty())throw std::invalid_argument("sceneClass requires an installed description");continue;}
    Json::Value profile(Json::objectValue);
    profile["model"]["kind"]=classes[space];
    auto& description=profile["model"]["description"];
    description["package"]=package;
    description["file"]=text(visual["descriptionFile"],"descriptionFile");
    description["statePublisher"]=flag(visual["robotStatePublisher"],false,"robotStatePublisher");
    description["jointStateTopic"]=text(visual["jointStateTopic"],"jointStateTopic");
    auto model=optionalText(visual["sceneModel"],"sceneModel");
    if(robot["kind"]=="scout_mini"&&robot["scout"].isObject()) {
      const auto mocap=trim(optionalText(robot["scout"]["mocapRigidBodyName"],"mocapRigidBodyName"));
      model=(mocap.empty()||mocap==name||name.compare(0,3,"ugv")==0)?"":mocap;
    }
    // A ground robot whose scene is switched off keeps its kind and its
    // description, and has no scene presence.
    const bool ground=scene_class=="scout"||scene_class=="mecanum";
    profile["model"]["scene"]=scene_class.empty()||(ground&&!publication.ground_scene)?"":model.empty()?name:model;
    if(!scene_class.empty()) {
      const auto path=optionalText(visual["pathTopic"],"pathTopic");
      if(path.empty())throw std::invalid_argument("a scene robot needs its path topic");
      profile["path"]["topic"]=path;
      profile["labels"]["color"]=color;profile["labels"]["scaleInvariant"]=invariant;
      profile["labels"]["fontSize"]=invariant?pixels:meters;profile["labels"]["opacity"]=opacity;
      profile["frames"]["labelOffset"]=scene_class=="fs150"?offsets.uav:scene_class=="scout"?offsets.scout:offsets.mecanum;
    }
    if(scene_class=="fs150") {
      arSource(profile,robot,context);
      if(height_projection)profile["model"]["heightProjectionColor"]=heightColor(panel,robot,ordered);
    }
    // The scene-wide switches decide which shared outputs exist; scene paths and
    // Path topics are the robot's own.
    profile["publication"]["scenePaths"]=publication.scene_paths;profile["publication"]["paths"]=publication.paths;
    if(visual.isMember("profile"))overlay(profile,visual["profile"]);
    spec.robots.emplace(name,parseRobotProfile(name,profile));
  }
  if(!value["displayRelays"].isArray())throw std::invalid_argument("displayRelays must be an array");
  for(const auto& relay:value["displayRelays"]) {
    if(!relay.isObject())throw std::invalid_argument("display relay must be an object");
    const std::set<std::string> allowed{"source","topic","messageType","maxRateHz","role"};
    const auto names=relay.getMemberNames();
    if(!relay.isMember("source")||!relay.isMember("topic")||!relay.isMember("messageType"))throw std::invalid_argument("display relay requires source, topic and messageType");
    for(const auto& name:names)if(!allowed.count(name))throw std::invalid_argument("display relay field is not source, topic, messageType, maxRateHz or role");
    const auto source=text(relay["source"],"relay source");
    std::string kind="global";for(const auto& item:classes)if(source.compare(0,item.first.size()+1,item.first+"/")==0)kind=item.second;
    const double max_rate=relay.isMember("maxRateHz")?ceilingHz(relay["maxRateHz"],0,"maxRateHz"):0;
    const RelayRole role=relay.isMember("role")?relayRole(relay["role"]):RelayRole::None;
    spec.scene.relays.push_back(relayConfig(source,text(relay["topic"],"relay topic"),text(relay["messageType"],"relay messageType"),parseKind(kind),max_rate,role));
  }
  checkInstance(spec);
  return spec;
}
} // namespace xgc2_ros_visualizer

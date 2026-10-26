#include "server.hpp"
#include <xgc2_ros_visualizer/instance_input.hpp>
#include <algorithm>
#include <chrono>
#include <regex>
#include <limits>
#include <cstdio>
#include <random>
#include <set>
#include <stdexcept>
#include <xmlrpcpp/XmlRpc.h>
namespace xgc2_ros_visualizer {
namespace {
constexpr std::size_t kMaxInstances=64;
thread_local bool master_probe=false;
std::int64_t steadyNs() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
// A request that names the revision it expects: absent means unconditional.
bool expectedRevision(const Json::Value& request,std::uint64_t* revision) {
  if(!request.isMember("expectedRevision"))return false;
  const auto& value=request["expectedRevision"];
  if((value.type()!=Json::intValue&&value.type()!=Json::uintValue)||!value.isUInt64())throw std::invalid_argument("expectedRevision must be a nonnegative integer");
  *revision=value.asUInt64();return true;
}
void requireFields(const Json::Value& request,const std::set<std::string>& required,const std::set<std::string>& optional) {
  if(!request.isObject())throw std::invalid_argument("request must be an object");
  for(const auto& name:request.getMemberNames())
    if(!required.count(name)&&!optional.count(name))throw std::invalid_argument("unknown request field: "+name);
  for(const auto& name:required)
    if(!request.isMember(name))throw std::invalid_argument("missing request field: "+name);
}
}
// XMLRPC's default handler prints each failed socket attempt. The watcher owns
// the outage summary; other ROS diagnostics retain their normal handler.
class Server::MasterErrors final : public XmlRpc::XmlRpcErrorHandler {
 public:
  MasterErrors():previous_(getErrorHandler()) {setErrorHandler(this);}
  ~MasterErrors() override {setErrorHandler(previous_);}
  void error(const char* message) override {if(!master_probe)previous_->error(message);}
 private:
  XmlRpc::XmlRpcErrorHandler* previous_;
};
Server::Server(std::size_t workers,Rates rates,std::string world_clock,const volatile std::sig_atomic_t* signal_stop)
    : input_(workers),master_errors_(new MasterErrors),master_uri_(ros::master::getURI()),world_clock_(std::move(world_clock)),
      snapshot_(new std::vector<std::shared_ptr<Instance>>),rates_(new Rates(std::move(rates))),signal_stop_(signal_stop) {
  rates_applied_steady_ns_=steadyNs();
  publisher_=std::thread(&Server::publishLoop,this);
}
Server::~Server() {stop();ros::shutdown();}
void Server::startMasterWatch(std::function<void()> on_bound) {
  on_master_bound_=std::move(on_bound);
  master_watch_=std::thread(&Server::watchMaster,this);
}
void Server::fail(std::exception_ptr failure) {
  {std::lock_guard<std::mutex> lock(failure_mutex_);if(!failure_)failure_=std::move(failure);}
  stopping_.store(true);wakePublisher();
}
void Server::activateRos() {
  if(stopping_.load()||native_active_)return;
  try {
    for(auto& item:instances_)item.second->activate(input_);
    native_active_=true;
    wakePublisher();
  } catch(...) {fail(std::current_exception());}
}
void Server::wakePublisher() {
  std::lock_guard<std::mutex> lock(wait_mutex_);
  ++revision_;wake_.notify_all();
}
void Server::replaceSnapshot() {
  std::shared_ptr<std::vector<std::shared_ptr<Instance>>> next(new std::vector<std::shared_ptr<Instance>>);
  next->reserve(instances_.size());for(const auto& item:instances_)next->push_back(item.second);
  std::shared_ptr<const std::vector<std::shared_ptr<Instance>>> immutable=next;std::atomic_store(&snapshot_,immutable);
  countMembers();
  wakePublisher();
}
void Server::countMembers() {
  std::size_t robots=0;for(const auto& item:instances_)robots+=item.second->spec().robots.size();
  instance_count_.store(instances_.size());robot_count_.store(robots);
}
void Server::publishLoop() {
  try {
    while(!stopping_.load()&&ros::ok()&&(!signal_stop_||!*signal_stop_)) {
      const auto revision=revision_.load();
      const auto started=std::chrono::steady_clock::now();auto rates=std::atomic_load(&rates_);auto members=std::atomic_load(&snapshot_);
      input_.rethrowFailure();
      double maximum=rates->maximum();
      if(ros_bound_.load()) {
        const auto now=ros::Time::now();
        for(const auto& instance:*members) {instance->tick(*rates,now);maximum=std::max(maximum,instance->maximumOverride());}
      }
      // One shared wait; membership/rates/Stop wake immediately. A bounded wall
      // heartbeat also observes signal flags safely, without notifying from a handler.
      const auto period=std::chrono::duration<double>(std::min(.1,1.0/maximum));
      std::unique_lock<std::mutex> wait(wait_mutex_);
      wake_.wait_until(wait,started+std::chrono::duration_cast<std::chrono::steady_clock::duration>(period),[&] {
        return stopping_.load()||revision_.load()!=revision||!ros::ok()||(signal_stop_&&*signal_stop_);
      });
    }
  } catch(...) {std::lock_guard<std::mutex> lock(failure_mutex_);failure_=std::current_exception();}
  stopping_.store(true);
}
// One watcher for the master; topic peers use roscpp's publisherUpdate events.
// XMLRPC has no master-identity event, so healthy checks are two seconds apart.
// Failed checks back off with full jitter from 250 ms to 30 s. Neither the
// scheduler nor the control/transport owner performs these probes.
void Server::watchMaster() {
  try {
    std::mt19937 random(std::random_device{}());
    int backoff_ms=250;
    std::uint64_t suppressed=0;
    bool logged=false;
    auto last_log=std::chrono::steady_clock::time_point::min();
    const auto call=[this](const char* method,const XmlRpc::XmlRpcValue& request,XmlRpc::XmlRpcValue* response) {
      struct ProbeScope {ProbeScope(){master_probe=true;}~ProbeScope(){master_probe=false;}} scope;
      XmlRpc::XmlRpcClient client(ros::master::getHost().c_str(),ros::master::getPort(),"/");
      if(!client.executeNonBlock(method,request))return false;
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
      while(!stopping_.load()&&std::chrono::steady_clock::now()<deadline) {
        client._disp.work(.05);
        if(client.executeCheckDone(*response))
          return response->getType()==XmlRpc::XmlRpcValue::TypeArray&&response->size()==3&&
              (*response)[0].getType()==XmlRpc::XmlRpcValue::TypeInt;
      }
      return false;
    };
    while(!stopping_.load()) {
      XmlRpc::XmlRpcValue request,response;
      request[0]=ros::this_node::getName();
      ++master_attempts_;
      bool reachable=call("getPid",request,&response)&&static_cast<int>(response[0])==1&&
          response[2].getType()==XmlRpc::XmlRpcValue::TypeInt;
      const int pid=reachable?static_cast<int>(response[2]):0;
      std::string run_id;
      if(reachable) {
        request[1]="/run_id";
        reachable=call("getParam",request,&response);
        if(reachable&&static_cast<int>(response[0])==1&&response[2].getType()==XmlRpc::XmlRpcValue::TypeString)
          run_id=static_cast<std::string>(response[2]);
      }
      if(reachable&&!ros_bound_.load()) {
        request[1]=ros::names::resolve("~use_sim_time");
        request[2]=world_clock_=="simulation";
        reachable=call("setParam",request,&response)&&static_cast<int>(response[0])==1;
      }
      std::chrono::milliseconds delay(2000);
      if(reachable) {
        {
          std::lock_guard<std::mutex> state(master_state_mutex_);
          if(master_pid_&&(pid!=master_pid_||(!master_run_id_.empty()&&run_id!=master_run_id_)))
            throw std::runtime_error("the ROS master was replaced by another run or process");
          master_pid_=pid;
          if(master_run_id_.empty())master_run_id_=run_id;
        }
        master_reachable_.store(true);
        if(!ros_bound_.load()) {
          // Avoid roscpp's unbounded retry loop if the master disappears during
          // this one native initialization. Later input connections are events.
          ros::master::setRetryTimeout(ros::WallDuration(.01));
          ros_runtime_.reset(new ros::NodeHandle);
          if(ros::Time::isSimTime()!=(world_clock_=="simulation"))
            throw std::invalid_argument("ROS clock remap conflicts with the explicit immutable world clock");
          ros_bound_.store(true);
          on_master_bound_();
        }
        backoff_ms=250;
      } else {
        master_reachable_.store(false);
        const auto now=std::chrono::steady_clock::now();
        if(!logged||now-last_log>=std::chrono::seconds(10)) {
          std::fprintf(stderr,"ROS master unreachable; retrying with backoff (%llu suppressed attempts)\n",
              static_cast<unsigned long long>(suppressed));
          last_log=now;suppressed=0;logged=true;
        } else ++suppressed;
        delay=std::chrono::milliseconds(std::uniform_int_distribution<int>(0,backoff_ms)(random));
        backoff_ms=std::min(30000,backoff_ms*2);
      }
      std::unique_lock<std::mutex> lock(wait_mutex_);
      wake_.wait_for(lock,delay,[this]{return stopping_.load();});
    }
  } catch(...) {fail(std::current_exception());}
}
void Server::stop() {
  stopping_.store(true);wakePublisher();
  if(master_watch_.joinable())master_watch_.join();
  if(publisher_.joinable())publisher_.join();
  input_.stop();
  for(auto& item:instances_)item.second->deactivate();
  instances_.clear();replaceSnapshot();
}
void Server::rethrowFailure() {std::lock_guard<std::mutex> lock(failure_mutex_);if(failure_)std::rethrow_exception(failure_);}
bool Server::ready() const {return !stopping_.load();}
Json::Value Server::facts() const {
  Json::Value result(Json::objectValue);
  result["ros_master_uri"]=master_uri_;
  {std::lock_guard<std::mutex> state(master_state_mutex_);
   result["ros_master_run_id"]=master_run_id_;result["ros_master_pid"]=master_pid_;}
  result["world_clock"]=world_clock_;
  result["ros_master_state"]=!ros_bound_.load()?"waiting for master":master_reachable_.load()?"connected":"unreachable";
  result["ros_master_connect_attempts"]=Json::UInt64(master_attempts_.load());
  result["callback_workers"]=Json::UInt64(input_.size());
  result["instances"]=Json::UInt64(instance_count_.load());
  result["robots"]=Json::UInt64(robot_count_.load());
  result["max_instances"]=Json::UInt64(kMaxInstances);
  result["max_robots_per_instance"]=Json::UInt64(kMaxRobots);
  if(stopping_.load())result["reason"]="stopping";
  else if(!master_reachable_.load())result["reason"]="ros master unreachable";
  return result;
}
Json::Value Server::ratesStatus() const {
  Json::Value result;result["ok"]=true;result["rates"]=std::atomic_load(&rates_)->json();
  result["desiredRevision"]=Json::UInt64(rates_revision_);result["appliedRevision"]=Json::UInt64(rates_revision_);
  result["persistedRevision"]=Json::Value();result["state"]="applied";result["appliedAtSteadyNs"]=Json::Int64(rates_applied_steady_ns_);
  return result;
}
void validateInstanceId(const std::string& id) {
  static const std::regex identity("^[A-Za-z0-9_.-]{1,128}$");
  if(!std::regex_match(id,identity))throw std::invalid_argument("invalid instance identity");
}
void validateRobotId(const std::string& id) {
  static const std::regex identity("^[A-Za-z_][A-Za-z0-9_]{0,126}$");
  if(!std::regex_match(id,identity))throw std::invalid_argument("invalid robot identity");
}
void Server::requireRunning() const {
  if(stopping_.load())throw ControlFailure(ControlError::Unavailable,"server is stopping");
}
void Server::requireMasterIfBound() const {
  if(native_active_&&!master_reachable_.load())throw ControlFailure(ControlError::Unavailable,"ros master unreachable");
}
Instance& Server::instance(const std::string& id) const {
  validateInstanceId(id);const auto found=instances_.find(id);
  if(found==instances_.end())throw ControlFailure(ControlError::NotFound,"instance not found");
  return *found->second;
}
void Server::requireFreeClaims(const std::string& id,const InstanceSpec& spec) const {
  const auto wanted=claims(spec);
  const auto listen=remoteListen(spec);
  for(const auto& item:instances_) {
    const auto existing=remoteListen(item.second->spec());
    if(!listen.empty() && !existing.empty() && listen!=existing)
      throw ControlFailure(ControlError::Conflict,"all active instances must share one Zenoh listen endpoint");
    if(item.first==id)continue;
    for(const auto& claim:item.second->claims())
      if(wanted.count(claim))throw ControlFailure(ControlError::Conflict,"instance output conflicts with "+item.first+": "+claim);
  }
}
Json::Value Server::status() const {
  Json::Value result;result["ok"]=true;result["ready"]=ready();result["callbackWorkers"]=Json::UInt64(input_.size());
  result["publisherWorkers"]=1;result["instanceCount"]=Json::UInt64(instances_.size());result["rates"]=std::atomic_load(&rates_)->json();
  Json::UInt64 robots=0,scene_robots=0,relays=0;result["instances"]=Json::Value(Json::arrayValue);
  for(const auto& item:instances_) {auto status=item.second->status();robots+=status["descriptionCount"].asUInt64();scene_robots+=status["robotCount"].asUInt64();relays+=status["relayCount"].asUInt64();result["instances"].append(status);}
  result["robotCount"]=scene_robots;result["descriptionCount"]=robots;result["relayCount"]=relays;return result;
}
Json::Value Server::replaceRates(std::uint64_t expected_revision,const Json::Value& complete_rates) {
  requireRunning();
  if(!expected_revision)throw std::invalid_argument("positive rate revision required");
  if(expected_revision!=rates_revision_)throw ControlFailure(ControlError::Conflict,"rate revision conflict");
  if(rates_revision_==std::numeric_limits<std::uint64_t>::max())throw ControlFailure(ControlError::Conflict,"rate revision exhausted");
  auto prepared=parseRates(complete_rates,true);std::shared_ptr<const Rates> replacement(new Rates(std::move(prepared)));std::atomic_store(&rates_,replacement);
  // This publication is the application boundary. A tick already holding
  // the old immutable snapshot finishes with that snapshot; subsequent
  // ticks load the new complete table, without partial channel updates.
  ++rates_revision_;rates_applied_steady_ns_=steadyNs();wakePublisher();return ratesStatus();
}
Json::Value Server::instanceStatus(const std::string& id) const {return instance(id).status();}
Json::Value Server::removeInstance(const std::string& id) {
  requireRunning();validateInstanceId(id);const auto found=instances_.find(id);
  if(found!=instances_.end()) {auto removed=found->second;instances_.erase(found);replaceSnapshot();removed->deactivate();}
  Json::Value result;result["ok"]=true;result["id"]=id;result["removed"]=true;return result;
}
Json::Value Server::applyInstance(const std::string& id,const Json::Value& input) {
  requireRunning();requireMasterIfBound();validateInstanceId(id);
  const auto spec=projectInstanceInput(input);
  const auto found=instances_.find(id);
  requireFreeClaims(id,spec);
  if(found==instances_.end()) {
    if(instances_.size()>=kMaxInstances)throw ControlFailure(ControlError::Conflict,"server supports at most 64 instances");
    std::shared_ptr<Instance> created(new Instance(id));
    const auto change=created->apply(spec,input_);
    if(native_active_)created->activate(input_);
    instances_.emplace(id,created);replaceSnapshot();
    auto status=created->status();status["created"]=true;status["unchanged"]=false;status["changes"]=change.json();return status;
  }
  const auto change=found->second->apply(spec,input_);
  countMembers();wakePublisher();
  auto status=found->second->status();status["created"]=false;status["unchanged"]=change.empty();status["changes"]=change.json();return status;
}
Json::Value Server::robotStatus(const std::string& id,const std::string& robot) const {
  auto& owner=instance(id);validateRobotId(robot);
  if(!owner.hasRobot(robot))throw ControlFailure(ControlError::NotFound,"robot not found");
  return owner.robotStatus(robot,*std::atomic_load(&rates_));
}
Json::Value Server::applyRobot(const std::string& id,const std::string& robot,const Json::Value& request) {
  requireRunning();requireMasterIfBound();auto& owner=instance(id);validateRobotId(robot);
  requireFields(request,{"profile"},{"expectedRevision"});
  std::uint64_t expected=0;
  if(expectedRevision(request,&expected)) {
    // 0 expects that the robot does not exist yet.
    const std::uint64_t current=owner.hasRobot(robot)?owner.robotRevision(robot):0;
    if(expected!=current)throw ControlFailure(ControlError::Conflict,"robot revision conflict");
  }
  InstanceSpec candidate=owner.spec();
  candidate.robots[robot]=parseRobotProfile(robot,request["profile"]);
  requireFreeClaims(id,candidate);
  const auto change=owner.apply(candidate,input_);
  countMembers();wakePublisher();
  auto status=owner.robotStatus(robot,*std::atomic_load(&rates_));status["unchanged"]=change.empty();status["changes"]=change.json();return status;
}
Json::Value Server::removeRobot(const std::string& id,const std::string& robot) {
  requireRunning();auto& owner=instance(id);validateRobotId(robot);
  Json::Value result;result["ok"]=true;result["instanceId"]=id;result["id"]=robot;result["removed"]=true;
  if(!owner.hasRobot(robot))return result;
  InstanceSpec candidate=owner.spec();candidate.robots.erase(robot);
  owner.apply(candidate,input_);
  countMembers();wakePublisher();return result;
}
Json::Value Server::robotRates(const std::string& id,const std::string& robot) const {
  auto status=robotStatus(id,robot);
  Json::Value result;
  for(const char* name:{"ok","instanceId","id","kind","configuration","rates"})result[name]=status[name];
  return result;
}
Json::Value Server::applyRobotRates(const std::string& id,const std::string& robot,const Json::Value& request) {
  requireRunning();auto& owner=instance(id);validateRobotId(robot);
  requireFields(request,{"rates"},{"expectedRevision"});
  if(!owner.hasRobot(robot))throw ControlFailure(ControlError::NotFound,"robot not found");
  std::uint64_t expected=0;
  if(expectedRevision(request,&expected)&&expected!=owner.robotRevision(robot))throw ControlFailure(ControlError::Conflict,"robot revision conflict");
  const auto overrides=parseRateOverrides(owner.spec().robots.at(robot).kind,request["rates"]);
  const bool changed=owner.setRobotRates(robot,overrides);
  if(changed)wakePublisher();
  auto result=robotRates(id,robot);result["unchanged"]=!changed;return result;
}
} // namespace xgc2_ros_visualizer

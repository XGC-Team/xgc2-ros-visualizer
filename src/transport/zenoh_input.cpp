#include "zenoh_input.hpp"
#include <zenoh.h>
#include <json/json.h>
#include <map>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <utility>
namespace xgc2_ros_visualizer {
namespace {
struct Session {
  z_owned_session_t value;
  explicit Session(const std::string& endpoint) {
    Json::Value config(Json::objectValue);
    config["mode"]="peer";
    config["connect"]["endpoints"]=Json::Value(Json::arrayValue);
    config["listen"]["endpoints"].append(endpoint);
    config["scouting"]["multicast"]["enabled"]=false;
    config["scouting"]["gossip"]["enabled"]=false;
    Json::StreamWriterBuilder writer;writer["indentation"]="";
    const auto document=Json::writeString(writer,config);
    z_owned_config_t c;
    if(zc_config_from_str(&c,document.c_str())<0) throw std::runtime_error("invalid Zenoh configuration");
    if(z_open(&value,z_move(c),nullptr)<0) throw std::runtime_error("cannot open Zenoh session");
  }
  ~Session() { z_drop(z_move(value)); }
};
std::shared_ptr<Session> sessionFor(const std::string& endpoint) {
  static std::mutex mutex;
  static std::map<std::string,std::weak_ptr<Session>> sessions;
  std::lock_guard<std::mutex> lock(mutex);
  auto existing=sessions[endpoint].lock();
  if(existing) return existing;
  auto created=std::make_shared<Session>(endpoint);
  sessions[endpoint]=created;
  // Expired entries are bounded by the number of endpoint changes, and are
  // removed whenever a session is opened; no timer or discovery loop.
  for(auto it=sessions.begin();it!=sessions.end();) {
    if(it->second.expired()) it=sessions.erase(it); else ++it;
  }
  return created;
}
struct CallbackState { std::size_t max_bytes; ZenohInput::Callback callback; };
void onSample(z_loaned_sample_t* sample,void* context) noexcept {
  if(z_sample_kind(sample)!=Z_SAMPLE_KIND_PUT) return;
  const auto bytes=z_sample_payload(sample);
  const auto* state=static_cast<CallbackState*>(context);
  if(!bytes || z_bytes_len(bytes)==0 || z_bytes_len(bytes)>state->max_bytes) return;
  z_owned_slice_t slice;
  if(z_bytes_to_slice(bytes,&slice)<0) return;
  try { state->callback(z_slice_data(z_loan(slice)),z_slice_len(z_loan(slice))); } catch (...) {}
  z_drop(z_move(slice));
}
void onDrop(void* context) { delete static_cast<CallbackState*>(context); }
}
void validateZenohSource(const std::string& endpoint,const std::string& key) {
  static const std::regex address("^tcp/([A-Za-z0-9_.-]+|\\[[0-9A-Fa-f:]+\\]):[0-9]{1,5}$");
  if(endpoint.size()>255 || !std::regex_match(endpoint,address)) throw std::invalid_argument("Zenoh listen must be an explicit TCP address and port");
  const auto port=std::stoul(endpoint.substr(endpoint.rfind(':')+1));
  if(port==0 || port>65535) throw std::invalid_argument("Zenoh listen port is invalid");
  if(key.empty() || key.size()>512 || key.find('*')!=std::string::npos) throw std::invalid_argument("Zenoh key must be exact");
  z_view_keyexpr_t expression;
  if(z_view_keyexpr_from_str(&expression,key.c_str())<0) throw std::invalid_argument("invalid Zenoh key");
}
std::shared_ptr<ZenohListener> ZenohListener::open(const std::string& endpoint) {
  // Reuse the same endpoint validation and session registry as subscriptions.
  validateZenohSource(endpoint,"xgc/listener-validation");
  return std::shared_ptr<ZenohListener>(new ZenohListener(sessionFor(endpoint)));
}
struct ZenohInput::Impl {
  std::shared_ptr<Session> session;
  z_owned_subscriber_t subscriber;
  ~Impl() { z_drop(z_move(subscriber)); }
};
ZenohInput::ZenohInput(std::unique_ptr<Impl> impl):impl_(std::move(impl)) {}
ZenohInput::~ZenohInput()=default;
std::unique_ptr<ZenohInput> ZenohInput::open(const std::string& endpoint,const std::string& key,
                                             std::size_t max_bytes,Callback callback) {
  validateZenohSource(endpoint,key);
  if(!callback || !max_bytes) throw std::invalid_argument("Zenoh input requires a callback and a byte limit");
  auto session=sessionFor(endpoint);
  z_view_keyexpr_t expression;
  if(z_view_keyexpr_from_str(&expression,key.c_str())<0) throw std::invalid_argument("invalid Zenoh key");
  z_owned_closure_sample_t closure;
  z_closure(&closure,onSample,onDrop,new CallbackState{max_bytes,std::move(callback)});
  z_owned_subscriber_t subscriber;
  if(z_declare_subscriber(z_loan(session->value),&subscriber,z_loan(expression),z_move(closure),nullptr)<0)
    throw std::runtime_error("cannot declare Zenoh subscriber");
  std::unique_ptr<Impl> impl(new Impl{std::move(session),subscriber});
  return std::unique_ptr<ZenohInput>(new ZenohInput(std::move(impl)));
}
} // namespace xgc2_ros_visualizer

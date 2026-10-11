#include "rpc.hpp"
#include <xgc2/xrpc/bounded_output.hpp>
#include <xgc2/xrpc/diagnostics.hpp>
#include <xgc2/xrpc/http.hpp>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace xgc2_ros_visualizer {
namespace {
using namespace xgc2::xrpc;

// Calls held by wait_ready_ms are bounded; each one retains host admission.
constexpr std::size_t kDescribeWaiters = 16;
// A held call answers shortly before its own deadline, with the current
// (not ready) document, instead of being cancelled by the host.
constexpr std::chrono::milliseconds kDescribeReplyMargin{50};

UnixOptions socketOptions(std::string path) {
  UnixOptions result;
  result.path = std::move(path);
  result.existing = ExistingPath::ReclaimUnreachable;
  return result;
}
Json::Value parseRequest(const std::string& text) {
  if (text.empty()) return Json::Value();
  Json::CharReaderBuilder builder;
  builder["allowComments"] = false;
  builder["collectComments"] = false;
  builder["allowTrailingCommas"] = false;
  builder["rejectDupKeys"] = true;
  builder["failIfExtra"] = true;
  builder["allowSpecialFloats"] = false;
  builder["stackLimit"] = 64;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  Json::Value result;
  std::string detail;
  if (!reader->parse(text.data(), text.data() + text.size(), &result, &detail))
    throw std::invalid_argument("invalid JSON request");
  return result;
}
void complete(const HttpReply& reply, RpcReply result, std::size_t limit) {
  BoundedOutput output(limit);
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(result.body, &output.stream());
  if (!output.good()) {
    reply.complete(http_error(500, "resource_exhausted", "JSON response exceeds limit"));
    return;
  }
  HttpResponse response;
  response.status = result.status;
  response.headers.emplace_back("Content-Type", "application/json");
  response.body = output.value();
  reply.complete(std::move(response));
}
// The only describe query: wait_ready_ms=<0..30000>.
std::chrono::milliseconds describeWait(const std::string& query) {
  if (query.empty()) return std::chrono::milliseconds::zero();
  const std::string key = "wait_ready_ms=";
  const auto digits = query.compare(0, key.size(), key) == 0 ? query.substr(key.size()) : std::string();
  if (digits.empty() || digits.size() > 5 || (digits.size() > 1 && digits[0] == '0') ||
      digits.find_first_not_of("0123456789") != std::string::npos)
    throw std::invalid_argument("describe takes only wait_ready_ms=<integer>");
  const auto milliseconds = std::stoul(digits);
  if (milliseconds > 30000) throw std::invalid_argument("wait_ready_ms exceeds 30000");
  return std::chrono::milliseconds(milliseconds);
}
DiagnosticsOptions diagnosticOptions() {
  DiagnosticsOptions options;
  // Only rejected, expired or failed calls; every successful call would
  // otherwise be one line of supervisor log.
  options.level = LogSeverity::Warn;
  return options;
}
bool writeRecord(void*, std::string_view record) {
  return std::fwrite(record.data(), 1, record.size(), stderr) == record.size();
}
}

class RpcServer::Impl {
 public:
  Impl(std::string path, RpcHandler handler, DescribeProvider describe, RpcOptions options,
       std::function<void()> quiesce_native, std::function<void()> activate_native)
      : handler_(std::move(handler)), describe_(std::move(describe)), slots_(limits_.inflight),
        diagnostics_(diagnosticOptions()), quiesce_native_(std::move(quiesce_native)),
        activate_native_(std::move(activate_native)),
        host_(socketOptions(std::move(path)), [this](HttpRequest request, HttpReply reply) {
          dispatch(std::move(request), std::move(reply));
        }, limits_, HttpIdentity{options.instance_id, {"/v1/describe"}}) {
    if (!handler_ || !describe_) throw std::invalid_argument("RPC handler and describe provider are required");
    if (options.instance_id.empty()) throw std::invalid_argument("instance_id is required");
    host_.set_diagnostics(&diagnostics_, "xgc2.visualization");
    worker_ = std::thread([this] { work(); });
  }
  ~Impl() { stop(); join(); quiesce(); }

  void dispatch(HttpRequest request, HttpReply reply) {
    const auto query_at = request.target.find('?');
    const std::string query = query_at == std::string::npos ? std::string() : request.target.substr(query_at + 1);
    request.target.resize(query_at == std::string::npos ? request.target.size() : query_at);
    if (request.target == "/v1/describe") {
      describe(request, query, std::move(reply));
      return;
    }
    if (!query.empty()) {
      reply.complete(http_error(400, "invalid_argument", "only describe takes a query"));
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_.load()) {
        reply.complete(http_error(503, "unavailable", "visualizer is stopping"));
        return;
      }
      if (size_ == slots_.size()) {
        reply.complete(http_error(503, "resource_exhausted", "domain handoff is full"));
        return;
      }
      slots_[tail_].emplace(Work{std::move(request), std::move(reply)});
      tail_ = (tail_ + 1) % slots_.size();
      ++size_;
    }
    wake_.notify_one();
  }
  void describe(const HttpRequest& request, const std::string& query, HttpReply reply) {
    if (request.method != "GET") {
      reply.complete(http_error(405, "invalid_argument", "describe supports GET"));
      return;
    }
    if (!request.body.empty()) {
      reply.complete(http_error(400, "invalid_argument", "GET resource requires an empty body"));
      return;
    }
    std::chrono::milliseconds wait;
    try {
      wait = describeWait(query);
    } catch (const std::invalid_argument& error) {
      reply.complete(http_error(400, "invalid_argument", error.what()));
      return;
    }
    auto document = describe_();
    if (wait.count() == 0 || document["ready"].asBool() || stopping_.load()) {
      complete(reply, {200, std::move(document)}, limits_.response_bytes);
      return;
    }
    if (waiters_.size() >= kDescribeWaiters) {
      reply.complete(http_error(503, "resource_exhausted", "describe waiter limit reached"));
      return;
    }
    waiters_.push_back({std::move(reply), std::min(Clock::now() + wait, request.deadline - kDescribeReplyMargin)});
  }
  // Transport owner thread only. A held call answers when the service becomes
  // ready, when its wait elapses or, at Stop, immediately.
  void answerDescribeWaiters(bool final) {
    if (waiters_.empty()) return;
    Json::Value document;
    bool loaded = false;
    const auto now = Clock::now();
    waiters_.erase(std::remove_if(waiters_.begin(), waiters_.end(), [&](const Waiter& waiter) {
      if (waiter.reply.cancelled()) return true;
      if (!loaded) {
        document = describe_();
        loaded = true;
      }
      if (!final && !document["ready"].asBool() && now < waiter.until) return false;
      complete(waiter.reply, {200, document}, limits_.response_bytes);
      return true;
    }), waiters_.end());
  }
  void work() noexcept {
    for (;;) {
      std::optional<Work> current;
      bool activate = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this] { return size_ || native_pending_ || stopping_.load(); });
        if (native_pending_ && !stopping_.load()) {
          native_pending_ = false;
          activate = true;
        } else {
          if (!size_) return;
          current = std::move(slots_[head_]);
          slots_[head_].reset();
          head_ = (head_ + 1) % slots_.size();
          --size_;
          busy_ = true;
        }
      }
      if (activate) {
        if (activate_native_) activate_native_();
        continue;
      }
      try {
        if (stopping_.load()) {
          current->reply.complete(http_error(503, "unavailable", "visualizer is stopping"));
        } else if (!current->reply.cancelled() && Clock::now() < current->request.deadline) {
          if ((current->request.method == "GET" || current->request.method == "DELETE") &&
              !current->request.body.empty())
            throw std::invalid_argument("GET and DELETE require an empty body");
          const auto body = parseRequest(current->request.body);
          // Admission does not assert native completion. Once started, native
          // work remains owned even when the peer/deadline cancels its reply.
          if (!current->reply.cancelled() && Clock::now() < current->request.deadline)
            complete(current->reply, handler_(current->request.method, current->request.target, body),
                     limits_.response_bytes);
        }
      } catch (const std::invalid_argument& error) {
        current->reply.complete(http_error(400, "invalid_argument", error.what()));
      } catch (...) {
        current->reply.complete(http_error(500, "internal", "internal RPC handler failure"));
      }
      current.reset();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
      }
    }
  }
  void notifyNativeReady() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      native_pending_ = true;
    }
    wake_.notify_one();
  }
  void stop() noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_.store(true);
    }
    // Wake the owner without releasing the endpoint. Its native quiescence
    // phase still owns the lease; only the later SDK drain closes it.
    host_.wake();
    wake_.notify_one();
  }
  void join() { if (worker_.joinable()) worker_.join(); }
  void quiesce() {
    if (!native_quiesced_) {
      native_quiesced_ = true;
      if (quiesce_native_) quiesce_native_();
    }
  }
  void drainDiagnostics() { diagnostics_.drain(32, writeRecord); }
  void run(const std::atomic<bool>& external_stop) {
    while (!external_stop.load() && !stopping_.load()) {
      host_.poll(std::chrono::milliseconds(50));
      answerDescribeWaiters(false);
      drainDiagnostics();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_.store(true);
    }
    answerDescribeWaiters(true);
    wake_.notify_one();
    join();
    // No new RPC is dispatched while this owner is outside poll(). Complete
    // native workers and publication fences before initiating SDK lease close.
    quiesce();
    host_.drain();
    drainDiagnostics();
  }
  const std::string& socket_path() const noexcept { return host_.socket_path(); }

 private:
  struct Work { HttpRequest request; HttpReply reply; };
  struct Waiter { HttpReply reply; Clock::time_point until; };
  RpcHandler handler_;
  DescribeProvider describe_;
  // The SDK defaults: 32 connections and calls, 16 KiB headers, 1 MiB bodies.
  HttpLimits limits_;
  std::vector<std::optional<Work>> slots_;
  Diagnostics diagnostics_;
  std::function<void()> quiesce_native_;
  std::function<void()> activate_native_;
  bool native_pending_{false};
  bool native_quiesced_{false};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::size_t head_{0}, tail_{0}, size_{0};
  bool busy_{false};
  std::atomic<bool> stopping_{false};
  std::vector<Waiter> waiters_;  // transport owner thread only
  HttpServer host_;
  std::thread worker_;
};

RpcServer::RpcServer(std::string path, RpcHandler handler, DescribeProvider describe, RpcOptions options,
                     std::function<void()> quiesce_native, std::function<void()> activate_native)
    : impl_(new Impl(std::move(path), std::move(handler), std::move(describe), std::move(options),
                     std::move(quiesce_native), std::move(activate_native))) {}
RpcServer::~RpcServer() = default;
void RpcServer::run(const std::atomic<bool>& stopping) { impl_->run(stopping); }
void RpcServer::stop() noexcept { impl_->stop(); }
void RpcServer::notifyNativeReady() { impl_->notifyNativeReady(); }
const std::string& RpcServer::socket_path() const noexcept { return impl_->socket_path(); }
} // namespace xgc2_ros_visualizer

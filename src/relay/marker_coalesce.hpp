#pragma once
#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace xgc2_ros_visualizer {
// Display copy of Marker / MarkerArray traffic.
//
// Epochs split on DELETEALL. Inside an epoch each (ns, id) keeps only its
// latest ADD or DELETE. A later ADD of a different id does not erase a DELETE.
//
// Lifetime. ros::Duration 0 means forever and is forwarded unchanged. A positive
// lifetime is always anchored at the ROS time this relay ingested that ADD.
// header.stamp is geometry/TF time and is copied through unchanged. The viewer
// expires a marker at receive time plus the lifetime in the message, so a
// delayed send must not forward the original duration. Each new ADD, including
// a periodic refresh of the same stamp and the same (ns, id), stores a new
// ingest anchor and starts the lifetime over.
//   - still alive: the sent lifetime is the time left since that ingest, never
//     longer than the original;
//   - already dead and this relay has published the id: emit DELETE;
//   - already dead and never published: drop it.
// Pending ADDs that die while waiting are dropped or turned into that DELETE
// before the next flush, so they do not keep points until a late send. A
// published id expires at the send time plus the lifetime that was advertised.
//
// Singular Marker uses takeOne: one op per due period. Unsent ops and barriers
// stay queued. The next id is the first key strictly after the last emitted
// id in (ns, id) order, wrapping to the lowest, so a busy low id cannot take
// every period. A barrier still goes out before ops ingested after it. An id
// is marked published only when that op is returned. MarkerArray uses take,
// which returns one bounded batch because that batch is one message.
//
// Bounds. At most kMaxKeys pending (ns, id) entries, kMaxKeys published ids,
// and kMaxMessageBytes of pending ADD payload (points, colors, text,
// mesh_resource). kMaxMessageBytes is the existing display-budget
// maxMessageBytes contract, 1 MiB, the same value as simpleLidarMaxMessageBytes.
//
// Overflow policy.
//   - A key-count or payload-byte overflow is namespace-local. The namespace
//     that did not fit is dropped from the pending map. The next flush emits
//     DELETEALL for that namespace only. Other namespaces stay, including
//     ones already published. An empty namespace cannot use DELETEALL, because
//     an empty namespace means every namespace; that case emits one DELETE per
//     published id in the empty namespace and drops its pending geometry.
//   - A published-id overflow uses the same namespace-local wipe: the namespace
//     of the ADD that would push the published set past kMaxKeys is cleared,
//     and other published namespaces stay.
//   - One global empty-namespace DELETEALL is used only when order cannot be
//     kept: an unknown action, or more than kMaxBarriers DELETEALL barriers.
//     That wipe forgets every namespace. It is the order boundary, not the
//     response to a large point list or a full namespace.
struct MarkerKey {
  std::string ns;
  int id{0};
  bool operator<(const MarkerKey& other) const {
    if (ns != other.ns) return ns < other.ns;
    return id < other.id;
  }
};

class MarkerCoalescer {
 public:
  static const std::size_t kMaxKeys = 4096;
  static const std::size_t kMaxBarriers = 32;
  static const std::size_t kMaxMessageBytes = 1048576;

  void ingest(const visualization_msgs::Marker& marker, ros::Time now = ros::Time());
  void ingest(const visualization_msgs::MarkerArray& array, ros::Time now = ros::Time()) {
    for (std::size_t i = 0; i < array.markers.size(); ++i) ingest(array.markers[i], now);
  }
  std::vector<visualization_msgs::Marker> take(ros::Time now = ros::Time());
  bool takeOne(ros::Time now, visualization_msgs::Marker* out);
  std::size_t keyCount() const;
  std::size_t barrierCount() const;
  std::size_t publishedCount() const { return published_.size(); }
  std::size_t payloadBytes() const { return payload_bytes_; }
  bool overflowed() const { return overflow_; }

 private:
  struct PendingOp {
    visualization_msgs::Marker marker;
    ros::Time anchor;
    bool immortal{true};
    std::size_t bytes{0};
  };
  struct Live {
    bool immortal{true};
    ros::Time expiry;
  };
  struct Epoch {
    int barrier{-1};  // -1 none, 0 global DELETEALL, 1 one namespace
    std::string barrier_ns;
    std::map<MarkerKey, PendingOp> ops;
  };
  std::vector<Epoch> epochs_;
  std::map<MarkerKey, Live> published_;
  std::set<std::string> overflow_ns_;
  bool overflow_{false};
  std::size_t payload_bytes_{0};
  bool have_cursor_{false};
  MarkerKey cursor_{};

  void resetOverflow();
  void ensureOpen();
  void pushBarrier(int kind, const std::string& ns);
  void overflowNamespace(const std::string& ns);
  void expirePending(const ros::Time& now);
  void recomputePayload();
  std::size_t lastWipe(const MarkerKey& key) const;
  void eraseKeyFrom(const MarkerKey& key, std::size_t begin);
  bool dropPending(const MarkerKey& key);
  bool pendingHas(const MarkerKey& key) const;
  void removeNamespace(const std::string& ns);
  bool coveredByGlobal() const;
  bool pendingNamespaceBarrier(const std::string& ns) const;
  bool namespaceCovered(const std::string& ns) const;
  void notePublished(const std::vector<visualization_msgs::Marker>& sent, const ros::Time& now);
  static bool liveExpired(const Live& live, const ros::Time& now);
  static bool pendingExpired(const PendingOp& op, const ros::Time& now);
  static visualization_msgs::Marker slimDelete(const visualization_msgs::Marker& marker);
  static visualization_msgs::Marker emitAdd(const PendingOp& op, const ros::Time& now, bool* alive);
  MarkerKey selectFair(const std::set<MarkerKey>& keys) const;
  void commitCursor(const MarkerKey& key);
  void fillDrainKeys(const ros::Time& now, std::set<MarkerKey>* keys) const;
};

inline std::size_t markerPayloadBytes(const visualization_msgs::Marker& marker) {
  // ROS1 wire sizes of the variable payload: Point is 3 float64, ColorRGBA is 4 float32.
  return marker.points.size() * 24 + marker.colors.size() * 16 + marker.text.size() + marker.mesh_resource.size();
}

inline void MarkerCoalescer::resetOverflow() {
  epochs_.clear();
  Epoch barrier;
  barrier.barrier = 0;
  epochs_.push_back(barrier);
  overflow_ = true;
  overflow_ns_.clear();
  payload_bytes_ = 0;
  have_cursor_ = false;
}
inline void MarkerCoalescer::ensureOpen() {
  if (epochs_.empty()) epochs_.push_back(Epoch());
}
inline void MarkerCoalescer::pushBarrier(int kind, const std::string& ns) {
  Epoch barrier;
  barrier.barrier = kind;
  barrier.barrier_ns = ns;
  if (!epochs_.empty() && epochs_.back().barrier < 0 && epochs_.back().ops.empty()) epochs_.insert(epochs_.end() - 1, barrier);
  else {
    epochs_.push_back(barrier);
    epochs_.push_back(Epoch());
  }
}
inline void MarkerCoalescer::recomputePayload() {
  std::size_t bytes = 0;
  for (std::size_t i = 0; i < epochs_.size(); ++i)
    for (std::map<MarkerKey, PendingOp>::const_iterator it = epochs_[i].ops.begin(); it != epochs_[i].ops.end(); ++it)
      bytes += it->second.bytes;
  payload_bytes_ = bytes;
}
inline std::size_t MarkerCoalescer::keyCount() const {
  std::size_t count = 0;
  for (std::size_t i = 0; i < epochs_.size(); ++i) count += epochs_[i].ops.size();
  return count;
}
inline std::size_t MarkerCoalescer::barrierCount() const {
  std::size_t count = 0;
  for (std::size_t i = 0; i < epochs_.size(); ++i) if (epochs_[i].barrier >= 0) ++count;
  return count;
}
inline std::size_t MarkerCoalescer::lastWipe(const MarkerKey& key) const {
  std::size_t begin = 0;
  for (std::size_t i = 0; i < epochs_.size(); ++i) {
    const Epoch& epoch = epochs_[i];
    if (epoch.barrier == 0 || (epoch.barrier == 1 && epoch.barrier_ns == key.ns)) begin = i + 1;
  }
  return begin;
}
inline void MarkerCoalescer::eraseKeyFrom(const MarkerKey& key, std::size_t begin) {
  for (std::size_t i = begin; i < epochs_.size(); ++i) epochs_[i].ops.erase(key);
}
inline bool MarkerCoalescer::dropPending(const MarkerKey& key) {
  const std::size_t begin = lastWipe(key);
  bool had_add = false;
  for (std::size_t i = begin; i < epochs_.size(); ++i) {
    const std::map<MarkerKey, PendingOp>::const_iterator found = epochs_[i].ops.find(key);
    if (found != epochs_[i].ops.end() && found->second.marker.action == visualization_msgs::Marker::ADD) had_add = true;
    epochs_[i].ops.erase(key);
  }
  return had_add;
}
inline bool MarkerCoalescer::pendingHas(const MarkerKey& key) const {
  const std::size_t begin = lastWipe(key);
  for (std::size_t i = begin; i < epochs_.size(); ++i)
    if (epochs_[i].ops.count(key)) return true;
  return false;
}
inline void MarkerCoalescer::removeNamespace(const std::string& ns) {
  for (std::size_t i = 0; i < epochs_.size(); ++i) {
    for (std::map<MarkerKey, PendingOp>::iterator it = epochs_[i].ops.begin(); it != epochs_[i].ops.end();) {
      if (it->first.ns == ns) it = epochs_[i].ops.erase(it);
      else ++it;
    }
  }
}
inline bool MarkerCoalescer::coveredByGlobal() const {
  for (std::size_t i = 0; i < epochs_.size(); ++i) if (epochs_[i].barrier == 0) return true;
  return false;
}
inline bool MarkerCoalescer::pendingNamespaceBarrier(const std::string& ns) const {
  for (std::size_t i = 0; i < epochs_.size(); ++i)
    if (epochs_[i].barrier == 1 && epochs_[i].barrier_ns == ns) return true;
  return false;
}
inline bool MarkerCoalescer::namespaceCovered(const std::string& ns) const {
  return coveredByGlobal() || pendingNamespaceBarrier(ns) || overflow_ns_.count(ns) > 0;
}
inline bool MarkerCoalescer::liveExpired(const Live& live, const ros::Time& now) {
  return !live.immortal && live.expiry <= now;
}
inline bool MarkerCoalescer::pendingExpired(const PendingOp& op, const ros::Time& now) {
  if (op.immortal) return false;
  return op.anchor + op.marker.lifetime <= now;
}
inline visualization_msgs::Marker MarkerCoalescer::slimDelete(const visualization_msgs::Marker& marker) {
  visualization_msgs::Marker out;
  out.header.frame_id = marker.header.frame_id;
  out.ns = marker.ns;
  out.id = marker.id;
  out.action = visualization_msgs::Marker::DELETE;
  return out;
}
inline visualization_msgs::Marker MarkerCoalescer::emitAdd(const PendingOp& op, const ros::Time& now, bool* alive) {
  visualization_msgs::Marker emit = op.marker;
  emit.action = visualization_msgs::Marker::ADD;
  if (op.immortal) {
    emit.lifetime = ros::Duration(0);
    *alive = true;
    return emit;
  }
  ros::Duration remain = (op.anchor + op.marker.lifetime) - now;
  if (remain > op.marker.lifetime) remain = op.marker.lifetime;
  *alive = remain > ros::Duration(0);
  emit.lifetime = *alive ? remain : ros::Duration(0);
  return emit;
}
inline void MarkerCoalescer::overflowNamespace(const std::string& ns) {
  removeNamespace(ns);
  overflow_ns_.insert(ns);
  // Empty ns DELETEALL would wipe every namespace. Per-id DELETEs are emitted at take.
  if (!ns.empty() && !coveredByGlobal() && !pendingNamespaceBarrier(ns)) pushBarrier(1, ns);
  recomputePayload();
  if (barrierCount() > kMaxBarriers) resetOverflow();
}
inline void MarkerCoalescer::expirePending(const ros::Time& now) {
  for (std::size_t i = 0; i < epochs_.size(); ++i) {
    for (std::map<MarkerKey, PendingOp>::iterator it = epochs_[i].ops.begin(); it != epochs_[i].ops.end();) {
      PendingOp& op = it->second;
      if (op.marker.action != visualization_msgs::Marker::ADD || !pendingExpired(op, now)) {
        ++it;
        continue;
      }
      if (published_.count(it->first) && !namespaceCovered(it->first.ns)) {
        op.marker = slimDelete(op.marker);
        op.immortal = true;
        op.bytes = 0;
        op.anchor = ros::Time();
        ++it;
      } else {
        it = epochs_[i].ops.erase(it);
      }
    }
  }
  recomputePayload();
}
inline void MarkerCoalescer::notePublished(const std::vector<visualization_msgs::Marker>& sent, const ros::Time& now) {
  for (std::size_t i = 0; i < sent.size(); ++i) {
    const visualization_msgs::Marker& marker = sent[i];
    if (marker.action == visualization_msgs::Marker::DELETEALL) {
      if (marker.ns.empty()) published_.clear();
      else {
        for (std::map<MarkerKey, Live>::iterator it = published_.begin(); it != published_.end();) {
          if (it->first.ns == marker.ns) it = published_.erase(it);
          else ++it;
        }
      }
      continue;
    }
    MarkerKey key;
    key.ns = marker.ns;
    key.id = marker.id;
    if (marker.action == visualization_msgs::Marker::DELETE) published_.erase(key);
    else {
      Live live;
      live.immortal = marker.lifetime <= ros::Duration(0);
      if (!live.immortal) live.expiry = now + marker.lifetime;
      published_[key] = live;
    }
  }
}
inline void MarkerCoalescer::ingest(const visualization_msgs::Marker& marker, ros::Time now) {
  if (overflow_) return;
  // A namespace already scheduled for a local wipe ignores further geometry until that DELETEALL is sent.
  // A source-wide DELETEALL still applies: the caller asked to clear every namespace.
  if (overflow_ns_.count(marker.ns) && !(marker.action == visualization_msgs::Marker::DELETEALL && marker.ns.empty())) return;
  expirePending(now);
  if (marker.action == visualization_msgs::Marker::DELETEALL) {
    if (marker.ns.empty()) {
      epochs_.clear();
      pushBarrier(0, "");
      overflow_ns_.clear();
      payload_bytes_ = 0;
    } else {
      removeNamespace(marker.ns);
      if (!coveredByGlobal() && !pendingNamespaceBarrier(marker.ns)) pushBarrier(1, marker.ns);
      recomputePayload();
    }
    if (barrierCount() > kMaxBarriers || keyCount() > kMaxKeys) resetOverflow();
    return;
  }
  if (marker.action != visualization_msgs::Marker::ADD && marker.action != visualization_msgs::Marker::DELETE) {
    resetOverflow();
    return;
  }
  MarkerKey key;
  key.ns = marker.ns;
  key.id = marker.id;
  ensureOpen();
  if (epochs_.back().barrier >= 0) epochs_.push_back(Epoch());
  if (marker.action == visualization_msgs::Marker::DELETE) {
    dropPending(key);
    recomputePayload();
    if (!published_.count(key) || namespaceCovered(key.ns)) return;
    if (keyCount() + 1 > kMaxKeys) {
      overflowNamespace(key.ns);
      return;
    }
    PendingOp op;
    op.marker = slimDelete(marker);
    op.immortal = true;
    epochs_.back().ops[key] = op;
    return;
  }
  eraseKeyFrom(key, lastWipe(key));
  recomputePayload();
  const std::size_t add_bytes = markerPayloadBytes(marker);
  if (add_bytes > kMaxMessageBytes || payload_bytes_ + add_bytes > kMaxMessageBytes || keyCount() + 1 > kMaxKeys) {
    overflowNamespace(key.ns);
    return;
  }
  PendingOp op;
  op.marker = marker;
  op.marker.action = visualization_msgs::Marker::ADD;
  op.bytes = add_bytes;
  op.immortal = marker.lifetime <= ros::Duration(0);
  // Always the ingest clock. A later ADD of this key replaces this record, so
  // the same header.stamp starts a new lifetime instead of shortening the old one.
  if (!op.immortal) op.anchor = now;
  epochs_.back().ops[key] = op;
  payload_bytes_ += add_bytes;
}
inline std::vector<visualization_msgs::Marker> MarkerCoalescer::take(ros::Time now) {
  std::vector<visualization_msgs::Marker> out;
  if (overflow_) {
    visualization_msgs::Marker wipe;
    wipe.action = visualization_msgs::Marker::DELETEALL;
    out.push_back(wipe);
    notePublished(out, now);
    epochs_.clear();
    epochs_.push_back(Epoch());
    overflow_ = false;
    overflow_ns_.clear();
    payload_bytes_ = 0;
    have_cursor_ = false;
    return out;
  }
  expirePending(now);
  if (overflow_ns_.count("")) {
    for (std::map<MarkerKey, Live>::const_iterator it = published_.begin(); it != published_.end(); ++it) {
      if (!it->first.ns.empty()) continue;
      visualization_msgs::Marker wipe;
      wipe.ns = it->first.ns;
      wipe.id = it->first.id;
      wipe.action = visualization_msgs::Marker::DELETE;
      out.push_back(wipe);
    }
  }
  for (std::map<MarkerKey, Live>::const_iterator it = published_.begin(); it != published_.end(); ++it) {
    if (!liveExpired(it->second, now) || pendingHas(it->first) || namespaceCovered(it->first.ns)) continue;
    visualization_msgs::Marker wipe;
    wipe.ns = it->first.ns;
    wipe.id = it->first.id;
    wipe.action = visualization_msgs::Marker::DELETE;
    out.push_back(wipe);
  }
  for (std::size_t i = 0; i < epochs_.size(); ++i) {
    const Epoch& epoch = epochs_[i];
    if (epoch.barrier == 0) {
      visualization_msgs::Marker wipe;
      wipe.action = visualization_msgs::Marker::DELETEALL;
      out.push_back(wipe);
    } else if (epoch.barrier == 1) {
      visualization_msgs::Marker wipe;
      wipe.action = visualization_msgs::Marker::DELETEALL;
      wipe.ns = epoch.barrier_ns;
      out.push_back(wipe);
    }
    for (std::map<MarkerKey, PendingOp>::const_iterator it = epoch.ops.begin(); it != epoch.ops.end(); ++it)
      if (it->second.marker.action == visualization_msgs::Marker::DELETE) out.push_back(it->second.marker);
    for (std::map<MarkerKey, PendingOp>::const_iterator it = epoch.ops.begin(); it != epoch.ops.end(); ++it) {
      if (it->second.marker.action == visualization_msgs::Marker::DELETE) continue;
      bool alive = false;
      visualization_msgs::Marker emit = emitAdd(it->second, now, &alive);
      if (!alive) {
        if (published_.count(it->first) && !namespaceCovered(it->first.ns)) out.push_back(slimDelete(emit));
        continue;
      }
      out.push_back(emit);
    }
  }
  std::set<std::string> shed;
  {
    std::map<MarkerKey, Live> sim = published_;
    for (std::size_t i = 0; i < out.size(); ++i) {
      const visualization_msgs::Marker& marker = out[i];
      if (marker.action == visualization_msgs::Marker::DELETEALL) {
        if (marker.ns.empty()) sim.clear();
        else {
          for (std::map<MarkerKey, Live>::iterator it = sim.begin(); it != sim.end();) {
            if (it->first.ns == marker.ns) it = sim.erase(it);
            else ++it;
          }
        }
        continue;
      }
      MarkerKey key;
      key.ns = marker.ns;
      key.id = marker.id;
      if (marker.action == visualization_msgs::Marker::DELETE) sim.erase(key);
      else if (!sim.count(key)) {
        if (sim.size() >= kMaxKeys) shed.insert(marker.ns);
        else {
          Live live;
          live.immortal = true;
          sim[key] = live;
        }
      }
    }
  }
  if (!shed.empty()) {
    std::vector<visualization_msgs::Marker> kept;
    std::set<std::string> have_barrier;
    for (std::size_t i = 0; i < out.size(); ++i) {
      const visualization_msgs::Marker& marker = out[i];
      if (marker.action == visualization_msgs::Marker::DELETEALL && shed.count(marker.ns)) have_barrier.insert(marker.ns);
      if (marker.action == visualization_msgs::Marker::ADD && shed.count(marker.ns)) continue;
      kept.push_back(marker);
    }
    for (std::set<std::string>::const_iterator it = shed.begin(); it != shed.end(); ++it) {
      if (have_barrier.count(*it)) continue;
      if (it->empty()) {
        for (std::map<MarkerKey, Live>::const_iterator live = published_.begin(); live != published_.end(); ++live) {
          if (!live->first.ns.empty()) continue;
          visualization_msgs::Marker wipe;
          wipe.id = live->first.id;
          wipe.action = visualization_msgs::Marker::DELETE;
          kept.push_back(wipe);
        }
      } else {
        visualization_msgs::Marker wipe;
        wipe.action = visualization_msgs::Marker::DELETEALL;
        wipe.ns = *it;
        kept.push_back(wipe);
      }
    }
    out.swap(kept);
  }
  if (!out.empty()) notePublished(out, now);
  epochs_.clear();
  epochs_.push_back(Epoch());
  overflow_ns_.clear();
  payload_bytes_ = 0;
  have_cursor_ = false;
  return out;
}
inline MarkerKey MarkerCoalescer::selectFair(const std::set<MarkerKey>& keys) const {
  if (!have_cursor_) return *keys.begin();
  const std::set<MarkerKey>::const_iterator next = keys.upper_bound(cursor_);
  if (next == keys.end()) return *keys.begin();
  return *next;
}
inline void MarkerCoalescer::commitCursor(const MarkerKey& key) {
  have_cursor_ = true;
  cursor_ = key;
}
inline void MarkerCoalescer::fillDrainKeys(const ros::Time& now, std::set<MarkerKey>* keys) const {
  if (!epochs_.empty()) {
    const Epoch& front = epochs_.front();
    for (std::map<MarkerKey, PendingOp>::const_iterator it = front.ops.begin(); it != front.ops.end(); ++it)
      keys->insert(it->first);
  }
  for (std::map<MarkerKey, Live>::const_iterator it = published_.begin(); it != published_.end(); ++it) {
    const bool owed = (overflow_ns_.count("") && it->first.ns.empty()) ||
                      (liveExpired(it->second, now) && !pendingHas(it->first) && !namespaceCovered(it->first.ns));
    if (owed) keys->insert(it->first);
  }
}
inline bool MarkerCoalescer::takeOne(ros::Time now, visualization_msgs::Marker* out) {
  if (!out) return false;
  for (int spin = 0; spin < 8; ++spin) {
    if (overflow_) {
      visualization_msgs::Marker wipe;
      wipe.action = visualization_msgs::Marker::DELETEALL;
      std::vector<visualization_msgs::Marker> sent(1, wipe);
      notePublished(sent, now);
      epochs_.clear();
      epochs_.push_back(Epoch());
      overflow_ = false;
      overflow_ns_.clear();
      payload_bytes_ = 0;
      have_cursor_ = false;
      *out = wipe;
      return true;
    }
    expirePending(now);
    while (!epochs_.empty() && epochs_.front().barrier < 0 && epochs_.front().ops.empty()) epochs_.erase(epochs_.begin());
    if (!epochs_.empty() && epochs_.front().barrier >= 0) {
      Epoch& front = epochs_.front();
      visualization_msgs::Marker wipe;
      wipe.action = visualization_msgs::Marker::DELETEALL;
      if (front.barrier == 1) wipe.ns = front.barrier_ns;
      front.barrier = -1;
      front.barrier_ns.clear();
      if (front.ops.empty()) epochs_.erase(epochs_.begin());
      std::vector<visualization_msgs::Marker> sent(1, wipe);
      notePublished(sent, now);
      *out = wipe;
      return true;
    }
    std::set<MarkerKey> keys;
    fillDrainKeys(now, &keys);
    if (keys.empty()) return false;
    const MarkerKey key = selectFair(keys);
    if (!epochs_.empty()) {
      const std::map<MarkerKey, PendingOp>::iterator found = epochs_.front().ops.find(key);
      if (found != epochs_.front().ops.end()) {
        const bool add = found->second.marker.action != visualization_msgs::Marker::DELETE;
        if (add && !published_.count(key) && published_.size() >= kMaxKeys) {
          overflowNamespace(key.ns);
          continue;
        }
        bool alive = true;
        visualization_msgs::Marker emit = found->second.marker;
        if (add) emit = emitAdd(found->second, now, &alive);
        if (!alive) {
          epochs_.front().ops.erase(found);
          recomputePayload();
          if (!published_.count(key) || namespaceCovered(key.ns)) continue;
          emit = slimDelete(emit);
        } else {
          epochs_.front().ops.erase(found);
          recomputePayload();
        }
        if (!epochs_.empty() && epochs_.front().barrier < 0 && epochs_.front().ops.empty()) epochs_.erase(epochs_.begin());
        std::vector<visualization_msgs::Marker> sent(1, emit);
        notePublished(sent, now);
        commitCursor(key);
        *out = emit;
        return true;
      }
    }
    visualization_msgs::Marker wipe;
    wipe.ns = key.ns;
    wipe.id = key.id;
    wipe.action = visualization_msgs::Marker::DELETE;
    std::vector<visualization_msgs::Marker> sent(1, wipe);
    notePublished(sent, now);
    commitCursor(key);
    bool empty_left = false;
    for (std::map<MarkerKey, Live>::const_iterator it = published_.begin(); it != published_.end(); ++it)
      if (it->first.ns.empty()) empty_left = true;
    if (!empty_left) overflow_ns_.erase("");
    *out = wipe;
    return true;
  }
  return false;
}
}  // namespace xgc2_ros_visualizer

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

// Sender-thread owned. The MCU checks spacing at reception, not at the
// Python sendcan deadline. A slow SPI transaction can compress the next gap.
class PsaT9TxTiming {
public:
  static constexpr uint64_t MIN_GAP_NS = 46000000ULL;  // MCU: >=45 ms, plus clock margin.
  uint64_t delay_ns(uint64_t now, bool nonzero_torque) const {
    if (!nonzero_torque || !completed_) return 0;
    const uint64_t earliest = completed_ + MIN_GAP_NS;
    return now < earliest ? earliest-now : 0;
  }
  void completed(uint64_t now) { completed_ = now; }
private:
  uint64_t completed_ = 0;
};

// Main-loop owned state. Only tx_ready is read by the sender thread.
class PsaT9Guard {
public:
  enum class Stage { WAITING, PROBING, ACTIVE, FAILED };
  static constexpr uint16_t ACTIVE_PARAM = 0x1308;
  static constexpr uint16_t PROBE_PARAM = 0x1309;
  explicit PsaT9Guard(bool rvv_only = false, bool combined = false, bool split_axes = false, bool eps_cycle = false)
    : rvv_only_(rvv_only && !combined), combined_(combined), split_axes_(combined && split_axes), eps_cycle_(combined && split_axes && eps_cycle) {}
  uint16_t active_param() const { return eps_cycle_ ? 0x1316 : split_axes_ ? 0x1314 : combined_ ? 0x1312 : rvv_only_ ? 0x1310 : ACTIVE_PARAM; }
  uint16_t probe_param() const { return eps_cycle_ ? 0x1317 : split_axes_ ? 0x1315 : combined_ ? 0x1313 : rvv_only_ ? 0x1311 : PROBE_PARAM; }
  bool rvv_only() const { return rvv_only_; }
  bool split_axes() const { return split_axes_; }
  std::atomic<bool> tx_ready{false};
  Stage stage = Stage::WAITING;
  const char *reason = "waiting_for_stationary_vehicle";

  void reset() {
    tx_ready = false; stage = Stage::WAITING; reason = "waiting_for_stationary_vehicle";
    started_ = 0; last_speed_ = 0; stationary_since_ = 0;
    stock_ = 0; eps_ = 0; stock_count_ = 0; eps_count_ = 0; wrong_ = false;
    rvv_stock_ = 0; rvv_engine_ = 0; rvv_stock_count_ = 0; rvv_engine_count_ = 0;
  }
  void fail(const char *why) {
    tx_ready = false;
    if (stage == Stage::FAILED) return;  // Preserve the first cause, not a later consequence.
    stage = Stage::FAILED; reason = why;
  }
  void update(uint32_t address, uint8_t bus, const uint8_t *data, size_t length, uint64_t now) {
    if ((address == 0x30DU) && ((bus == 0U) || (bus == 2U)) && (length == 8)) {
      bool stopped = true;
      for (int i = 0; i < 8; i += 2) stopped &= ((data[i] << 8) | data[i + 1]) <= 10; // 0.1 km/h
      if (!stopped || (last_speed_ && now-last_speed_ > 150000000ULL)) stationary_since_ = 0;
      else if (!stationary_since_) stationary_since_ = now;
      last_speed_ = now;
    }
    if ((stage != Stage::PROBING) && (stage != Stage::ACTIVE)) return;
    if (now < started_) { fail("clock_regression"); return; }
    // Drain in-flight frames immediately after the relay command.
    if (now-started_ < 250000000ULL) return;
    if (address == (rvv_only_ ? 0x50EU : 0x3F2U) && (bus == 0U || bus == 2U)) {
      if (bus != 2U || length != 8) wrong_ = true;
      else { stock_ = now; ++stock_count_; }
    }
    if (address == (rvv_only_ ? 0x208U : 0x495U) && (bus == 0U || bus == 2U)) {
      if (bus != 0U || length != (rvv_only_ ? 8U : 4U)) wrong_ = true;
      else { eps_ = now; ++eps_count_; }
    }
    if (combined_ && address == 0x50EU && (bus == 0U || bus == 2U)) {
      if (bus != 2U || length != 8U) wrong_ = true;
      else { rvv_stock_ = now; ++rvv_stock_count_; }
    }
    if (combined_ && address == 0x208U && (bus == 0U || bus == 2U)) {
      if (bus != 0U || length != 8U) wrong_ = true;
      else { rvv_engine_ = now; ++rvv_engine_count_; }
    }
    if (wrong_) fail("stock_command_and_eps_not_isolated");
  }
  bool stationary(uint64_t now) const {
    return stationary_since_ && now >= last_speed_ && now-last_speed_ <= 150000000ULL &&
           now >= stationary_since_ && now-stationary_since_ >= 500000000ULL;
  }
  void begin_probe(uint64_t now) {
    stage = Stage::PROBING; reason = "checking_harness_forwarding"; started_ = now;
    stock_ = 0; eps_ = 0; stock_count_ = 0; eps_count_ = 0; wrong_ = false; tx_ready = false;
    rvv_stock_ = 0; rvv_engine_ = 0; rvv_stock_count_ = 0; rvv_engine_count_ = 0;
  }
  bool probe_complete(uint64_t now) {
    if (stage != Stage::PROBING) return false;
    if (!stationary(now)) { fail("vehicle_moved_during_wiring_check"); return false; }
    if (now < started_ || now-started_ < 3000000000ULL) return false;
    if (!streams_fresh(now) || stock_count_ < (rvv_only_ ? 20U : 30U) || eps_count_ < 15 ||
        (combined_ && (rvv_stock_count_ < 20U || rvv_engine_count_ < 15U))) {
      fail("missing_isolated_stock_or_eps_stream"); return false;
    }
    return true;
  }
  void begin_active(uint64_t now) {
    stage = Stage::ACTIVE; reason = "waiting_for_panda_settle"; started_ = now; tx_ready = false;
  }
  bool active_healthy(uint64_t now) {
    if (stage != Stage::ACTIVE) return false;
    if (now < started_) { fail("clock_regression"); return false; }
    if (now-started_ < 2000000000ULL) return true;
    if (!streams_fresh(now)) { fail("isolated_can_stream_stale"); return false; }
    tx_ready = true;
    reason = split_axes_ ? "split_axes_test_ready" : combined_ ? "lateral_rvv_test_ready" : rvv_only_ ? "rvv_test_ready" : "lateral_test_ready";
    return true;
  }
private:
  bool streams_fresh(uint64_t now) const {
    return !wrong_ && stock_ && eps_ && now >= stock_ && now >= eps_ &&
           now-stock_ <= 150000000ULL && now-eps_ <= 250000000ULL &&
           (!combined_ || (rvv_stock_ && rvv_engine_ && now >= rvv_stock_ && now >= rvv_engine_ &&
             now-rvv_stock_ <= 150000000ULL && now-rvv_engine_ <= 250000000ULL));
  }
  uint64_t started_ = 0, last_speed_ = 0, stationary_since_ = 0, stock_ = 0, eps_ = 0;
  unsigned int stock_count_ = 0, eps_count_ = 0;
  bool wrong_ = false;
  const bool rvv_only_;
  const bool combined_;
  const bool split_axes_;
  const bool eps_cycle_;
  uint64_t rvv_stock_ = 0, rvv_engine_ = 0;
  unsigned int rvv_stock_count_ = 0, rvv_engine_count_ = 0;
};

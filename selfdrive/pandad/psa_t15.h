#pragma once

#include <cstddef>
#include <cstdint>

// Host-side ignition for the dedicated passive PSA branch. Runs only on the
// pandad receive thread; no Panda firmware, safety hooks or CAN TX are involved.
class PsaT15Ignition {
public:
  static constexpr uint64_t ON_CONFIRM_NS = 100000000ULL;
  static constexpr uint64_t MAX_CONFIRM_GAP_NS = 250000000ULL;
  static constexpr uint64_t TIMEOUT_NS = 1000000000ULL;

  explicit PsaT15Ignition(bool enabled = false) : enabled_(enabled) {}
  bool enabled() const { return enabled_; }

  void reset() {
    seen_ = false;
    active_ = false;
    samples_ = 0;
  }

  void update(uint32_t address, uint8_t bus, const uint8_t *data, size_t length, uint64_t now_ns) {
    if (!enabled_ || address != 0x348U || bus != 0U || length != 8U) return;

    // A gap, clock regression or explicit T15=0 requires a new confirmation.
    if (seen_ && (now_ns < last_ns_ || now_ns - last_ns_ >= TIMEOUT_NS ||
                  (!active_ && now_ns - last_ns_ > MAX_CONFIRM_GAP_NS))) reset();
    if ((data[6] & 0x40U) == 0U) {
      reset();
      return;
    }
    if (!seen_) {
      first_ns_ = now_ns;
      seen_ = true;
    }
    last_ns_ = now_ns;
    if (samples_ < 3U) ++samples_;
    if (samples_ >= 3U && now_ns - first_ns_ >= ON_CONFIRM_NS) active_ = true;
  }

  // Harness and transport checks share this path with the replay tests.
  bool ignition(uint64_t now_ns, bool transport_healthy = true, bool harness_connected = true) {
    if (!enabled_ || !transport_healthy || !harness_connected ||
        (seen_ && (now_ns < last_ns_ || now_ns - last_ns_ >= TIMEOUT_NS))) {
      reset();
    }
    return active_;
  }

private:
  const bool enabled_;
  bool seen_ = false;
  bool active_ = false;
  unsigned int samples_ = 0;
  uint64_t first_ns_ = 0;
  uint64_t last_ns_ = 0;
};

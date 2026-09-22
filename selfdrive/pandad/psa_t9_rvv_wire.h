#pragma once
#include <cstddef>
#include <cstdint>

// customReservedRawData0: identical 32-byte format to psa/rvv_wire.py.
// Only the dedicated RVV safety profiles consume it. Never send as CAN.
// RVV2 distinguishes an intentional axis release from a shared input/transport
// failure. It cannot be decoded by the legacy profile or sent to old firmware.
inline uint16_t t9_rvv_command(const uint8_t *data, size_t size, uint64_t now, uint64_t event_time, bool valid,
                               bool split_axes = false) {
  const uint16_t release = split_axes ? 0x2000U : 0x1000U;
  const uint16_t fault = split_axes ? 0x2200U : 0x1000U;
  if (!valid || size != 32U || data[0] != 'R' || data[1] != 'V' || data[2] != 'V' || data[3] != (split_axes ? '2' : '1') ||
      data[6] || data[7] || (data[5] & ~(split_axes ? 7U : 3U)) ||
      now < event_time || now-event_time > 100000000ULL) return fault;
  const auto read_time = [data](size_t start) {
    uint64_t value = 0;
    for (size_t i = 0; i < 8; ++i) value |= uint64_t(data[start+i]) << (8U*i);
    return value;
  };
  const auto fresh = [now](uint64_t time, uint64_t timeout) { return time && now >= time && now-time <= timeout; };
  if (!fresh(read_time(8), 100000000ULL) || (split_axes && (data[5] & 4U))) return fault;
  if ((data[5] & 2U) || !(data[5] & 1U)) {
    if (split_axes && (data[4] || read_time(16) || read_time(24))) return fault;
    // An inhibition published during an enabled session is an explicit
    // longitudinal stop, including before its first nonzero setpoint.
    if (split_axes && data[5] == 3U) return 0x2400U;
    return release;
  }
  if (data[4] == 0U) {
    if (split_axes && (read_time(16) || read_time(24))) return fault;
    return release | 0x100U;  // enabled, waiting for a lead, no target
  }
  if (data[4] < 40U || data[4] > 140U || !fresh(read_time(16), 250000000ULL) ||
      !fresh(read_time(24), 500000000ULL)) return fault;
  return release | 0x100U | data[4];
}

#include <array>
#include <cassert>
#include <utility>
#include "selfdrive/pandad/psa_t9_rvv_wire.h"
#include "selfdrive/pandad/psa_t9_guard.h"

int main() {
  constexpr uint64_t now = 1000000000ULL;
  std::array<uint8_t, 32> data{'R', 'V', 'V', '1', 74, 1, 0, 0};
  auto stamp = [&data](int offset, uint64_t value) {
    for (int i = 0; i < 8; ++i) data[offset+i] = (value >> (8*i)) & 0xff;
  };
  for (int i : {8, 16, 24}) stamp(i, now);
  assert(t9_rvv_command(data.data(), data.size(), now, now, true) == (0x1100U | 74U));
  assert(t9_rvv_command(data.data(), data.size(), now, now, false) == 0x1000U);
  assert(t9_rvv_command(data.data(), data.size()-1, now, now, true) == 0x1000U);
  assert(t9_rvv_command(data.data(), data.size(), now+100000001, now, true) == 0x1000U);
  for (int offset : {8, 16, 24}) {
    stamp(offset, now+1);
    assert(t9_rvv_command(data.data(), data.size(), now, now, true) == 0x1000U);
    stamp(offset, now);
  }
  for (int flag : {0, 2, 3, 4, 255}) {
    data[5] = flag;
    assert(t9_rvv_command(data.data(), data.size(), now, now, true) == 0x1000U);
  }
  data[5] = 1;
  data[4] = 140;
  assert(t9_rvv_command(data.data(), data.size(), now, now, true) == (0x1100U | 140U));
  for (int target : {39, 141, 255}) {
    data[4] = target;
    assert(t9_rvv_command(data.data(), data.size(), now, now, true) == 0x1000U);
  }
  data[4] = 0;
  assert(t9_rvv_command(data.data(), data.size(), now, now, true) == 0x1100U);

  // Split-profile wire distinguishes deliberate RVV stops from common faults.
  data[3] = '2'; data[4] = 74; data[5] = 1;
  for (int i : {8, 16, 24}) stamp(i, now);
  auto split_command = [&]() { return t9_rvv_command(data.data(), data.size(), now, now, true, true); };
  assert(split_command() == (0x2100U | 74U));
  assert(t9_rvv_command(data.data(), data.size(), now, now, true) == 0x1000U);
  data[3] = '1';
  assert(split_command() == 0x2200U);
  data[3] = '2';
  assert(t9_rvv_command(data.data(), data.size(), now, now, false, true) == 0x2200U);
  assert(t9_rvv_command(nullptr, 0, now, now, true, true) == 0x2200U);
  assert(t9_rvv_command(data.data(), data.size(), now+100000001, now, true, true) == 0x2200U);
  for (int offset : {8, 16, 24}) {
    stamp(offset, now+1);
    assert(split_command() == 0x2200U);
    stamp(offset, 0);
    assert(split_command() == 0x2200U);
    stamp(offset, now);
  }
  stamp(8, now-100000000); stamp(16, now-250000000); stamp(24, now-500000000);
  assert(split_command() == (0x2100U | 74U));
  for (auto [offset, age] : {std::pair{8, 100000001ULL}, {16, 250000001ULL}, {24, 500000001ULL}}) {
    stamp(offset, now-age);
    assert(split_command() == 0x2200U);
    stamp(offset, now);
  }
  for (int i : {8, 16, 24}) stamp(i, now);
  for (int target : {40, 140}) {
    data[4] = target;
    assert(split_command() == (0x2100U | target));
  }
  for (int target : {39, 141, 255}) {
    data[4] = target;
    assert(split_command() == 0x2200U);
  }
  data[4] = 0; stamp(16, 0); stamp(24, 0);
  for (int flag : {0, 2, 3}) {
    data[5] = flag;
    assert(split_command() == (flag == 3 ? 0x2400U : 0x2000U));
    data[4] = 74;  // contradictory release must not hide corrupt data
    assert(split_command() == 0x2200U);
    data[4] = 0;
  }
  data[5] = 1;
  assert(split_command() == 0x2100U);
  stamp(16, now);
  assert(split_command() == 0x2200U);
  stamp(16, 0);
  for (int flag : {4, 5, 6, 7, 8, 255}) {
    data[5] = flag;
    assert(split_command() == 0x2200U);
  }
  data[5] = 1; data[6] = 1;
  assert(split_command() == 0x2200U);
  data[6] = 0;

  PsaT9Guard incomplete_split(false, false, true);
  assert(!incomplete_split.split_axes() && incomplete_split.active_param() == 0x1308);

  // RVV probe observes the actual BSI/engine split, independently of EPS.
  PsaT9Guard guard(true);
  assert(guard.active_param() == 0x1310 && guard.probe_param() == 0x1311);
  std::array<uint8_t, 8> frame{};
  uint64_t t = now;
  guard.begin_probe(t);
  for (int i = 0; i < 65; ++i) {
    t += 50000000ULL;
    guard.update(0x30d, 0, frame.data(), 8, t);
    guard.update(0x50e, 2, frame.data(), 8, t);
    guard.update(0x208, 0, frame.data(), 8, t);
  }
  assert(guard.probe_complete(t));
  guard.begin_active(t);
  for (int i = 0; i < 45; ++i) {
    t += 50000000ULL;
    guard.update(0x50e, 2, frame.data(), 8, t);
    guard.update(0x208, 0, frame.data(), 8, t);
  }
  assert(guard.active_healthy(t) && guard.tx_ready.load());
  guard.update(0x50e, 0, frame.data(), 8, t);
  assert(!guard.tx_ready.load() && guard.stage == PsaT9Guard::Stage::FAILED);

  // Combined mode must prove BOTH isolated command/feedback pairs.
  for (bool split_axes : {false, true}) {
   for (int omitted : {0, 0x50e, 0x208, 0x3f2, 0x495}) {
    PsaT9Guard combined(true, true, split_axes);
    assert(!combined.rvv_only());
    assert(combined.split_axes() == split_axes);
    assert(combined.active_param() == (split_axes ? 0x1314 : 0x1312));
    assert(combined.probe_param() == (split_axes ? 0x1315 : 0x1313));
    t = now;
    combined.begin_probe(t);
    for (int i = 0; i < 65; ++i) {
      t += 50000000ULL;
      combined.update(0x30d, 0, frame.data(), 8, t);
      for (int addr : {0x50e, 0x208, 0x3f2, 0x495}) {
        if (addr != omitted) combined.update(addr, (addr == 0x50e || addr == 0x3f2) ? 2 : 0,
                                             frame.data(), addr == 0x495 ? 4 : 8, t);
      }
    }
    assert(combined.probe_complete(t) == (omitted == 0));
    if (omitted == 0) {
      combined.begin_active(t);
      for (int i = 0; i < 45; ++i) {
        t += 50000000ULL;
        for (int addr : {0x50e, 0x208, 0x3f2, 0x495}) {
          combined.update(addr, (addr == 0x50e || addr == 0x3f2) ? 2 : 0,
                          frame.data(), addr == 0x495 ? 4 : 8, t);
        }
      }
      assert(combined.active_healthy(t));
      combined.update(0x208, 2, frame.data(), 8, t);
      assert(!combined.tx_ready.load());
    }
   }
  }
  return 0;
}

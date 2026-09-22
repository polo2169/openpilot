#include <array>
#include <cassert>
#include <iostream>
#include "selfdrive/pandad/psa_t9_guard.h"

using Data = std::array<uint8_t, 8>;
constexpr uint64_t MS = 1000000ULL;

void feed(PsaT9Guard &guard, uint64_t ms, int stock_bus = 2, int eps_bus = 0, bool moving = false) {
  Data speed{}; if (moving) speed = {0x1d, 0x4c, 0x1d, 0x4c, 0x1d, 0x4c, 0x1d, 0x4c};
  Data stock{}; Data eps{};
  guard.update(0x30D, 0, speed.data(), 8, ms*MS);
  guard.update(0x3F2, stock_bus, stock.data(), 8, ms*MS);
  guard.update(0x495, eps_bus, eps.data(), 4, ms*MS);
}

int main() {
  PsaT9Guard cycle(true, true, true, true);
  assert(cycle.active_param() == 0x1316 && cycle.probe_param() == 0x1317);
  assert(cycle.split_axes() && !cycle.tx_ready);
  PsaT9Guard cycle_off(true, true, true, false);
  assert(cycle_off.active_param() == 0x1314 && cycle_off.probe_param() == 0x1315);
  PsaT9Guard not_split(true, true, false, true);
  assert(not_split.active_param() == 0x1312);
  PsaT9TxTiming timing;
  assert(timing.delay_ns(100*MS, true) == 0);
  // A 50 ms publication interval becomes only 44 ms at the MCU if the
  // previous write took 6 ms. Wait from completed transport, not publication.
  timing.completed(106*MS);
  assert(timing.delay_ns(150*MS, true) == 2*MS);
  assert(timing.delay_ns(152*MS, true) == 0);
  assert(timing.delay_ns(107*MS, false) == 0);  // emergency release
  timing.completed(108*MS);  // a release also resets the spacing reference
  assert(timing.delay_ns(150*MS, true) == 4*MS);
  uint64_t previous_completed = 108*MS;
  for (uint64_t tick = 0; tick < 5000; ++tick) {
    uint64_t proposed = 150*MS + tick*50*MS;
    uint64_t sent = proposed + timing.delay_ns(proposed, true);
    assert(sent-previous_completed >= PsaT9TxTiming::MIN_GAP_NS);
    // Include occasional SPI recovery delays and scheduler jitter.
    previous_completed = sent + (tick % 11)*MS;
    timing.completed(previous_completed);
  }
  std::cout << "T9 TX timing: delayed transport is paced; zero releases stay immediate\n";

  PsaT9Guard guard;
  assert(!guard.stationary(0)); assert(!guard.tx_ready);
  for (uint64_t t = 10; t <= 600; t += 10) feed(guard, t);
  assert(guard.stationary(600*MS));
  guard.begin_probe(600*MS);
  for (uint64_t t = 650; t <= 3600; t += 50) feed(guard, t);
  assert(guard.probe_complete(3600*MS)); assert(!guard.tx_ready);
  guard.begin_active(3600*MS);
  for (uint64_t t = 3650; t <= 5550; t += 50) {
    feed(guard, t, 2, 0, true);
    assert(guard.active_healthy(t*MS)); assert(!guard.tx_ready);
  }
  feed(guard, 5600, 2, 0, true);
  assert(guard.active_healthy(5600*MS)); assert(guard.tx_ready);
  assert(!guard.active_healthy(5900*MS)); assert(!guard.tx_ready);
  assert(guard.stage == PsaT9Guard::Stage::FAILED);

  // Same frames on an unseparated bus cannot produce a positive result.
  for (const auto &sides : {std::array<int, 2>{0, 0}, std::array<int, 2>{2, 2}}) {
    guard.reset(); for (uint64_t t = 10; t <= 600; t += 10) feed(guard, t);
    guard.begin_probe(600*MS);
    feed(guard, 900, sides[0], sides[1]);
    assert(guard.stage == PsaT9Guard::Stage::FAILED); assert(!guard.tx_ready);
  }
  // Missing EPS, movement during interception, transport loss and clock reversal.
  guard.reset(); for (uint64_t t = 10; t <= 600; t += 10) feed(guard, t);
  guard.begin_probe(600*MS); feed(guard, 900, 2, 0, true);
  assert(!guard.probe_complete(900*MS)); assert(guard.stage == PsaT9Guard::Stage::FAILED);
  guard.reset(); guard.begin_probe(600*MS); feed(guard, 590);
  assert(guard.stage == PsaT9Guard::Stage::FAILED);
  guard.reset(); guard.begin_active(600*MS); guard.fail("transport_lost");
  assert(!guard.active_healthy(900*MS)); assert(!guard.tx_ready);
  const char *first_reason = guard.reason;
  guard.fail("later_mirrored_frames");
  assert(guard.reason == first_reason);
  std::cout << "T9 startup guard: isolation, no moving startup, stale data and failure latch pass\n";
}

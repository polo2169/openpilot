#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "selfdrive/pandad/psa_t15.h"

// Standalone: clang++ -std=c++17 -I. selfdrive/pandad/tests/test_psa_t15.cc -o /tmp/test_psa_t15
// Optional replay input: monotonic_ns address_decimal bus_decimal data_hex.
#define CHECK(expr) do { if (!(expr)) { std::cerr << "Failed line " << __LINE__ << ": " << #expr << '\n'; std::exit(1); } } while (0)

using Data = std::array<uint8_t, 8>;
const Data ON = {0x00, 0x32, 0x47, 0x32, 0xD1, 0x01, 0x44, 0x00};
const Data OFF = {0x00, 0x32, 0x4F, 0x32, 0x11, 0x01, 0x00, 0x59};

void feed(PsaT15Ignition &detector, uint64_t ms, const Data &data = ON, uint32_t addr = 0x348, uint8_t bus = 0, size_t size = 8) {
  detector.update(addr, bus, data.data(), size, ms * 1000000ULL);
}

void confirm(PsaT15Ignition &detector, uint64_t base = 0) {
  for (uint64_t t = 0; t <= 100; t += 20) feed(detector, base + t);
}

void unit_tests() {
  PsaT15Ignition disabled;
  confirm(disabled);
  CHECK(!disabled.ignition(100000000ULL));

  PsaT15Ignition detector(true);
  CHECK(!detector.ignition(0));
  feed(detector, 0);
  feed(detector, 20);
  CHECK(!detector.ignition(20000000ULL));
  feed(detector, 99);
  CHECK(!detector.ignition(99000000ULL));
  feed(detector, 100);
  CHECK(detector.ignition(100000000ULL));
  feed(detector, 110, OFF);
  CHECK(!detector.ignition(110000000ULL));

  // A queued burst with only one reception time cannot assert ignition.
  for (int i = 0; i < 20; ++i) feed(detector, 120);
  CHECK(!detector.ignition(120000000ULL));
  detector.reset();

  // Two isolated frames are insufficient even when 100 ms apart.
  feed(detector, 0); feed(detector, 100);
  CHECK(!detector.ignition(100000000ULL));
  detector.reset();
  for (uint64_t t = 0; t <= 100; t += 20) {
    feed(detector, t, ON, 0x348, 1);
    feed(detector, t, ON, 0x348, 2);
    feed(detector, t, ON, 0x348, 128);
    feed(detector, t, ON, 0x208);
    feed(detector, t, ON, 0x1348);
    feed(detector, t, ON, 0x348, 0, 7);
  }
  CHECK(!detector.ignition(100000000ULL));

  confirm(detector);
  feed(detector, 1000, ON, 0x208);  // RPM does not refresh contact.
  CHECK(detector.ignition(1099999999ULL));
  CHECK(!detector.ignition(1100000000ULL));
  feed(detector, 1120);
  CHECK(!detector.ignition(1120000000ULL));
  confirm(detector, 1140);
  CHECK(detector.ignition(1240000000ULL));
  CHECK(!detector.ignition(1240000000ULL, false, true));
  confirm(detector, 1300);
  CHECK(!detector.ignition(1400000000ULL, true, false));

  detector.reset();
  feed(detector, 0);
  feed(detector, 400);  // A >250 ms gap cannot confirm an initial ON.
  CHECK(!detector.ignition(400000000ULL));
  confirm(detector, 420);
  CHECK(detector.ignition(520000000ULL));
  feed(detector, 900);  // Confirmed ignition tolerates a sub-timeout gap.
  CHECK(detector.ignition(900000000ULL));
  feed(detector, 1900);
  CHECK(!detector.ignition(1900000000ULL));
  confirm(detector, 1920);
  CHECK(detector.ignition(2020000000ULL));
  CHECK(!detector.ignition(2019999999ULL));  // Clock regression.

  // Engine states 3 -> 4 -> 5 -> 3 and zero RPM keep T15 asserted.
  detector.reset();
  confirm(detector);
  uint64_t t = 120;
  for (uint8_t state : {3, 4, 5, 3}) {
    Data data = ON;
    data[5] = state;
    for (int i = 0; i < 30; ++i, t += 20) {
      feed(detector, t, data);
      feed(detector, t, OFF, 0x208);
      CHECK(detector.ignition(t * 1000000ULL));
    }
  }
  feed(detector, t, OFF);
  CHECK(!detector.ignition(t * 1000000ULL));
  std::cout << "PSA T15 unit tests passed\n";
}

void replay(const char *path) {
  std::ifstream input(path);
  CHECK(input.good());
  PsaT15Ignition detector(true);
  uint64_t now = 0, previous = 0, frames = 0;
  uint32_t addr, bus;
  std::string hex;
  bool state = false;
  while (input >> now >> addr >> bus >> hex) {
    CHECK(frames == 0 || now >= previous);
    CHECK(hex.size() % 2 == 0 && hex.size() <= 128);
    std::array<uint8_t, 64> data{};
    for (size_t i = 0; i < hex.size()/2; ++i) data[i] = std::stoul(hex.substr(i*2,2), nullptr, 16);
    detector.update(addr, bus, data.data(), hex.size()/2, now);
    bool current = detector.ignition(now);
    if (current != state) std::cout << now << ' ' << current << '\n';
    state = current;
    previous = now;
    ++frames;
  }
  CHECK(input.eof());
  CHECK(!detector.ignition(now + PsaT15Ignition::TIMEOUT_NS));
  std::cout << "frames " << frames << '\n';
}

int main(int argc, char **argv) {
  unit_tests();
  if (argc == 2) replay(argv[1]);
  return 0;
}

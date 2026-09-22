#pragma once

// First RVV-only experiment: adjust the existing BSI setpoint in the MCU.
// No host-generated vehicle CAN, steering command, activation or cancellation.
// This is a conventional cruise setpoint adapter, not a service-brake API.
#define PSA_T9_RVV_PARAM 0x1310U
#define PSA_T9_RVV_PROBE_PARAM 0x1311U
#define PSA_T9_COMBINED_PARAM 0x1312U
#define PSA_T9_COMBINED_PROBE_PARAM 0x1313U
#define RVV_LEASE_US 150000U
#define RVV_STEP_US 500000U
#define RVV_DOWN_STEP_US 100000U

static bool rvv_probe, rvv_fault, rvv_rearm, rvv_pending, rvv_prev_engaged, rvv_settled;
static bool rvv_seen[8], rvv_seen_seq, rvv_requested, rvv_applied_seen;
static bool rvv_host_engaged_seen;
static bool rvv_t15, rvv_reverse, rvv_doors, rvv_stock_active, rvv_stock_off, rvv_pedal_invalid;
static int rvv_speed, rvv_gear, rvv_park, rvv_belt, rvv_engine;
static uint8_t rvv_ceiling, rvv_target, rvv_applied, rvv_side[2048];
static uint16_t rvv_seq;
static uint32_t rvv_init_ts, rvv_pending_ts, rvv_request_ts, rvv_step_ts, rvv_rx_ts[8], rvv_rewrites;

static bool rvv_combined_profile(void) {
  return (current_safety_mode == SAFETY_PSA) &&
    ((current_safety_param == PSA_T9_COMBINED_PARAM) || (current_safety_param == PSA_T9_COMBINED_PROBE_PARAM) || t9_split_profile());
}

static bool rvv_profile(void) {
  return (current_safety_mode == SAFETY_PSA) &&
    ((current_safety_param == PSA_T9_RVV_PARAM) || (current_safety_param == PSA_T9_RVV_PROBE_PARAM) || rvv_combined_profile());
}

static uint8_t rvv_permission_bits(void) {
  uint8_t bits = controls_allowed ? 1U : 0U;
  if (t9_split_profile()) {
    // A read-only query never synchronizes or changes authority. AND the
    // common flag so an externally imposed common stop is visible at once.
    if (controls_allowed && t9_split_lateral_allowed) { bits |= 2U; }
    if (controls_allowed && t9_split_rvv_allowed) { bits |= 4U; }
  }
  return bits;
}

static uint8_t rvv_parity(uint8_t speed) {
  uint8_t low = 0U, high = 0U;
  for (int bit = 0; bit < 4; bit++) {
    low ^= (speed >> bit) & 1U;
    high ^= (speed >> (bit + 4)) & 1U;
  }
  return (high << 1) | low;
}

static bool rvv_stock_valid(const CANPacket_t *msg) {
  return !msg->extended && !msg->fd && (GET_LEN(msg) == 8) &&
    (((msg->data[0] >> 4) & 3U) == rvv_parity(msg->data[6]));
}

static bool rvv_ready(uint32_t now) {
  // Latch the one-time startup delay; the 32-bit microsecond timer wraps
  // every ~71 minutes and must not create another startup delay mid-trip.
  if (safety_get_ts_elapsed(now, rvv_init_ts) >= 2000000U) { rvv_settled = true; }
  bool ready = !rvv_probe && !rvv_fault && !relay_malfunction && !safety_rx_checks_invalid &&
    rvv_settled &&
    !brake_pressed && !gas_pressed && !rvv_pedal_invalid && rvv_t15 && !rvv_reverse && !rvv_doors &&
    (rvv_gear >= 1) && (rvv_gear <= 6) && (rvv_park == 0) && (rvv_belt == 2) &&
    (rvv_speed >= 4000) && (rvv_speed <= 14000) && (rvv_engine == 2) && rvv_stock_active &&
    (rvv_ceiling >= 40U) && (rvv_ceiling <= 140U);
  for (int i = 0; i < 8; i++) { ready = ready && t9_fresh(now, rvv_rx_ts[i], rvv_seen[i], T9_FRESH_US); }
  // The combined profile inherits the lateral envelope and driver/EPS gates.
  if (rvv_combined_profile()) {
    ready = ready && (t9_split_profile() ? t9_common_ready(now) : t9_ready(now)) && !t9_output_lease_expired(now);
  }
  return ready;
}

static void rvv_release(bool disengage) {
  rvv_requested = false;
  if (disengage) {
    if (t9_split_profile()) { t9_split_common_stop(); }
    else { controls_allowed = false; }
    rvv_pending = false;
  }
}

static bool rvv_allowed(void) {
  return controls_allowed && (!t9_split_profile() || t9_split_rvv_allowed);
}

static void rvv_split_axis_release(void) {
  t9_split_sync_common_stop();
  rvv_requested = false;
  rvv_pending = false;
  t9_split_rvv_allowed = false;
  t9_split_rvv_stopped = true;
  controls_allowed = t9_split_lateral_allowed;
}

static void rvv_rx(const CANPacket_t *msg) {
  uint32_t now = microsecond_timer_get();
  int slot = -1;
  if (msg->addr == 0x30DU) {
    slot = 0; int sum = 0, low = 65535, high = 0;
    for (int i = 0; i < 8; i += 2) {
      int speed = (msg->data[i] << 8) | msg->data[i + 1];
      sum += speed; low = SAFETY_MIN(low, speed); high = SAFETY_MAX(high, speed);
    }
    rvv_speed = ((high - low) <= 500) ? sum / 4 : -1;
    vehicle_moving = high > 10;
    UPDATE_VEHICLE_SPEED(SAFETY_MAX(rvv_speed, 0) * 0.01 * KPH_TO_MS);
  } else if (msg->addr == 0x348U) {
    slot = 1; rvv_gear = msg->data[0] >> 4; rvv_t15 = (msg->data[6] & 0x40U) != 0U;
  } else if (msg->addr == 0x412U) {
    slot = 2; brake_pressed = (msg->data[0] & 0x20U) != 0U;
    rvv_reverse = (msg->data[0] & 4U) != 0U; rvv_doors = (msg->data[6] & 0x78U) != 0U;
  } else if (msg->addr == 0x3ADU) {
    slot = 3; rvv_park = msg->data[3] & 7U;
  } else if (msg->addr == 0x572U) {
    slot = 4; rvv_belt = msg->data[0] >> 6;
  } else if (msg->addr == 0x228U) {
    slot = 5; gas_pressed = msg->data[2] > 0U; rvv_pedal_invalid = msg->data[2] > 200U;
  } else if (msg->addr == 0x50EU) {
    slot = 6;
    bool valid = rvv_stock_valid(msg) && (((msg->data[7] >> 5) & 3U) == 1U);
    rvv_ceiling = msg->data[6];
    rvv_stock_active = valid && ((msg->data[7] & 0x80U) != 0U) && (rvv_ceiling != 255U);
    rvv_stock_off = valid && ((msg->data[7] & 0x80U) == 0U) && (rvv_ceiling == 255U);
  } else if (msg->addr == 0x208U) {
    slot = 7; rvv_engine = (msg->data[4] >> 2) & 3U;
  } else { }
  if (slot >= 0) { rvv_rx_ts[slot] = now; rvv_seen[slot] = true; }
  if ((msg->addr == 0x50EU) || (msg->addr == 0x208U)) {
    bool off = rvv_stock_off && ((rvv_engine == 0) || (rvv_engine == 3)) &&
      t9_fresh(now, rvv_rx_ts[6], rvv_seen[6], T9_FRESH_US) && t9_fresh(now, rvv_rx_ts[7], rvv_seen[7], T9_FRESH_US);
    if (off) {
      rvv_host_engaged_seen = false;
      rvv_rearm = true; rvv_applied_seen = false; rvv_release(true);
    }
    bool engaged = rvv_stock_active && (rvv_engine == 2);
    if (engaged && !rvv_prev_engaged) {
      rvv_pending = rvv_rearm; rvv_pending_ts = now; rvv_rearm = false;
    }
    rvv_prev_engaged = engaged;
  }
  if (rvv_pending && (!rvv_stock_active || (rvv_engine != 2) ||
      (safety_get_ts_elapsed(now, rvv_pending_ts) > T9_FRESH_US))) { rvv_pending = false; }
  if (rvv_pending && rvv_ready(now)) {
    // In the combined profile only the lateral physical OFF/ON admission
    // may enable controls. RVV must never undo a driver or EPS interruption.
    if (!rvv_combined_profile()) { controls_allowed = true; }
    rvv_pending = false;
  }
  if (!rvv_ready(now)) {
    rvv_release(false);
    if (t9_split_profile()) {
      if (t9_split_lateral_allowed || t9_split_rvv_allowed) { t9_split_common_stop(); }
    } else { controls_allowed = false; }
  }
}

// USB/SPI control request 0xA9, version 1. It only leases a target; CAN RX
// supplies every physical outgoing frame. Duplicate SPI retries cannot renew
// the lease. A target of zero releases; it cannot activate stock cruise.
static uint8_t rvv_request(uint16_t command, uint16_t sequence) {
  uint32_t now = microsecond_timer_get();
  t9_split_sync_common_stop();
  if (!rvv_profile() || rvv_probe) { return 1U; }
  if (t9_split_profile() &&
      ((rvv_requested && (safety_get_ts_elapsed(now, rvv_request_ts) > RVV_LEASE_US)) ||
       ((t9_split_lateral_allowed || t9_split_rvv_allowed) && !rvv_ready(now)))) { rvv_release(true); }
  uint16_t prefix = t9_split_profile() ? 0x2000U : 0x1000U;
  bool split_axis_fault = t9_split_profile() && (command == 0x2400U);
  if (((command & 0xFE00U) != prefix) && !split_axis_fault) { rvv_release(true); return 2U; }
  uint16_t delta = (sequence - rvv_seq) & 0xFFFFU;
  if (rvv_seen_seq && ((delta == 0U) || (delta > 0x7FFFU))) { return 3U; }
  rvv_seq = sequence; rvv_seen_seq = true;
  if (split_axis_fault) {
    rvv_host_engaged_seen = true;
    rvv_split_axis_release();
    return 0U;
  }
  uint8_t target = command & 0xFFU;
  bool enabled = (command & 0x100U) != 0U;
  if (t9_split_profile()) {
    if (!enabled && (target != 0U)) { rvv_release(true); return 2U; }
    if (enabled) { rvv_host_engaged_seen = true; }
  }
  if (!enabled || (target == 0U)) {
    if (t9_split_profile()) {
      // CAN admission can precede the host's native engagement by a batch.
      // Initial disabled heartbeats do not consume that explicit admission.
      // Once the host has engaged, a disable or released takeover latches off.
      if ((!enabled && rvv_host_engaged_seen) || rvv_requested || rvv_applied_seen) { rvv_split_axis_release(); }
      else { rvv_release(false); }
    } else { rvv_release(rvv_applied_seen); }
    return 0U;
  }
  // An expired takeover must not be revived by a late host command.
  if (rvv_requested && (safety_get_ts_elapsed(now, rvv_request_ts) > RVV_LEASE_US)) { rvv_release(true); }
  if (!rvv_allowed() || !rvv_ready(now)) { rvv_release(false); return 4U; }
  if ((target < 40U) || (target > 140U) || (target > rvv_ceiling)) { rvv_release(true); return 5U; }
  rvv_target = target; rvv_request_ts = now; rvv_requested = true;
  return 0U;
}

// Capability queries are read-only even outside a PSA safety mode. New hosts
// must verify status 7 before selecting the new split parameter: older PSA
// firmware interprets unknown parameters as its legacy angle-control mode.
static uint8_t rvv_control_request(uint16_t command, uint16_t sequence) {
  if (command == 0U) {
    if (sequence == 0U) { return 6U; }
    if (sequence == 2U) { return 7U; }
    if (sequence == 4U) { return 8U; }
  }
  return rvv_request(command, sequence);
}

// Called on the forwarding copy BEFORE safety_rx_hook, matching the FDCAN
// driver order. Inspect THIS stock frame, including cancel and lower ceiling.
// Preserve counter, mode, activation and every other unknown bit verbatim.
static bool rvv_rewrite(CANPacket_t *msg, int destination) {
  uint32_t now = microsecond_timer_get();
  t9_split_sync_common_stop();
  if (!rvv_profile() || rvv_probe || (msg->bus != 2U) || (destination != 0) || (msg->addr != 0x50EU)) { return false; }
  if (!rvv_requested) { return false; }
  bool stock_active = rvv_stock_valid(msg) && (((msg->data[7] >> 5) & 7U) == 5U) &&
    (msg->data[6] >= 40U) && (msg->data[6] <= 140U);
  if (!rvv_allowed() || !rvv_ready(now) || !stock_active ||
      (safety_get_ts_elapsed(now, rvv_request_ts) > RVV_LEASE_US)) {
    rvv_release(true); return false;
  }
  if (!rvv_applied_seen) {
    rvv_applied = msg->data[6]; rvv_step_ts = now; rvv_applied_seen = true;
  }
  rvv_applied = SAFETY_MIN(rvv_applied, msg->data[6]);
  uint8_t target = SAFETY_MIN(rvv_target, msg->data[6]);
  uint32_t step_us = target < rvv_applied ? RVV_DOWN_STEP_US : RVV_STEP_US;
  if (safety_get_ts_elapsed(now, rvv_step_ts) >= step_us) {
    if (rvv_applied < target) { rvv_applied++; }
    else if (rvv_applied > target) { rvv_applied--; }
    rvv_step_ts = now;
  }
  msg->data[6] = rvv_applied;
  msg->data[0] = (msg->data[0] & 0xCFU) | (rvv_parity(rvv_applied) << 4);
  rvv_rewrites++;
  return true;
}

static bool rvv_tx(const CANPacket_t *msg) { (void)msg; return false; }

static bool rvv_fwd(int bus, int addr) {
  if ((addr >= 0) && (addr < 2048) && ((bus == 0) || (bus == 2))) {
    rvv_side[addr] |= (bus == 0) ? 1U : 2U;
    if (rvv_side[addr] == 3U) { rvv_fault = true; relay_malfunction = true; }
  }
  if (rvv_settled &&
      (((addr == 0x50E) && (bus == 0)) || ((addr == 0x208) && (bus == 2)))) { rvv_fault = true; }
  if (rvv_fault) { rvv_release(true); }
  return rvv_fault;
}

static safety_config rvv_init(uint16_t param) {
  rvv_probe = param == PSA_T9_RVV_PROBE_PARAM;
  rvv_settled = false; rvv_fault = false; rvv_rearm = false; rvv_pending = false; rvv_prev_engaged = false;
  rvv_seen_seq = false; rvv_requested = false; rvv_applied_seen = false;
  rvv_host_engaged_seen = false;
  rvv_t15 = false; rvv_reverse = false; rvv_doors = false; rvv_stock_active = false; rvv_stock_off = false; rvv_pedal_invalid = true;
  rvv_speed = 0; rvv_gear = 0; rvv_park = 0; rvv_belt = 0; rvv_engine = -1;
  rvv_ceiling = 255U; rvv_target = 0U; rvv_applied = 0U; rvv_seq = 0U;
  rvv_init_ts = microsecond_timer_get(); rvv_pending_ts = 0U; rvv_request_ts = 0U; rvv_step_ts = 0U; rvv_rewrites = 0U;
  for (int i = 0; i < 8; i++) { rvv_seen[i] = false; rvv_rx_ts[i] = 0U; }
  for (int i = 0; i < 2048; i++) { rvv_side[i] = 0U; }
  static RxCheck rx[] = {
    T9_RX_BOTH(0x30D, 8, 50), T9_RX_BOTH(0x348, 8, 50), T9_RX_BOTH(0x412, 8, 20),
    T9_RX_CHECKED(0x3AD, 8, 50), T9_RX_BOTH(0x572, 8, 10), T9_RX_CHECKED(0x228, 8, 100),
    {.msg = {{0x50E, 2, 8, 10, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x208, 0, 8, 100, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };
  return (safety_config){.rx_checks = rx, .rx_checks_len = sizeof(rx) / sizeof(rx[0]), .tx_msgs = NULL, .tx_msgs_len = 0};
}

const safety_hooks psa_t9_rvv_hooks = {
  .init = rvv_init, .rx = rvv_rx, .tx = rvv_tx, .fwd = rvv_fwd,
  .get_checksum = t9_checksum_received, .compute_checksum = t9_checksum, .get_counter = t9_counter,
};

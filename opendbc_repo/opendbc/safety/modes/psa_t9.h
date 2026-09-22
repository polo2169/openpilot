#pragma once

#include "opendbc/safety/declarations.h"

// Dedicated development parameter: never inherit the R3 angle safety model.
#define PSA_T9_LATERAL_PARAM 0x1308U
#define PSA_T9_PROBE_PARAM 0x1309U
#define PSA_T9_SPLIT_PARAM 0x1314U
#define PSA_T9_SPLIT_PROBE_PARAM 0x1315U
#define PSA_T9_CYCLE_PARAM 0x1316U
#define PSA_T9_CYCLE_PROBE_PARAM 0x1317U
#define T9_CYCLE_PERIOD_US 12000000U
#define T9_CYCLE_TIMEOUT_US 2000000U
#define T9_MAX_TORQUE 15
#define T9_DRIVER_OVERRIDE_LIMIT 15
#define T9_FRESH_US 250000U

static bool t9_probe;
static uint8_t t9_template[8];
static uint32_t t9_stock_ts, t9_eps_ts, t9_init_ts, t9_request_ts, t9_last_tx_ts;
static bool t9_stock_seen, t9_eps_seen, t9_wiring_fault, t9_request_seen, t9_tx_seen, t9_eps_ack, t9_started_control;
static bool t9_replacing_stock;
static int t9_eps_state, t9_last_factor;
static uint32_t t9_rx_ts[10];
static bool t9_seen[10];
static bool t9_blinker;
static int t9_driver, t9_speed_centi_kph, t9_gear, t9_park, t9_belt;
static bool t9_reverse, t9_doors, t9_t15, t9_cruise, t9_mode, t9_pedal_invalid;
static bool t9_stock_cruise_off, t9_cruise_rearm;
static int t9_cruise_state;
static bool t9_engage_pending;
static uint32_t t9_engage_ts;
static uint8_t t9_source_side[2048];

// Explicit bench profile only. A recovered input never grants authority:
// both bits are admitted only by the validated physical RVV OFF/ON edge.
static bool t9_split_lateral_allowed, t9_split_rvv_allowed;
static bool t9_split_lateral_stopped, t9_split_rvv_stopped;
// 0 idle, 1 explicit zero-factor marker, 2 release, 3 prepare, 4 await ACK.
static int t9_cycle_phase;
static bool t9_cycle_release_seen, t9_active_seen, t9_activity_seen, t9_eps_activity;
static uint32_t t9_cycle_ts, t9_cycle_prepare_ts, t9_active_ts, t9_activity_ts;

static bool t9_cycle_profile(void) {
  return (current_safety_mode == SAFETY_PSA) &&
    ((current_safety_param == PSA_T9_CYCLE_PARAM) || (current_safety_param == PSA_T9_CYCLE_PROBE_PARAM));
}

static bool t9_split_profile(void) {
  return (current_safety_mode == SAFETY_PSA) &&
    ((current_safety_param == PSA_T9_SPLIT_PARAM) || (current_safety_param == PSA_T9_SPLIT_PROBE_PARAM) || t9_cycle_profile());
}

static void t9_split_common_stop(void) {
  t9_cycle_phase = 0; t9_active_seen = false;
  t9_split_lateral_allowed = false;
  t9_split_rvv_allowed = false;
  t9_engage_pending = false;
  controls_allowed = false;
}

static void t9_split_sync_common_stop(void) {
  if (t9_split_profile() && !controls_allowed && (t9_split_lateral_allowed || t9_split_rvv_allowed)) {
    t9_split_common_stop();
  }
}

static void t9_split_lateral_stop(void) {
  t9_split_sync_common_stop();
  t9_split_lateral_allowed = false;
  t9_cycle_phase = 0; t9_active_seen = false;
  t9_split_lateral_stopped = true;
  controls_allowed = t9_split_rvv_allowed;
}

static bool t9_lateral_allowed(void) {
  return controls_allowed && (!t9_split_profile() || t9_split_lateral_allowed);
}

// A suspension keeps only an already acknowledged session. Initial refusal,
// EPS withdrawal and all common faults still require physical RVV rearming.
static bool t9_pause_session(void) {
  return t9_split_profile() && t9_lateral_allowed() && t9_started_control && t9_eps_ack && (t9_cycle_phase == 0);
}

static bool t9_driver_pause(void) {
  return (t9_driver < -T9_DRIVER_OVERRIDE_LIMIT) || (t9_driver > T9_DRIVER_OVERRIDE_LIMIT);
}

static bool t9_fresh(uint32_t now, uint32_t then, bool seen, uint32_t timeout) {
  return seen && (safety_get_ts_elapsed(now, then) <= timeout);
}

// While replacing the factory stream, even zero prepare commands must keep
// arriving. A state-2 release ends replacement; idle factory forwarding has
// no host-command lease. The firmware tick reconnects an expired takeover.
static bool t9_output_lease_expired(uint32_t now) {
  return t9_replacing_stock && (safety_get_ts_elapsed(now, t9_last_tx_ts) > 250000U);
}

static bool t9_topology_ready(uint32_t now) {
  return !t9_wiring_fault && !relay_malfunction && (safety_get_ts_elapsed(now, t9_init_ts) >= 2000000U) &&
    t9_fresh(now, t9_stock_ts, t9_stock_seen, 150000U) && t9_fresh(now, t9_eps_ts, t9_eps_seen, T9_FRESH_US);
}

static bool t9_common_ready(uint32_t now) {
  bool ready = t9_topology_ready(now) && !safety_rx_checks_invalid && !brake_pressed && !gas_pressed &&
    !t9_pedal_invalid && !t9_reverse && !t9_doors && t9_t15 && (t9_gear >= 1) && (t9_gear <= 6) &&
    (t9_park == 0) && (t9_belt == 2) &&
    (t9_speed_centi_kph >= (t9_split_profile() ? 4000 : 6710)) && (t9_speed_centi_kph <= 14000) && t9_cruise && t9_mode;
  for (int i = 0; i < (t9_split_profile() ? 10 : 9); i++) {
    ready = ready && t9_fresh(now, t9_rx_ts[i], t9_seen[i], T9_FRESH_US);
  }
  return ready;
}

static bool t9_ready(uint32_t now) {
  return t9_common_ready(now) && (t9_speed_centi_kph >= 6710) &&
    ((t9_cycle_phase == 0) || (t9_cycle_profile() &&
      (safety_get_ts_elapsed(now, t9_cycle_ts) < T9_CYCLE_TIMEOUT_US) &&
      t9_fresh(now, t9_activity_ts, t9_activity_seen, T9_CYCLE_TIMEOUT_US + T9_FRESH_US) &&
      ((t9_cycle_phase != 3) || (t9_eps_state != 3)))) &&
    (t9_pause_session() || (t9_split_profile() ? (!t9_driver_pause() && !t9_blinker) :
      ((t9_driver >= -T9_DRIVER_OVERRIDE_LIMIT) && (t9_driver <= T9_DRIVER_OVERRIDE_LIMIT)))) &&
    (t9_eps_state >= 1) && (t9_eps_state <= 3) && (!t9_eps_ack || (t9_eps_state == 3)) &&
    (((t9_template[4] >> 2) & 7U) >= 3U) && (((t9_template[4] >> 2) & 7U) <= 4U) &&
    !(t9_request_seen && !t9_eps_ack && (safety_get_ts_elapsed(now, t9_request_ts) >= 500000U));
}

static uint8_t t9_get_checksum(const CANPacket_t *msg) {
  if (msg->addr == 0x2F5U) { return msg->data[0] >> 4; }
  if (msg->addr == 0x228U) { return msg->data[3] & 15U; }
  return msg->data[7] & 15U;  // 0x3AD
}

static uint32_t t9_checksum(const CANPacket_t *msg) {
  uint8_t sum = 0U;
  const uint8_t seed = (msg->addr == 0x2F5U) ? 11U : ((msg->addr == 0x228U) ? 3U : 13U);
  for (int i = 0; i < GET_LEN(msg); i++) {
    sum += (msg->data[i] >> 4) + (msg->data[i] & 15U);
  }
  return (seed - sum + t9_get_checksum(msg)) & 15U;
}

static uint32_t t9_checksum_received(const CANPacket_t *msg) { return t9_get_checksum(msg); }

static uint8_t t9_counter(const CANPacket_t *msg) {
  if (msg->addr == 0x2F5U) { return msg->data[0] & 15U; }
  if (msg->addr == 0x228U) { return msg->data[3] >> 4; }
  return msg->data[7] >> 4;
}

static void t9_rx(const CANPacket_t *msg) {
  t9_split_sync_common_stop();
  uint32_t now = microsecond_timer_get();
  int slot = -1;
  if (msg->addr == 0x3F2U) {
    if ((msg->bus != 2U) && (safety_get_ts_elapsed(now, t9_init_ts) >= 2000000U)) {
      t9_wiring_fault = true;
      controls_allowed = false;
    } else if (msg->bus == 2U) {
      for (int i = 0; i < 8; i++) { t9_template[i] = msg->data[i]; }
      t9_stock_ts = now; t9_stock_seen = true;
    }
  } else if (msg->addr == 0x495U) {
    // EPS must be on the destination side. No assumed echo/acknowledgement.
    if ((msg->bus != 0U) && (safety_get_ts_elapsed(now, t9_init_ts) >= 2000000U)) {
      t9_wiring_fault = true;
      controls_allowed = false;
    } else if (msg->bus == 0U) {
      t9_eps_state = (msg->data[2] >> 2) & 7U;
      t9_eps_ts = now; t9_eps_seen = true;
      t9_eps_activity = (msg->data[2] & 2U) != 0U;
      if (t9_eps_activity && (t9_eps_state == 3)) { t9_activity_ts = now; t9_activity_seen = true; }
      if ((t9_cycle_phase == 2) && ((t9_eps_state == 1) || (t9_eps_state == 2)) &&
          (safety_get_ts_elapsed(now, t9_cycle_ts) > 0U)) { t9_cycle_release_seen = true; }
      uint32_t delay = safety_get_ts_elapsed(now, t9_request_ts);
      if (t9_request_seen && !t9_eps_ack && (t9_eps_state == 3) && (delay > 0U) && (delay < 500000U)) {
        t9_eps_ack = true; t9_active_ts = now; t9_active_seen = true;
        if (t9_cycle_phase == 4) { t9_cycle_phase = 0; }
      }
    }
  } else if (msg->addr == 0x2F5U) {
    slot = 0; t9_driver = to_signed(msg->data[1], 8);
    update_sample(&torque_driver, t9_driver);
    steering_disengage = !t9_pause_session() && (t9_split_profile() ? t9_driver_pause() :
      ((t9_driver < -T9_DRIVER_OVERRIDE_LIMIT) || (t9_driver > T9_DRIVER_OVERRIDE_LIMIT)));
  } else if (msg->addr == 0x452U) {
    slot = 9; t9_blinker = (msg->data[0] & 0x30U) != 0U;
  } else if (msg->addr == 0x30DU) {
    slot = 1; int sum = 0; int low = 65535; int high = 0;
    for (int i = 0; i < 8; i += 2) {
      int speed = (msg->data[i] << 8) | msg->data[i + 1];
      sum += speed; low = SAFETY_MIN(low, speed); high = SAFETY_MAX(high, speed);
    }
    t9_speed_centi_kph = ((high - low) <= 500) ? sum / 4 : -1;
    vehicle_moving = high > 10;
    UPDATE_VEHICLE_SPEED(SAFETY_MAX(t9_speed_centi_kph, 0) * 0.01 * KPH_TO_MS);
  } else if (msg->addr == 0x348U) {
    slot = 2; t9_gear = msg->data[0] >> 4; t9_t15 = (msg->data[6] & 0x40U) != 0U;
  } else if (msg->addr == 0x412U) {
    slot = 3; brake_pressed = (msg->data[0] & 0x20U) != 0U;
    t9_reverse = (msg->data[0] & 4U) != 0U; t9_doors = (msg->data[6] & 0x78U) != 0U;
  } else if (msg->addr == 0x3ADU) {
    slot = 4; t9_park = msg->data[3] & 7U;
  } else if (msg->addr == 0x572U) {
    slot = 5; t9_belt = msg->data[0] >> 6;
  } else if (msg->addr == 0x228U) {
    slot = 6; gas_pressed = msg->data[2] > 0U; t9_pedal_invalid = msg->data[2] > 200U;
  } else if (msg->addr == 0x50EU) {
    slot = 7;
    uint8_t low = 0U, high = 0U;
    for (int bit = 0; bit < 4; bit++) {
      low ^= (msg->data[6] >> bit) & 1U;
      high ^= (msg->data[6] >> (bit + 4)) & 1U;
    }
    bool valid = (((msg->data[0] >> 4) & 3U) == (((uint32_t)high << 1U) | low));
    bool rvv_mode = (((msg->data[7] >> 5) & 3U) == 1U);
    bool activation = (msg->data[7] & 0x80U) != 0U;
    t9_mode = valid && rvv_mode && activation && (msg->data[6] < 255U);
    t9_stock_cruise_off = valid && rvv_mode && !activation && (msg->data[6] == 255U);
  } else if (msg->addr == 0x208U) {
    slot = 8; t9_cruise_state = (msg->data[4] >> 2) & 3U; t9_cruise = t9_cruise_state == 2;
  } else { }
  if (slot >= 0) { t9_rx_ts[slot] = now; t9_seen[slot] = true; }
  bool ready = t9_ready(now);
  // Only a fresh physical RVV off->on transition can arm this lateral test.
  // A safety cut cannot re-arm itself while the stock RVV remains active.
  if ((msg->addr == 0x208U) || (msg->addr == 0x50EU)) {
    bool physical_off = t9_stock_cruise_off && ((t9_cruise_state == 0) || (t9_cruise_state == 3)) &&
      t9_fresh(now, t9_rx_ts[7], t9_seen[7], T9_FRESH_US) && t9_fresh(now, t9_rx_ts[8], t9_seen[8], T9_FRESH_US);
    if (physical_off) {
      t9_cycle_phase = 0; t9_active_seen = false;
      t9_cruise_rearm = true;
      t9_request_seen = false; t9_eps_ack = false;
      t9_split_lateral_stopped = false; t9_split_rvv_stopped = false;
    }
    // Evaluate the combined physical state on either message: the BSI frame
    // may arrive after the engine's rising edge. Never treat pedal override
    // (engine state 1, BSI still active) as a new driver engagement.
    bool engaged = t9_cruise && t9_mode;
    if (engaged && !cruise_engaged_prev) {
      t9_engage_pending = t9_cruise_rearm;
      t9_engage_ts = now;
      t9_cruise_rearm = false;
    }
    cruise_engaged_prev = engaged;
  }
  // RX arrives one frame at a time, whereas the host evaluates a batch.
  // Complete this one explicit engagement after the other fresh inputs
  // arrive, for at most one freshness window. Consume it on admission so a
  // later fault/recovery cannot re-arm controls without another physical off.
  if (t9_engage_pending && (!t9_cruise || !t9_mode || (safety_get_ts_elapsed(now, t9_engage_ts) > T9_FRESH_US))) {
    t9_engage_pending = false;
  }
  if (t9_split_profile()) {
    bool common_ready = t9_common_ready(now) && !t9_output_lease_expired(now);
    if (t9_engage_pending && common_ready && !t9_probe) {
      t9_split_lateral_allowed = ready && !t9_split_lateral_stopped;
      t9_split_rvv_allowed = !t9_split_rvv_stopped;
      controls_allowed = t9_split_lateral_allowed || t9_split_rvv_allowed;
      t9_engage_pending = false;
    }
    if (!common_ready) {
      // Before admission, retain the bounded explicit engagement while the
      // rest of this CAN batch arrives. Once active, any common fault cuts.
      if (t9_split_lateral_allowed || t9_split_rvv_allowed) { t9_split_common_stop(); }
    } else if (!ready) { t9_split_lateral_stop(); }
  } else if (t9_engage_pending && ready && !t9_probe) {
    controls_allowed = true;
    t9_engage_pending = false;
  }
  if (!t9_split_profile() && !ready) { controls_allowed = false; }
}

static bool t9_tx(const CANPacket_t *msg) {
  t9_split_sync_common_stop();
  const TorqueSteeringLimits limits = {
    // The shared RT helper includes both endpoints and rolls over after >250 ms.
    .max_torque = T9_MAX_TORQUE, .max_rate_up = 1, .max_rate_down = 1, .max_rt_delta = 6,
    .type = TorqueDriverLimited, .driver_torque_allowance = T9_DRIVER_OVERRIDE_LIMIT, .driver_torque_multiplier = 1,
  };
  uint32_t now = microsecond_timer_get();
  if (t9_split_profile() && (!t9_common_ready(now) || t9_output_lease_expired(now))) { t9_split_common_stop(); }
  if (t9_probe || msg->extended || (msg->addr != 0x3F2U) || (msg->bus != 0U) ||
      (GET_LEN(msg) != 8) || !t9_topology_ready(now) || t9_output_lease_expired(now)) { return false; }
  static const uint8_t opaque_mask[8] = {255U, 255U, 255U, 0U, 3U, 1U, 255U, 255U};
  for (int i = 0; i < 8; i++) {
    if ((msg->data[i] & opaque_mask[i]) != (t9_template[i] & opaque_mask[i])) { return false; }
  }
  if ((msg->data[5] & 1U) || msg->data[6] || (msg->data[7] & 0xFCU)) { return false; }
  int torque = to_signed((msg->data[3] << 3) | (msg->data[4] >> 5), 11);
  int state = (msg->data[4] >> 2) & 7U;
  int factor = msg->data[5] >> 1;
  int stock_state = (t9_template[4] >> 2) & 7U;
  if ((state < 2) || (state > 4) || (factor > 100) || (stock_state < 2) || (stock_state > 4)) { return false; }
  bool ready = t9_ready(now);
  if (t9_split_profile()) {
    if (!t9_common_ready(now) || t9_output_lease_expired(now)) { t9_split_common_stop(); }
    else if (!ready) { t9_split_lateral_stop(); }
  } else if (!ready) { controls_allowed = false; }
  // Do not replace the healthy factory stream with an idle host stream.
  // Starting interception requires the physical cruise edge and all gates;
  // a zero state-2 command is reserved for releasing an existing takeover.
  if (!t9_replacing_stock && ((state == 2) || !t9_lateral_allowed() || !ready)) { return false; }
  if ((state == 4) && (!t9_lateral_allowed() || !ready || (stock_state < 3))) { return false; }
  if ((state == 2) && ((factor != 0) || (torque != 0))) { return false; }
  if ((state == 3) && ((torque != 0) || (factor > t9_last_factor))) { return false; }
  if ((torque != 0) && ((state != 4) || (factor != 100) || (t9_eps_state != 3) ||
      !t9_request_seen || !t9_eps_ack)) { return false; }
  // Physical turn signals and raw driver effort independently reject every
  // nonzero command. Zero keeps the EPS session; it cannot reset its watchdog.
  if (t9_split_profile() && (torque != 0) && (t9_blinker || t9_driver_pause())) { return false; }
  bool cycle_marker = t9_cycle_profile() && (t9_cycle_phase == 0) && (state == 4) && (factor == 0);
  bool cycle_invalid = false;
  if (cycle_marker) {
    cycle_invalid = !t9_lateral_allowed() || !t9_started_control || !t9_eps_ack || (t9_eps_state != 3) ||
      !t9_active_seen || (safety_get_ts_elapsed(now, t9_active_ts) < T9_CYCLE_PERIOD_US) ||
      !t9_eps_activity || !t9_fresh(now, t9_activity_ts, t9_activity_seen, T9_FRESH_US) ||
      t9_driver_pause() || t9_blinker || (t9_last_factor != 100);
  }
  if (t9_cycle_phase != 0) {
    cycle_invalid = torque != 0;
    if (t9_cycle_phase == 1) { cycle_invalid |= ((state != 2) && (state != 4)) || (factor != 0); }
    if (t9_cycle_phase == 2) {
      cycle_invalid |= ((state != 2) && (state != 3)) || (factor != 0) ||
        ((state == 3) && (!t9_cycle_release_seen || (t9_eps_state == 3)));
    }
    if (t9_cycle_phase == 3) {
      cycle_invalid |= ((state == 3) && (factor != 0)) ||
        ((state == 4) && ((factor == 0) || (safety_get_ts_elapsed(now, t9_cycle_prepare_ts) < 90000U)));
    }
    if (t9_cycle_phase == 4) { cycle_invalid |= state == 3; }
  }
  if (cycle_invalid) { t9_split_lateral_stop(); return false; }
  // A gap cannot resume a nonzero command; neither can an overfrequency stream.
  uint32_t gap = safety_get_ts_elapsed(now, t9_last_tx_ts);
  if ((torque != 0) && (!t9_tx_seen || (gap > 150000U) || (gap < 45000U))) { return false; }
  if ((state == 4) && !t9_request_seen) {
    if ((torque != 0) || (t9_eps_state == 3)) { return false; }
    t9_request_ts = now; t9_request_seen = true;
  }
  // Zero torque is always an immediate release, not subject to a slow ramp.
  if (torque == 0) { desired_torque_last = 0; rt_torque_last = 0; }
  if (steer_torque_cmd_checks(torque, state == 4, limits)) { return false; }
  if (cycle_marker) {
    t9_cycle_phase = 1; t9_cycle_ts = now; t9_cycle_release_seen = false;
  } else if ((t9_cycle_phase == 1) && (state == 2)) {
    t9_cycle_phase = 2;
  } else if ((t9_cycle_phase == 2) && (state == 3)) {
    t9_cycle_phase = 3; t9_cycle_prepare_ts = now;
  } else if ((t9_cycle_phase == 3) && (state == 4)) {
    t9_cycle_phase = 4;
  } else if ((t9_cycle_phase >= 3) && (state == 2)) {
    t9_cycle_phase = 0;
  }
  if (state != 4) { t9_request_seen = false; t9_eps_ack = false; }
  if ((state == 2) && t9_started_control && (t9_cycle_phase == 0)) {
    if (t9_split_profile()) { t9_split_lateral_stop(); }
    else { controls_allowed = false; }
    t9_started_control = false;
  }
  if ((state >= 3) && t9_lateral_allowed()) { t9_started_control = true; }
  t9_last_factor = factor; t9_last_tx_ts = now; t9_tx_seen = true;
  t9_replacing_stock = (state != 2) || (t9_cycle_phase != 0);
  return true;
}

#define T9_RX_BOTH(addr_, len_, hz_) \
  {.msg = {{addr_, 0, len_, hz_, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, \
           {addr_, 2, len_, hz_, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}}}
#define T9_RX_CHECKED(addr_, len_, hz_) \
  {.msg = {{addr_, 0, len_, hz_, .max_counter = 15, .ignore_quality_flag = true}, \
           {addr_, 2, len_, hz_, .max_counter = 15, .ignore_quality_flag = true}, {0}}}

static safety_config t9_init(uint16_t param) {
  t9_cycle_phase = 0; t9_cycle_release_seen = false; t9_active_seen = false;
  t9_activity_seen = false; t9_eps_activity = false;
  t9_cycle_ts = 0U; t9_cycle_prepare_ts = 0U; t9_active_ts = 0U; t9_activity_ts = 0U;
  t9_split_lateral_allowed = false; t9_split_rvv_allowed = false;
  t9_split_lateral_stopped = false; t9_split_rvv_stopped = false;
  t9_probe = param == PSA_T9_PROBE_PARAM;
  t9_stock_seen = false; t9_eps_seen = false; t9_wiring_fault = false;
  t9_request_seen = false; t9_tx_seen = false; t9_eps_ack = false; t9_started_control = false; t9_last_factor = 0;
  t9_replacing_stock = false;
  t9_init_ts = microsecond_timer_get(); t9_stock_ts = 0; t9_eps_ts = 0;
  t9_request_ts = 0; t9_last_tx_ts = 0; t9_eps_state = 0;
  t9_driver = 0; t9_speed_centi_kph = 0; t9_gear = 0; t9_park = 0; t9_belt = 0;
  t9_reverse = false; t9_doors = false; t9_t15 = false; t9_cruise = false; t9_mode = false; t9_pedal_invalid = true;
  t9_stock_cruise_off = false; t9_cruise_rearm = false; t9_cruise_state = -1;
  t9_engage_pending = false; t9_engage_ts = 0U;
  t9_blinker = false;
  for (int i = 0; i < 10; i++) { t9_rx_ts[i] = 0; t9_seen[i] = false; }
  for (int i = 0; i < 8; i++) { t9_template[i] = 0; }
  for (int i = 0; i < 2048; i++) { t9_source_side[i] = 0; }
  // Keep relay malfunction detection, but defer blocking the factory stream
  // until the first accepted replacement. Static blocking here creates an
  // EPS watchdog gap during the two-second host/Panda startup settle.
  static const CanMsg tx[] = {{0x3F2, 0, 8, .check_relay = true, .disable_static_blocking = true}};
  static RxCheck rx[] = {
    {.msg = {{0x3F2, 2, 8, 20, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x495, 0, 4, 10, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    T9_RX_CHECKED(0x2F5, 7, 100), T9_RX_BOTH(0x30D, 8, 50), T9_RX_BOTH(0x348, 8, 50),
    T9_RX_BOTH(0x412, 8, 20), T9_RX_CHECKED(0x3AD, 8, 50), T9_RX_BOTH(0x572, 8, 10),
    T9_RX_CHECKED(0x228, 8, 100), T9_RX_BOTH(0x50E, 8, 10), T9_RX_BOTH(0x208, 8, 100),
    T9_RX_BOTH(0x452, 6, 20),
  };
  safety_config config = BUILD_SAFETY_CFG(rx, tx);
  if (t9_probe) { config.tx_msgs = NULL; config.tx_msgs_len = 0; }
  return config;
}

static bool t9_fwd(int bus_num, int addr) {
  // A closed/miswired split must not create an amplifying 0<->2 forwarding
  // loop. The first identifier received from both sides aborts interception.
  if ((addr >= 0) && (addr < 2048) && ((bus_num == 0) || (bus_num == 2))) {
    t9_source_side[addr] |= (bus_num == 0) ? 1U : 2U;
    if (t9_source_side[addr] == 3U) {
      t9_wiring_fault = true; controls_allowed = false; relay_malfunction = true;
    }
  }
  // FWD sees every physical RX, including a later unexpected second source.
  if ((safety_get_ts_elapsed(microsecond_timer_get(), t9_init_ts) >= 2000000U) &&
      (((addr == 0x3F2) && (bus_num == 0)) || ((addr == 0x495) && (bus_num == 2)))) {
    t9_wiring_fault = true;
    controls_allowed = false;
  }
  return t9_wiring_fault || (!t9_probe && t9_replacing_stock && (bus_num == 2) && (addr == 0x3F2));
}

const safety_hooks psa_t9_hooks = {
  .init = t9_init, .rx = t9_rx, .tx = t9_tx, .fwd = t9_fwd,
  .get_checksum = t9_checksum_received, .compute_checksum = t9_checksum, .get_counter = t9_counter,
};

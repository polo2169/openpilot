#pragma once

// Separate opt-in profile for a P87 lateral experiment. The 5008 observation
// interface uses NO_OUTPUT. No legacy radar/ACC/spoofing TX is authorized here.
#define PSA_5008_LATERAL_PARAM 0x5008U

static bool psa_5008_profile = false;
static bool p87_t15, p87_fault, p87_body_ok, p87_acc_ok;
static int p87_engine, p87_eps, p87_speed, p87_driver;

static safety_config p87_init(void) {
  p87_t15 = false;
  p87_fault = true;
  p87_body_ok = false;
  p87_acc_ok = false;
  p87_engine = 0;
  p87_eps = 0;
  p87_speed = 0;
  p87_driver = 0;
  static const CanMsg tx[] = {
    {0x3F2, 0, 8, .check_relay = true},
  };
  static RxCheck rx[] = {
    {.msg = {{0x228, 0, 8, 100, .max_counter = 15, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x348, 0, 8, 50, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x30D, 0, 8, 50, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x2F5, 0, 7, 100, .max_counter = 15, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x495, 0, 4, 10, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x412, 2, 8, 20, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x452, 1, 6, 20, .max_counter = 15, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x2B6, 1, 8, 50, .max_counter = 15, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x32D, 1, 8, 50, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };
  return BUILD_SAFETY_CFG(rx, tx);
}

static void p87_rx(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Driver pedal, not 0x208 calculated demand (positive during stock ACC).
    // Reserved/out-of-range values also inhibit steering.
    if (msg->addr == 0x228U) { gas_pressed = msg->data[2] != 0U; }
    if (msg->addr == 0x348U) {
      p87_t15 = (msg->data[6] & 0x40U) != 0U;
      p87_engine = msg->data[5] & 0xFU;
    }
    if (msg->addr == 0x30DU) {
      int sum = 0;
      for (int i = 0; i < 8; i += 2) { sum += (msg->data[i] << 8) | msg->data[i + 1]; }
      p87_speed = sum / 4;
      vehicle_moving = p87_speed > 10;
      UPDATE_VEHICLE_SPEED(p87_speed * 0.01 * KPH_TO_MS);
    }
    if (msg->addr == 0x2F5U) {
      p87_driver = to_signed(msg->data[1], 8) * 3;
      update_sample(&torque_driver, p87_driver);
    }
    if (msg->addr == 0x495U) { p87_eps = (msg->data[2] >> 2) & 7U; }
  }
  if ((msg->bus == 2U) && (msg->addr == 0x412U)) {
    brake_pressed = (msg->data[0] & 0x20U) != 0U;
    p87_body_ok = ((msg->data[0] & 5U) == 0U) && ((msg->data[6] & 0x18U) == 0U);
  }
  if (msg->bus == 1U) {
    if (msg->addr == 0x452U) { pcm_cruise_check((msg->data[2] & 0x80U) != 0U); }
    if (msg->addr == 0x2B6U) {
      int acc = msg->data[3] & 0xFU;
      p87_acc_ok = (acc == 4) || (acc == 6);
    }
    if (msg->addr == 0x32DU) {
      p87_fault = ((msg->data[2] >> 3) & 3U) == 3U || (msg->data[3] & 0x80U) != 0U;
    }
  }
  if (p87_fault || !p87_acc_ok || !p87_t15 || !p87_body_ok || gas_pressed || brake_pressed) {
    controls_allowed = false;
  }
}

static bool p87_ready(void) {
  bool ready = controls_allowed && p87_t15 && (p87_engine == 3) && !p87_fault && p87_acc_ok && p87_body_ok &&
    !gas_pressed && !brake_pressed && (p87_speed >= 5200) && (p87_speed <= 14000) &&
    (p87_eps >= 1) && (p87_eps <= 3) && (SAFETY_ABS(p87_driver) <= 45);
  const uint32_t now = microsecond_timer_get();
  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    const RxCheck *rx = &current_safety_config.rx_checks[i];
    ready = ready && rx->status.msg_seen && rx->status.valid_checksum && rx->status.valid_quality_flag &&
      (rx->status.wrong_counters < MAX_WRONG_COUNTERS) &&
      (safety_get_ts_elapsed(now, rx->status.last_timestamp) <= 250000U);
  }
  return ready;
}

static bool p87_tx(const CANPacket_t *msg) {
  if (msg->extended || msg->fd || (msg->addr != 0x3F2U) || (msg->bus != 0U) || (GET_LEN(msg) != 8)) { return false; }
  int torque = to_signed((msg->data[3] << 3) | (msg->data[4] >> 5), 11);
  int factor = msg->data[5] >> 1;
  int status = (msg->data[4] >> 2) & 7U;
  int angle = (msg->data[6] << 6) | (msg->data[7] >> 2);
  bool neutral = (torque == 0) && (factor == 0) && (status == 2);
  bool allowed = (factor <= 100) && (angle == 0) && (status >= 2) && (status <= 4);
  if (!neutral) {
    allowed = allowed && p87_ready() && (status >= 3) && (factor > 0) &&
      ((torque == 0) || ((p87_eps == 3) && (status == 4)));
  }
  static const TorqueSteeringLimits limits = {
    .max_torque = 150, .max_rate_up = 8, .max_rate_down = 38, .max_rt_delta = 75,
    .driver_torque_allowance = 50, .driver_torque_multiplier = 1, .type = TorqueDriverLimited,
  };
  // Enforce the check result. The legacy 3008 hook currently discards it.
  if (!allowed) { torque = 0; }
  bool violation = steer_torque_cmd_checks(torque, factor != 0, limits);
  return allowed && !violation;
}

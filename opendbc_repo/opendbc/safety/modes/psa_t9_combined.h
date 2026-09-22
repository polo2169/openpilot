#pragma once

// Combined experiment: retain the complete lateral authorization lifecycle.
// RVV may only reduce/recover a stock setpoint while those same gates hold.
// Only the explicit 0x1316/17 profile permits the bounded planned EPS cycle.
static safety_config t9_combined_init(uint16_t param) {
  bool probe = (param == PSA_T9_COMBINED_PROBE_PARAM) || (param == PSA_T9_SPLIT_PROBE_PARAM) || (param == PSA_T9_CYCLE_PROBE_PARAM);
  safety_config config = t9_init(probe ? PSA_T9_PROBE_PARAM : PSA_T9_LATERAL_PARAM);
  // The lateral RX list already includes every RVV input and its checks.
  (void)rvv_init(probe ? PSA_T9_RVV_PROBE_PARAM : PSA_T9_RVV_PARAM);
  return config;
}

static void t9_combined_rx(const CANPacket_t *msg) {
  t9_split_sync_common_stop();
  t9_rx(msg);
  rvv_rx(msg);
}

static bool t9_combined_tx(const CANPacket_t *msg) {
  t9_split_sync_common_stop();
  bool accepted = t9_tx(msg);
  if (!controls_allowed) { rvv_release(false); }
  return accepted;
}

static bool t9_combined_fwd(int bus, int addr) {
  t9_split_sync_common_stop();
  bool lateral_block = t9_fwd(bus, addr);
  bool rvv_block = rvv_fwd(bus, addr);
  t9_split_sync_common_stop();
  return lateral_block || rvv_block;
}

const safety_hooks psa_t9_combined_hooks = {
  .init = t9_combined_init, .rx = t9_combined_rx, .tx = t9_combined_tx, .fwd = t9_combined_fwd,
  .get_checksum = t9_checksum_received, .compute_checksum = t9_checksum, .get_counter = t9_counter,
};

#include "selfdrive/pandad/pandad.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "common/timing.h"

void PandaSafety::configureSafetyMode(bool is_onroad) {
  if (t9_guard_ != nullptr) {
    auto &guard = *t9_guard_;
    const auto no_output = [this]() { panda_->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT); };
    const auto report_stop = [this, &guard, &no_output]() {
      // Observe before issuing our own noOutput. An MCU watchdog may have
      // already restored the factory path and caused the mirrored RX frames.
      auto observed = panda_->get_state();
      if (observed) {
        LOGE("T9 lateral stopped: %s; observed mode=%u param=%u controls=%u faults=%u tx_blocked=%u rx_invalid=%u",
             guard.reason, (unsigned)observed->safety_mode_pkt, (unsigned)observed->safety_param_pkt,
             (unsigned)observed->controls_allowed_pkt, (unsigned)observed->faults_pkt,
             (unsigned)observed->safety_tx_blocked_pkt, (unsigned)observed->safety_rx_invalid_pkt);
      } else {
        LOGE("T9 lateral stopped: %s; Panda health unavailable", guard.reason);
      }
      no_output(); safety_configured_ = true;
    };
    if (!is_onroad) {
      if (guard.stage != PsaT9Guard::Stage::WAITING) no_output();
      guard.reset(); safety_configured_ = false; t9_wait_reason_.clear(); return;
    }
    if (guard.stage == PsaT9Guard::Stage::FAILED) {
      if (!safety_configured_) report_stop();
      return;
    }
    uint64_t now = nanos_since_boot();
    if (guard.stage == PsaT9Guard::Stage::WAITING) {
      std::string raw = fetchCarParams();
      if (raw.empty() || !guard.stationary(now)) {
        const char *reason = raw.empty() ? "waiting_for_controls_ready" : "waiting_for_stationary_vehicle";
        if (t9_wait_reason_ != reason) { t9_wait_reason_ = reason; LOGW("T9 waiting: %s", reason); }
        return;
      }
      AlignedBuffer buf;
      capnp::FlatArrayMessageReader reader(buf.align(raw.data(), raw.size()));
      auto cp = reader.getRoot<cereal::CarParams>();
      auto configs = cp.getSafetyConfigs();
      if (cp.getDashcamOnly() || cp.getPassive() || cp.getOpenpilotLongitudinalControl() || configs.size() != 1 ||
          configs[0].getSafetyModel() != cereal::CarParams::SafetyModel::PSA || configs[0].getSafetyParam() != guard.active_param()) {
        guard.fail("unexpected_car_params"); report_stop(); return;
      }
      panda_->set_safety_model(cereal::CarParams::SafetyModel::PSA, guard.probe_param());
      guard.begin_probe(now); LOGW("T9: stationary harness check, forwarding only, zero host CAN output");
      return;
    }
    auto health = panda_->get_state();
    uint16_t expected = guard.stage == PsaT9Guard::Stage::PROBING ? guard.probe_param() : guard.active_param();
    if (!health || !panda_->comms_healthy() || health->faults_pkt != 0 ||
        health->safety_mode_pkt != (uint8_t)cereal::CarParams::SafetyModel::PSA || health->safety_param_pkt != expected) {
      guard.fail("panda_mode_transport_or_hardware_fault"); report_stop(); return;
    }
    if (guard.stage == PsaT9Guard::Stage::PROBING && guard.probe_complete(now)) {
      panda_->set_safety_model(cereal::CarParams::SafetyModel::PSA, guard.active_param());
      guard.begin_active(now); LOGW("T9: isolated command/feedback confirmed, selecting bounded T9 safety");
    } else if (guard.stage == PsaT9Guard::Stage::ACTIVE) {
      guard.active_healthy(now);
      if (guard.tx_ready.load() && health->safety_rx_checks_invalid_pkt) guard.fail("panda_rx_checks_invalid");
    }
    if (guard.stage == PsaT9Guard::Stage::FAILED) report_stop();
    return;
  }
  // Passive PSA identification never needs ELM327 or an active safety model.
  if (dashcam_only_) {
    if (!safety_configured_) {
      panda_->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
      safety_configured_ = true;
    }
    return;
  }

  if (is_onroad && !safety_configured_) {
    updateMultiplexingMode();

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (!is_onroad) {
    initialized_ = false;
    safety_configured_ = false;
    log_once_ = false;
  }
}

void PandaSafety::updateMultiplexingMode() {
  // Initialize to ELM327 without OBD multiplexing for initial fingerprinting
  if (!initialized_) {
    prev_obd_multiplexing_ = false;
    panda_->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
    initialized_ = true;
  }

  // Switch between multiplexing modes based on the OBD multiplexing request
  bool obd_multiplexing_requested = params_.getBool("ObdMultiplexingEnabled");
  if (obd_multiplexing_requested != prev_obd_multiplexing_) {
    const uint16_t safety_param = obd_multiplexing_requested ? 0U : 1U;
    panda_->set_safety_model(cereal::CarParams::SafetyModel::ELM327, safety_param);
    prev_obd_multiplexing_ = obd_multiplexing_requested;
    params_.putBool("ObdMultiplexingChanged", true);
  }
}

std::string PandaSafety::fetchCarParams() {
  if (!params_.getBool("FirmwareQueryDone")) {
    return {};
  }

  if (!log_once_) {
    LOGW("Finished FW query, Waiting for params to set safety model");
    log_once_ = true;
  }

  if (!params_.getBool("ControlsReady")) {
    return {};
  }
  return params_.get("CarParams");
}

void PandaSafety::setSafetyMode(const std::string &params_string) {
  AlignedBuffer aligned_buf;
  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
  cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

  auto safety_configs = car_params.getSafetyConfigs();
  uint16_t alternative_experience = car_params.getAlternativeExperience();

  cereal::CarParams::SafetyModel safety_model = safety_configs[0].getSafetyModel();
  uint16_t safety_param = safety_configs[0].getSafetyParam();

  LOGW("setting safety model: %d, param: %d, alternative experience: %d", (int)safety_model, safety_param, alternative_experience);
  panda_->set_alternative_experience(alternative_experience);
  panda_->set_safety_model(safety_model, safety_param);
}

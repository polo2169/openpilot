#pragma once

#include <string>

#include "common/params.h"
#include "selfdrive/pandad/panda.h"
#include "selfdrive/pandad/psa_t9_guard.h"

void pandad_main_thread(std::string serial);

class PandaSafety {
public:
  PandaSafety(Panda *panda, bool dashcam_only = false, PsaT9Guard *t9_guard = nullptr) : panda_(panda), dashcam_only_(dashcam_only), t9_guard_(t9_guard) {}
  void configureSafetyMode(bool is_onroad);

private:
  void updateMultiplexingMode();
  std::string fetchCarParams();
  void setSafetyMode(const std::string &params_string);

  bool initialized_ = false;
  bool log_once_ = false;
  bool safety_configured_ = false;
  bool prev_obd_multiplexing_ = false;
  std::string t9_wait_reason_;
  Panda *panda_;
  const bool dashcam_only_;
  PsaT9Guard *t9_guard_;
  Params params_;
};

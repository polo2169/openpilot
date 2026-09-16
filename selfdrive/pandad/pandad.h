#pragma once

#include <string>

#include "common/params.h"
#include "selfdrive/pandad/panda.h"

void pandad_main_thread(std::string serial);

class PandaSafety {
public:
  PandaSafety(Panda *panda, bool dashcam_only = false) : panda_(panda), dashcam_only_(dashcam_only) {}
  void configureSafetyMode(bool is_onroad);

private:
  void updateMultiplexingMode();
  std::string fetchCarParams();
  void setSafetyMode(const std::string &params_string);

  bool initialized_ = false;
  bool log_once_ = false;
  bool safety_configured_ = false;
  bool prev_obd_multiplexing_ = false;
  Panda *panda_;
  const bool dashcam_only_;
  Params params_;
};

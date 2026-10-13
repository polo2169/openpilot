from opendbc.car.psa.values import CAR


def independent_stock_cruise(CP):
  # This experimental port emits only LKA. It neither controls nor cancels ACC.
  return CP.carFingerprint == CAR.PSA_PEUGEOT_5008_P87 and not CP.openpilotLongitudinalControl


def cruise_mismatch(CP, enabled, stock_cruise_enabled):
  if independent_stock_cruise(CP):
    return False
  return stock_cruise_enabled and (not enabled or not CP.pcmCruise)

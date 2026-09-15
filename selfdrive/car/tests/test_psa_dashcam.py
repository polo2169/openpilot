import os
import unittest  # noqa: TID251 -- runs on the device without installing pytest
from unittest.mock import MagicMock, patch  # noqa: TID251 -- stdlib-only device validation

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import fingerprint
from opendbc.car.psa.fingerprints import FINGERPRINTS
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from openpilot.selfdrive.car.card import Car


class TestPSADashcamDeployment(unittest.TestCase):
  def test_passive_can_identification_never_queries_ecus(self):
    frames = [CanData(address, bytes(length), 0) for address, length in FINGERPRINTS[CAR.PSA_PEUGEOT_308_T9][0].items()]
    send = MagicMock()
    multiplex = MagicMock()
    with patch.dict(os.environ, {"SKIP_FW_QUERY": "1", "FINGERPRINT": ""}):
      result = fingerprint(lambda **kwargs: [frames], send, multiplex, None)
    self.assertEqual(result[0], CAR.PSA_PEUGEOT_308_T9)
    self.assertEqual(result[3], [])
    self.assertEqual(result[4], structs.CarParams.FingerprintSource.can)
    send.assert_not_called()
    multiplex.assert_called_once_with(False)

  def test_device_branch_stays_passive_even_with_control_enabled(self):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    ci = CarInterface(cp)
    # Simulate a mistaken non-dashcam classification and a user enabling control.
    cp.dashcamOnly = False
    cp.safetyConfigs = [structs.CarParams.SafetyConfig(safetyModel=structs.CarParams.SafetyModel.psa)]
    params = MagicMock()
    params.get.return_value = None
    params.get_bool.return_value = True
    with patch.dict(os.environ, {"PSA_DASHCAM_ONLY": "1"}), \
         patch("openpilot.selfdrive.car.card.Params", return_value=params), \
         patch("openpilot.selfdrive.car.card.messaging.sub_sock"), \
         patch("openpilot.selfdrive.car.card.messaging.SubMaster"), \
         patch("openpilot.selfdrive.car.card.messaging.PubMaster"):
      car = Car(CI=ci, RI=MagicMock())
    self.assertTrue(car.CP.passive)
    self.assertEqual(len(car.CP.safetyConfigs), 1)
    self.assertEqual(car.CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)
    self.assertEqual(car.CP.safetyConfigs[0].safetyParam, 0)
    saved = next(call for call in params.put.call_args_list if call.args[0] == "CarParams")
    self.assertEqual(saved.kwargs, {"block": True})
    with structs.CarParams.from_bytes(saved.args[1]) as stored:
      self.assertTrue(stored.passive)
      self.assertEqual(stored.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)
    ready = next(call for call in params.method_calls if call[0] == "put_bool" and call.args[0] == "ControlsReady")
    persisted = next(call for call in params.method_calls if call[0] == "put" and call.args[0] == "CarParams")
    self.assertLess(params.method_calls.index(persisted), params.method_calls.index(ready))
    self.assertEqual(ready.args, ("ControlsReady", True))
    self.assertEqual(ready.kwargs, {"block": True})

    car.sm.__getitem__.return_value = []
    car.sm.seen = {'onroadEvents': True}
    with patch.object(car, "state_update", return_value=(structs.CarState(), None)), \
         patch.object(car, "state_publish"), patch.object(ci, "init") as init, patch.object(ci, "apply") as apply:
      car.step()
      car.step()
    init.assert_not_called()
    apply.assert_not_called()
    car.pm.send.assert_not_called()

  def test_active_car_still_waits_for_control_initialization(self):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    ci = CarInterface(cp)
    cp.dashcamOnly = False
    ci.CC = MagicMock()
    params = MagicMock()
    params.get.return_value = None
    params.get_bool.return_value = True
    with patch.dict(os.environ, {"PSA_DASHCAM_ONLY": "0"}), \
         patch("openpilot.selfdrive.car.card.Params", return_value=params), \
         patch("openpilot.selfdrive.car.card.messaging.sub_sock"), \
         patch("openpilot.selfdrive.car.card.messaging.SubMaster"), \
         patch("openpilot.selfdrive.car.card.messaging.PubMaster"):
      car = Car(CI=ci, RI=MagicMock())
    self.assertFalse(car.CP.passive)
    self.assertFalse(any(call.args[0] == "ControlsReady" for call in params.put_bool.call_args_list))


if __name__ == "__main__":
  unittest.main()

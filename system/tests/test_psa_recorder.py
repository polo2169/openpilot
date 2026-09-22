import os
from pathlib import Path
import tempfile
import unittest  # noqa: TID251 -- stdlib-only validation on the device
from unittest.mock import MagicMock, patch  # noqa: TID251 -- stdlib-only validation on the device

from openpilot.system.psa_recorder import RotatingLog, SERVICES, record


class TestPSARecorder(unittest.TestCase):
  def test_rotation_retention_and_restart_ignore_wall_clock(self):
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      unrelated = root / "user-notes.txt"
      unrelated.write_text("keep")
      writer = RotatingLog(root, segment_bytes=8, max_segments=2, min_free_bytes=0)
      try:
        for payload in (b"first123", b"second45", b"third678"):
          self.assertTrue(writer.write(payload))
        writer.flush()
        self.assertEqual([p.read_bytes() for p in writer.files()], [b"second45", b"third678"])
        os.utime(writer.files()[-1], (1, 1))
      finally:
        writer.close()
      restarted = RotatingLog(root, segment_bytes=8, max_segments=2, min_free_bytes=0)
      try:
        self.assertTrue(restarted.write(b"fourth90"))
        restarted.flush()
        self.assertEqual([p.name for p in restarted.files()], ["capture-00000003.rlog", "capture-00000004.rlog"])
        self.assertEqual([p.read_bytes() for p in restarted.files()], [b"third678", b"fourth90"])
        self.assertEqual(len(list(root.glob("capture-*.json"))), 2)
        self.assertEqual(unrelated.read_text(), "keep")
      finally:
        restarted.close()

  def test_complete_messages_remain_separate_and_flushed(self):
    with tempfile.TemporaryDirectory() as directory:
      writer = RotatingLog(directory, segment_bytes=8, min_free_bytes=0)
      try:
        for payload in (b"abc", b"def", b"ghij"):
          writer.write(payload)
        writer.flush()
        self.assertEqual([p.read_bytes() for p in writer.files()], [b"abcdef", b"ghij"])
        with self.assertRaises(ValueError):
          writer.write(b"too large" * 5)
      finally:
        writer.close()

  def test_low_space_preserves_existing_data(self):
    with tempfile.TemporaryDirectory() as directory:
      writer = RotatingLog(directory, segment_bytes=8, min_free_bytes=0)
      try:
        writer.write(b"preserve")
        writer.flush()
        writer.next_check = 0
        with patch("openpilot.system.psa_recorder.shutil.disk_usage") as usage:
          usage.return_value.free = 0
          self.assertFalse(writer.write(b"skip"))
        self.assertEqual([p.read_bytes() for p in writer.files()], [b"preserve"])
      finally:
        writer.close()

  def test_second_writer_cannot_interleave_the_log(self):
    with tempfile.TemporaryDirectory() as directory:
      writer = RotatingLog(directory)
      try:
        with self.assertRaises(BlockingIOError):
          RotatingLog(directory)
      finally:
        writer.close()

  def test_entrypoint_keeps_recording_in_each_psa_profile(self):
    from openpilot.system.psa_recorder import main
    modes = ('PSA_DASHCAM_ONLY', 'PSA_T9_LATERAL_TEST', 'PSA_T9_RVV_TEST')
    with patch.dict(os.environ, dict.fromkeys(modes, '0')):
      with patch('openpilot.system.psa_recorder.record') as run:
        main()
        run.assert_not_called()
      for mode in modes:
        with patch.dict(os.environ, {mode: '1'}), \
             patch('openpilot.system.psa_recorder.record', side_effect=KeyboardInterrupt) as run:
          with self.assertRaises(KeyboardInterrupt):
            main()
          run.assert_called_once_with()

  def test_offroad_can_and_state_events_are_readable_without_transmission(self):
    from cereal import messaging
    from openpilot.tools.lib.logreader import LogReader

    can = messaging.new_message("can", 1, valid=True)
    can.can[0] = {"address": 0x208, "dat": b"\x12\x34", "src": 0}
    panda = messaging.new_message("pandaStates", 1, valid=True)
    panda.pandaStates[0].ignitionLine = False
    panda.pandaStates[0].ignitionCan = False
    panda.pandaStates[0].canState2.totalErrorCnt = 6404
    device = messaging.new_message("deviceState", valid=True)
    device.deviceState.started = False
    peripheral = messaging.new_message("peripheralState", valid=True)
    events = [can, panda, device, peripheral]
    for service in SERVICES[4:]:
      if service == 'logMessage':
        event = messaging.new_message(None)
        event.logMessage = '{"msg":"psa_t9_lateral test_cut"}'
      else:
        event = messaging.new_message(service, 0, valid=True) if service in ('sendcan', 'onroadEvents', 'customReservedRawData0') else messaging.new_message(service, valid=True)
      events.append(event)
    sockets = [MagicMock() for _ in SERVICES]
    # msgq.Poller returns fresh wrappers, not the subscription objects.
    polled_sockets = [MagicMock() for _ in SERVICES]
    for sock, event in zip(polled_sockets, events, strict=True):
      sock.receive.side_effect = [event.to_bytes(), None]
    pending = polled_sockets.copy()

    def poll(_timeout):
      result = pending.copy()
      pending.clear()
      return result

    with tempfile.TemporaryDirectory() as directory, \
         patch.object(messaging, "sub_sock", side_effect=sockets) as subscribe, \
         patch.object(messaging, "Poller") as poller, \
         patch.object(messaging, "PubMaster") as publish, \
         patch("openpilot.system.psa_recorder.shutil.disk_usage") as usage:
      # Device /tmp is a small tmpfs; the production recorder writes under /data.
      usage.return_value.free = 10 * 1024**3
      poller.return_value.poll.side_effect = poll
      record(directory, duration=0.02)
      data = b"".join(p.read_bytes() for p in sorted(Path(directory).glob("*.rlog")))
      saved = list(LogReader.from_bytes(data))
      self.assertEqual([m.which() for m in saved], list(SERVICES))
      self.assertEqual(saved[0].can[0].dat, b"\x12\x34")
      self.assertEqual(saved[1].pandaStates[0].canState2.totalErrorCnt, 6404)
      self.assertFalse(saved[2].deviceState.started)
      self.assertIn('psa_t9_lateral test_cut', saved[SERVICES.index('logMessage')].logMessage)
      self.assertEqual([call.args[0] for call in subscribe.call_args_list], list(SERVICES))
      publish.assert_not_called()
      # An abrupt power cut can leave a partial last event. Keep the complete prefix.
      with self.assertWarns(RuntimeWarning):
        recovered = list(LogReader.from_bytes(data + can.to_bytes()[:12]))
      self.assertEqual([m.which() for m in recovered], list(SERVICES))


if __name__ == "__main__":
  unittest.main()

import json
from pathlib import Path

from openpilot.system.home_uploader import HomeUploadConfig, UploadState, completed_files


def test_config_is_optional_and_validated(tmp_path):
  path = tmp_path / "upload.json"
  assert HomeUploadConfig.load(path) is None
  path.write_text(json.dumps({"server_url": "http://server:8060", "token": "secret", "chunk_bytes": 1}))
  config = HomeUploadConfig.load(path)
  assert config is not None
  assert config.chunk_bytes == 256 * 1024


def test_completed_files_skip_locked_segments_and_remember_upload(tmp_path, monkeypatch):
  root = tmp_path / "realdata"
  done = root / "2026-09-24--08-00-00--0"
  active = root / "2026-09-24--08-00-00--1"
  done.mkdir(parents=True)
  active.mkdir()
  log = done / "rlog.zst"
  log.write_bytes(b"data")
  (active / "rlog.zst").write_bytes(b"active")
  (active / "rlog.lock").touch()
  monkeypatch.setattr("openpilot.system.home_uploader.time.time", lambda: log.stat().st_mtime + 200)
  config = HomeUploadConfig("http://server", "token")
  state = UploadState(tmp_path / "state.json")
  assert completed_files(root, config, state) == [log]
  state.mark(log)
  assert completed_files(root, config, state) == []

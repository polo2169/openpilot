#!/usr/bin/env python3
"""Resumable, offroad-only upload of comma routes to a private OBD server.

The process is inert unless /data/comma_home_upload.json exists. Source files
are never deleted. Progress is acknowledged chunk by chunk by the server, so a
power or Wi-Fi interruption resumes at the last durable offset.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import random
import threading
import time

import requests
import zstandard as zstd

from cereal import log
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.hw import Paths


NetworkType = log.DeviceState.NetworkType
DEFAULT_CONFIG_PATH = Path("/data/comma_home_upload.json")
DEFAULT_STATE_PATH = Path("/data/comma-home-uploader/state.json")
UPLOADABLE_FILES = {
  "rlog", "rlog.zst", "qlog", "qlog.zst", "qcamera.ts",
  "fcamera.hevc", "dcamera.hevc", "ecamera.hevc",
}


@dataclass(frozen=True)
class HomeUploadConfig:
  server_url: str
  token: str
  chunk_bytes: int = 4 * 1024 * 1024
  min_age_seconds: int = 90
  request_timeout_seconds: int = 30
  verify_tls: bool = True
  upload_video: bool = True
  upload_logs: bool = True

  @classmethod
  def load(cls, path: Path = DEFAULT_CONFIG_PATH) -> HomeUploadConfig | None:
    try:
      raw = json.loads(path.read_text())
      server_url = str(raw["server_url"]).strip().rstrip("/")
      token = str(raw["token"]).strip()
      if not server_url.startswith(("http://", "https://")) or not token:
        raise ValueError("server_url or token missing")
      chunk_bytes = max(256 * 1024, min(int(raw.get("chunk_bytes", cls.chunk_bytes)), 8 * 1024 * 1024))
      return cls(
        server_url=server_url,
        token=token,
        chunk_bytes=chunk_bytes,
        min_age_seconds=max(30, int(raw.get("min_age_seconds", cls.min_age_seconds))),
        request_timeout_seconds=max(5, int(raw.get("request_timeout_seconds", cls.request_timeout_seconds))),
        verify_tls=bool(raw.get("verify_tls", True)),
        upload_video=bool(raw.get("upload_video", True)),
        upload_logs=bool(raw.get("upload_logs", True)),
      )
    except FileNotFoundError:
      return None
    except (KeyError, TypeError, ValueError, json.JSONDecodeError, OSError) as exc:
      cloudlog.event("home_uploader_config_invalid", error=str(exc))
      return None


class UploadState:
  def __init__(self, path: Path = DEFAULT_STATE_PATH):
    self.path = path
    try:
      self.completed: dict[str, dict] = json.loads(path.read_text()).get("completed", {})
    except (FileNotFoundError, OSError, ValueError, json.JSONDecodeError):
      self.completed = {}

  @staticmethod
  def identity(path: Path) -> dict:
    stat = path.stat()
    return {"size": stat.st_size, "mtime_ns": stat.st_mtime_ns}

  def contains(self, path: Path) -> bool:
    try:
      return self.completed.get(str(path)) == self.identity(path)
    except OSError:
      return False

  def mark(self, path: Path) -> None:
    self.completed[str(path)] = self.identity(path)
    if len(self.completed) > 20_000:
      self.completed = dict(list(self.completed.items())[-15_000:])
    self.path.parent.mkdir(parents=True, exist_ok=True)
    temporary = self.path.with_suffix(".tmp")
    temporary.write_text(json.dumps({"completed": self.completed}, separators=(",", ":")))
    os.replace(temporary, self.path)


def _eligible(path: Path, config: HomeUploadConfig, now: float) -> bool:
  if path.name not in UPLOADABLE_FILES or not path.is_file():
    return False
  if path.name.endswith((".hevc", ".ts")) and not config.upload_video:
    return False
  if path.name.startswith(("rlog", "qlog")) and not config.upload_logs:
    return False
  try:
    return path.stat().st_size > 0 and now - path.stat().st_mtime >= config.min_age_seconds
  except OSError:
    return False


def completed_files(root: Path, config: HomeUploadConfig, state: UploadState) -> list[Path]:
  now = time.time()
  candidates = []
  try:
    segments = sorted((path for path in root.iterdir() if path.is_dir()), key=lambda path: path.stat().st_mtime)
  except OSError:
    return []
  for segment in segments:
    try:
      names = {path.name for path in segment.iterdir()}
      if any(name.endswith(".lock") for name in names):
        continue
      for path in sorted(segment.iterdir(), key=lambda item: item.name):
        if _eligible(path, config, now) and not state.contains(path):
          candidates.append(path)
    except OSError:
      continue
  return candidates


def compressed_copy(source: Path) -> tuple[Path, bool]:
  if source.name not in {"rlog", "qlog"}:
    return source, False
  cache_root = Path("/data/comma-home-uploader/cache")
  cache_root.mkdir(parents=True, exist_ok=True)
  target = cache_root / f"{source.parent.name}-{source.name}.zst"
  source_identity = f"{source.stat().st_size}:{source.stat().st_mtime_ns}"
  identity_path = target.with_suffix(target.suffix + ".source")
  if target.exists() and identity_path.exists() and identity_path.read_text() == source_identity:
    return target, True
  temporary = target.with_suffix(target.suffix + ".tmp")
  compressor = zstd.ZstdCompressor(level=3)
  with source.open("rb") as input_stream, temporary.open("wb") as output_stream:
    compressor.copy_stream(input_stream, output_stream)
    output_stream.flush()
    os.fsync(output_stream.fileno())
  os.replace(temporary, target)
  identity_path.write_text(source_identity)
  return target, True


class HomeUploader:
  def __init__(self, config: HomeUploadConfig, session: requests.Session | None = None):
    self.config = config
    self.session = session or requests.Session()
    self.headers = {"Authorization": f"Bearer {config.token}"}

  def _request(self, method: str, path: str, **kwargs) -> requests.Response:
    response = self.session.request(
      method,
      f"{self.config.server_url}{path}",
      headers={**self.headers, **kwargs.pop("headers", {})},
      timeout=self.config.request_timeout_seconds,
      verify=self.config.verify_tls,
      **kwargs,
    )
    response.raise_for_status()
    return response

  def upload(self, source: Path) -> None:
    prepared, cached = compressed_copy(source)
    filename = f"{source.name}.zst" if cached else source.name
    stat = prepared.stat()
    started = self._request("POST", "/api/comma/v1/uploads", json={
      "segment": source.parent.name,
      "filename": filename,
      "size": stat.st_size,
      "mtime_ns": source.stat().st_mtime_ns,
    }).json()
    upload_id = started["upload_id"]
    offset = int(started["offset"])
    if not started.get("complete"):
      with prepared.open("rb") as stream:
        stream.seek(offset)
        while offset < stat.st_size:
          block = stream.read(self.config.chunk_bytes)
          if not block:
            raise IOError("source file ended before announced size")
          acknowledged = self._request(
            "PATCH",
            f"/api/comma/v1/uploads/{upload_id}",
            params={"offset": offset},
            headers={"Content-Type": "application/octet-stream"},
            data=block,
          ).json()
          next_offset = int(acknowledged["offset"])
          if next_offset == offset:
            stream.seek(offset)
            continue
          if next_offset < 0 or next_offset > stat.st_size:
            raise IOError("server returned an invalid upload offset")
          offset = next_offset
          stream.seek(offset)
      self._request("POST", f"/api/comma/v1/uploads/{upload_id}/complete")
    if cached:
      prepared.unlink(missing_ok=True)
      prepared.with_suffix(prepared.suffix + ".source").unlink(missing_ok=True)


def main(exit_event: threading.Event | None = None) -> None:
  exit_event = exit_event or threading.Event()
  params = Params()
  state = UploadState()
  device_state = messaging.SubMaster(["deviceState"])
  backoff = 5.0
  while not exit_event.is_set():
    config = HomeUploadConfig.load(Path(os.getenv("COMMA_HOME_UPLOAD_CONFIG", str(DEFAULT_CONFIG_PATH))))
    device_state.update(0)
    is_offroad = params.get_bool("IsOffroad")
    on_wifi = device_state["deviceState"].networkType == NetworkType.wifi
    if config is None or not is_offroad or not on_wifi:
      exit_event.wait(30 if is_offroad else 10)
      continue

    files = completed_files(Path(Paths.log_root()), config, state)
    if not files:
      backoff = 30
      exit_event.wait(backoff)
      continue

    try:
      source = files[0]
      HomeUploader(config).upload(source)
      state.mark(source)
      cloudlog.event("home_upload_success", path=str(source), size=source.stat().st_size)
      backoff = 1
    except Exception:
      cloudlog.exception("home_upload_failed")
      backoff = min(max(backoff * 2, 10), 300)
    exit_event.wait(backoff + random.uniform(0, min(backoff, 10)))


if __name__ == "__main__":
  main()

"""Bounded local CAN recording before ignition, for the PSA dashcam branch.

Only subscribes to existing cereal messages. No Panda access, Params writes,
CAN transmission, camera activation, ignition override or automatic upload.
Read the uncompressed events with LogReader.from_bytes(path.read_bytes()).
After power loss, the final event may be incomplete; preceding events survive.
"""
import datetime
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import time
import uuid

SERVICES = ("can", "pandaStates", "deviceState", "peripheralState")
LOG_ROOT = Path("/data/psa-diagnostics")
SEGMENT_BYTES = 16 * 1024 * 1024
MAX_SEGMENTS = 32
MIN_FREE_BYTES = 1024 * 1024 * 1024


class RotatingLog:
  def __init__(self, root, segment_bytes=SEGMENT_BYTES, max_segments=MAX_SEGMENTS, min_free_bytes=MIN_FREE_BYTES):
    if segment_bytes <= 0 or max_segments < 2 or min_free_bytes < 0:
      raise ValueError("Invalid recording limits")
    self.root = Path(root)
    self.root.mkdir(parents=True, exist_ok=True)
    self.lock = (self.root / ".recorder.lock").open("a")
    try:
      fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
      self.lock.close()
      raise
    self.segment_bytes, self.max_segments, self.min_free_bytes = segment_bytes, max_segments, min_free_bytes
    self.session = uuid.uuid4().hex
    self.file = None
    self.size = 0
    self.written = 0
    self.next_check = 0.0
    self.has_space = False
    files = self.files()
    self.sequence = int(files[-1].stem.split("-")[1]) + 1 if files else 1

  def files(self):
    return sorted((p for p in self.root.iterdir()
                   if re.fullmatch(r"capture-\d{8,}\.rlog", p.name) and p.is_file() and not p.is_symlink()),
                  key=lambda p: int(p.stem.split("-")[1]))

  def rotate(self):
    self.flush()
    if self.file is not None:
      self.file.close()
      self.file = None
    # Order by persistent sequence, not wall clock: the offline RTC can reset.
    for old in self.files()[:max(0, len(self.files()) - self.max_segments + 1)]:
      old.unlink()
      old.with_suffix(".json").unlink(missing_ok=True)
    path = self.root / f"capture-{self.sequence:08d}.rlog"
    self.file = path.open("xb")
    self.sequence += 1
    self.size = 0
    metadata = {"format": "cereal.Event (uncompressed)", "session": self.session,
                "created_utc_untrusted": datetime.datetime.now(datetime.UTC).isoformat(),
                "monotonic_ns": time.monotonic_ns(), "services": SERVICES}
    path.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")

  def write(self, payload):
    if len(payload) > self.segment_bytes:
      raise ValueError("One event exceeds the segment size limit")
    now = time.monotonic()
    if now >= self.next_check:
      self.has_space = shutil.disk_usage(self.root).free >= self.min_free_bytes + self.segment_bytes
      self.next_check = now + 1
    if not self.has_space:
      return False
    if self.file is None or self.size + len(payload) > self.segment_bytes:
      self.rotate()
    self.file.write(payload)
    self.size += len(payload)
    self.written += len(payload)
    return True

  def flush(self):
    if self.file is not None:
      self.file.flush()
      os.fsync(self.file.fileno())

  def close(self):
    try:
      if self.file is not None:
        try:
          self.flush()
        finally:
          self.file.close()
          self.file = None
    finally:
      self.lock.close()


def record(root=LOG_ROOT, duration=None):
  from cereal import messaging

  writer = RotatingLog(root)
  poller = messaging.Poller()
  # Poller retains the subscriptions and returns new wrappers on every poll.
  # Determine the service from the event, never from Python socket identity.
  for service in SERVICES:
    messaging.sub_sock(service, poller=poller)
  counts = dict.fromkeys(SERVICES, 0)
  storage_drops = 0
  started = time.monotonic()
  next_flush = started
  try:
    while duration is None or time.monotonic() - started < duration:
      for sock in poller.poll(100):
        # Bound each drain so continuous CAN cannot starve state messages/fsync.
        for _ in range(200):
          payload = sock.receive(non_blocking=True)
          if payload is None:
            break
          counts[messaging.log_from_bytes(payload).which()] += 1
          if not writer.write(payload):
            storage_drops += 1
      now = time.monotonic()
      if now >= next_flush:
        writer.flush()
        status = {"session": writer.session, "uptime_s": now - started, "messages_received": counts,
                  "bytes_written": writer.written, "storage_drops": storage_drops,
                  "transport_loss": "unknown; compare Panda RX/overflow counters with saved CAN",
                  "current_file": Path(writer.file.name).name if writer.file else None}
        temporary = Path(root) / "status.json.tmp"
        temporary.write_text(json.dumps(status, indent=2) + "\n")
        temporary.replace(Path(root) / "status.json")
        next_flush = now + 1
  finally:
    writer.close()


def main():
  if os.environ.get("PSA_DASHCAM_ONLY") != "1":
    return
  while True:
    try:
      record()
    except OSError as exc:
      print(f"PSA recorder storage unavailable: {exc}", flush=True)
      time.sleep(30)


if __name__ == "__main__":
  main()

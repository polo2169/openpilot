#!/usr/bin/env python3
"""Build a bounded T9 dataset archive with the matching source code.

The input is either an extracted ``psa-diagnostics`` directory or the archive
created by ``fetch_comma_308_diagnostics.sh``. The exporter selects one recorder
session and a bounded number of its newest segments. It never changes or
deletes the source capture.

The event stream is copied byte for byte. The recorder does not subscribe to
camera or GPS services, but raw CAN, CarParams and log messages can still carry
vehicle or device identifiers. Review a dataset before sharing it publicly.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import datetime
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import tarfile
from typing import Iterable


CAPTURE_METADATA = re.compile(r"capture-(\d{8,})\.json$")
MAX_METADATA_BYTES = 1024 * 1024
MAX_SEGMENT_BYTES = 32 * 1024 * 1024
DEFAULT_SEGMENTS = 4
DEFAULT_MAX_BYTES = 128 * 1024 * 1024

SOURCE_LIST = "tools/peugeot/t9_source_files.txt"
SUPPORT_FILES = (
  "PEUGEOT_308_T9.md",
  "tools/export_t9_dataset.py",
  SOURCE_LIST,
)


@dataclass(frozen=True)
class Capture:
  sequence: int
  session: str
  metadata_name: str
  metadata: dict
  rlog_name: str
  rlog: bytes


def sha256(data: bytes) -> str:
  return hashlib.sha256(data).hexdigest()


def _safe_archive_name(name: str) -> str:
  path = PurePosixPath(name)
  if path.is_absolute() or ".." in path.parts:
    raise ValueError(f"Unsafe archive member: {name}")
  return str(path)


def _load_metadata(raw: bytes, name: str) -> dict:
  if len(raw) > MAX_METADATA_BYTES:
    raise ValueError(f"Metadata is unexpectedly large: {name}")
  try:
    value = json.loads(raw)
  except (UnicodeDecodeError, json.JSONDecodeError) as exc:
    raise ValueError(f"Invalid recorder metadata: {name}") from exc
  if not isinstance(value, dict) or not isinstance(value.get("session"), str) or not value["session"]:
    raise ValueError(f"Recorder session missing from: {name}")
  return value


def _captures_from_directory(root: Path) -> tuple[list[Capture], dict | None]:
  captures = []
  for metadata_path in sorted(root.rglob("capture-*.json")):
    match = CAPTURE_METADATA.fullmatch(metadata_path.name)
    if match is None or metadata_path.is_symlink():
      continue
    rlog_path = metadata_path.with_suffix(".rlog")
    if not rlog_path.is_file() or rlog_path.is_symlink():
      continue
    if rlog_path.stat().st_size > MAX_SEGMENT_BYTES:
      raise ValueError(f"Recorder segment is unexpectedly large: {rlog_path}")
    metadata = _load_metadata(metadata_path.read_bytes(), str(metadata_path))
    captures.append(Capture(int(match.group(1)), metadata["session"], metadata_path.name,
                            metadata, rlog_path.name, rlog_path.read_bytes()))
  status_path = next((path for path in sorted(root.rglob("status.json"))
                      if path.is_file() and not path.is_symlink()), None)
  status = json.loads(status_path.read_text()) if status_path is not None else None
  return captures, status if isinstance(status, dict) else None


def _captures_from_archive(path: Path) -> tuple[list[Capture], dict | None]:
  captures = []
  status = None
  with tarfile.open(path, "r:*") as archive:
    members = {PurePosixPath(_safe_archive_name(member.name)): member for member in archive.getmembers()
               if member.isfile() and not member.issym() and not member.islnk()}
    for member_path, member in sorted(members.items(), key=lambda item: str(item[0])):
      match = CAPTURE_METADATA.fullmatch(member_path.name)
      if match is None:
        if member_path.name == "status.json" and status is None:
          stream = archive.extractfile(member)
          candidate = json.loads(stream.read()) if stream is not None else None
          status = candidate if isinstance(candidate, dict) else None
        continue
      stream = archive.extractfile(member)
      if stream is None:
        continue
      metadata = _load_metadata(stream.read(MAX_METADATA_BYTES + 1), str(member_path))
      rlog_path = member_path.with_suffix(".rlog")
      rlog_member = members.get(rlog_path)
      if rlog_member is None:
        continue
      if rlog_member.size > MAX_SEGMENT_BYTES:
        raise ValueError(f"Recorder segment is unexpectedly large: {rlog_path}")
      rlog_stream = archive.extractfile(rlog_member)
      if rlog_stream is None:
        continue
      captures.append(Capture(int(match.group(1)), metadata["session"], member_path.name,
                              metadata, rlog_path.name, rlog_stream.read()))
  return captures, status


def read_captures(source: Path) -> tuple[list[Capture], dict | None]:
  if source.is_dir():
    captures, status = _captures_from_directory(source)
  elif source.is_file() and tarfile.is_tarfile(source):
    captures, status = _captures_from_archive(source)
  else:
    raise ValueError("Source must be a recorder directory or tar archive")
  if not captures:
    raise ValueError("No complete capture/metadata pair found")
  return captures, status


def select_captures(captures: Iterable[Capture], session: str | None, segments: int) -> list[Capture]:
  if not 1 <= segments <= 32:
    raise ValueError("segments must be between 1 and 32")
  groups: dict[str, list[Capture]] = {}
  for capture in captures:
    groups.setdefault(capture.session, []).append(capture)
  if session is None:
    session = max(groups, key=lambda key: max(c.sequence for c in groups[key]))
  if session not in groups:
    raise ValueError("Requested session is not present")
  return sorted(groups[session], key=lambda capture: capture.sequence)[-segments:]


def _code_files(repo_root: Path) -> list[tuple[str, bytes]]:
  source_list = repo_root / SOURCE_LIST
  if not source_list.is_file():
    raise ValueError(f"T9 source list missing below {repo_root}")
  names = [line.strip() for line in source_list.read_text().splitlines()
           if line.strip() and not line.lstrip().startswith("#")]
  files = []
  for name in sorted(set(names) | set(SUPPORT_FILES)):
    path = repo_root / name
    if not path.is_file() or path.is_symlink():
      raise ValueError(f"T9 source file missing: {name}")
    files.append((f"code/{name}", path.read_bytes()))
  return files


def _git_provenance(repo_root: Path) -> tuple[str | None, bool | None]:
  try:
    head = subprocess.run(("git", "rev-parse", "HEAD"), cwd=repo_root, check=True,
                          capture_output=True, text=True).stdout.strip()
    status = subprocess.run(("git", "status", "--porcelain"), cwd=repo_root, check=True,
                            capture_output=True, text=True).stdout
    return head, bool(status)
  except (OSError, subprocess.CalledProcessError):
    return None, None


def _json_bytes(value: object) -> bytes:
  return (json.dumps(value, indent=2, sort_keys=True) + "\n").encode()


def _sanitized_metadata(capture: Capture, session_hash: str) -> bytes:
  metadata = dict(capture.metadata)
  metadata.pop("session", None)
  metadata["session_sha256_prefix"] = session_hash
  return _json_bytes(metadata)


def _sanitized_status(status: dict | None, session: str, session_hash: str) -> bytes | None:
  if status is None or status.get("session") != session:
    return None
  result = dict(status)
  result.pop("session", None)
  result["session_sha256_prefix"] = session_hash
  return _json_bytes(result)


def _archive_readme() -> bytes:
  return b"""# Peugeot 308 T9 dataset\n\nThis archive contains a bounded recorder session and the exact T9 source files\nneeded to interpret it. `manifest.json` lists every file, size and SHA-256.\n\nThe `.rlog` files are uncompressed concatenated `cereal.Event` messages. Read\nthem with `LogReader.from_bytes()`. Recorder metadata uses a hash prefix instead\nof the local session UUID. The event stream itself is byte-for-byte unchanged.\n\nNo camera or GPS service is subscribed by this recorder. Raw CAN, CarParams and\nlog messages may still contain vehicle or device identifiers. This archive is\nnot automatically anonymized; review it before public redistribution.\n\nThe active-control overlay is experimental. Its EPS-cycle setting defaults to\nOFF, and software/USB validation does not establish safe road operation.\n"""


def _tar_add(archive: tarfile.TarFile, name: str, payload: bytes) -> None:
  info = tarfile.TarInfo(_safe_archive_name(name))
  info.size = len(payload)
  info.mode = 0o644
  info.mtime = 0
  info.uid = info.gid = 0
  info.uname = info.gname = ""
  archive.addfile(info, io.BytesIO(payload))


def build_dataset(source: Path, output: Path, *, repo_root: Path, segments: int = DEFAULT_SEGMENTS,
                  session: str | None = None, label: str = "peugeot-308-t9",
                  max_bytes: int = DEFAULT_MAX_BYTES) -> dict:
  captures, status = read_captures(source)
  selected = select_captures(captures, session, segments)
  selected_session = selected[0].session
  session_hash = sha256(selected_session.encode())[:16]
  entries: list[tuple[str, bytes]] = [("README.md", _archive_readme())]
  for capture in selected:
    entries.append((f"dataset/{capture.metadata_name}", _sanitized_metadata(capture, session_hash)))
    entries.append((f"dataset/{capture.rlog_name}", capture.rlog))
  sanitized_status = _sanitized_status(status, selected_session, session_hash)
  if sanitized_status is not None:
    entries.append(("dataset/status.json", sanitized_status))
  entries.extend(_code_files(repo_root))
  total_bytes = sum(len(payload) for _, payload in entries)
  if total_bytes > max_bytes:
    raise ValueError(f"Selected dataset is {total_bytes} bytes; limit is {max_bytes}")
  files = {name: {"bytes": len(payload), "sha256": sha256(payload)} for name, payload in entries}
  git_head, git_dirty = _git_provenance(repo_root)
  manifest = {
    "schema": 1,
    "label": label,
    "created_utc": datetime.datetime.now(datetime.UTC).isoformat(),
    "session_sha256_prefix": session_hash,
    "capture_sequences": [capture.sequence for capture in selected],
    "capture_services": selected[0].metadata.get("services", []),
    "event_stream_anonymized": False,
    "privacy_review_required": True,
    "git_head_at_export": git_head,
    "git_worktree_dirty_at_export": git_dirty,
    "files": dict(sorted(files.items())),
  }
  entries.append(("manifest.json", _json_bytes(manifest)))
  output.parent.mkdir(parents=True, exist_ok=True)
  partial = output.with_name(output.name + ".partial")
  try:
    with tarfile.open(partial, "w:gz") as archive:
      for name, payload in sorted(entries):
        _tar_add(archive, name, payload)
    partial.replace(output)
  finally:
    partial.unlink(missing_ok=True)
  return manifest | {"archive": str(output), "archive_bytes": output.stat().st_size,
                     "archive_sha256": sha256(output.read_bytes())}


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("source", type=Path, help="psa-diagnostics directory or archive")
  parser.add_argument("--output", type=Path, required=True, help="output .tar.gz")
  parser.add_argument("--segments", type=int, default=DEFAULT_SEGMENTS)
  parser.add_argument("--session", help="exact recorder session; newest session is selected by default")
  parser.add_argument("--label", default="peugeot-308-t9")
  parser.add_argument("--max-bytes", type=int, default=DEFAULT_MAX_BYTES,
                      help="uncompressed payload limit, default 128 MiB")
  args = parser.parse_args()
  repo_root = Path(__file__).resolve().parents[2]
  try:
    report = build_dataset(args.source, args.output, repo_root=repo_root, segments=args.segments,
                           session=args.session, label=args.label, max_bytes=args.max_bytes)
  except (OSError, ValueError, tarfile.TarError) as exc:
    parser.exit(1, f"error: {exc}\n")
  print(json.dumps(report, indent=2))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

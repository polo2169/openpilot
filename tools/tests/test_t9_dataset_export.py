import hashlib
import io
import json
from pathlib import Path
import tarfile

from openpilot.tools.export_t9_dataset import build_dataset


def write_capture(root: Path, sequence: int, session: str, payload: bytes) -> None:
  stem = f"capture-{sequence:08d}"
  (root / f"{stem}.json").write_text(json.dumps({"session": session, "services": ["can", "carControl"]}))
  (root / f"{stem}.rlog").write_bytes(payload)


def code_root(root: Path) -> Path:
  source = root / "opendbc_repo/opendbc/car/psa/feature.py"
  source.parent.mkdir(parents=True)
  source.write_text("VALUE = 15\n")
  source_list = root / "tools/peugeot/t9_source_files.txt"
  source_list.parent.mkdir(parents=True)
  source_list.write_text("opendbc_repo/opendbc/car/psa/feature.py\n")
  (root / "tools/export_t9_dataset.py").parent.mkdir(parents=True, exist_ok=True)
  (root / "tools/export_t9_dataset.py").write_text("# exporter\n")
  (root / "PEUGEOT_308_T9.md").write_text("# test port\n")
  return root


def test_export_selects_one_bounded_session_and_bundles_code(tmp_path):
  source = tmp_path / "source"
  source.mkdir()
  write_capture(source, 1, "old-private-session", b"old")
  for sequence in (2, 3, 4):
    write_capture(source, sequence, "new-private-session", f"segment-{sequence}".encode())
  (source / "status.json").write_text(json.dumps({"session": "new-private-session", "bytes_written": 30}))
  output = tmp_path / "share.tar.gz"

  report = build_dataset(source, output, repo_root=code_root(tmp_path / "repo"), segments=2)

  expected_session_hash = hashlib.sha256(b"new-private-session").hexdigest()[:16]
  assert report["capture_sequences"] == [3, 4]
  assert report["session_sha256_prefix"] == expected_session_hash
  with tarfile.open(output) as archive:
    names = set(archive.getnames())
    assert "dataset/capture-00000002.rlog" not in names
    assert "dataset/capture-00000003.rlog" in names
    assert "dataset/capture-00000004.rlog" in names
    assert "code/opendbc_repo/opendbc/car/psa/feature.py" in names
    metadata = archive.extractfile("dataset/capture-00000003.json").read()
    status = archive.extractfile("dataset/status.json").read()
    manifest = json.load(archive.extractfile("manifest.json"))
  assert b"new-private-session" not in metadata + status
  assert json.loads(metadata)["session_sha256_prefix"] == expected_session_hash
  assert manifest["privacy_review_required"] is True
  assert manifest["event_stream_anonymized"] is False
  assert manifest["files"]["dataset/capture-00000003.rlog"]["sha256"] == hashlib.sha256(b"segment-3").hexdigest()


def test_export_reads_fetch_archive_without_extracting(tmp_path):
  source = tmp_path / "source"
  source.mkdir()
  write_capture(source, 12, "archive-session", b"captured")
  fetched = tmp_path / "fetched.tar.gz"
  with tarfile.open(fetched, "w:gz") as archive:
    for path in source.iterdir():
      data = path.read_bytes()
      info = tarfile.TarInfo(f"psa-diagnostics/{path.name}")
      info.size = len(data)
      archive.addfile(info, io.BytesIO(data))

  output = tmp_path / "share.tar.gz"
  report = build_dataset(fetched, output, repo_root=code_root(tmp_path / "repo"), segments=1)

  assert report["capture_sequences"] == [12]
  with tarfile.open(output) as archive:
    assert archive.extractfile("dataset/capture-00000012.rlog").read() == b"captured"


def test_export_refuses_an_unbounded_payload(tmp_path):
  source = tmp_path / "source"
  source.mkdir()
  write_capture(source, 1, "session", b"larger-than-limit")
  output = tmp_path / "share.tar.gz"

  try:
    build_dataset(source, output, repo_root=code_root(tmp_path / "repo"), max_bytes=1)
  except ValueError as exc:
    assert "limit" in str(exc)
  else:
    raise AssertionError("size limit was not enforced")
  assert not output.exists()

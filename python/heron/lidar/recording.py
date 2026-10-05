"""Capture file formats: ``.rpraw`` (wire bytes) and ``.jsonl`` (decoded).

Stdlib only, so the same reader runs unchanged on the Pi and on the host.

Two sibling files are written per capture, sharing a basename::

    desk-open_20260915T231204Z.rpraw   byte-exact wire log
    desk-open_20260915T231204Z.jsonl   decoded records, one JSON object per line

The raw file is ground truth. It is the only artifact that lets a parser --
Python or C++ -- be developed and regression-tested offline with no hardware
and no spinning motor. The JSONL file is what a human and matplotlib read.
Keeping them separate means the raw file stays byte-exact with no escaping or
base64 bloat, while the JSONL stays greppable and ``jq``-able.
"""

from __future__ import annotations

import gzip
import json
import struct
from dataclasses import dataclass
from typing import Any, Iterator

SCHEMA_VERSION = 1

#: Direction markers. Four printable bytes, so they double as visible resync
#: markers when the raw file is inspected with a hexdump.
MAGIC_TX = b"RPTX"  # host -> sensor
MAGIC_RX = b"RPRX"  # sensor -> host

#: magic(4) + seq(uint32) + t_mono_ns(uint64) + length(uint32)
_CHUNK_HEADER = struct.Struct("<4sIQI")
CHUNK_HEADER_LEN = _CHUNK_HEADER.size  # 20

# Record types in the .jsonl stream.
REC_HEADER = "header"
REC_COMMAND = "command"
REC_DESCRIPTOR = "descriptor"
REC_SAMPLE = "sample"
REC_REVOLUTION = "revolution"
REC_EVENT = "event"
REC_FOOTER = "footer"


class RecordingError(Exception):
    """A capture file was malformed or truncated in a way that matters."""


# --------------------------------------------------------------------------
# .rpraw
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Chunk:
    """One read() or write() worth of bytes, as it crossed the wire."""

    direction: str  # "tx" or "rx"
    seq: int
    t_mono_ns: int
    data: bytes
    offset: int  # Byte offset of this chunk's payload within the file.


class RawWriter:
    """Append-only writer for ``.rpraw``.

    Chunk boundaries are preserved deliberately: they are the evidence for USB
    and kernel batching behaviour, which is what bounds the measurable part of
    sensor-to-Python latency.
    """

    def __init__(self, path: str, header: dict[str, Any]) -> None:
        self._file = open(path, "wb")
        self._seq = {"tx": 0, "rx": 0}

        # The header is a plain JSON line so that `head -1 file.rpraw` works.
        self._file.write(json.dumps(header, separators=(",", ":")).encode())
        self._file.write(b"\n")
        self._file.flush()

    @property
    def offset(self) -> int:
        """Current end-of-file offset, i.e. where the next chunk header goes."""
        return self._file.tell()

    def write_chunk(self, direction: str, t_mono_ns: int, data: bytes) -> int:
        """Record one chunk. Returns the file offset of its payload."""
        if direction not in ("tx", "rx"):
            raise ValueError(f"direction must be 'tx' or 'rx', got {direction!r}")

        magic = MAGIC_TX if direction == "tx" else MAGIC_RX
        seq = self._seq[direction]
        self._seq[direction] = seq + 1

        self._file.write(_CHUNK_HEADER.pack(magic, seq, t_mono_ns, len(data)))
        payload_offset = self._file.tell()
        self._file.write(data)

        return payload_offset

    def close(self) -> None:
        if not self._file.closed:
            self._file.flush()
            self._file.close()

    def __enter__(self) -> RawWriter:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


class RawReader:
    """Reader for ``.rpraw``. Tolerates a truncated final chunk."""

    def __init__(self, path: str) -> None:
        self._file = open(path, "rb")

        line = self._file.readline()
        if not line:
            raise RecordingError(f"{path}: empty file")
        try:
            self.header: dict[str, Any] = json.loads(line)
        except json.JSONDecodeError as exc:
            raise RecordingError(f"{path}: first line is not a JSON header") from exc

        self.truncated = False

    def chunks(self) -> Iterator[Chunk]:
        while True:
            head = self._file.read(CHUNK_HEADER_LEN)
            if not head:
                return
            if len(head) < CHUNK_HEADER_LEN:
                # A capture killed mid-write. Report it rather than raising:
                # the data up to this point is still good.
                self.truncated = True
                return

            magic, seq, t_mono_ns, length = _CHUNK_HEADER.unpack(head)
            if magic not in (MAGIC_TX, MAGIC_RX):
                raise RecordingError(
                    f"bad chunk magic {magic!r} at offset "
                    f"{self._file.tell() - CHUNK_HEADER_LEN}"
                )

            offset = self._file.tell()
            data = self._file.read(length)
            if len(data) < length:
                self.truncated = True
                return

            yield Chunk(
                direction="tx" if magic == MAGIC_TX else "rx",
                seq=seq,
                t_mono_ns=t_mono_ns,
                data=data,
                offset=offset,
            )

    def rx_bytes(self) -> bytes:
        """All sensor->host bytes concatenated, alignment preserved."""
        return b"".join(c.data for c in self.chunks() if c.direction == "rx")

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def __enter__(self) -> RawReader:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


# --------------------------------------------------------------------------
# .jsonl
# --------------------------------------------------------------------------


class JsonlWriter:
    """One compact JSON object per line."""

    def __init__(self, path: str, gzip_output: bool = False) -> None:
        self._file = (
            gzip.open(path, "wt", encoding="utf-8")
            if gzip_output
            else open(path, "w", encoding="utf-8")
        )

    def write(self, record: dict[str, Any]) -> None:
        self._file.write(json.dumps(record, separators=(",", ":")))
        self._file.write("\n")

    def flush(self) -> None:
        self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.flush()
            self._file.close()

    def __enter__(self) -> JsonlWriter:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


class JsonlReader:
    """Reader for ``.jsonl``, transparently handling ``.gz``.

    A truncated final line is discarded rather than raising: JSONL is
    append-only precisely so that a capture killed with Ctrl-C is still
    readable up to the last complete record.
    """

    def __init__(self, path: str) -> None:
        self.path = path
        self._open = gzip.open if path.endswith(".gz") else open
        self.truncated = False

    def records(self) -> Iterator[dict[str, Any]]:
        with self._open(self.path, "rt", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    self.truncated = True
                    return

    def of_type(self, record_type: str) -> Iterator[dict[str, Any]]:
        for record in self.records():
            if record.get("type") == record_type:
                yield record

    def header(self) -> dict[str, Any]:
        for record in self.records():
            if record.get("type") == REC_HEADER:
                return record
        raise RecordingError(f"{self.path}: no header record")

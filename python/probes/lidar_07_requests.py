#!/usr/bin/env python3
"""Probe 7 -- do requests get answered while the motor runs?

On 2026-11-08 the LiDAR answered 20 of 20 GET_INFO requests with the motor
stopped, but only 11 of 20 with it running, requests 50 ms apart, on the
Pi's mains adapter (navigation's Inbox entry). How that test counted a
miss is not recorded. This probe repeats it so a miss can be classified.

For each motor phase (``--phases``) it sends ``--count`` requests and logs,
per request: when it went out, how long after the motor changed, every
byte that came back with its arrival time, and one outcome:

    ok        a valid descriptor and payload, first bytes after the request
    garbled   bytes arrived, but not a valid reply (or not only one)
    partial   a valid descriptor, payload cut short by the timeout
    none      nothing at all within ``--timeout-ms``

The latency of every ``ok`` is kept, so a shorter timeout elsewhere can be
judged afterwards: a reply at 300 ms is "late", not "missing". Bytes found
waiting *before* a request are logged as stray, never flushed silently,
because a late reply to the previous request lands there.

A background thread polls ``vcgencmd get_throttled`` every 0.25 s, so an
under-voltage while the motor runs shows up beside the requests it overlaps.

The gap (``--gap-ms``) is idle time from the end of one exchange (reply
complete, or timeout) to the next request. Several gaps may be given; each
phase is then run once per gap.

Writes ``<tag>_<UTC>.rpraw`` (both directions, byte exact) and ``.jsonl``
(one record per request, plus throttled samples and a summary per phase).

Usage
  deploy_pi.sh <user>@<pi> probes/lidar_07_requests.py
  deploy_pi.sh <user>@<pi> probes/lidar_07_requests.py --gap-ms 10,50,200,1000 \
      --phases running
  deploy_pi.sh <user>@<pi> probes/lidar_07_requests.py --command health
"""

from __future__ import annotations

import signal
import statistics
import sys
import threading
import time

from heron.lidar import cli, platform_info, protocol, recording, render
from heron.lidar.protocol import Command
from heron.lidar.transport import TappedTransport

#: Expected payload length for each command's single response.
PAYLOAD_LEN = {
    "info": (Command.GET_INFO, 20),
    "health": (Command.GET_HEALTH, 3),
    "rate": (Command.GET_SAMPLERATE, 4),
}


class ThrottlePoller:
    """Polls ``vcgencmd get_throttled`` on a thread; records every sample."""

    def __init__(self, t0_ns: int, period_s: float = 0.25) -> None:
        self._t0_ns = t0_ns
        self._period_s = period_s
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.samples: list[tuple[float, str | None]] = []
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._stop.is_set():
            value = platform_info.throttled()
            t = (time.monotonic_ns() - self._t0_ns) / 1e9
            with self._lock:
                self.samples.append((t, value))
            self._stop.wait(self._period_s)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join()

    def between(self, t_from: float, t_to: float) -> list[tuple[float, str | None]]:
        with self._lock:
            return [s for s in self.samples if t_from <= s[0] <= t_to]


def under_voltage_now(value: str | None) -> bool:
    """Bit 0 of get_throttled: under-voltage at this moment."""
    if not value or "=" not in value:
        return False
    return bool(int(value.split("=", 1)[1], 16) & 0x1)


def drain(transport) -> list[tuple[int, bytes]]:
    """Read whatever is already waiting, without blocking."""
    chunks = []
    while True:
        t_ns, data = transport.read_chunk(4096, 0.0)
        if not data:
            return chunks
        chunks.append((t_ns, data))


def exchange(transport, command: int, payload_len: int, timeout_s: float):
    """Send one request and read until a full reply or the timeout.

    Returns (t_sent_ns, chunks, outcome, latency_ms, payload).
    """
    want = protocol.DESCRIPTOR_LEN + payload_len
    request = protocol.build_request(command)

    t_sent_ns = time.monotonic_ns()
    transport.write(request)
    deadline_ns = t_sent_ns + int(timeout_s * 1e9)

    chunks: list[tuple[int, bytes]] = []
    received = b""
    t_complete_ns = None
    while True:
        remaining = (deadline_ns - time.monotonic_ns()) / 1e9
        if remaining <= 0:
            break
        t_ns, data = transport.read_chunk(4096, min(remaining, 0.05))
        if data:
            chunks.append((t_ns, data))
            received += data
            if len(received) >= want:
                t_complete_ns = t_ns
                # Give a trailing extra byte 5 ms to show itself.
                t_ns, extra = transport.read_chunk(4096, 0.005)
                if extra:
                    chunks.append((t_ns, extra))
                    received += extra
                break

    if not received:
        return t_sent_ns, chunks, "none", None, None

    outcome = "garbled"
    payload = None
    try:
        descriptor = protocol.parse_descriptor(received[: protocol.DESCRIPTOR_LEN])
        if descriptor.length == payload_len and not descriptor.is_stream:
            if len(received) < want:
                outcome = "partial"
            elif len(received) == want:
                outcome = "ok"
                payload = received[protocol.DESCRIPTOR_LEN:want]
    except protocol.ProtocolError:
        pass

    latency_ms = None
    if outcome == "ok" and t_complete_ns is not None:
        latency_ms = (t_complete_ns - t_sent_ns) / 1e6
    return t_sent_ns, chunks, outcome, latency_ms, payload


def main() -> int:
    parser = cli.base_parser(__doc__)
    parser.add_argument("--phases", default="stopped,running,stopped",
                        help="comma-separated motor states, run in order")
    parser.add_argument("--count", type=int, default=50,
                        help="requests per phase and gap")
    parser.add_argument("--gap-ms", default="50",
                        help="idle ms between exchanges; comma-separated "
                             "values run each phase once per gap")
    parser.add_argument("--timeout-ms", type=float, default=500.0)
    parser.add_argument("--command", choices=sorted(PAYLOAD_LEN), default="info")
    parser.add_argument("--settle", type=float, default=0.0,
                        help="seconds after a motor change before the first "
                             "request (0 keeps the spin-up in the data)")
    parser.add_argument("--pause", type=float, default=1.0,
                        help="seconds between phases, motor as it was")
    parser.add_argument("--tag", default="requests")
    parser.add_argument("--note", default="",
                        help="the conditions: supply, what else is plugged in")
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args()

    phases = [p.strip() for p in args.phases.split(",") if p.strip()]
    for phase in phases:
        if phase not in ("stopped", "running"):
            sys.exit(f"ERROR: unknown phase '{phase}': use stopped or running")
    gaps_ms = [float(g) for g in args.gap_ms.split(",")]
    command, payload_len = PAYLOAD_LEN[args.command]
    timeout_s = args.timeout_ms / 1000.0

    cli.print_header("Probe 7 -- Requests With the Motor Running")
    print()
    inner = cli.open_transport(args)
    if args.replay:
        sys.exit("ERROR: this probe needs the device; --replay cannot answer requests")

    base = cli.capture_basename(args.tag)
    out_dir = cli.capture_dir(args.out_dir)
    raw_path = f"{out_dir}/{base}.rpraw"
    jsonl_path = f"{out_dir}/{base}.jsonl"
    t0_ns = time.monotonic_ns()
    header = {
        "type": "header",
        "probe": "lidar_07_requests",
        "port": inner.port,
        "command": args.command,
        "phases": phases,
        "gaps_ms": gaps_ms,
        "count": args.count,
        "timeout_ms": args.timeout_ms,
        "settle_s": args.settle,
        "t0_mono_ns": t0_ns,
        "host": platform_info.snapshot(),
        "notes": args.note,
    }

    def rel(t_ns: int) -> float:
        return round((t_ns - t0_ns) / 1e9, 6)

    raw_writer = recording.RawWriter(raw_path, header)
    transport = TappedTransport(inner, raw_writer)
    jsonl = recording.JsonlWriter(jsonl_path)
    jsonl.write(header)
    poller = ThrottlePoller(t0_ns)
    poller.start()

    # A dropped ssh session sends SIGHUP: leave through the finally and the
    # transport's close, which stops the motor, rather than dying spinning.
    def leave(signum: int, frame: object) -> None:
        raise SystemExit(128 + signum)

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, leave)

    print(f"  command         {args.command} (0x{int(command):02x}), "
          f"timeout {args.timeout_ms:.0f} ms")
    print(f"  writing         {raw_path}")
    print(f"                  {jsonl_path}")

    summaries = []
    try:
        with transport:
            for phase in phases:
                for gap_ms in gaps_ms:
                    running = phase == "running"
                    changed = running != transport.motor_running
                    if changed:
                        transport.set_motor(running)
                        t_motor_ns = time.monotonic_ns()
                    elif not summaries:
                        t_motor_ns = t0_ns
                    if changed or not summaries:
                        jsonl.write({"type": "motor", "t": rel(t_motor_ns),
                                     "running": running})
                    if args.settle > 0:
                        time.sleep(args.settle)

                    print(render.section(f"Motor {phase}, gap {gap_ms:g} ms"))
                    t_phase_start = rel(time.monotonic_ns())
                    outcomes: list[str] = []
                    latencies: list[float] = []
                    payloads: set[bytes] = set()
                    stray_total = 0

                    for index in range(args.count):
                        stray = drain(transport)
                        stray_bytes = sum(len(d) for _, d in stray)
                        stray_total += stray_bytes
                        t_sent_ns, chunks, outcome, latency_ms, payload = exchange(
                            transport, command, payload_len, timeout_s)
                        outcomes.append(outcome)
                        if latency_ms is not None:
                            latencies.append(latency_ms)
                        if payload is not None:
                            payloads.add(payload)

                        jsonl.write({
                            "type": "request",
                            "phase": phase,
                            "gap_ms": gap_ms,
                            "index": index,
                            "t_sent": rel(t_sent_ns),
                            "since_motor_s": round((t_sent_ns - t_motor_ns) / 1e9, 4),
                            "outcome": outcome,
                            "latency_ms": None if latency_ms is None
                            else round(latency_ms, 3),
                            "stray_before": [(rel(t), d.hex()) for t, d in stray],
                            "rx": [(round((t - t_sent_ns) / 1e6, 3), d.hex())
                                   for t, d in chunks],
                        })
                        mark = {"ok": ".", "none": "N", "garbled": "G",
                                "partial": "P"}[outcome]
                        if stray_bytes:
                            mark = "s" + mark
                        print(mark, end="", flush=True)
                        time.sleep(gap_ms / 1000.0)
                    print()

                    t_phase_end = rel(time.monotonic_ns())
                    samples = poller.between(t_phase_start, t_phase_end)
                    uv = sum(1 for _, v in samples if under_voltage_now(v))
                    counts = {k: outcomes.count(k)
                              for k in ("ok", "none", "garbled", "partial")}
                    summary = {
                        "type": "summary",
                        "phase": phase,
                        "gap_ms": gap_ms,
                        "counts": counts,
                        "misses_at": [i for i, o in enumerate(outcomes) if o != "ok"],
                        "latency_ms": None if not latencies else {
                            "min": round(min(latencies), 3),
                            "median": round(statistics.median(latencies), 3),
                            "max": round(max(latencies), 3),
                        },
                        "distinct_payloads": len(payloads),
                        "stray_bytes": stray_total,
                        "throttled_samples": len(samples),
                        "under_voltage_samples": uv,
                        "throttled_values": sorted({v for _, v in samples if v}),
                    }
                    jsonl.write(summary)
                    summaries.append(summary)

                    print(f"  ok {counts['ok']}  none {counts['none']}  "
                          f"garbled {counts['garbled']}  partial {counts['partial']}"
                          f"   (of {args.count})")
                    if latencies:
                        lat = summary["latency_ms"]
                        print(f"  latency ms      min {lat['min']}  "
                              f"median {lat['median']}  max {lat['max']}")
                    print(f"  misses at       {summary['misses_at'] or '-'}")
                    print(f"  distinct replies {len(payloads)}   "
                          f"stray bytes {stray_total}")
                    print(f"  under-voltage   {uv} of {len(samples)} samples   "
                          f"{', '.join(summary['throttled_values'])}")
                    time.sleep(args.pause)
    finally:
        poller.stop()
        jsonl.close()

    print(render.section("All phases"))
    for s in summaries:
        c = s["counts"]
        print(f"  {s['phase']:8s} gap {s['gap_ms']:>6g} ms   ok {c['ok']:3d}/"
              f"{args.count}   none {c['none']}  garbled {c['garbled']}  "
              f"partial {c['partial']}   uv {s['under_voltage_samples']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

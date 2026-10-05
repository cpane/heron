#!/usr/bin/env python3
"""Probe 4 -- sustained capture with statistics. The workhorse.

Writes two sibling files per run:

    <tag>_<UTC>.rpraw   byte-exact wire log, every chunk in both directions
    <tag>_<UTC>.jsonl   decoded records, one JSON object per line

The raw file is ground truth: it lets a parser be developed and regression
tested offline with no hardware, and it is the reference the C++ library is
checked against. The JSONL is what a human and the host-side plots read.

Nothing is filtered. Invalid returns, resyncs, timeouts and throttling events
are all recorded, because at roughly 68% invalid on this unit, dropping the
invalid samples would discard the primary finding.

What this probe teaches
  * The real sustained sample rate and rotation rate, versus what the device
    advertises.
  * How much the rotation period jitters.
  * What fraction of returns are invalid, and how quality is distributed.
  * Angular coverage and the real worst-case gap.
  * Whether under-voltage or throttling correlates with any of it.

Usage
  deploy_pi.sh cpane@<pi> probes/lidar_04_capture.py --seconds 60 --tag desk-open
  deploy_pi.sh cpane@<pi> probes/lidar_04_capture.py --seconds 30 --tag wall-close \
      --note "sensor 0.5 m from a wall"
"""

from __future__ import annotations

import sys
import time

from heron.lidar import cli, platform_info, recording, render
from heron.lidar.driver import Lidar, LidarError, MotorGuard
from heron.lidar.scan import RevolutionAssembler
from heron.lidar.stats import ScanStats
from heron.lidar.transport import TappedTransport


def main() -> int:
    parser = cli.base_parser(__doc__)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--tag", default="capture",
                        help="name prefix for the capture files")
    parser.add_argument("--note", default="",
                        help="free-text description of the scene; recorded "
                             "in the header and worth the ten seconds")
    parser.add_argument("--out-dir", default=None)
    parser.add_argument("--gzip", action="store_true",
                        help="gzip the .jsonl (about 9x smaller)")
    parser.add_argument("--no-jsonl", action="store_true",
                        help="record only the raw wire log")
    parser.add_argument("--spinup", type=float, default=2.0)
    parser.add_argument("--stats-interval", type=float, default=1.0)
    parser.add_argument(
        "--express",
        action="store_true",
        help="record EXPRESS_SCAN instead of legacy SCAN (about twice the "
             "angular resolution; see probe 5 for the trade-offs)",
    )
    parser.add_argument(
        "--induce-overflow",
        type=float,
        default=0.0,
        metavar="MS",
        help="sleep this many ms between reads to force a kernel tty overrun, "
             "so you can see what silent data loss looks like",
    )
    args = parser.parse_args()

    cli.print_header("Probe 4 -- Sustained Capture")
    print()

    out_dir = cli.capture_dir(args.out_dir)
    base = cli.capture_basename(args.tag)
    raw_path = f"{out_dir}/{base}.rpraw"
    jsonl_path = f"{out_dir}/{base}.jsonl" + (".gz" if args.gzip else "")

    inner = cli.open_transport(args)

    # Identity is read before the tap is installed, so the capture file
    # contains only the scan session itself.
    probe_lidar = Lidar(inner)
    device = health = rate = None
    try:
        probe_lidar.stop()
        _, _, device = probe_lidar.get_info()
        _, _, health = probe_lidar.get_health()
        _, _, rate = probe_lidar.get_samplerate()
    except LidarError as exc:
        print(f"  WARNING: identity query failed: {exc}")

    t0_mono_ns = time.monotonic_ns()
    t0_wall_ns = time.time_ns()
    platform_start = platform_info.snapshot()

    header = {
        "type": recording.REC_HEADER,
        "schema": recording.SCHEMA_VERSION,
        "capture_id": base,
        "t": 0.0,
        "t0_mono_ns": t0_mono_ns,
        "t0_wall_ns": t0_wall_ns,
        "raw_file": f"{base}.rpraw",
        "probe": "lidar_04_capture.py",
        "argv": sys.argv[1:],
        "scan_mode": "express" if args.express else "legacy",
        "port": getattr(inner, "port", args.replay or "?"),
        "baud": args.baud,
        "device": None if device is None else {
            "model": device.model,
            "firmware": device.firmware,
            "hardware": device.hardware,
            "serial": device.serial_text,
        },
        "health": None if health is None else {
            "status": health.status,
            "status_name": health.state_name,
            "error_code": health.error_code,
        },
        "samplerate": None if rate is None else {
            "t_standard_us": rate.t_standard_us,
            "t_express_us": rate.t_express_us,
        },
        "host": platform_start,
        "notes": args.note,
    }

    def elapsed(t_mono_ns: int) -> float:
        return round((t_mono_ns - t0_mono_ns) / 1e9, 6)

    raw_writer = recording.RawWriter(raw_path, header)
    transport = TappedTransport(inner, raw_writer)
    jsonl = None if args.no_jsonl else recording.JsonlWriter(jsonl_path, args.gzip)
    if jsonl:
        jsonl.write(header)

    stats = ScanStats()
    assembler = RevolutionAssembler()
    events: list[str] = []

    def on_command(name: str, frame: bytes, offset: int) -> None:
        if jsonl:
            jsonl.write({
                "type": recording.REC_COMMAND, "t": elapsed(time.monotonic_ns()),
                "name": name, "tx": frame.hex(), "raw_off": offset,
            })

    def on_descriptor(descriptor, offset: int) -> None:
        if jsonl:
            jsonl.write({
                "type": recording.REC_DESCRIPTOR,
                "t": elapsed(time.monotonic_ns()),
                "bytes": descriptor.raw.hex(), "length": descriptor.length,
                "mode": descriptor.mode.value, "mode_name": descriptor.mode.name,
                "data_type": descriptor.data_type, "raw_off": offset,
            })

    def on_resync(event) -> None:
        events.append(f"resync: {event.bytes_discarded} byte(s) discarded")
        if jsonl:
            jsonl.write({
                "type": recording.REC_EVENT, "t": elapsed(event.t_mono_ns),
                "event": "resync", "bytes_discarded": event.bytes_discarded,
                "raw_off": event.raw_offset, "detail": event.detail,
            })

    lidar = Lidar(transport, on_command=on_command,
                  on_descriptor=on_descriptor, on_resync=on_resync)

    print(render.section("Recording"))
    print(f"  raw             {raw_path}")
    print(f"  jsonl           {jsonl_path if jsonl else '(disabled)'}")
    print(f"  duration        {args.seconds:.0f} s")
    print(f"  mode            {'express' if args.express else 'legacy'}")
    if args.note:
        print(f"  note            {args.note}")
    if args.induce_overflow:
        print(f"  overflow        forcing a {args.induce_overflow:.1f} ms "
              "stall between reads")
    print()

    try:
        with transport, lidar, MotorGuard(lidar):
            if not args.replay:
                lidar.spin_up(args.spinup)
                inner.flush_input()

            if args.express:
                lidar.start_express_scan()
                stream = lidar.express_samples(duration=args.seconds)
            else:
                lidar.start_scan()
                stream = lidar.samples(duration=args.seconds)

            last_report = time.monotonic()
            last_chunk_ns = 0
            last_samples = 0
            last_throttled = platform_start.get("throttled")

            print("    elapsed   samples/s   rev Hz   invalid   in_waiting")

            for scan in stream:
                sample = scan.sample
                stats.add_sample(sample, scan.t_mono_ns)

                if last_chunk_ns and scan.t_mono_ns != last_chunk_ns:
                    stats.add_chunk_gap((scan.t_mono_ns - last_chunk_ns) / 1e9)
                last_chunk_ns = scan.t_mono_ns

                if jsonl:
                    jsonl.write({
                        "type": recording.REC_SAMPLE,
                        "t": elapsed(scan.t_mono_ns),
                        "seq": stats.samples, "rev": assembler.index,
                        "angle_deg": round(sample.angle_deg, 3),
                        "dist_mm": round(sample.dist_mm, 2),
                        "quality": sample.quality, "start": sample.start,
                        "valid": sample.valid, "raw_off": scan.raw_offset,
                    })

                revolution = assembler.push(scan)
                if revolution is not None:
                    stats.add_revolution(revolution)
                    stats.wraps = assembler.wraps
                    stats.disagreements = assembler.disagreements
                    if jsonl and revolution.index >= 0:
                        jsonl.write({
                            "type": recording.REC_REVOLUTION,
                            "t": elapsed(revolution.t_end_ns),
                            "rev": revolution.index, "n": revolution.count,
                            "n_valid": revolution.valid_count,
                            "period_s": round(revolution.duration_s, 6),
                            "rate_hz": round(revolution.rate_hz, 3),
                            "max_gap_deg": round(revolution.max_gap_deg(), 3),
                            "out_of_order": revolution.out_of_order_count(),
                            "start_flag_ok": revolution.start_flag_ok,
                            "wrap_agrees": revolution.wrap_agrees,
                        })

                if args.induce_overflow:
                    time.sleep(args.induce_overflow / 1000.0)

                now = time.monotonic()
                if now - last_report >= args.stats_interval:
                    window = stats.samples - last_samples
                    interval = now - last_report
                    last_samples, last_report = stats.samples, now
                    in_waiting = getattr(inner, "in_waiting", 0)
                    print(f"    {stats.elapsed_s:7.1f}s   {window / interval:9.0f}"
                          f"   {stats.rev_rate_hz:6.2f}"
                          f"   {100.0 * stats.invalid_fraction:6.1f}%"
                          f"   {in_waiting:10d}")

                    current = platform_info.throttled()
                    if current != last_throttled:
                        events.append(f"throttled changed: {last_throttled} "
                                      f"-> {current}")
                        if jsonl:
                            jsonl.write({
                                "type": recording.REC_EVENT,
                                "t": elapsed(time.monotonic_ns()),
                                "event": "platform", "throttled": current,
                                "cpu_temp_c": platform_info.cpu_temp_c(),
                            })
                        last_throttled = current

            partial = assembler.flush()
            if partial is not None:
                stats.add_revolution(partial)
            lidar.stop()
    except KeyboardInterrupt:
        print("\n  Interrupted; the capture up to this point is still valid.")

    platform_end = platform_info.snapshot()

    # Compare against the rate the device claims FOR THE MODE THAT RAN.
    # Checking express throughput against the standard figure reports a
    # nonsensical -98% and hides whether the mode actually met its spec.
    if rate is None:
        claimed = None
    else:
        claimed = rate.t_express_us if args.express else rate.t_standard_us

    footer = {
        "type": recording.REC_FOOTER,
        "t": elapsed(time.monotonic_ns()),
        **stats.as_dict(),
        "resyncs": lidar.counters.resyncs,
        "bytes_read": lidar.counters.bytes_read,
        "bytes_discarded": lidar.counters.bytes_discarded,
        "chunks": lidar.counters.chunks,
        "timeouts": lidar.counters.timeouts,
        "max_in_waiting": lidar.counters.max_in_waiting,
        "throttled_start": platform_start.get("throttled"),
        "throttled_end": platform_end.get("throttled"),
        "cpu_temp_start_c": platform_start.get("cpu_temp_c"),
        "cpu_temp_end_c": platform_end.get("cpu_temp_c"),
    }
    if jsonl:
        jsonl.write(footer)
        jsonl.close()

    print(render.section("Results"))
    for line in stats.report_lines(claimed_us=claimed):
        print(line)

    print(render.section("Transport"))
    counters = lidar.counters
    print(f"  chunks read        {counters.chunks}")
    print(f"  bytes read         {counters.bytes_read}")
    print(f"  resyncs            {counters.resyncs} "
          f"({counters.bytes_discarded} bytes discarded)")
    print(f"  read timeouts      {counters.timeouts}")
    print(f"  max in_waiting     {counters.max_in_waiting} bytes")
    if counters.bytes_read and counters.samples:
        # Legacy spends 5 bytes per sample. Express packs 32 measurements into
        # an 84-byte capsule, so 84/32 = 2.625 bytes/sample is the floor --
        # part of why it doubles the rate over the same 115200 baud link.
        ideal = 84.0 / 32.0 if args.express else 5.0
        print(f"  bytes/sample       "
              f"{counters.bytes_read / counters.samples:.3f} "
              f"({ideal:.3f} = every byte accounted for)")

    print(render.section("Power"))
    print(f"  throttled start    {platform_start.get('throttled')}")
    print(f"  throttled end      {platform_end.get('throttled')}")
    for flag in platform_end.get("throttled_flags") or []:
        print(f"                       - {flag}")

    if events:
        print(render.section("Events"))
        for event in events[:20]:
            print(f"  {event}")

    print(render.section("Files"))
    print(f"  {raw_path}")
    if jsonl:
        print(f"  {jsonl_path}")
    print()
    print("  Fetch them to the host with:")
    print("    scripts/fetch_captures.sh cpane@<pi>")

    return 0


if __name__ == "__main__":
    sys.exit(main())

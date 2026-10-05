#!/usr/bin/env python3
"""Offline analysis of a .jsonl capture. Host-side; needs numpy + matplotlib.

Reads a capture through the same heron.lidar.recording reader the Pi used
to write it, and produces the figures that answer the questions this whole
exercise exists to answer.

Figures
  polar.png     one revolution over a faint overlay of all of them, with
                invalid returns drawn as rim ticks so 68% is seen, not stated
  coverage.png  angular-delta histogram and per-degree sample counts
  rate.png      samples/s, rotation rate and revolution period over time
  quality.png   quality vs distance, and the quality/validity contingency
  range.png     distance distribution and invalid fraction vs range
  summary.txt   the footer statistics as a readable table

Usage
  .venv/bin/python python/tools/lidar_report.py captures/desk-open_*.jsonl
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import matplotlib  # noqa: E402

matplotlib.use("Agg")  # No display on either machine; write files.

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from heron.lidar import recording  # noqa: E402

INVALID_COLOUR = "#c44"
VALID_COLOUR = "#246"
ACCENT = "#e08214"


def load(path: str):
    """One pass over the file, into arrays."""
    reader = recording.JsonlReader(path)
    header = None
    footer = None
    events = []
    revs = []
    angle, dist, quality, valid, rev_id, t = [], [], [], [], [], []

    for record in reader.records():
        kind = record.get("type")
        if kind == recording.REC_SAMPLE:
            angle.append(record["angle_deg"])
            dist.append(record["dist_mm"])
            quality.append(record["quality"])
            valid.append(record["valid"])
            rev_id.append(record["rev"])
            t.append(record["t"])
        elif kind == recording.REC_HEADER:
            header = record
        elif kind == recording.REC_FOOTER:
            footer = record
        elif kind == recording.REC_EVENT:
            events.append(record)
        elif kind == recording.REC_REVOLUTION:
            revs.append(record)

    return {
        "header": header or {},
        "footer": footer,
        "events": events,
        "revs": revs,
        "truncated": reader.truncated,
        "angle": np.array(angle),
        "dist": np.array(dist),
        "quality": np.array(quality, dtype=int),
        "valid": np.array(valid, dtype=bool),
        "rev": np.array(rev_id, dtype=int),
        "t": np.array(t),
    }


def figure_polar(data, out_dir: str, rev_index: int | None) -> None:
    angle, dist, valid, rev = data["angle"], data["dist"], data["valid"], data["rev"]
    complete = rev[rev >= 0]
    if complete.size == 0:
        return
    chosen = rev_index if rev_index is not None else int(np.median(complete))

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw={"projection": "polar"})
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)  # Clockwise; verified with probe 3.

    good = valid & (rev >= 0)
    ax.scatter(np.radians(angle[good]), dist[good] / 1000.0,
               s=1, c="#bbb", alpha=0.35, label="all revolutions")

    one = good & (rev == chosen)
    ax.scatter(np.radians(angle[one]), dist[one] / 1000.0,
               s=14, c=VALID_COLOUR, label=f"revolution {chosen}")

    # Invalid returns carry a real bearing but no range, so they are drawn on
    # the rim. At ~68% of all samples this is most of the data.
    rim = (~valid) & (rev == chosen)
    if rim.any():
        edge = float(np.nanmax(dist[good]) / 1000.0) if good.any() else 1.0
        ax.scatter(np.radians(angle[rim]), np.full(rim.sum(), edge * 1.02),
                   s=8, marker="|", c=INVALID_COLOUR,
                   label=f"no return ({rim.sum()})")

    ax.set_title(f"Revolution {chosen} -- {data['header'].get('notes') or ''}\n"
                 f"0 deg up, angle increasing clockwise", pad=18)
    ax.legend(loc="lower right", bbox_to_anchor=(1.15, -0.05), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "polar.png"), dpi=130)
    plt.close(fig)


def figure_coverage(data, out_dir: str) -> None:
    angle, rev = data["angle"], data["rev"]
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 7))

    # Angular deltas within each revolution, computed in ASCENDING angle order
    # -- the arrival order is not angular order on this sensor.
    deltas = []
    for index in np.unique(rev[rev >= 0]):
        ordered = np.sort(angle[rev == index])
        if ordered.size > 1:
            deltas.append(np.diff(ordered))
    if deltas:
        alld = np.concatenate(deltas)
        top.hist(alld, bins=np.arange(0, 12, 0.1), color=VALID_COLOUR)
        top.set_yscale("log")
        top.set_xlabel("angular gap between adjacent samples (deg, angle-sorted)")
        top.set_ylabel("count (log)")
        top.set_title(f"Angular spacing -- median {np.median(alld):.3f} deg, "
                      f"p99 {np.percentile(alld, 99):.2f}, max {alld.max():.2f}")

    counts, edges = np.histogram(angle, bins=np.arange(0, 361, 1))
    bottom.bar(edges[:-1], counts, width=1.0, color=ACCENT)
    bottom.set_xlabel("bearing (deg)")
    bottom.set_ylabel("samples")
    empty = int((counts == 0).sum())
    bottom.set_title(f"Coverage per 1-degree bearing bin -- "
                     f"{empty} bins never sampled")
    bottom.set_xlim(0, 360)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "coverage.png"), dpi=130)
    plt.close(fig)


def figure_rate(data, out_dir: str) -> None:
    t = data["t"]
    if t.size == 0:
        return
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    bins = np.arange(0, t.max() + 1, 1.0)
    counts, _ = np.histogram(t, bins=bins)
    axes[0].plot(bins[:-1], counts, color=VALID_COLOUR)
    claimed = (data["header"].get("samplerate") or {}).get("t_standard_us")
    if claimed:
        axes[0].axhline(1e6 / claimed, color=INVALID_COLOUR, ls="--",
                        label=f"device claims {1e6 / claimed:.0f}/s")
        axes[0].legend(fontsize=8)
    axes[0].set_ylabel("samples/s")
    axes[0].set_title("Throughput, rotation rate and period over the capture")

    revs = data["revs"]
    if revs:
        rt = np.array([r["t"] for r in revs])
        rate = np.array([r["rate_hz"] for r in revs])
        period = np.array([r["period_s"] for r in revs]) * 1000.0
        axes[1].plot(rt, rate, color=ACCENT)
        axes[1].set_ylabel("rotation (Hz)")
        axes[2].plot(rt, period, color=VALID_COLOUR)
        axes[2].set_ylabel("rev period (ms)")
        axes[2].set_title(f"period stdev {period.std():.2f} ms "
                          f"({100 * period.std() / period.mean():.1f}% of mean)")

    # Events are where the story usually is: resyncs and throttle changes.
    for event in data["events"]:
        for ax in axes:
            ax.axvline(event["t"], color=INVALID_COLOUR, alpha=0.4, lw=0.8)

    axes[2].set_xlabel("time (s, monotonic from capture start)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "rate.png"), dpi=130)
    plt.close(fig)


def figure_quality(data, out_dir: str) -> None:
    quality, dist, valid = data["quality"], data["dist"], data["valid"]
    fig, (left, right) = plt.subplots(1, 2, figsize=(12, 5))

    good = valid & (dist > 0)
    if good.any():
        hexes = left.hexbin(dist[good] / 1000.0, quality[good],
                            gridsize=40, bins="log", cmap="viridis")
        fig.colorbar(hexes, ax=left, label="count (log)")
    left.set_xlabel("distance (m)")
    left.set_ylabel("quality (raw 6-bit field)")
    left.set_title("Quality vs distance, valid returns only")

    levels = np.arange(0, 64)
    valid_counts = np.array([(quality[valid] == q).sum() for q in levels])
    invalid_counts = np.array([(quality[~valid] == q).sum() for q in levels])
    right.bar(levels, valid_counts, color=VALID_COLOUR, label="valid")
    right.bar(levels, invalid_counts, bottom=valid_counts,
              color=INVALID_COLOUR, label="no return")
    right.set_yscale("log")
    right.set_xlabel("quality")
    right.set_ylabel("count (log)")

    q0_invalid = int(((quality == 0) & (~valid)).sum())
    q0_valid = int(((quality == 0) & valid).sum())
    nonzero_invalid = int(((quality != 0) & (~valid)).sum())
    right.set_title(
        f"quality==0: {q0_invalid} invalid / {q0_valid} valid\n"
        f"quality!=0 but no return: {nonzero_invalid}"
    )
    right.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "quality.png"), dpi=130)
    plt.close(fig)


def figure_range(data, out_dir: str) -> None:
    dist, valid = data["dist"], data["valid"]
    good = valid & (dist > 0)
    if not good.any():
        return
    fig, (left, right) = plt.subplots(1, 2, figsize=(12, 5))

    left.hist(dist[good] / 1000.0, bins=120, color=VALID_COLOUR)
    left.set_yscale("log")
    left.set_xlabel("distance (m)")
    left.set_ylabel("count (log)")
    left.set_title(f"Valid returns: {dist[good].min():.0f} .. "
                   f"{dist[good].max():.0f} mm")

    # Invalid fraction per bearing: which directions the sensor cannot see.
    angle = data["angle"]
    bins = np.arange(0, 361, 5)
    frac = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        sel = (angle >= lo) & (angle < hi)
        frac.append(1.0 - valid[sel].mean() if sel.any() else np.nan)
    right.bar(bins[:-1], frac, width=5, color=INVALID_COLOUR)
    right.set_xlabel("bearing (deg)")
    right.set_ylabel("fraction with no return")
    right.set_ylim(0, 1)
    right.set_xlim(0, 360)
    right.set_title(f"No-return fraction by bearing "
                    f"(overall {1.0 - valid.mean():.1%})")

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "range.png"), dpi=130)
    plt.close(fig)


def write_summary(data, out_dir: str, path: str) -> str:
    header, footer = data["header"], data["footer"]
    lines = [
        "Capture summary",
        "=" * 60,
        f"file            {path}",
        f"capture_id      {header.get('capture_id')}",
        f"notes           {header.get('notes') or '(none)'}",
        f"device          model {(header.get('device') or {}).get('model')}, "
        f"fw {(header.get('device') or {}).get('firmware')}",
        f"host            {(header.get('host') or {}).get('model')}",
        f"throttled       {(header.get('host') or {}).get('throttled')}",
        "",
    ]

    if data["truncated"]:
        lines.append("WARNING: the .jsonl ends mid-line; the run was interrupted.")
        lines.append("")

    if footer is None:
        lines.append("No footer record: the capture did not finish cleanly.")
    else:
        width = max(len(k) for k in footer)
        for key, value in footer.items():
            if key == "type":
                continue
            lines.append(f"{key:<{width}}  {value}")

    if data["events"]:
        lines += ["", f"Events ({len(data['events'])}):"]
        for event in data["events"][:20]:
            lines.append(f"  t={event['t']:9.3f}  {event.get('event')}  "
                         + str({k: v for k, v in event.items()
                                if k not in ("type", "t", "event")}))

    text = "\n".join(lines)
    with open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    return text


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("jsonl")
    parser.add_argument("--out-dir", default=None,
                        help="default: reports/<capture_id>/")
    parser.add_argument("--rev", type=int, default=None,
                        help="which revolution to draw in polar.png")
    args = parser.parse_args()

    data = load(args.jsonl)
    if data["angle"].size == 0:
        print("ERROR: no sample records in that file.")
        return 1

    capture_id = data["header"].get("capture_id") or os.path.basename(args.jsonl)
    out_dir = args.out_dir or os.path.join("reports", capture_id)
    os.makedirs(out_dir, exist_ok=True)

    figure_polar(data, out_dir, args.rev)
    figure_coverage(data, out_dir)
    figure_rate(data, out_dir)
    figure_quality(data, out_dir)
    figure_range(data, out_dir)
    print(write_summary(data, out_dir, args.jsonl))

    print()
    print(f"Figures written to {out_dir}/")
    for name in sorted(os.listdir(out_dir)):
        print(f"  {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

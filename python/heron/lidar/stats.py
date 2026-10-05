"""Streaming statistics over a scan.

All accumulators are O(1) in memory so a long soak does not grow without
bound. Percentiles use fixed-width histograms rather than keeping every
sample, which is accurate enough for the questions being asked and keeps the
Pi's 905 MiB out of the picture.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field
from typing import Any


@dataclass
class Running:
    """Count, min, max, mean and standard deviation via Welford's method."""

    n: int = 0
    total: float = 0.0
    minimum: float = math.inf
    maximum: float = -math.inf
    _mean: float = 0.0
    _m2: float = 0.0

    def add(self, value: float) -> None:
        self.n += 1
        self.total += value
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)
        delta = value - self._mean
        self._mean += delta / self.n
        self._m2 += delta * (value - self._mean)

    @property
    def mean(self) -> float:
        return self._mean if self.n else 0.0

    @property
    def stdev(self) -> float:
        return math.sqrt(self._m2 / (self.n - 1)) if self.n > 1 else 0.0

    def as_dict(self) -> dict[str, Any]:
        if not self.n:
            return {"n": 0}
        return {
            "n": self.n,
            "min": round(self.minimum, 6),
            "max": round(self.maximum, 6),
            "mean": round(self.mean, 6),
            "stdev": round(self.stdev, 6),
        }


class Histogram:
    """Fixed-width bins with overflow, for approximate percentiles."""

    def __init__(self, width: float, bins: int) -> None:
        self.width = width
        self.bins = [0] * bins
        self.overflow = 0
        self.n = 0

    def add(self, value: float) -> None:
        self.n += 1
        index = int(value / self.width)
        if 0 <= index < len(self.bins):
            self.bins[index] += 1
        else:
            self.overflow += 1

    def percentile(self, fraction: float) -> float:
        """Upper edge of the bin containing the requested percentile."""
        if not self.n:
            return 0.0
        target = fraction * self.n
        cumulative = 0
        for index, count in enumerate(self.bins):
            cumulative += count
            if cumulative >= target:
                return (index + 1) * self.width
        return len(self.bins) * self.width

    def as_dict(self) -> dict[str, Any]:
        return {
            "p50": round(self.percentile(0.50), 6),
            "p95": round(self.percentile(0.95), 6),
            "p99": round(self.percentile(0.99), 6),
            "overflow": self.overflow,
        }


@dataclass
class ScanStats:
    """Everything probe 4 measures, accumulated as the scan runs."""

    samples: int = 0
    invalid: int = 0
    revolutions: int = 0

    t_first_ns: int = 0
    t_last_ns: int = 0

    quality: Counter[int] = field(default_factory=Counter)
    quality_when_valid: Counter[int] = field(default_factory=Counter)
    distance: Running = field(default_factory=Running)
    rev_period: Running = field(default_factory=Running)
    samples_per_rev: Running = field(default_factory=Running)
    max_gap: Running = field(default_factory=Running)

    #: Angular gaps, 0.1 degree bins out to 36 degrees.
    angle_delta: Histogram = field(
        default_factory=lambda: Histogram(width=0.1, bins=360)
    )
    #: Chunk inter-arrival, 0.5 ms bins out to 250 ms.
    chunk_gap_ms: Histogram = field(
        default_factory=lambda: Histogram(width=0.5, bins=500)
    )
    #: Samples seen per 1-degree bearing bin, for coverage analysis.
    bearing_bins: list[int] = field(default_factory=lambda: [0] * 360)

    start_flags: int = 0
    wraps: int = 0
    disagreements: int = 0
    non_monotonic_revs: int = 0
    out_of_order: int = 0
    partial_revs: int = 0
    start_angle: Running = field(default_factory=Running)

    def add_sample(self, sample: Any, t_mono_ns: int) -> None:
        if not self.samples:
            self.t_first_ns = t_mono_ns
        self.t_last_ns = t_mono_ns
        self.samples += 1

        self.quality[sample.quality] += 1
        if sample.valid:
            self.quality_when_valid[sample.quality] += 1
            self.distance.add(sample.dist_mm)
        else:
            self.invalid += 1

        bearing = int(sample.angle_deg) % 360
        self.bearing_bins[bearing] += 1

        if sample.start:
            self.start_flags += 1
            self.start_angle.add(sample.angle_deg)

    def add_revolution(self, revolution: Any) -> None:
        if revolution.index < 0 or revolution.partial:
            # A truncated fragment is not a measurement of a rotation. Folding
            # one in makes min samples-per-rev and min period nonsense.
            self.partial_revs += 1
            return

        self.revolutions += 1
        self.samples_per_rev.add(revolution.count)
        if revolution.duration_s > 0:
            self.rev_period.add(revolution.duration_s)

        # Angular coverage is measured on angle-sorted order. The raw arrival
        # order is not ascending on this sensor, so gaps computed from it
        # describe the transmission sequence, not where the beams pointed.
        self.max_gap.add(revolution.max_gap_deg(ascending=True))
        for delta in revolution.angle_deltas_deg(ascending=True):
            if delta >= 0:
                self.angle_delta.add(delta)

        self.out_of_order += revolution.out_of_order_count()
        if not revolution.is_monotonic():
            self.non_monotonic_revs += 1
        if not revolution.wrap_agrees:
            self.disagreements += 1

    def add_chunk_gap(self, gap_s: float) -> None:
        self.chunk_gap_ms.add(gap_s * 1000.0)

    @property
    def elapsed_s(self) -> float:
        return (self.t_last_ns - self.t_first_ns) / 1e9

    @property
    def sample_rate_hz(self) -> float:
        return self.samples / self.elapsed_s if self.elapsed_s > 0 else 0.0

    @property
    def rev_rate_hz(self) -> float:
        return self.revolutions / self.elapsed_s if self.elapsed_s > 0 else 0.0

    @property
    def invalid_fraction(self) -> float:
        return self.invalid / self.samples if self.samples else 0.0

    def coverage_gaps(self) -> list[tuple[int, int]]:
        """Bearing bins with no samples at all, as (start_deg, end_deg) runs."""
        gaps: list[tuple[int, int]] = []
        start: int | None = None
        for degree, count in enumerate(self.bearing_bins):
            if count == 0 and start is None:
                start = degree
            elif count and start is not None:
                gaps.append((start, degree - 1))
                start = None
        if start is not None:
            gaps.append((start, 359))
        return gaps

    def as_dict(self) -> dict[str, Any]:
        return {
            "samples": self.samples,
            "revolutions": self.revolutions,
            "elapsed_s": round(self.elapsed_s, 6),
            "sample_rate_hz": round(self.sample_rate_hz, 1),
            "rev_rate_hz": round(self.rev_rate_hz, 3),
            "invalid": self.invalid,
            "invalid_fraction": round(self.invalid_fraction, 4),
            "rev_period_s": self.rev_period.as_dict(),
            "samples_per_rev": self.samples_per_rev.as_dict(),
            "max_gap_deg": self.max_gap.as_dict(),
            "distance_mm": self.distance.as_dict(),
            "angle_delta_deg": self.angle_delta.as_dict(),
            "chunk_gap_ms": self.chunk_gap_ms.as_dict(),
            "quality_hist": dict(sorted(self.quality.items())),
            "quality_hist_valid_only": dict(sorted(self.quality_when_valid.items())),
            "start_flags": self.start_flags,
            "start_flag_angle_deg": self.start_angle.as_dict(),
            "wraps": self.wraps,
            "segmentation_disagreements": self.disagreements,
            "non_monotonic_revs": self.non_monotonic_revs,
            "out_of_order_samples": self.out_of_order,
            "empty_bearing_bins": sum(1 for c in self.bearing_bins if c == 0),
        }

    def report_lines(self, claimed_us: int | None = None) -> list[str]:
        """The end-of-run summary, formatted for a terminal."""
        lines = [
            f"  samples            {self.samples}",
            f"  elapsed            {self.elapsed_s:.2f} s",
            f"  sample rate        {self.sample_rate_hz:.0f} samples/s",
        ]

        if claimed_us:
            claimed_hz = 1e6 / claimed_us
            shortfall = 100.0 * (1.0 - self.sample_rate_hz / claimed_hz)
            lines.append(
                f"  device claims      {claimed_hz:.0f} samples/s "
                f"({claimed_us} us/sample)  ->  {shortfall:+.1f}% vs measured"
            )

        lines += [
            f"  revolutions        {self.revolutions}",
            f"  rotation rate      {self.rev_rate_hz:.2f} Hz",
        ]

        if self.rev_period.n:
            period = self.rev_period
            lines.append(
                f"  rev period         min {period.minimum:.4f}  "
                f"max {period.maximum:.4f}  mean {period.mean:.4f}  "
                f"stdev {period.stdev:.4f} s"
            )
            if period.mean:
                jitter = 100.0 * (period.maximum - period.minimum) / period.mean
                lines.append(f"  rev period spread  {jitter:.1f}% of mean")

        if self.samples_per_rev.n:
            per_rev = self.samples_per_rev
            lines.append(
                f"  samples/rev        min {per_rev.minimum:.0f}  "
                f"max {per_rev.maximum:.0f}  mean {per_rev.mean:.1f}"
            )

        lines += [
            f"  invalid returns    {self.invalid} "
            f"({100.0 * self.invalid_fraction:.1f}%)",
        ]

        if self.distance.n:
            lines.append(
                f"  distance (valid)   {self.distance.minimum:.1f} .. "
                f"{self.distance.maximum:.1f} mm  "
                f"(mean {self.distance.mean:.1f})"
            )

        lines += [
            f"  angular delta      p50 {self.angle_delta.percentile(0.50):.2f}  "
            f"p95 {self.angle_delta.percentile(0.95):.2f} deg",
            f"  max gap per rev    mean {self.max_gap.mean:.2f}  "
            f"worst {self.max_gap.maximum:.2f} deg",
            f"  chunk arrival gap  p50 {self.chunk_gap_ms.percentile(0.50):.1f}  "
            f"p95 {self.chunk_gap_ms.percentile(0.95):.1f} ms",
        ]

        empty = sum(1 for c in self.bearing_bins if c == 0)
        lines.append(f"  bearings unseen    {empty} of 360 one-degree bins")

        lines += [
            f"  start flags        {self.start_flags}",
            f"  angle wraps        {self.wraps}  "
            "(independent check on the same boundary)",
            f"  non-monotonic revs {self.non_monotonic_revs} of "
            f"{self.revolutions}",
            f"  out-of-order       {self.out_of_order} samples "
            f"({100.0 * self.out_of_order / max(self.samples, 1):.1f}%) "
            "arrived behind the previous angle",
        ]

        if self.start_angle.n:
            angle = self.start_angle
            lines.append(
                f"  S flag fires at    {angle.minimum:.1f} .. "
                f"{angle.maximum:.1f} deg (mean {angle.mean:.1f}, "
                f"stdev {angle.stdev:.1f})"
            )
            lines.append(
                "                     NOT at 0 deg -- the S flag marks a "
                "transmission boundary,"
            )
            lines.append(
                "                     not an angular one, so sort by angle "
                "before using a scan."
            )

        top = self.quality.most_common(8)
        lines.append(
            "  quality histogram  "
            + ", ".join(f"{q}:{n}" for q, n in sorted(top, key=lambda x: -x[1]))
        )

        return lines

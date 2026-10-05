"""Assembling a stream of samples into revolutions.

The sensor emits an unbroken stream of samples; "a scan" is a software
concept. Two independent signals could mark a revolution boundary:

  1. The S flag in bit 0 of each node's first byte.
  2. The angle wrapping from near 360 degrees back towards 0.

They are tracked separately and their disagreements are counted, because
"can I trust the S bit?" is an open question that the production driver has to
answer -- and segmenting on the wrong signal produces revolutions that are
subtly short or long in a way that is very hard to see downstream.

MEASURED ON THIS UNIT, AND IMPORTANT
------------------------------------
The legacy scan stream is **not in angular order**. Roughly 12% of samples
arrive behind the angle of the sample before them, and about 82% of those
out-of-order samples are quality 15 -- that is, real returns. The smooth
1.359-degree progression comes mostly from quality-0 no-return filler.

The S flag likewise does not fire at angle 0: on this unit it lands wherever
the device happens to be when it decides a revolution has elapsed (357.8
degrees in one observed case), so the S-flagged sample often belongs at the
*end* of the revolution once sorted.

The consequence for any consumer is that a revolution must be **sorted by
angle** before it is treated as a 360-degree scan. This is not a quirk of this
program: SLAMTEC's own C++ SDK ships ``ascendScanData()`` for exactly this
reason. :meth:`Revolution.sorted_samples` is the equivalent here, and probe 3
prints the statistics both ways so the difference is visible.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator

from .driver import ScanSample

#: Half a turn. Used to unfold angle differences into (-180, +180].
WRAP_THRESHOLD_DEG = 180.0

#: A revolution boundary is only credited when the angle crosses from the top
#: of the circle to the bottom. A bare "the angle went backwards by more than
#: 180 degrees" test does not work on this sensor: samples arrive out of
#: angular order (see the module docstring), so a single sample landing just
#: the other side of the seam reads as an extra wrap and inflates the count.
WRAP_HIGH_DEG = 300.0
WRAP_LOW_DEG = 60.0

#: After a wrap is credited, further wraps are suppressed until the angle has
#: advanced past this point. Without the debounce, out-of-order samples near
#: the seam cross it repeatedly and each crossing is counted again -- which is
#: how a genuine 138 revolutions gets reported as 249.
WRAP_REARM_DEG = 180.0

#: 360 degrees in the wire's Q6 angle units.
FULL_CIRCLE_Q6 = 360 * 64


@dataclass
class Revolution:
    """One revolution's worth of samples.

    ``index`` is -1 for the leading partial revolution -- the samples that
    arrived before the first start flag was seen. It is kept rather than
    discarded so that startup behaviour is visible.
    """

    index: int
    samples: list[ScanSample] = field(default_factory=list)
    t_start_ns: int = 0
    t_end_ns: int = 0
    start_flag_ok: bool = True
    wrap_agrees: bool = True
    partial: bool = False  # Leading or trailing fragment, not a full rotation.

    @property
    def count(self) -> int:
        return len(self.samples)

    @property
    def valid_count(self) -> int:
        return sum(1 for s in self.samples if s.sample.valid)

    @property
    def duration_s(self) -> float:
        return (self.t_end_ns - self.t_start_ns) / 1e9

    @property
    def rate_hz(self) -> float:
        return 1.0 / self.duration_s if self.duration_s > 0 else 0.0

    @property
    def angles_deg(self) -> list[float]:
        return [s.sample.angle_deg for s in self.samples]

    def sorted_samples(self) -> list[ScanSample]:
        """Samples in ascending angular order.

        The raw stream is not angle-sorted (see the module docstring), so this
        is what any consumer treating a revolution as a 360-degree scan must
        use. Equivalent to ``ascendScanData()`` in SLAMTEC's C++ SDK.
        """
        return sorted(self.samples, key=lambda s: s.sample.angle_q6)

    @staticmethod
    def _unfold(delta: float) -> float:
        """Map a raw angle difference into (-180, +180].

        Unfolding must happen in both directions. A small backward step taken
        across the 0/360 seam shows up as roughly +354, and reporting that as
        a forward gap makes every seam look like a catastrophic data loss.
        """
        if delta < -WRAP_THRESHOLD_DEG:
            return delta + 360.0
        if delta > WRAP_THRESHOLD_DEG:
            return delta - 360.0
        return delta

    def angle_deltas_deg(self, ascending: bool = False) -> list[float]:
        """Gaps between consecutive samples, with the 360 wrap unfolded.

        ``ascending`` sorts by angle first, which is the order that actually
        describes the scan's angular coverage.
        """
        source = self.sorted_samples() if ascending else self.samples
        angles = [s.sample.angle_deg for s in source]
        return [
            self._unfold(current - previous)
            for previous, current in zip(angles, angles[1:])
        ]

    def max_gap_deg(self, ascending: bool = True) -> float:
        """Largest angular gap. Sorted by default: that is the real coverage."""
        deltas = self.angle_deltas_deg(ascending=ascending)
        return max(deltas) if deltas else 0.0

    def sampling_gap_q6(self) -> int:
        """Widest angular gap between samples, in Q6, including the 360 seam.

        This is the drop detector, and it is measured over EVERY sample --
        no-returns included. A no-return still tells you the beam pointed
        there; only a sample that never arrived leaves a hole. Measured over
        valid returns alone the figure means nothing: with 61-84% of samples
        returning nothing, a 30-115 degree run without a return is ordinary
        (measured across all captures, 2026-10-01), and healthy revolutions
        would look like data loss.

        Unlike :meth:`max_gap_deg` this closes the circle, so a hole that
        straddles 0 degrees is not missed. Integer Q6 throughout so the C++
        adapter has an exact reference to match.
        """
        angles = sorted(s.sample.angle_q6 for s in self.samples)
        if len(angles) < 2:
            return FULL_CIRCLE_Q6
        widest = max(b - a for a, b in zip(angles, angles[1:]))
        seam = angles[0] + FULL_CIRCLE_Q6 - angles[-1]
        return max(widest, seam)

    def out_of_order_count(self) -> int:
        """Samples that arrived behind the one before them."""
        return sum(1 for d in self.angle_deltas_deg() if d < 0)

    def angle_span_deg(self) -> float:
        angles = self.angles_deg
        return (angles[-1] - angles[0]) % 360.0 if len(angles) > 1 else 0.0

    def is_monotonic(self) -> bool:
        """True when the raw arrival order is also ascending angular order.

        Expected to be False on this sensor -- see the module docstring. It is
        measured rather than assumed, because it is the single fact that most
        shapes how a consumer has to treat a revolution.
        """
        return all(d >= 0 for d in self.angle_deltas_deg())


class RevolutionAssembler:
    """Turn a sample stream into revolutions.

    Segmentation uses the S flag, which keeps the numbering canonical and
    simple. The angle-wrap detector runs alongside purely as a cross-check;
    when the two disagree, the revolution is flagged rather than renumbered.
    """

    def __init__(self) -> None:
        self.index = -1
        self._current = Revolution(index=-1, partial=True)
        self._last_angle: float | None = None
        self._wrap_armed = True
        self.start_flags = 0
        self.wraps = 0
        self.disagreements = 0
        #: The angle at which each S flag fired. The interesting question is
        #: not "do S and the wrap fire on the same sample" -- they never do,
        #: since they are different samples -- but "where in the circle does
        #: the S flag land?" On this unit it is nowhere near 0 degrees.
        self.start_angles: list[float] = []

    def push(self, scan: ScanSample) -> Revolution | None:
        """Add a sample. Returns a completed revolution, if one just ended."""
        angle = scan.sample.angle_deg

        if not self._wrap_armed and WRAP_REARM_DEG <= angle < WRAP_HIGH_DEG:
            self._wrap_armed = True

        wrapped = (
            self._wrap_armed
            and self._last_angle is not None
            and self._last_angle >= WRAP_HIGH_DEG
            and angle <= WRAP_LOW_DEG
        )
        if wrapped:
            self._wrap_armed = False

        self._last_angle = angle

        if scan.sample.start:
            self.start_flags += 1
            self.start_angles.append(angle)
        if wrapped:
            self.wraps += 1

        # The two signals should fire on the same sample. Count the cases
        # where exactly one of them does.
        disagreed = scan.sample.start != wrapped
        if disagreed and self._current.samples:
            self.disagreements += 1

        if not scan.sample.start:
            if not self._current.samples:
                self._current.t_start_ns = scan.t_mono_ns
            self._current.samples.append(scan)
            self._current.t_end_ns = scan.t_mono_ns
            if disagreed:
                self._current.wrap_agrees = False
            return None

        completed = self._current if self._current.samples else None
        if completed is not None:
            completed.t_end_ns = scan.t_mono_ns
            completed.start_flag_ok = completed.index >= 0
            if not wrapped:
                completed.wrap_agrees = False

        self.index += 1
        self._current = Revolution(index=self.index, t_start_ns=scan.t_mono_ns)
        self._current.samples.append(scan)
        self._current.t_end_ns = scan.t_mono_ns

        return completed

    def flush(self) -> Revolution | None:
        """Return the in-progress revolution, which is by definition partial.

        It is marked ``partial`` so that callers do not average a truncated
        fragment in with complete rotations -- doing so drags the minimum
        samples-per-revolution and rotation period down to meaningless values.
        """
        if not self._current.samples:
            return None
        partial, self._current = self._current, Revolution(index=self.index + 1)
        partial.partial = True
        return partial


def revolutions(
    stream: Iterator[ScanSample],
    skip: int = 0,
    limit: int | None = None,
) -> Iterator[Revolution]:
    """Assemble ``stream`` into revolutions.

    ``skip`` drops the first N complete revolutions, which is how you avoid
    measuring the motor's spin-up transient. The leading partial revolution
    (index -1) is never counted against ``skip`` and is always yielded, so
    startup behaviour stays visible.
    """
    assembler = RevolutionAssembler()
    emitted = 0
    skipped = 0

    for scan in stream:
        revolution = assembler.push(scan)
        if revolution is None:
            continue

        if revolution.index >= 0 and skipped < skip:
            skipped += 1
            continue

        yield revolution
        emitted += 1
        if limit is not None and emitted >= limit:
            return

    partial = assembler.flush()
    if partial is not None and (limit is None or emitted < limit):
        yield partial

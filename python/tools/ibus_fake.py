#!/usr/bin/env python3
"""A fake FlySky iBUS receiver on a pseudo-terminal, for testing ibus_reader
on the host without hardware.

Creates a pty, prints the slave's path (and optionally symlinks it to a
stable name), then writes 32-byte iBUS frames to it every --period-ms. Point
the host-native build at it:

    python3 python/tools/ibus_fake.py --link /tmp/ibus &
    build-host/bin/ibus_reader monitor --device /tmp/ibus

Standard library only. Frames are built here from the protocol description
in docs/remote-ibus-protocol.md, not from any third-party code.

What it can simulate:

  --set CH=VAL          a fixed channel value (CH is 1-based)
  --move CH@T0-T1       sweep a channel 1000->2000->1000 between T0 and T1 s,
                        as a person moving one control would (for identify)
  --corrupt P           flip a bit in a fraction P of frames (checksum errors)
  --junk P              insert 1-8 random bytes before a fraction P of frames
  --split               write each frame in random-sized pieces
  --loss-at T           the transmitter goes away T seconds in, and then:
  --loss-mode MODE        stop   frames stop entirely
                          hold   frames continue with the last values
                          values frames continue with --loss-set values
                          flag   like values, with 0x2000 set in each word
  --loss-set CH=VAL     a failsafe value for 'values' and 'flag' modes
  --recover-at T        the transmitter comes back T seconds in
  --duration T          exit after T seconds (default: run until killed)

The pty ignores baud rate and stop bits, so line settings are not exercised.
"""

import argparse
import os
import random
import struct
import sys
import time
import tty

CHANNELS = 14
HEADER = b"\x20\x40"


def make_frame(channels):
    body = HEADER + struct.pack("<14H", *channels)
    checksum = (0xFFFF - sum(body)) & 0xFFFF
    return body + struct.pack("<H", checksum)


def parse_assign(text):
    ch, _, val = text.partition("=")
    ch, val = int(ch), int(val, 0)
    if not 1 <= ch <= CHANNELS or not 0 <= val <= 0xFFFF:
        raise argparse.ArgumentTypeError(f"bad channel assignment: {text}")
    return ch - 1, val


def parse_move(text):
    ch, _, span = text.partition("@")
    t0, _, t1 = span.partition("-")
    ch, t0, t1 = int(ch), float(t0), float(t1)
    if not 1 <= ch <= CHANNELS or t1 <= t0:
        raise argparse.ArgumentTypeError(f"bad move: {text}")
    return ch - 1, t0, t1


def swept(t, t0, t1):
    """1000 -> 2000 -> 1000 over [t0, t1], a triangle."""
    phase = (t - t0) / (t1 - t0)
    return int(1000 + 1000 * (1 - abs(2 * phase - 1)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--link", help="symlink this path to the pty slave")
    ap.add_argument("--period-ms", type=float, default=7.0)
    ap.add_argument("--set", type=parse_assign, action="append", default=[])
    ap.add_argument("--move", type=parse_move, action="append", default=[])
    ap.add_argument("--corrupt", type=float, default=0.0)
    ap.add_argument("--junk", type=float, default=0.0)
    ap.add_argument("--split", action="store_true")
    ap.add_argument("--loss-at", type=float)
    ap.add_argument("--loss-mode", choices=["stop", "hold", "values", "flag"], default="stop")
    ap.add_argument("--loss-set", type=parse_assign, action="append", default=[])
    ap.add_argument("--recover-at", type=float)
    ap.add_argument("--duration", type=float)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    master, slave = os.openpty()
    # Raw, so the slave's line discipline neither echoes nor translates. The
    # slave stays open here so the pty survives the reader closing it.
    tty.setraw(slave)
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    if args.link:
        if os.path.islink(args.link):
            os.unlink(args.link)
        os.symlink(path, args.link)
    print(f"ibus_fake: writing frames to {path}"
          + (f" (linked as {args.link})" if args.link else ""), flush=True)

    base = [1500] * CHANNELS
    for ch, val in args.set:
        base[ch] = val
    failsafe = list(base)
    for ch, val in args.loss_set:
        failsafe[ch] = val

    start = time.monotonic()
    next_at = start
    last_sent = list(base)
    frames = 0
    try:
        while True:
            now = time.monotonic()
            t = now - start
            if args.duration is not None and t >= args.duration:
                break
            lost = (args.loss_at is not None and t >= args.loss_at
                    and (args.recover_at is None or t < args.recover_at))

            if lost and args.loss_mode == "stop":
                channels = None
            elif lost and args.loss_mode == "hold":
                channels = last_sent
            elif lost:
                channels = list(failsafe)
                if args.loss_mode == "flag":
                    channels = [v | 0x2000 for v in channels]
            else:
                channels = list(base)
                for ch, t0, t1 in args.move:
                    if t0 <= t <= t1:
                        channels[ch] = swept(t, t0, t1)
                last_sent = channels

            if channels is not None:
                frame = bytearray(make_frame(channels))
                if rng.random() < args.corrupt:
                    frame[rng.randrange(2, 32)] ^= 1 << rng.randrange(8)
                if rng.random() < args.junk:
                    frame[0:0] = bytes(rng.randrange(256) for _ in range(rng.randint(1, 8)))
                pieces = [bytes(frame)]
                if args.split:
                    cut = sorted(rng.sample(range(1, len(frame)), 2))
                    pieces = [frame[:cut[0]], frame[cut[0]:cut[1]], frame[cut[1]:]]
                for piece in pieces:
                    try:
                        os.write(master, piece)
                    except BlockingIOError:
                        pass  # nobody reading yet: drop, as a wire would
                frames += 1

            # Absolute schedule, so sleep overshoot does not accumulate.
            next_at += args.period_ms / 1000.0
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        if args.link and os.path.islink(args.link):
            os.unlink(args.link)
    print(f"ibus_fake: {frames} frames in {time.monotonic() - start:.1f} s", file=sys.stderr)


if __name__ == "__main__":
    main()

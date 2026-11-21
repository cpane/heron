# Maker Faire checklist: the rubber floor

The Faire's floor is rubber (a field-house floor), and the bench rig's
numbers are the hardwood's. Nothing can be measured on rubber beforehand
(decided 2026-11-18), so the floor's numbers are taken on the morning,
before the doors open, in this order. Robot 1 (the bench rig) first.
About 45 minutes; the commands are run from the Mac, in the repository.

`<pi>` is the robot's address on the Faire's network (below).

## Before leaving home

- [ ] **A network the Mac and the Pi both join.** The Pi knows only the
      home Wi-Fi, and there is no other way in. A phone's hotspot,
      added to the Pi at home while it is still reachable (sudo, on the
      Pi): `sudo nmcli device wifi connect "<hotspot name>" password
      "<password>"`, then check the Mac reaches it on the hotspot and
      note the Pi's address there (`ip -br addr show wlan0` on the Pi,
      or the phone's list of connected devices). [unverified: not tried]
- [ ] Fresh motor alkalines in, and spares: the tables are measured on
      fresh cells (RESUME-navigation, Decided 2026-11-16), and so is the
      rubber's.
- [ ] A tape measure, masking tape, the box used at home (about 30 cm).
- [ ] The Mac with this repository on `develop`, built and deployed:
      `scripts/build_cpp.sh`, then `scripts/deploy_cpp.sh <user>@<pi>`.
      The deploy copies `profiles/surfaces/bench-mecanum/rubber-1.conf`,
      today a copy of the hardwood marked not measured.

## 1. Switch the robot to the rubber

```
scripts/set_surface.sh <user>@<pi> rubber-1
```

From then on `heron_robot`, boot service included, starts on `rubber-1`;
its first lines say `Surface: rubber-1 (this Pi's, ~/heron/surface)`.
Until step 4 its numbers are still the hardwood's.

## 2. Does the OTOS see the rubber? (5 minutes; decides the rest)

```
ssh -t <user>@<pi> '~/heron/bin/heron_robot --lines 10 2>&1 | tee ~/heron/captures/rubber-otos.log'
```

Manual. Two strips of tape 1 m apart; drive forward slowly from one to
the other and stop.

- The status line's `speed` reads while moving, and near 0 standing
  (within about 0.04 m/s, as on the hardwood).
- The distance agrees with the tape within about 10%: tell Claude, or
  compare the OTOS's track in the log. The hardwood needed a linear
  scalar of 0.908; rubber may need its own.
- No `TILT` or `POSE_NOT_FRESH` while driving.

**If the OTOS does not track the rubber:** Assisted and Navigate cannot
run on it (both stop without a fresh pose; `MotionWatch` stops a robot
the OTOS calls still). Demo in Manual, which needs neither, and stop
here. Nothing below helps.

## 3. The motion table (about 20 minutes)

Same session (or start again with `rubber-table.log` in the name).
Manual, armed. For each command: from rest, the throttle set (the
status line shows it to 0.01), the right stick pushed fully the way
given, held **4 s**, let go, wait 2 s. Slide the robot back with the
stick centred whenever it runs out of room; that does not show in the
numbers.

| Stick | Throttle | Replaces on hardwood |
|---|---|---|
| fully forward | 0.2, 0.4, 0.5, 0.6, 0.8, 1.0 | p01 to p05, p32 |
| fully right | 0.2, 0.4, 0.5, 0.6, 0.8, 1.0 | p15 to p19, p33 |
| fully left | 0.4, 0.6, 0.8, 1.0 | p20, p21, p27, p28 |
| forward-right corner | 0.3, 0.55, 0.85, 1.0 | p11 to p14 |
| forward-left corner | 0.3, 0.55, 0.7, 0.85 | p23 to p26 |
| fully back | 0.6 | p22 |
| *if there is time:* 1 o'clock (about -15°) | 0.2, 0.4, 0.6, 0.8, 1.0 | p06 to p10 |
| *if there is time:* 10 o'clock (about 65°) | 0.4, 0.55, 0.7 | p29 to p31 |

The two rows marked "if there is time" keep the hardwood's numbers if
skipped; the calibration record (step 4) says so.

## 4. Into the profile (10 minutes, on the Mac)

```
scripts/fetch_captures.sh <user>@<pi>
python3 python/tools/motion_points.py captures/rubber-table.log
```

It lists each held command (log line, throttle, direction, size, what it
made) and the `p<n>` lines to paste. In
`profiles/surfaces/bench-mecanum/rubber-1.conf`:

- Replace each hardwood line with the measured one for the same stick
  and size (the table above says which); keep the numbering.
- `max_speed` = forward at 1.0's speed; `min_speed` = forward at 0.2's.
- `linear_scalar` from step 2 if the OTOS was off by more than a few
  percent (`docs/rig-calibration.md` §8.5); else leave it.
- The header and `[surface.rubber-1.calibration]`: the date, the log,
  what was measured and what is still the hardwood's.

Then `scripts/deploy_cpp.sh <user>@<pi>`. A `rig_profile show` run
on the Pi checks the file loads:
`ssh <user>@<pi> '~/heron/bin/rig_profile show'`.

## 5. Navigate on the rubber (10 minutes)

```
ssh -t <user>@<pi> '~/heron/bin/heron_robot --lines 10 2>&1 | tee ~/heron/captures/rubber-navigate.log'
```

- **From rest:** Navigate, stick forward from standing, three times: it
  must move off within about a second, not stand and hum. (The from-rest
  thresholds were set on the hardwood.)
- **Round the box:** full stick, the box centred on the line, then 10 cm
  left, 0.6 m clear each side and 2 m past it, as at home on 2026-11-18.
- **Stopping:** Assisted, full stick toward the box: it must stop short.

Anything wrong: Manual still works, and is the demo.

## 6. Robot 2

Its own rig profile (`mini-2`, RESUME-navigation, "Picking up" 0.4),
its own transmitter (`docs/wiring.md`, "Transmitter set-up"), then
`scripts/set_surface.sh <user>@<pi2> rubber-1` and step 5. Both robots
share the drive, so its table is robot 1's [unverified: two builds can
differ; step 5 is the check].

## Back home

```
scripts/set_surface.sh <user>@<pi> --default
```

The robot is on the hardwood again from its next start. Commit the
measured `rubber-1.conf` on a branch: the Faire's numbers stay, beside
the hardwood's.

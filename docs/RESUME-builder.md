# Builder workstream — resume point

Started 2026-11-02. Scripts and a guide for people who build, set up and
drive a Heron without changing its code: they need a terminal, not a
compiler or the design docs.

## What exists

| File | What |
|---|---|
| [`builder-guide.md`](builder-guide.md) | The guide, one section per script, written for builders |
| `builder/README.md` | A short index of the scripts, pointing to the guide |
| `builder/wheel_check.sh` | Interactive: each motor alone (which wheel, which way), the fix as wiring or as profile lines; then each remote stick through `heron_teleop` |

`wheel_check.sh` was tested on the host against a fake `ssh` (part 1 with
the two wiring faults the bench rig had on 2026-11-02, part 2 with scripted
status lines). **Not yet run on the robot.**

## How builder scripts are written

- **One script per task, in `builder/`,** taking `<user>@<pi>` like the
  host scripts in `scripts/`, and following the same shell conventions
  (`CLAUDE.md`).
- **Plain words, no code knowledge assumed.** Questions with short answers;
  every failure says what to do next. Name things as a builder sees them
  (front-left wheel, terminal M1, SwA), not by code names.
- **Use the programs already on the Pi** (`motor_link`, `heron_teleop`,
  ...); a builder script adds no robot code. If it needs something the
  programs cannot do, that is a change in the owning workstream first.
- **Leave the robot as it was found:** a service stopped for the check is
  offered back at the end, and the script says so if it exits early.
- **Each script gets a section in the guide and a row in both indexes**
  (the guide's and `builder/README.md`).

## Picking up

1. Run `wheel_check.sh` on the bench rig, both parts, and record it here.
2. Next scripts, as builders need them. Candidates: setting up a new Pi end
   to end (provision, rig, deploy, autostart) as one guided script; turning
   the teleop service on and off; a status check (is the Pi reachable, are
   the devices there, what is running).

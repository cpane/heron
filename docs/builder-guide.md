# Heron builder's guide

For people who want to build a Heron, set it up and drive it, without
changing its code. Each task here is one script in [`builder/`](../builder/)
that asks you questions as it goes and tells you what to do when something
is wrong. You need no programming, only a terminal.

This guide grows as builders take on more of the robot. Today it has one
script: checking that the wheels turn the right way.

## What you need

- **A computer with this repository on it**, a Mac or Linux, with a terminal
  open in the repository's folder. Every command below is typed there.
- **A Heron whose Pi is set up and has Heron's programs on it.** The
  developer setting up the robot does this once (`scripts/provision_pi.sh`,
  `scripts/set_rig.sh`, `scripts/deploy_cpp.sh`). The scripts check for it
  and say what is missing.
- **The Pi's address and user name**, written below as `<user>@<pi>`: for
  example `pat@192.168.1.40`. Whoever set up the Pi can tell you. If the
  scripts ask for a password on every step, set up a key once with
  `ssh-copy-id <user>@<pi>`.
- **The FlySky FS-i6X remote,** switched on, for the steps that use it.

## Scripts

| Script | When to run it |
|---|---|
| [`wheel_check.sh`](#check-the-wheels) | After building the robot, and after any change to the motor wiring |

## Check the wheels

```
builder/wheel_check.sh <user>@<pi>
```

Run it after wiring the motors for the first time, and again whenever a
motor wire has been moved. A wheel that turns the wrong way gives no error:
the robot just drives the wrong way, or turns when it should slide. So each
wheel is checked by eye.

**Lift the robot first,** on a box under its frame, so no wheel touches
anything. Turn the motor battery on. The script reminds you of both.

**"Forward"** for a wheel means the way that would roll the robot forward:
the top of the wheel moves toward the robot's front.

### Part 1: the motors

The script turns one motor at a time, for 2 seconds, and asks:

1. **Which wheel turned:** front-left, front-right, back-right, back-left,
   none, or more than one. Answer `r` to see it again.
2. **Which way:** forward or backward.

It goes in this order, and each should turn its own wheel forward:

| Motor | Wheel |
|---|---|
| 1 | front-left |
| 2 | front-right |
| 3 | back-right |
| 4 | back-left |

If every answer is right, it says so and moves on to part 2. If not, it
works out the fix and prints it two ways. **Pick one:**

- **A, move wires on the motor board.** It names each motor to move, which
  terminal (M1 to M4) it is on, where it belongs, and whether to swap its
  two wires. Turn the motor battery off first.
- **B, change the profile instead.** It prints two lines, `terminal = ...`
  and `sign = ...`, to put in `profiles/<rig>.conf` under `[motor]`. Then
  copy the profile to the Pi with `scripts/deploy_cpp.sh <user>@<pi>`.
  Use B when the wires are hard to reach.

Then run the check again. It is right when every motor turns its own wheel
forward.

Two answers it cannot work out a fix for:

- **A motor turned no wheel, or more than one.** That is not a mix-up of
  terminals. Check that the motor battery is on, that each motor's two
  wires are tight in their terminal, and that no wheel is jammed.
- **Two motors turned the same wheel.** One of the answers was wrong: run
  it again and watch closely.

An example, from the bench rig on 2026-11-02, after its motors were
rewired: motor 1 turned the front-right wheel and motor 2 the front-left,
both forward. The fix was to swap the two front motors between terminals M1
and M2, keeping each motor's wires in order.

### Part 2: the remote

Once part 1 passes, the script offers to check the remote. It starts the
program that drives with the remote, and asks you to arm it:

1. Left stick (the throttle) all the way down.
2. Switch SwA (top left) down.
3. Throttle up to about half way.

The robot starts driving as soon as the throttle reaches the bottom, but
with the throttle there the sticks do nothing, so the script waits for it
to come up. If it reminds you about the throttle, push it up.

Then it asks you to push each stick and hold it. It reads what the robot
saw, checks the wheel commands, and asks whether the wheels did it:

| Stick | The wheels should |
|---|---|
| Right stick up | All four forward |
| Right stick down | All four backward |
| Right stick left | Front-left and back-right backward, front-right and back-left forward (slide left) |
| Left stick left | Front-left and back-left backward, front-right and back-right forward (turn left) |
| SwA up | All stop at once |

**The remote reads wrong.** If the robot reads a stick as a different
movement, or the opposite way, the script first asks whether it was a
slip: the other stick, or the other way, is easy to do. Say yes and try
that stick again. If it still reads wrong, the transmitter's channels are
set up differently from Heron's: the right stick should be channels 1
(left-right) and 2 (up-down), the left stick 3 (throttle) and 4
(left-right), SwA channel 7. Check the transmitter's settings, or ask the
developer: the fix is in Heron's settings, not in the wiring.

### The teleop service

If the Pi is set to start the remote driving program at boot
(`scripts/teleop_autostart.sh`), it is using the motors, and the check
cannot run beside it. The script offers to stop it, and to start it again
at the end. Both need the Pi user's password, for `sudo`.

If the check ends early, on an error or Ctrl-C, it still offers to start
the service again. If you say no, it prints the command to start it later.
Restarting the Pi also starts it.

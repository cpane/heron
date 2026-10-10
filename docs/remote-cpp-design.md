# Remote control C++ — design and intended use

How the iBUS code is layered, and how the robot is meant to use it. The
protocol itself is in [`remote-ibus-protocol.md`](remote-ibus-protocol.md);
the diagnostic tool in [`remote-ibus-reader.md`](remote-ibus-reader.md).

## 1. Layers

```
heron_ibus         pure: IBusParser (+ IBusStats), LinkMonitor, Normalize,
                   channelValue()/channelHighNibble(). No I/O, no threads.
  ^
heron_ibus_port    IBusPort: receive-only termios, poll() with a timeout.
                   IBusReceiver: port + parser + link rule, synchronous.
  ^
heron_remote       RemoteControl: a reader thread publishing the newest frame
                   and link state. What robot code links.

ibus_reader        tool, synchronous (IBusReceiver): needs every frame
remote_adapter     probe: RemoteControl live, as a 50 Hz loop sees it
```

Each row links the rows above it, publicly, so linking `heron_remote` brings
the headers of all three.

## 2. Two ways in, for two kinds of consumer

| Consumer | Wants | Uses |
|---|---|---|
| Robot control loop | the newest channels and whether to trust them, at its own rate, never waiting | **`RemoteControl::getLatest()`** |
| Diagnostics, recording, tests | every frame, in order | `IBusReceiver::pump()` with a callback, or `IBusParser` fed directly |

`RemoteControl` is shaped like `Heron::Lidar::Lidar` on purpose, so the
robot core deals with every sensor the same way:

- `start()` and `stop()` belong to one owning thread.
- One reader thread (`std::jthread`, named `heron-ibus` on Linux) runs per
  instance. It blocks in `poll()` for at most 20 ms, which also bounds how
  long `stop()` takes.
- `getLatest()` and `getHealth()` are callable from any thread and hold a
  mutex only to copy about 100 bytes.
- **Latest wins.** The thread decodes every frame; a consumer polling slower
  than 130 Hz simply skips frames. `seq` counts frames, so a consumer can see
  how many it skipped.

## 3. The link is judged when you ask

`RemoteSnapshot::link` is computed **at the time of the `getLatest()`
call**, from the time of the newest frame. It is not stored when the frame
arrives. So every failure looks the same to a consumer, with no special case:
a quiet line, a dead receiver, a port that failed, or a reader thread that
stopped. A frame older than `staleAfter` (100 ms) reads as STALE. A
consumer that only checks `isOk()` stops the robot in every one of those cases.

`getLatest(now)` takes an explicit time for a control loop that stamps each
cycle once and wants every input judged at that same instant.

## 4. Intended use in the robot

A sketch, not yet built. The robot core does not exist yet.

```cpp
Heron::Remote::RemoteControl remote(Heron::Remote::ReceiverConfig{});
if (!remote.start()) { fail(remote.getHealth().lastError); }

// each control cycle, e.g. 50 Hz
const auto now = Heron::Remote::Clock::now();
const Heron::Remote::RemoteSnapshot s = remote.getLatest(now);
if (!s.isOk() || isFailsafe(s.frame)) {        // isFailsafe: after §5 is measured
  commandStop();
} else {
  const auto &ch = s.frame.channels;
  // check channelHighNibble() == 0 on each channel used, then:
  drive = normalizeAxis(ch[kDriveCh], driveAxis);
  ...
}
```

What the robot owns, not this library:

- **The channel map and the meaning of each control.** These come from
  `identify` (protocol §4) and the control rules (protocol §7). The library
  hands out raw channel words and pure helpers. It does not know which
  stick is which.
- **Arming policy.** For example, "only armed if the switch was seen
  disarmed since the link came up". That is state across cycles, and it
  belongs with the motion arbiter.
- **What to do on loss.** The library says the link is not OK. Stopping is
  the robot's decision.

## 5. Failsafe detection: deliberately not here yet

Every source says the FS-iA6B keeps sending frames on radio loss: with the
last values (failsafe Off), or with set values, possibly flagged in the top
nibble (protocol §5, **unverified**). If that holds, `isOk()` stays true
while the transmitter is off, and the stale rule alone is not enough.

What to add depends on what `failsafe-test` shows, so nothing has been
guessed yet. Once it has been measured, the check goes into `heron_ibus`
as a pure function of a frame (and config). `RemoteSnapshot` then gets a
`failsafe` flag, so `isOk()` can cover both cases and the robot keeps a
single test. `RemoteSnapshot` already carries the raw frame for exactly this.

## 6. Errors

- `start()` returns false with `lastError` set: device missing, permission
  denied, or bad settings. It never throws.
- A port failure while running (a read error, or a hung-up device) ends the
  reader thread and sets `state = Failed` with `lastError`. New frames stop,
  so consumers see STALE within 100 ms without checking the state. `start()`
  can be called again.
- A quiet line is **not** an error. The state stays Running and the link
  reads STALE.

`IBusPort` treats "poll() reports data, read() returns nothing" as a hangup.
Without that, a vanished device turned the reader into a busy loop. The pty
test caught this, and 30 s of `stats` on the Pi confirmed the real UART
never triggers it.

## 7. Tests

| Test | What it covers |
|---|---|
| `ibus_parser_test` | parser, stats, `LinkMonitor`: fed bytes, no port |
| `normalize_test` | axis, knob and switch helpers at their edges |
| `remote_control_test` | `RemoteControl` over a real pseudo-terminal: start, publication, stale and recovery, stop and restart, missing device, hangup mid-run |

All run under `ctest`, on the host and on the Pi. A pty ignores baud rate and
stop bits, so those are only exercised against the real receiver
(`remote_adapter`, `ibus_reader stats`).

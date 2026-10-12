# Remote control C++ — design and intended use

How the iBUS code is layered, and how the robot is meant to use it. The
protocol itself is in [`remote-ibus-protocol.md`](remote-ibus-protocol.md);
the diagnostic tool in [`remote-ibus-reader.md`](remote-ibus-reader.md).

## 1. Layers

```
heron_ibus         pure: IBusParser (+ IBusStats), LinkMonitor, Failsafe,
                   Normalize, channelValue()/channelHighNibble(). No I/O,
                   no threads.
  ^
heron_ibus_port    IBusPort: receive-only termios, poll() with a timeout.
                   IBusReceiver: port + parser + link rule, synchronous.
  ^
heron_remote       RemoteControl: a reader thread publishing the newest frame
                   and link and failsafe state. What robot code links.

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

### Examples, one per layer

These compile against the headers and link with the libraries (checked
2026-10-12). `setMotors()`, `check()`, `handle()` and the choice of sticks
are placeholders for code that does not exist yet.

**`heron_remote`: robot code, almost always.** The newest sticks, at the
loop's own rate. With the failsafe configured, one `isOk()` covers a
stale link, a disarm and a lost radio.

```cpp
#include "heron/remote/RemoteControl.h"
using namespace Heron::Remote;

void driveLoop(const std::atomic<bool> &running) {
  ReceiverConfig config;                          // /dev/serial0, stale after 100 ms
  config.failsafe = FailsafeConfig{6, 1000, 50};  // CH7 = SwA: 1000 is disarmed or radio lost
  RemoteControl remote(config);
  if (!remote.start()) {                          // never throws
    std::fprintf(stderr, "remote: %s\n", remote.getHealth().lastError.c_str());
    return;
  }
  const AxisConfig stick{.deadband = 10};        // measured, protocol §7
  while (running) {
    const RemoteSnapshot s = remote.getLatest();  // never waits
    if (!s.isOk()) {
      setMotors(0, 0);                            // stale, disarmed, or radio lost
    } else {
      const std::int32_t speed = normalizeAxis(s.frame.channels[1], stick);  // CH2
      const std::int32_t turn = normalizeAxis(s.frame.channels[0], stick);   // CH1
      setMotors(speed + turn, speed - turn);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50 Hz
  }
  remote.stop();
}
```

The real robot adds the arming rule (§4): after a stop, SwA must be seen
disarmed and then armed before it drives again.

**`heron_ibus_port`: every frame, in order.** Logging, timing and
diagnostics, with no background thread. This is how `ibus_reader`'s
`dump`, `record` and `stats` work; `RemoteControl` would skip frames here,
because it keeps only the newest.

```cpp
#include "heron/remote/IBusReceiver.h"
using namespace Heron::Remote;

// Log every SwA change with frame-accurate times.
int main() {
  IBusReceiver rx(ReceiverConfig{});              // throws std::system_error if the port fails
  const TimePoint start = Clock::now();
  std::uint16_t last = 0;
  while (Clock::now() - start < std::chrono::seconds(10)) {
    rx.pump(std::chrono::milliseconds(20), [&](const IBusFrame &f) {
      if (f.channels[6] != last) {
        const double t = std::chrono::duration<double>(f.stamp - start).count();
        std::printf("%6.3f s  CH7 %u -> %u\n", t, last, f.channels[6]);
        last = f.channels[6];
      }
    });
  }
  std::printf("%" PRIu64 " frames, %" PRIu64 " checksum errors\n",
              rx.getStats().framesDecoded, rx.getStats().checksumErrors);
}
```

**`heron_ibus`: rarely on its own.** Tests, replays, and bytes that do not
come from a serial port. Robot code still uses its helpers
(`normalizeAxis()`, `decodeSwitch3()`) all the time, through
`heron_remote`.

```cpp
#include "heron/remote/Failsafe.h"
#include "heron/remote/Normalize.h"
using namespace Heron::Remote;

// Test control logic with no hardware: build the frame by hand.
IBusFrame f{};
f.channels.fill(1500);
f.channels[6] = 1000;                             // SwA up
check(isFailsafe(f, FailsafeConfig{6, 1000, 50}), "SwA up reads as failsafe");
check(normalizeAxis(1750, AxisConfig{}) == 500, "three-quarter stick is +500");
check(decodeSwitch3(1500) == Switch3::Mid, "SwC middle");

// Or decode raw bytes from anywhere: a file, a test vector, a socket.
IBusParser parser;
parser.feed(bytes, Clock::now(), [](const IBusFrame &frame) { handle(frame); });
```

## 3. The link is judged when you ask

`RemoteSnapshot::link` is computed **at the time of the `getLatest()`
call**, from the time of the newest frame. It is not stored when the frame
arrives. So every failure looks the same to a consumer, with no special case:
a quiet line, a dead receiver, a port that failed, or a reader thread that
stopped. A frame older than `staleAfter` (100 ms) reads as STALE. A
consumer that only checks `isOk()` stops the robot in every one of those cases,
and on radio loss too, once a failsafe channel is configured (§5).

`getLatest(now)` takes an explicit time for a control loop that stamps each
cycle once and wants every input judged at that same instant.

## 4. Intended use in the robot

A sketch, not yet built. The robot core does not exist yet.

```cpp
Heron::Remote::ReceiverConfig config;
config.failsafe = Heron::Remote::FailsafeConfig{kArmCh, 1000, 50};  // §5
Heron::Remote::RemoteControl remote(config);
if (!remote.start()) { fail(remote.getHealth().lastError); }

// each control cycle, e.g. 50 Hz
const auto now = Heron::Remote::Clock::now();
const Heron::Remote::RemoteSnapshot s = remote.getLatest(now);
if (!s.isOk()) {             // stale, no signal, or failsafe (radio lost or disarmed)
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
  stick is which, so `ReceiverConfig::failsafe` is empty by default and
  the robot fills it in.
- **Arming policy.** For example, "only armed if the switch was seen
  disarmed since the link came up". That is state across cycles, and it
  belongs with the motion arbiter.
- **What to do on loss.** The library says the link is not OK. Stopping is
  the robot's decision.

## 5. Failsafe detection: radio loss through the arm switch

**What was measured** (protocol §5, [ours]): the FS-iA6B never stops
sending frames when the radio is lost, so the stale rule cannot see it.
Channels whose transmitter failsafe is Off freeze at their last value. A
channel with a failsafe value moves to it about 1 s after the loss, with
no flag in the top 4 bits. A channel reading its failsafe value is
therefore the only sign of radio loss.

**What the library does.** In `heron_ibus`, `Failsafe.h`:

```cpp
struct FailsafeConfig { std::size_t channel; std::uint16_t value; std::uint16_t tolerance; };
void checkFailsafeConfig(const FailsafeConfig &config);   // throws std::invalid_argument
bool isFailsafe(const IBusFrame &frame, const FailsafeConfig &config);
```

`isFailsafe()` is true when `channelValue()` of the configured channel is
within `tolerance` (inclusive) of `value`. It is also true when that
channel's top 4 bits are not zero. No such word has been seen on our
receiver, so it is unexplained, and an unexplained word on the arm channel
must not read as "armed". The config is rejected if the channel is not
below 14 or the value does not fit in 12 bits.

`ReceiverConfig::failsafe` (a `std::optional<FailsafeConfig>`, empty by
default) turns it on. `IBusReceiver` only checks it, so a bad one fails
`start()` with `lastError` like any other bad setting. `RemoteControl`
judges each frame as it arrives and publishes the result:

- `RemoteSnapshot::failsafe`: the newest frame is a failsafe frame. False
  before the first frame and when no failsafe is configured. It is a
  property of the frame, so it stays set if the link then goes stale.
- `RemoteSnapshot::isOk()` is link OK **and** not failsafe, so the robot
  keeps one test.
- `RemoteHealth::failsafeEvents`: entries into failsafe, one per run of
  failsafe frames, like `staleEvents`.

**Our setup.** SwA on CH7 (index 6) is the arm switch: up 1000 = disarmed,
down 2000 = armed. Its transmitter failsafe is 1000. So the robot sets
`FailsafeConfig{6, 1000, 50}`, and `remote_adapter` does the same.

**Verified on the receiver** (2026-10-11, `remote_adapter 40`) **[ours]**:
flipping SwA up, and switching the transmitter off, each set `failsafe` with
the link staying OK throughout. Over 1992 polls, every one that was not OK
was a failsafe one (742), with 0 stale events, 2 failsafe events, and 0
checksum errors. After switch-on and SwA down it cleared.

**Limits:**

- **A deliberate disarm reads exactly like radio loss.** That is intended:
  both mean stop. The library does not try to tell them apart, and a
  robot that needs to (to report "radio lost") cannot do it from the
  frames.
- **Detection lags the loss by about 1 s** (1027 and 1028 ms in runs 2 and
  4, [ours]). Until then the receiver repeats the last values and the
  robot keeps acting on the last stick positions. Nothing on our side can
  shorten this.
- **It depends on the transmitter's failsafe actually being set.** With
  CH7's failsafe Off, radio loss freezes CH7 at armed and is invisible.
  Run 3 found the settings lost on the transmitter. Check the Failsafe menu
  (after any model or channel change, and before trusting the robot to
  stop), and confirm with `remote_adapter`: switch the transmitter off and
  watch `failsafe` go `YES`.
- **Arming policy is still the robot's** (§4). Failsafe ends when CH7
  leaves 1000; a robot that requires a fresh disarm-then-arm after a loss
  must keep that state itself.

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
| `failsafe_test` | `isFailsafe()`: match, tolerance edges, the high nibble, bad configs |
| `remote_control_test` | `RemoteControl` over a real pseudo-terminal: start, publication, stale and recovery, failsafe with the link OK and its recovery, stop and restart, missing device, hangup mid-run |

All run under `ctest`, on the host and on the Pi. A pty ignores baud rate and
stop bits, so those are only exercised against the real receiver
(`remote_adapter`, `ibus_reader stats`).

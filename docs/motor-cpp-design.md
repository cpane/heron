# Motors C++ — design and intended use

How the Pi's motor library, `heron_motor`, is layered, and how the robot
is meant to use it. **Agreed 2026-10-25, not built** (motor step 5). The
wire format it speaks is [`motor-protocol.md`](motor-protocol.md); the
rig's numbers it sends come from its profile
([`rig-profiles.md`](rig-profiles.md)).

It is shaped like `Lidar`, `RemoteControl` and `Odometry`: one owning
thread starts and stops it, a reader thread publishes the newest state,
and any thread may read that state. What differs is that this library also
*sends*: the robot's control loop drives the motors through it.

## 1. Layers

```
heron_motor_protocol   the wire format, shared with the firmware: frames,
                       messages, the CRC, FrameParser. Header-only, no I/O
  ^
heron_motor_port       MotorPort: /dev/arduino, 115200 8N1, raw; read with a
                       timeout, write
  ^
heron_motor            Motors: start-up handshake, reader thread, drive(),
                       the newest Status. What robot code links.
                       Also links heron_profile: its configuration holds
                       the profile's [motor] section.

motor_link             tool, as now: the low-level bench tool, frame by frame
motor_adapter          probe (new): Motors live, as a 50 Hz loop sees it
```

`toMotorConfig()`, the conversion from a profile's `[motor]` section to the
Configure message, moves from `motor_link` into `heron_motor`, so the tool
and the library cannot convert differently.

## 2. Decisions (2026-10-25)

| # | Decision | Why |
|---|---|---|
| D1 | **The robot's control loop sends Drive**, one frame per `drive()` call, every 20 ms. The library has no heartbeat thread | If the robot loop stalls, the drives stop and the Arduino's 250 ms watchdog stops the motors. A library thread repeating the last command would keep the watchdog fed while the robot hangs, and would need its own timeout to be safe |
| D2 | **`start()` blocks until the Arduino is configured**, up to 3 s, and fails with a message that says what to do | Opening the port resets the Uno: Boot follows about 1.5 s later, then Configure and its checksum. A robot must not start its loop on a board that cannot drive. `Odometry::start()` blocks too, for its calibration |
| D3 | **`drive()` takes −1.0..1.0 per wheel**, rounded to the wire's −1000..1000 and clamped. A NaN or infinite value sends Stop | The mixing works in fractions of full drive (motor-design.md §4). A bad number is a bug upstream; stopping is the safe answer to it |
| D4 | **The caller passes the profile in.** `MotorsConfig` holds the profile's platform and `[motor]` section | The robot loads its profile once and gives each library its section, as `[odometry]` and `[lidar]` arrive. Tests pass made-up profiles with no files |
| D5 | **The class is `Heron::Motor::Motors`** | Like `Lidar` and `Odometry`. `MotorController` is taken by the firmware's logic, in the same namespace |

## 3. The robot's view: `Motors`

### 3.1 Configuration

```cpp
struct MotorsConfig {
  std::string device{"/dev/arduino"};
  std::string platform;                    // the profile's [rig] platform
  Heron::Profile::MotorProfile motor;      // the profile's [motor] section
  Duration startTimeout{std::chrono::seconds(3)};
  Duration staleAfter{std::chrono::milliseconds(300)};   // 3 Status periods
};

/** The configuration for a rig: device default, platform and [motor] from the profile. */
MotorsConfig toMotorsConfig(const Heron::Profile::RigProfile &profile);
```

`start()` checks it first: a known platform, and a `[motor]` section that
makes a valid Configure (`isValidConfig()`). A profile from
`loadRigProfile()` always passes; a hand-built one may not.

### 3.2 Driving

```cpp
bool drive(const std::array<double, 4> &wheels);   // front-left, front-right, back-right, back-left
bool stopMotors();                                  // the Stop message: release all four now
```

- **`drive()` sends one Drive frame now**, and returns. Call it every
  20 ms, also when nothing changes: it is the heartbeat (D1).
- **Values** are −1.0..1.0 of full drive, positive driving the robot
  forward, in protocol order. They are multiplied by 1000, rounded and
  clamped. Any non-finite value sends Stop instead (D3).
- **Returns false, sending nothing**, unless running: before `start()`,
  after `stop()`, or after the reader failed. Whether the Arduino *acts* on
  a Drive is in the snapshot: after a reset it refuses Drive until the
  library has configured it again (§5).
- **`stopMotors()`**, not `stop()`: `stop()` is the lifecycle, as on every
  library here. `stop()` also sends a Stop before closing the port.

### 3.3 What a snapshot holds

```cpp
struct MotorsSnapshot {
  bool fresh{false};          // a Status within staleAfter, judged at the call
  bool configured{false};     // the newest Status's checksum is ours
  std::uint64_t seq{0};       // Status messages so far
  Duration age{};             // from the newest Status to the call
  Status status{};            // the newest, decoded: state, stop reason,
                              // target, applied, the board's link counters
  bool isOk() const { return fresh && configured; }
};
```

`isOk()` is the one test for "the motors will do what I send": the board
is alive and holds this rig's calibration. Stopped is still OK; why it
stopped is `status.stopReason`. As on the other libraries, freshness is
judged when you ask, so a dead reader reads as stale.

### 3.4 Health

```cpp
struct MotorsHealth {
  MotorsState state{MotorsState::Stopped};   // Stopped, Running, Failed
  BootInfo boot{};                // the newest Boot: firmware, protocol, platform
  std::uint64_t boots{0};         // Boots seen, the one start() causes included
  std::uint64_t resets{0};        // Boots after start(): the Arduino reset while running
  std::uint64_t configuresSent{0};
  std::uint64_t staleEvents{0};   // times the link went from fresh to stale
  ParserStats link{};             // the Pi side's frame counters
  std::string lastError;
};
```

`resets` matters most on a real robot: a reset under load means motor
current pulled the Uno's supply down (a brown-out), which the Uno's
bootloader hides from the reset cause ([`RESUME-motor.md`](RESUME-motor.md),
step 2).

## 4. Start-up: `start()` blocks until configured

1. Check the configuration (§3.1). Open the port, which resets the Uno.
   Start the reader thread.
2. Wait for Boot, up to `startTimeout`. None: fail with *"no Boot from
   /dev/arduino within 3 s: is the firmware flashed?"*. If frames arrived
   but all with another protocol version (the parser counts them), say so
   instead: *"the board speaks another protocol version: reflash it"*.
3. Check Boot: protocol version 2, and its platform the profile's
   (`uno-l293d` is 1). A mismatch fails, naming both.
4. Send Configure; wait for a Status carrying its checksum, up to 1 s.
5. Running. Any failure stops the thread, closes the port, and leaves
   `state = Failed` with `lastError`; `start()` may be called again.

About 1.6 s in all on the bench rig, almost all of it the Uno's bootloader.

## 5. The reader thread

| Aspect | Choice |
|---|---|
| Each pass | Read the port, 20 ms timeout; feed the parser |
| Status | Published, latest wins; `configured` from its checksum |
| A Boot after start-up | The Arduino reset: counted in `resets`; Configure sent again at once |
| A Status without our checksum | Configure sent again, at most once per 500 ms. Covers a lost or corrupted Configure, not only a reset |
| A device error | The thread ends, `state = Failed`, `lastError` set; `drive()` returns false from then on |

The reader and `drive()` both write to the port, so writes go under a
mutex; a frame is never interleaved with another. Reads happen only on the
reader thread.

## 6. Intended use in the robot

A sketch; the robot core does not exist yet.

```cpp
const auto profile = Heron::Profile::loadRigProfile(Heron::Profile::findRigProfile());
Heron::Motor::Motors motors(Heron::Motor::toMotorsConfig(profile));
if (!motors.start()) {                       // ~1.6 s: reset, Boot, Configure
  fail(motors.getHealth().lastError);
}

// each 20 ms
const auto r = remote.getLatest(now);
if (!r.isOk()) {
  motors.stopMotors();                       // disarmed, radio lost or stale
} else {
  motors.drive(mix(sticksToVelocity(r.frame)));   // four values, -1..1
}
if (!motors.getLatest(now).isOk()) {
  // the board is gone, or reset and not yet configured again: report it
}
```

The robot owns the loop timing, the mixing, and what to do when the motors
are not OK. The library owns the wire, the handshake and keeping the board
configured.

## 7. Left out of the first version

- **The battery voltage**: Status carries it, but no divider is fitted
  yet; it reads 0.
- **Automatic restart after `Failed`**: the robot decides whether to call
  `start()` again (which resets the Uno).
- **`motor_link` rebuilt on `Motors`**: it stays the frame-level bench
  tool; it shares `toMotorConfig()` with the library.
- **Several Arduinos**: one board per rig.

## 8. Tests

| Test | Covers |
|---|---|
| `motors_test` (`ctest`) | `Motors` against a **fake Arduino on a pseudo-terminal**, as `remote_control_test` fakes the receiver. The fake runs the firmware's own `MotorController` and `FrameParser` on the host, so it refuses, configures and reports exactly as the board does. Cases: start-up and its checksum; no Boot; another protocol version; another platform; an invalid profile; `drive()` scaling, clamping and NaN to Stop; `drive()` before `start()`; stale when the fake goes quiet; a reset mid-run, configured again; a lost Configure, sent again; `stop()` sends Stop |
| `motor_adapter` (probe) | `Motors` on the real board: the snapshot and health once a second, and optionally four wheel values driven at 50 Hz |

The conversion `toMotorConfig()` gets its own checks in the same test.

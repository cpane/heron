# Telemetry — design

How the robot streams what it knows to programs on other computers, live,
while it drives. **Proposed 2026-10-28**, not built. The rules it must keep
are [`robot-telemetry.md`](robot-telemetry.md) (a shared document); this is
the design that keeps them. The LiDAR is the first sensor it carries, and
the design is a pattern every later one (odometry first) follows without
changing the server or any client.

## 1. What it is

A **telemetry server inside the robot program**: one thread that reads the
newest data from each library through `getLatest()`, encodes it once, and
sends it to every connected client over TCP. Clients only watch: they never
open a device, never command the robot, and never slow it.

```
 robot program (heron_teleop today)                        clients (host)
 +-----------------------------------------------+
 | Lidar ----getLatest()---+                     |        lidar_gui.py  (as is)
 | RemoteControl           |  TelemetryServer    |  TCP   telemetry_watch.py
 | Motors                  +- channel "lidar" ---+------> nc <pi> 5555
 | control loop -publish()-+- channel "robot"    |  5555  later: a dashboard
 |   (50 Hz, never waits)                        |
 +-----------------------------------------------+
```

A **channel** is one stream of one kind of data: `lidar` (scans), `robot`
(the control loop's state), later `odometry`. The server knows nothing
about any sensor; a channel is what connects a library to it.

## 2. Decisions (2026-10-28)

| # | Decision | Why |
|---|---|---|
| D1 | **One general server, sensors added as channels** | The LiDAR is the first of several. A server per sensor would repeat the hard part (sockets, slow clients, limits) and give clients one connection per sensor |
| D2 | **The wire format stays `wire.py`'s newline-delimited JSON, version 1, extended only by additions** | Rule 6 of the requirements: one format. The existing viewer ignores message types and fields it does not know (checked in `lidar_gui.py`, `_handle()` and `_on_hello()`), so additions reach new clients without breaking it. `nc` remains a working client |
| D3 | **Read-only.** Clients may ask for `hello` and `subscribe`; every other request is refused with a reason | Rule 2. The viewer's motor, scan, mode and reset buttons get an answer it already displays |
| D4 | **A client gets every channel unless it subscribes to fewer** | Bandwidth is not a constraint (requirements, "Bandwidth"), and old clients ignore what they do not know. A client that wants less says so |
| D5 | **Latest wins, per client and per channel** | Rule 3. A slow client misses items rather than queueing them, and a large scan never delays a small pose message behind it |
| D6 | **Each item is encoded once**, and the same bytes go to every client | Four viewers cost the robot the same encoding as one |
| D7 | **Up to 4 clients** at once; a fifth is told why and closed | Several people can watch. The existing servers allow one because their client controls the sensor; here none does |
| D8 | **The robot program owns the LiDAR and runs it whenever it runs**, in Express (mode 1); `--no-lidar` turns it off | It is a robot sensor, and the safety arbiter will need it running. A client starting it would be a client controlling a device (D3) |
| D9 | **A `robot` channel from the first version**: arming, sticks as velocity, wheels, motors' health, loop timing, the server's own counters | Two channels at once prove the pattern is not LiDAR-shaped. The loop timing makes "a client never slows the robot" a measured fact, visible to every client |
| D10 | **Telemetry failing never stops the robot.** A port already in use, or a source that fails, is reported and the robot drives on | Watching is optional; driving is not |
| D11 | **Our own small JSON writer and command reader**, no library | The standard library only, as the rest of `cpp/` (`CPP_CODING_STANDARD.md`). The server writes JSON and reads only tiny commands, so a writer plus a strict reader for flat objects is all it needs |
| D12 | **New messages use SI units** (m, rad, m/s, rad/s) as numbers, and integer nanoseconds for times. The LiDAR's scan keeps its v1 fields as they are | One rule for every later channel. The scan's centidegrees and millimetres stay, because the existing viewer reads them |

## 3. Layers

```
heron_telemetry_wire   pure, no I/O: JsonWriter, the command reader, and
                       the encoders, one file per channel
                       (LidarMessages, RobotMessages; later
                       OdometryMessages). Golden-message tests.
  ^
heron_telemetry        Channel, Published<T>, TelemetryServer (sockets,
                       one thread). Links the libraries whose channels
                       it builds: heron_lidar now, heron_odometry later.

heron_teleop           the robot program: owns the devices, adds the
                       channels, runs the server
lidar_gui_server       bench tool, as now, but encoding through
                       LidarMessages: the two cannot drift
telemetry_watch.py     host tool (new): any channel, as text
```

The dependencies run one way. **A sensor library never knows telemetry
exists**: it keeps its `getLatest()` and nothing more (requirements, "What
already fits"). The telemetry library depends on the sensor libraries, not
the reverse.

## 4. The protocol

`python/heron/link/wire.py` remains the reference; every message below is
added to it, with a constructor, before any client reads it.

### 4.1 On connect

The server sends, in order: `hello`, then each channel's opening messages,
then items as they arrive.

```json
{"type":"hello","version":1,"server":"heron-robot",
 "robot":{"rig":"bench","host":"heron-9d9324"},
 "steady_ns":123456789012,
 "channels":[
   {"name":"lidar","messages":["scan"],"max_hz":15,"frame":"robot"},
   {"name":"robot","messages":["robot"],"max_hz":10,"frame":"none"}],
 "device":{...},"health":{...},"modes":[...],"decodable":[1],
 "typical_mode":1,"port":"/dev/rplidar"}
```

- `version` is 1, and stays 1 while every change is an addition (D2). The
  viewer refuses any other, so a breaking change would need it bumped
  there too.
- `steady_ns` is the Pi's steady clock when the hello was written. Every
  item's times are on that clock (rule 4), so a client can tell how old an
  item is: its time against `steady_ns` plus the time since the hello
  arrived. They mean nothing as wall-clock times.
- `channels` is what this robot publishes. A client learns from it what
  exists, rather than from a fixed list.
- The last six fields belong to the `lidar` channel. A channel may add
  fields to the hello; these are the ones the existing viewer needs for its
  device panel and mode list (one mode, the one in use).

The `lidar` channel's opening message is a v1 `status` with `"scanning":
true`, which is what makes the viewer draw at once:

```json
{"type":"status","motor":true,"scanning":true,"mode":1,
 "detail":"robot: the LiDAR is the robot's; this view is read-only"}
```

### 4.2 Items

Every item a channel sends carries `channel` and `seq`, and its time on the
steady clock. The `scan` keeps every v1 field (`seq`, `mode`, `t_start_ns`,
`t_end_ns`, `total`, `max_gap_q6`, `frame`, `rpts`, `coverage_ok`,
`unobserved`, `dropped`) and adds `"channel":"lidar"`. It is the message
`lidar_gui_server` sends today, from the same encoder.

The `robot` item, 10 times a second:

```json
{"type":"robot","channel":"robot","seq":4182,"t_ns":123456789012,
 "mode":"DRIVING","link":"OK","throttle":0.40,
 "velocity":{"vx":0.31,"vy":0.0,"wz":-0.05},
 "wheels":[0.36,0.26,0.26,0.36],
 "motors":{"ok":true,"resets":0,"stop_reason":"none"},
 "loop":{"period_ns":20000000,"late_max_ns":412000,"overruns":0},
 "telemetry":{"clients":2,"skipped":17}}
```

- `velocity` is the stick command as fractions of full speed, not a
  measured speed: the robot measures none yet. The units say so in
  `wire.py`.
- `loop` covers the time since the previous item: the worst lateness of a
  20 ms cycle, and cycles that missed their slot entirely.
- `telemetry.skipped` counts items replaced before a client could take them
  (D5), summed over clients.

### 4.3 Channel state

A channel's source can fail or go quiet without the robot stopping (D10).
Each channel reports it, on connect and whenever it changes:

```json
{"type":"channel","channel":"lidar","state":"failed",
 "detail":"no answer from /dev/rplidar"}
```

`state` is `ok`, `waiting` (running, nothing yet: a LiDAR spinning up),
`failed`, or `off` (`--no-lidar`). For the `lidar` channel the server also
sends the v1 `error` and a `status` with `"scanning": false`, so the
existing viewer marks its plot NOT LIVE rather than freezing it as though
it were current.

### 4.4 Commands

| Command | Answer |
|---|---|
| `{"cmd":"hello"}`, or the viewer's `{"cmd":"refresh"}` | the hello and opening messages again |
| `{"cmd":"subscribe","channels":["robot"]}` | `{"type":"subscribed","channels":["robot"]}`; only those from then on. Unknown names are listed in an `error`. An empty list means none |
| `{"cmd":"subscribe","channels":"all"}` | back to every channel, the default (D4) |
| anything else (`motor`, `scan_start`, `scan_stop`, `reset`) | `{"type":"error","message":"read-only: <what was asked> is not offered by the robot","fatal":false}`, and for the LiDAR's v1 commands a `status` again, so the viewer's buttons show the true state |

A command line longer than 4 KB, or one that is not a JSON object, closes
that client: it is not speaking this protocol.

## 5. The server

| Aspect | Choice |
|---|---|
| Thread | One, started by `start()`, joined by `stop()`. It does all socket work and all encoding. The robot's threads never touch a socket |
| Listening | TCP 5555 by default, on every interface: the wired link and Wi-Fi alike. `SO_REUSEADDR`, so a restart does not wait for old connections to time out |
| Each pass | `poll()` the listener and every client, 10 ms timeout. Then ask each channel for a newer item, if its `max_hz` allows one; encode it once; put it in every subscribed client's slot for that channel, replacing an unsent one (counted as skipped). Then write what each client's socket will take |
| Sockets | Non-blocking. A client is written to only as far as its socket accepts; the rest of that message waits for the next pass. A client gets the next message only once the current one has gone, so no message is ever interleaved |
| Limits | 4 clients (D7). A client whose current message has not moved for 5 s is closed: it has stopped reading |
| A failed send or a closed socket | That client is closed and forgotten; nothing else changes |
| Start-up failure | The port in use (by `lidar_gui_server`, say): `start()` returns false with `lastError`; the robot program reports it and drives on (D10) |
| Health | `TelemetryHealth`: state, clients now, connections and refusals so far, items sent and skipped, bytes sent, `lastError`. Shaped like the other libraries' health |

The cost to the robot's own threads is what `getLatest()` already costs: a
shared pointer copy for a scan, a small struct copy under a short lock for a
snapshot.

## 6. Channels in C++

```cpp
namespace Heron::Telemetry {

/** One encoded item: its sequence number, and the line to send. */
struct Item {
  std::uint64_t seq{0};
  std::string line;            // one JSON object, no newline
};

/** One stream. The server calls these from its own thread only. */
struct Channel {
  std::string name;            // "lidar"
  std::vector<std::string> messages;   // the item types, for the hello
  double maxHz{10.0};
  std::string frame;           // "robot", "odom", "none"
  /** Fields this channel adds to the hello, as `"key":value,...`. */
  std::function<std::string()> helloFields;
  /** Messages sent once to each client after the hello. */
  std::function<std::vector<std::string>()> openingMessages;
  /** A newer item than lastSeq, or nothing. */
  std::function<std::optional<Item>(std::uint64_t lastSeq)> poll;
  /** The source's state, for the channel message (§4.3). */
  std::function<ChannelState()> state;
};

Channel lidarChannel(const Heron::Lidar::Lidar &lidar);
Channel robotChannel(const Published<RobotStatus> &status);

}  // namespace Heron::Telemetry
```

**`Published<T>`** is for state that no library owns: what the control loop
decided. The loop calls `publish(value)` each cycle; it copies under a
mutex that the server holds only for a copy, and it gives the value a
sequence number. Reading it is `getLatest()`, as on every library. So the
robot's own state is published exactly as a sensor's is.

**Lifetimes.** A channel holds references to its sources. The robot
program stops the server before it stops or destroys any of them; declaring
the server after its sources does that by itself.

## 7. In the robot program

`heron_teleop` is the robot program until a robot core exists
(`motor-design.md` §4). It gains:

```cpp
Heron::Lidar::Lidar lidar(lidarConfig);          // Express, /dev/rplidar
if (useLidar && !lidar.start()) {
  report(lidar.getHealth().lastError);            // drives on without it
}
Heron::Telemetry::Published<Heron::Telemetry::RobotStatus> robotStatus;
Heron::Telemetry::TelemetryServer telemetry(telemetryConfig);   // after its sources
telemetry.addChannel(Heron::Telemetry::lidarChannel(lidar));
telemetry.addChannel(Heron::Telemetry::robotChannel(robotStatus));
if (!telemetry.start()) {
  report(telemetry.getHealth().lastError);        // drives on without it
}

// each 20 ms, after deciding the wheels
robotStatus.publish({mode, r.link, throttle, v, wheels, motorsSnapshot, loopTiming,
                     telemetry.getHealth()});
```

- New options: `--no-lidar`, `--no-telemetry`, `--telemetry-port N`.
- Its status line adds the LiDAR's state and the number of clients.
- Start-up order: remote, motors, LiDAR (about 2 s to spin up; it does not
  delay driving, since `start()` returns once scanning has begun), then the
  server.
- The boot service (`heron-teleop@.service`) runs it unchanged, so the
  robot streams from power-on.

## 8. Clients

| Client | What it shows |
|---|---|
| `lidar_gui.py`, **unchanged** | The live scan in the robot frame, the device panel, and one scan mode. Its motor, scan, mode and reset buttons are answered "read-only", and the plot stays live. It ignores the `robot` channel |
| `telemetry_watch.py` (new, host) | Any channel as text, one line per item (`--channel robot`), or a summary that updates in place: each channel's rate, age and skipped count. Standard library only, through `wire.py`, so it runs from a bare checkout and reads any channel `wire.py` describes |
| `nc <pi> 5555` | The raw lines |

```
.venv/bin/python python/tools/lidar_gui.py --host <pi> --connect
python3 python/tools/telemetry_watch.py --host <pi> --channel robot
```

Over Wi-Fi, `<pi>` may be its mDNS name, `heron-<suffix>.local`, which
resolves to the Wi-Fi address.

## 9. Adding a sensor: the recipe

1. **The library** keeps the shape: a reader thread, `getLatest()` safe
   from any thread, a `seq`, its times on the steady clock, and a stated
   frame (requirements, rules 4 and 5).
2. **An encoder** in `heron_telemetry_wire`, `<Sensor>Messages.cpp`, with
   golden-message tests, and a `<sensor>Channel()` factory beside
   `lidarChannel()`.
3. **One line** in the robot program: `telemetry.addChannel(...)`.
4. **The message in `wire.py`**: a constructor and its units, so Python
   clients, `telemetry_watch.py` among them, read it.

Nothing changes in the server, the protocol's version, or any existing
client. Odometry is the first to follow it; it also needs the reading's own
timestamp, a gap the requirements list.

## 10. Considered, not chosen

- **OpenTelemetry.** Built for the health of services: metrics, traces and
  logs, batched and pushed to a collector every few seconds, stamped with
  wall-clock time. Live scans fit none of that, and its C++ SDK is a large
  dependency for a Pi 3. It may suit the robot's *health* later (loop
  timing, resets, on a Grafana dashboard), as a separate channel out.
- **Foxglove's WebSocket protocol and MCAP.** Open robotics standards, with
  a ready-made viewer. They would replace `wire.py` (D2) and the existing
  viewer. Worth a second look if a full dashboard is wanted; not now.
- **A server per sensor**, or extending `lidar_gui_server`: rejected by D1.

## 11. Tests

| Test | Covers |
|---|---|
| `telemetry_wire_test` (`ctest`) | `JsonWriter`: escaping, numbers, nesting. The command reader: valid commands, malformed lines, oversized lines. Encoders against golden lines: the scan's bytes match what `lidar_gui_server` sends today |
| `telemetry_server_test` (`ctest`) | A real server on the loopback interface with fake channels: hello and opening messages; items arrive; `subscribe`; refused commands; latest wins for a client that stops reading, while another keeps up; a stalled client closed after 5 s; the fifth client refused; disconnects; `max_hz` honoured; a port in use reported |
| Bench, with the boot service | The existing viewer live while driving with the remote; `telemetry_watch` on the `robot` channel; the loop's `late_max_ns` with no client, with two, with one frozen (Ctrl-Z) and one closed mid-scan; over Ethernet and Wi-Fi |

## 12. Left out of the first version

- **Authentication and encryption.** Anyone on the same network can watch.
  Read-only, so acceptable on a home network; open for anything wider.
- **Recording** a session to a file, for replay into the same clients.
- **Commands from a client.** A separate, later decision, and only ever
  through the safety arbiter (rule 2).
- **Discovery**: advertising the service over mDNS, so a client lists the
  robots on the network.
- **Mounting offsets** in the hello, from the profile's `[lidar]` and
  `[odometry]` sections once those exist.

## 13. Steps

1. This design, reviewed.
2. `heron_telemetry_wire`: `JsonWriter`, the command reader,
   `LidarMessages`; `lidar_gui_server` encodes through it, unchanged on the
   wire.
3. `heron_telemetry`: `Channel`, `Published<T>`, `TelemetryServer`, and
   their tests.
4. The robot program: LiDAR, server, `lidar` and `robot` channels;
   `telemetry_watch.py`; the bench check.
5. Odometry, by the recipe, when its workstream resumes.

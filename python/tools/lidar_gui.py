"""Live LiDAR viewer -- host-side GUI for the Pi's lidar_server.

    .venv/bin/python python/tools/lidar_gui.py --host <pi>

Start the server on the Pi first, in another terminal:

    scripts/deploy_pi.sh cpane@<pi> server/lidar_server.py

Host-side only. Tkinter is in the standard library, so this adds nothing to
requirements-host.txt, and it lives in tools/ rather than probes/ because it
never runs on the target.

Two things about this display that are easy to get wrong
--------------------------------------------------------
**Orientation depends on the server.** With the Python server, 0 degrees is
drawn straight up and the angle increases clockwise: the sensor's own
convention as measured on this unit (docs/rplidar-a1m8-findings.md), which is
*left-handed*. With the C++ server (cpp/tools/LidarGuiServer.cpp) scans arrive
already in the robot frame, REP 103: 0 still points up, but bearings increase
anticlockwise, so positive is to the LEFT. The caption under the plot always
says which frame is on screen. The C++ server also sends coverage and the
unobserved arcs, which are shaded on the plot.

**The range filter is a filter, not a zoom.** Points beyond it are not drawn
*and* the plot rescales so the kept region fills the canvas. That is the point
of it -- pull the range in to 30 cm and a nearby object stops being three
pixels near the origin.
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import socket
import sys
import threading
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import ttk

# python/tools/lidar_gui.py -> python/, so the heron package imports with
# no packaging step and no dependence on the caller's working directory.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from heron.link import wire  # noqa: E402

# -- look -------------------------------------------------------------------

BG = "#0e1116"
GRID = "#243040"
GRID_TEXT = "#5c7086"
POINT = "#39d3a0"
POINT_NEAR = "#f2c14e"
#: Stale points are drawn in this instead. It has to read as obviously dead,
#: because the whole failure it fixes was a frozen plot passing for a live one.
POINT_STALE = "#43535f"
#: Directions the sensor did not look at in this revolution (C++ server only).
UNOBSERVED = "#3a2230"
STALE_TEXT = "#f2c14e"
AXIS = "#3c5068"
FRONT = "#e06c75"

#: Redraw at most this often. The sensor produces revolutions at ~7.3 Hz; a
#: faster poll only burns CPU, a slower one looks laggy when you move a hand
#: through the beam.
POLL_MS = 30

RANGE_PRESETS_CM = [15, 30, 50, 100, 200, 500, 1200]


@dataclass
class Frame:
    """The most recent revolution, already converted out of fixed point."""

    seq: int = 0
    mode: int = 0
    #: (bearing_deg, distance_mm, quality)
    points: list[tuple[float, float, int]] = field(default_factory=list)
    total: int = 0
    max_gap_deg: float = 0.0
    span_s: float = 0.0
    #: True when the server sent robot-frame points (positive bearing = left).
    robot: bool = False
    #: Unobserved arcs as (start_deg, width_deg), anticlockwise, robot frame.
    unobserved: list[tuple[float, float]] = field(default_factory=list)
    #: None when the server does not report coverage (the Python server).
    coverage_ok: bool | None = None
    dropped: int = 0


class Link(threading.Thread):
    """Socket reader. Parses on this thread, touches no widgets.

    Tkinter is not thread-safe, so everything this produces goes through a
    queue that the UI drains from the main loop.
    """

    def __init__(self, host: str, port: int, inbox: queue.Queue) -> None:
        super().__init__(name="lidar-link", daemon=True)
        self.host = host
        self.port = port
        self.inbox = inbox
        self.sock: socket.socket | None = None
        self._stop = threading.Event()
        self._send_lock = threading.Lock()

    def run(self) -> None:
        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=5.0)
            self.sock.settimeout(None)
        except OSError as exc:
            self.inbox.put({"type": "_link", "state": "failed", "detail": str(exc)})
            return

        self.inbox.put({"type": "_link", "state": "up", "detail": f"{self.host}:{self.port}"})
        reader = wire.LineReader()
        try:
            while not self._stop.is_set():
                chunk = self.sock.recv(262144)
                if not chunk:
                    break
                for message in reader.feed(chunk):
                    self.inbox.put(message)
        except (OSError, wire.WireError) as exc:
            if not self._stop.is_set():
                self.inbox.put({"type": "_link", "state": "lost", "detail": str(exc)})
                return
        self.inbox.put({"type": "_link", "state": "down", "detail": "closed"})

    def send(self, message: dict) -> None:
        """Safe to call from the UI thread; failure is reported, not raised."""
        if self.sock is None:
            return
        with self._send_lock:
            try:
                self.sock.sendall(wire.encode(message))
            except OSError as exc:
                self.inbox.put({"type": "_link", "state": "lost", "detail": str(exc)})

    def close(self) -> None:
        self._stop.set()
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass


class App:
    def __init__(self, root: tk.Tk, host: str, port: int) -> None:
        self.root = root
        self.inbox: queue.Queue = queue.Queue()
        self.link: Link | None = None

        self.frame = Frame()
        self.modes: list[dict] = []
        self.decodable: set[int] = set()
        self.scanning = False
        self.motor_on = False
        self.raw_note = ""
        self.raw_stats: dict | None = None
        #: True when the points on screen are left over from a stream that has
        #: since stopped. They are kept rather than cleared -- the last
        #: revolution is worth looking at -- but they must not look live.
        self.stale = False

        root.title("Heron -- LiDAR viewer")
        root.configure(bg=BG)
        root.geometry("1180x760")
        root.minsize(900, 600)

        self._build_top(host, port)
        self._build_body()

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(POLL_MS, self._pump)

    # -- layout ------------------------------------------------------------

    def _build_top(self, host: str, port: int) -> None:
        bar = ttk.Frame(self.root, padding=(10, 8))
        bar.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(bar, text="Server").pack(side=tk.LEFT)
        self.host_var = tk.StringVar(value=host)
        ttk.Entry(bar, textvariable=self.host_var, width=16).pack(side=tk.LEFT, padx=(6, 2))
        ttk.Label(bar, text=":").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=str(port))
        ttk.Entry(bar, textvariable=self.port_var, width=6).pack(side=tk.LEFT, padx=(2, 8))

        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_link)
        self.connect_btn.pack(side=tk.LEFT)

        self.link_lbl = ttk.Label(bar, text="disconnected")
        self.link_lbl.pack(side=tk.LEFT, padx=12)

    def _build_body(self) -> None:
        body = ttk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True)

        side = ttk.Frame(body, padding=(10, 4))
        side.pack(side=tk.LEFT, fill=tk.Y)
        side.pack_propagate(False)
        side.configure(width=310)

        # -- device ---------------------------------------------------------
        dev = ttk.LabelFrame(side, text="Device", padding=8)
        dev.pack(fill=tk.X, pady=(0, 8))
        self.dev_lbl = ttk.Label(dev, text="not connected", justify=tk.LEFT)
        self.dev_lbl.pack(anchor="w")

        # -- control --------------------------------------------------------
        ctl = ttk.LabelFrame(side, text="Control", padding=8)
        ctl.pack(fill=tk.X, pady=(0, 8))

        self.motor_var = tk.BooleanVar(value=False)
        self.motor_chk = ttk.Checkbutton(
            ctl, text="Motor", variable=self.motor_var, command=self.on_motor
        )
        self.motor_chk.pack(anchor="w")

        ttk.Label(ctl, text="Scan mode").pack(anchor="w", pady=(8, 2))
        self.mode_var = tk.StringVar()
        self.mode_box = ttk.Combobox(
            ctl, textvariable=self.mode_var, state="readonly", width=34
        )
        self.mode_box.pack(anchor="w")
        self.mode_box.bind("<<ComboboxSelected>>", self.on_mode)

        self.mode_note = ttk.Label(ctl, text="", wraplength=280, justify=tk.LEFT)
        self.mode_note.pack(anchor="w", pady=(4, 0))

        btns = ttk.Frame(ctl)
        btns.pack(fill=tk.X, pady=(10, 0))
        self.scan_btn = ttk.Button(btns, text="Start scan", command=self.on_scan)
        self.scan_btn.pack(side=tk.LEFT)
        ttk.Button(btns, text="Reset", command=self.on_reset).pack(side=tk.LEFT, padx=6)

        # -- range ----------------------------------------------------------
        rng = ttk.LabelFrame(side, text="Range filter", padding=8)
        rng.pack(fill=tk.X, pady=(0, 8))

        self.range_var = tk.IntVar(value=100)
        self.range_lbl = ttk.Label(rng, text="")
        self.range_lbl.pack(anchor="w")
        # The initial set() must happen before the command is wired, or Tk
        # fires on_range() while this method is still building the widget and
        # self.range_scale does not exist yet.
        self.range_scale = ttk.Scale(rng, from_=10, to=1200, orient=tk.HORIZONTAL)
        self.range_scale.set(100)
        self.range_scale.configure(command=lambda _v: self.on_range())
        self.range_scale.pack(fill=tk.X)

        # Grid, not a single packed row: seven buttons do not fit the side
        # panel's width and the tail of them was being clipped off-screen.
        presets = ttk.Frame(rng)
        presets.pack(fill=tk.X, pady=(6, 0))
        # Three per row, not four: sticky="ew" makes the grid column width win
        # over the button's requested width, and four columns inside a 310 px
        # panel truncate "15cm" to "15cr".
        per_row = 3
        for index, cm in enumerate(RANGE_PRESETS_CM):
            text = f"{cm}cm" if cm < 100 else f"{cm // 100}m"
            ttk.Button(
                presets, text=text, width=7,
                command=lambda c=cm: self.set_range(c),
            ).grid(row=index // per_row, column=index % per_row, padx=1, pady=1, sticky="ew")
        for column in range(per_row):
            presets.columnconfigure(column, weight=1)

        # -- readouts -------------------------------------------------------
        stat = ttk.LabelFrame(side, text="Stream", padding=8)
        stat.pack(fill=tk.X)
        self.stat_lbl = ttk.Label(stat, text="idle", justify=tk.LEFT)
        self.stat_lbl.pack(anchor="w")

        # -- canvas ---------------------------------------------------------
        self.canvas = tk.Canvas(body, bg=BG, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10), pady=(4, 10))
        self.canvas.bind("<Configure>", lambda _e: self.redraw())

        self.on_range()

    # -- connection --------------------------------------------------------

    def toggle_link(self) -> None:
        if self.link is not None:
            self.teardown_link("disconnected by user")
            return
        try:
            port = int(self.port_var.get())
        except ValueError:
            self.link_lbl.configure(text="port must be a number")
            return
        self.link_lbl.configure(text="connecting...")
        self.link = Link(self.host_var.get().strip(), port, self.inbox)
        self.link.start()

    def teardown_link(self, why: str) -> None:
        if self.link is not None:
            self.link.close()
            self.link = None
        self.scanning = False
        self.motor_on = False
        self.motor_var.set(False)
        self.frame = Frame()
        self.raw_stats = None
        self.stale = False
        self.connect_btn.configure(text="Connect")
        self.link_lbl.configure(text=why)
        self.scan_btn.configure(text="Start scan")
        self.dev_lbl.configure(text="not connected")
        self.redraw()

    def send(self, message: dict) -> None:
        if self.link is None:
            self.link_lbl.configure(text="not connected")
            return
        self.link.send(message)

    # -- commands ----------------------------------------------------------

    def on_motor(self) -> None:
        self.send({"cmd": wire.CMD_MOTOR, "on": bool(self.motor_var.get())})

    def on_scan(self) -> None:
        if self.scanning:
            self.send({"cmd": wire.CMD_SCAN_STOP})
        else:
            mode = self.selected_mode()
            if mode is None:
                return
            self.send({"cmd": wire.CMD_SCAN_START, "mode": mode})

    def on_reset(self) -> None:
        self.send({"cmd": wire.CMD_RESET})

    def on_mode(self, _event=None) -> None:
        self.update_mode_note()
        # Switching mode mid-stream restarts it in the new mode. The whole
        # point of this tool is comparing modes, so making that a three-click
        # stop/change/start would be a poor trade.
        if self.scanning:
            mode = self.selected_mode()
            if mode is not None:
                self.send({"cmd": wire.CMD_SCAN_START, "mode": mode})

    def selected_mode(self) -> int | None:
        label = self.mode_var.get()
        if not label:
            return None
        try:
            return int(label.split(":", 1)[0].strip())
        except ValueError:
            return None

    def update_mode_note(self) -> None:
        mode = self.selected_mode()
        if mode is None:
            self.mode_note.configure(text="")
            return
        entry = next((m for m in self.modes if int(m["id"]) == mode), None)
        if entry is None:
            self.mode_note.configure(text="")
            return
        if mode in self.decodable:
            self.mode_note.configure(
                text=f"{entry['us_per_sample']:.0f} us/sample, "
                     f"max {entry['max_distance_m']:.0f} m, {entry['kind']}"
            )
        else:
            self.mode_note.configure(
                text=f"{entry['kind']} -- this build cannot turn these frames "
                     "into points. The stream still starts, and the Stream "
                     "panel reports its byte and frame rate."
            )

    def set_range(self, cm: int) -> None:
        self.range_scale.set(cm)
        self.on_range()

    def on_range(self) -> None:
        cm = int(float(self.range_scale.get()))
        self.range_var.set(cm)
        shown = f"{cm} cm" if cm < 100 else f"{cm / 100:.2f} m"
        self.range_lbl.configure(text=f"show returns within {shown}")
        self.redraw()

    # -- inbound -----------------------------------------------------------

    def _pump(self) -> None:
        """Drain the queue, then redraw once.

        Draining without redrawing per message matters: if the UI thread
        stalls, several revolutions can pile up, and rendering every one of
        them would make the backlog worse rather than better. Only the newest
        is worth drawing.
        """
        dirty = False
        try:
            while True:
                message = self.inbox.get_nowait()
                dirty |= self._handle(message)
        except queue.Empty:
            pass
        if dirty:
            self.redraw()
        self.root.after(POLL_MS, self._pump)

    def _handle(self, message: dict) -> bool:
        kind = message.get("type")

        if kind == "_link":
            state = message.get("state")
            if state == "up":
                self.connect_btn.configure(text="Disconnect")
                self.link_lbl.configure(text=f"connected to {message['detail']}")
            elif state == "failed":
                self.teardown_link(f"connect failed: {message['detail']}")
            elif state in ("lost", "down"):
                self.teardown_link(f"link {state}: {message['detail']}")
            return True

        if kind == wire.MSG_HELLO:
            self._on_hello(message)
            return True

        if kind == wire.MSG_STATUS:
            self.scanning = bool(message.get("scanning"))
            self.motor_on = bool(message.get("motor"))
            self.motor_var.set(self.motor_on)
            self.scan_btn.configure(text="Stop scan" if self.scanning else "Start scan")
            detail = message.get("detail") or ""
            if detail:
                self.link_lbl.configure(text=detail)
            if not self.scanning:
                self.raw_stats = None
                # Whatever is on screen is now history. Say so rather than
                # letting it keep looking like the current state of the room.
                self.stale = bool(self.frame.points)
            return True

        if kind == wire.MSG_SCAN:
            self._on_scan(message)
            return True

        if kind == wire.MSG_RAW:
            self.raw_stats = message
            self.frame = Frame(seq=self.frame.seq + 1, mode=int(message.get("mode", 0)))
            return True

        if kind == wire.MSG_ERROR:
            self.link_lbl.configure(text=f"server: {message.get('message')}")
            return True

        return False

    def _on_hello(self, message: dict) -> None:
        if message.get("version") != wire.PROTOCOL_VERSION:
            self.teardown_link(
                f"protocol mismatch: server speaks v{message.get('version')}, "
                f"this client speaks v{wire.PROTOCOL_VERSION}"
            )
            return

        dev = message.get("device", {})
        health = message.get("health", {})
        self.dev_lbl.configure(
            text=(
                f"model    0x{dev.get('model', 0):02x}\n"
                f"firmware {dev.get('firmware', '?')}   hw {dev.get('hardware', '?')}\n"
                f"health   {health.get('state', '?')}\n"
                f"port     {message.get('port', '?')}"
            )
        )

        self.modes = message.get("modes", [])
        self.decodable = set(message.get("decodable", []))
        labels = []
        for entry in self.modes:
            mark = "" if int(entry["id"]) in self.decodable else "  [no decoder]"
            star = " *" if int(entry["id"]) == message.get("typical_mode") else ""
            labels.append(f"{entry['id']}: {entry['name']}{star}{mark}")
        self.mode_box.configure(values=labels)

        if labels and not self.mode_var.get():
            # Default to the server's announced mode when it can render it.
            # The Python server announces the device's own default, mode 3,
            # which it cannot decode, so fall back to the fastest mode that
            # renders; the C++ server announces its default, Express.
            typical = message.get("typical_mode")
            if typical in self.decodable:
                best = typical
            else:
                best = max(self.decodable) if self.decodable else 0
            for label in labels:
                if label.startswith(f"{best}:"):
                    self.mode_var.set(label)
                    break
        self.update_mode_note()

    def _on_scan(self, message: dict) -> None:
        points = []
        robot = "rpts" in message
        if robot:
            # Robot frame: [bearing in centidegrees, range in mm].
            for bearing_cdeg, range_mm in message["rpts"]:
                points.append((bearing_cdeg / 100.0, float(range_mm), 0))
        else:
            for angle_q6, dist_q2, quality in message.get("pts", []):
                points.append((wire.deg(angle_q6), wire.mm(dist_q2), quality))

        span_ns = int(message.get("t_end_ns", 0)) - int(message.get("t_start_ns", 0))
        self.frame = Frame(
            seq=int(message.get("seq", 0)),
            mode=int(message.get("mode", 0)),
            points=points,
            total=int(message.get("total", 0)),
            max_gap_deg=wire.deg(int(message.get("max_gap_q6", 0))),
            span_s=span_ns / 1e9 if span_ns > 0 else 0.0,
            robot=robot,
            unobserved=[(a / 100.0, w / 100.0) for a, w in message.get("unobserved", [])],
            coverage_ok=message.get("coverage_ok"),
            dropped=int(message.get("dropped", 0)),
        )
        self.raw_stats = None
        self.stale = False

    # -- drawing -----------------------------------------------------------

    def redraw(self) -> None:
        c = self.canvas
        c.delete("all")

        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        cx, cy = w / 2.0, h / 2.0
        radius_px = min(w, h) / 2.0 - 34.0
        limit_mm = self.range_var.get() * 10.0

        self._draw_grid(cx, cy, radius_px, limit_mm)
        self._draw_unobserved(cx, cy, radius_px)

        # The sensor frame grows clockwise on screen; the robot frame grows
        # anticlockwise (positive = left), which flips the sign of x.
        x_sign = -1.0 if self.frame.robot else 1.0
        kept = 0
        for bearing, dist_mm, _quality in self.frame.points:
            if dist_mm > limit_mm:
                continue
            kept += 1
            # 0 degrees up in both frames; x_sign sets the direction.
            theta = math.radians(bearing)
            r = (dist_mm / limit_mm) * radius_px
            x = cx + x_sign * r * math.sin(theta)
            y = cy - r * math.cos(theta)
            near = dist_mm < limit_mm * 0.25
            if self.stale:
                colour = POINT_STALE
                size = 2.0
            else:
                colour = POINT_NEAR if near else POINT
                size = 3.0 if near else 2.0
            c.create_oval(x - size, y - size, x + size, y + size,
                          fill=colour, outline="", tags="point")

        self._draw_sensor(cx, cy)
        if self.stale:
            self._draw_stale_banner()
        self._update_stats(kept)

        if self.raw_stats is not None:
            self._draw_raw_overlay(cx, cy, radius_px)

    def _draw_grid(self, cx: float, cy: float, radius_px: float, limit_mm: float) -> None:
        c = self.canvas

        # Four rings, labelled in whatever unit reads better at this range.
        for i in range(1, 5):
            frac = i / 4.0
            r = radius_px * frac
            c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=GRID, tags="grid")
            value_mm = limit_mm * frac
            label = (
                f"{value_mm / 10:.0f} cm" if value_mm < 1000 else f"{value_mm / 1000:.2f} m"
            )
            c.create_text(cx + 4, cy - r - 8, text=label, fill=GRID_TEXT,
                          anchor="w", font=("TkDefaultFont", 8))

        for deg_ in range(0, 360, 30):
            theta = math.radians(deg_)
            x = cx + radius_px * math.sin(theta)
            y = cy - radius_px * math.cos(theta)
            c.create_line(cx, cy, x, y, fill=GRID if deg_ % 90 else AXIS)

        if self.frame.robot:
            # Screen angle clockwise from up -> label. Left of the plot is +90.
            labels = ((0, "0 (front)"), (90, "-90 (right)"), (180, "180"), (270, "+90 (left)"))
            caption = "robot frame (REP 103)\n0 up, + is LEFT (anticlockwise)"
        else:
            labels = ((0, "0 (front)"), (90, "90"), (180, "180"), (270, "270"))
            caption = "sensor frame (left-handed)\n0 up, angle grows clockwise"
        for deg_, text in labels:
            theta = math.radians(deg_)
            x = cx + (radius_px + 18) * math.sin(theta)
            y = cy - (radius_px + 18) * math.cos(theta)
            c.create_text(x, y, text=text, fill=FRONT if deg_ == 0 else GRID_TEXT,
                          font=("TkDefaultFont", 8))

        # Bottom-left corner, in two short lines. Centred under the plot it
        # landed on the "180" label and could not be read, and any single
        # full-width line near the top or bottom crosses the 0 or 180 label.
        # Top-left is the stale banner's place.
        c.create_text(12, c.winfo_height() - 10, text=caption, anchor="sw", fill=GRID_TEXT,
                      font=("TkDefaultFont", 9))

    def _draw_unobserved(self, cx: float, cy: float, radius_px: float) -> None:
        """Shade the directions the sensor did not look at in this revolution.

        Robot frame only. Tk measures arc angles anticlockwise from 3 o'clock,
        and a robot bearing is anticlockwise from 12 o'clock, so add 90.
        """
        if not self.frame.robot or self.stale:
            return
        for start_deg, width_deg in self.frame.unobserved:
            self.canvas.create_arc(
                cx - radius_px, cy - radius_px, cx + radius_px, cy + radius_px,
                start=90.0 + start_deg, extent=width_deg,
                style=tk.PIESLICE, fill=UNOBSERVED, outline="", tags="unobserved",
            )

    def _draw_sensor(self, cx: float, cy: float) -> None:
        c = self.canvas
        c.create_line(cx - 6, cy, cx + 6, cy, fill=FRONT)
        c.create_line(cx, cy - 6, cx, cy + 6, fill=FRONT)

    def _draw_stale_banner(self) -> None:
        """Unmissable, because the bug this fixes was being quietly misled.

        The motor stopped, the plot froze, and an object that had physically
        been removed stayed on screen looking current.
        """
        # Top-left, not centred above the plot: the 0 degree label and the
        # outermost ring label both live up there, and the banner was landing
        # on top of them.
        self.canvas.create_text(
            12, 12, anchor="nw",
            text="NOT LIVE -- stream stopped",
            fill=STALE_TEXT, font=("TkDefaultFont", 11, "bold"),
        )

    def _draw_raw_overlay(self, cx: float, cy: float, radius_px: float) -> None:
        stats = self.raw_stats or {}
        elapsed = float(stats.get("elapsed_s", 0.0)) or 1e-9
        lines = [
            f"mode {stats.get('mode')} streams 0x{int(stats.get('ans_type', 0)):02x}",
            "no decoder in this build -- no points to draw",
            "",
            f"{stats.get('frames', 0)} framed  ({stats.get('frame_bytes', 0)} B each)",
            f"{float(stats.get('bytes_read', 0)) / elapsed / 1024:.1f} KiB/s",
            f"{float(stats.get('frames', 0)) / elapsed:.0f} frames/s",
        ]
        self.canvas.create_text(
            cx, cy - radius_px / 2, text="\n".join(lines),
            fill=GRID_TEXT, justify=tk.CENTER, font=("TkFixedFont", 10),
        )

    def _update_stats(self, kept: int) -> None:
        if self.raw_stats is not None:
            stats = self.raw_stats
            elapsed = float(stats.get("elapsed_s", 0.0)) or 1e-9
            self.stat_lbl.configure(
                text=(
                    f"mode      {stats.get('mode')}  (undecoded)\n"
                    f"frames    {stats.get('frames', 0)}\n"
                    f"rate      {float(stats.get('frames', 0)) / elapsed:.0f}/s\n"
                    f"bytes     {float(stats.get('bytes_read', 0)) / elapsed / 1024:.1f} KiB/s"
                )
            )
            return

        f = self.frame
        if not self.scanning and not f.points:
            self.stat_lbl.configure(text="idle")
            return

        rate = 1.0 / f.span_s if f.span_s > 0 else 0.0
        valid = len(f.points)
        invalid = max(0, f.total - valid)
        invalid_pct = (100.0 * invalid / f.total) if f.total else 0.0

        prefix = "STOPPED -- not live\n\n" if self.stale else ""
        extra = ""
        if f.coverage_ok is not None:
            extra = (
                f"\ncoverage   {'ok' if f.coverage_ok else 'NOT OK'}"
                f"\nunobserved {len(f.unobserved)} arc(s)"
                f"\ndropped    ~{f.dropped} samples"
            )
        self.stat_lbl.configure(
            text=prefix + (
                f"frame      {'robot, REP 103 (+ = left)' if f.robot else 'sensor (CW)'}\n"
                f"rev        #{f.seq}\n"
                f"shown      {kept} of {valid} valid\n"
                f"samples    {f.total} in the revolution\n"
                f"invalid    {invalid} ({invalid_pct:.0f}%)\n"
                f"sample gap {f.max_gap_deg:.1f} deg\n"
                f"rev rate   {rate:.1f} Hz"
            ) + extra
        )

    # -- teardown ----------------------------------------------------------

    def on_close(self) -> None:
        if self.link is not None:
            # Leave the sensor idle; the server also does this on disconnect,
            # but asking politely first means the motor stops before the
            # socket does.
            self.send({"cmd": wire.CMD_SCAN_STOP})
            self.send({"cmd": wire.CMD_MOTOR, "on": False})
            self.link.close()
        self.root.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="", help="server address (the Pi's wired link)")
    parser.add_argument("--port", type=int, default=wire.DEFAULT_TCP_PORT)
    parser.add_argument(
        "--connect", action="store_true", help="connect immediately on launch"
    )
    args = parser.parse_args()
    if args.connect and not args.host:
        parser.error("--connect needs --host")

    root = tk.Tk()
    app = App(root, args.host, args.port)
    if args.connect:
        root.after(100, app.toggle_link)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Drone Desk: camera, spatial maps, telemetry and CSV flight in one window."""

from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
from datetime import datetime, timedelta, timezone
import json
import logging
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

import cv2
import numpy as np
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from drone_app import REPO_ROOT
from drone_app.maps import StartMap, TagPlot, draw_tag
from drone_app.session import DroneSession, Route
from protocol import CMD_HOLD, CMD_LAND

BG, PANEL, EDGE = "#0b1220", "#111c2e", "#243247"
TEXT, MUTED, ACCENT, RED = "#e7edf6", "#94a3b8", "#54d5c4", "#f27686"
SGT = timezone(timedelta(hours=8))
log = logging.getLogger(__name__)


class QueueLogHandler(logging.Handler):
    def __init__(self, output):
        super().__init__()
        self.output = output

    def emit(self, record):
        try:
            stamp = datetime.fromtimestamp(record.created, SGT).strftime("%H:%M:%S")
            self.output.put_nowait((record.levelname, f"{stamp}  {record.getMessage()}"))
        except queue.Full:
            pass


class DroneDesk:
    def __init__(self, root, *, drone_id=4, esp_ip="192.168.1.204", demo=False,
                 config=None, smoke_test=False):
        self.root, self.demo_mode = root, demo
        self.config = config
        self.smoke_test = smoke_test
        self.session = None
        self.route = None
        self.last_image = None
        self.last_frame_number = -1
        self.photo = None
        self.disconnecting = False
        self.closing = False
        self.destroyed = False
        self.display_errors = []
        self._disconnect_finished = threading.Event()
        self._disconnect_error = None
        self._log_queue = queue.Queue(maxsize=2000)
        self._log_handler = QueueLogHandler(self._log_queue)
        logging.getLogger().addHandler(self._log_handler)
        logging.getLogger().setLevel(logging.INFO)
        self.drone_id = tk.StringVar(value=str(drone_id))
        self.esp_ip = tk.StringVar(value=esp_ip)
        self.route_name = tk.StringVar(value="ellipse_min_snap_traj.csv")
        self.interval = tk.StringVar(value="0.05")
        self.takeoff_wait = tk.StringVar(value="12")
        self.takeoff_first = tk.BooleanVar(value=True)
        self.finish = tk.StringVar(value="hold")
        self.link_text = tk.StringVar(value="OFFLINE")
        self.stage_text = tk.StringVar(value="Connect to begin")
        self.camera_text = tk.StringVar(value="Camera offline")
        self.route_text = tk.StringVar(value="Choose a CSV to preview")
        self.fields = {name: tk.StringVar(value="—") for name in
                       ("Navigation", "Position N / E", "Heading", "Height", "Armed / mode", "Telemetry")}
        self._build()
        self._load_route()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.bind("<Control-s>", lambda event: self._save_session())
        root.after(100, self._poll)
        if demo:
            root.after(150, self._connect)

    def _build(self):
        root = self.root
        root.title("Drone Desk" + (" — DEMO" if self.demo_mode else ""))
        root.configure(bg=BG)
        width = max(1120, min(1600, root.winfo_screenwidth() - 50))
        height = max(720, min(1000, root.winfo_screenheight() - 70))
        root.geometry(f"{width}x{height}")
        root.minsize(1120, 720)
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=TEXT, font=("Segoe UI", 10))
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=PANEL)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("Muted.TLabel", foreground=MUTED)
        style.configure("Title.TLabel", font=("Segoe UI", 23, "bold"))
        style.configure("Section.TLabel", font=("Segoe UI", 11, "bold"))
        style.configure("Accent.TLabel", foreground=ACCENT, font=("Segoe UI", 10, "bold"))
        style.configure("TButton", padding=(10, 7), background=EDGE, borderwidth=0)
        style.map("TButton", background=[("active", "#34475e")], foreground=[("disabled", "#65758c")])
        style.configure("Accent.TButton", background=ACCENT, foreground=BG)
        style.map("Accent.TButton", background=[("active", "#8ce8dc"), ("disabled", EDGE)])
        style.configure("Danger.TButton", background="#793b4b", foreground="white")
        style.configure("TEntry", fieldbackground=PANEL, foreground=TEXT, padding=6)
        style.configure("TCombobox", fieldbackground=PANEL, background=EDGE, foreground=TEXT, padding=5)
        style.map("TCombobox", fieldbackground=[("readonly", PANEL)], foreground=[("readonly", TEXT)])
        style.configure("TCheckbutton", background=BG, foreground=TEXT)
        style.configure("Treeview", background=PANEL, fieldbackground=PANEL, foreground=TEXT,
                        rowheight=25, borderwidth=0)
        style.configure("Treeview.Heading", background=EDGE, foreground=TEXT, padding=5)
        style.configure("TPanedwindow", background=BG)

        header = ttk.Frame(root, padding=(20, 14))
        header.pack(fill="x")
        ttk.Label(header, text="DRONE DESK", style="Title.TLabel").pack(side="left")
        subtitle = "DEMO · simulated data · flight controls disabled" if self.demo_mode else "Camera · telemetry · AprilTag mapping"
        ttk.Label(header, text=subtitle, style="Muted.TLabel").pack(side="left", padx=20)
        ttk.Label(header, textvariable=self.link_text, style="Accent.TLabel").pack(side="right")

        body = ttk.Frame(root, padding=(16, 0, 16, 12))
        body.pack(fill="both", expand=True)
        sidebar = ttk.Frame(body, width=285)
        sidebar.pack(side="left", fill="y", padx=(0, 15))
        sidebar.pack_propagate(False)
        actions = ttk.Frame(sidebar)
        actions.pack(side="bottom", fill="x", pady=(8, 0))
        form_area = ttk.Frame(sidebar)
        form_area.pack(side="top", fill="both", expand=True)
        scroll_canvas = tk.Canvas(form_area, bg=BG, highlightthickness=0, width=265)
        scroll = ttk.Scrollbar(form_area, orient="vertical", command=scroll_canvas.yview)
        scroll.pack(side="right", fill="y")
        scroll_canvas.pack(side="left", fill="both", expand=True)
        scroll_canvas.configure(yscrollcommand=scroll.set)
        side = ttk.Frame(scroll_canvas)
        form_window = scroll_canvas.create_window((0, 0), window=side, anchor="nw")
        side.bind("<Configure>", lambda event: scroll_canvas.configure(scrollregion=scroll_canvas.bbox("all")))
        scroll_canvas.bind("<Configure>", lambda event: scroll_canvas.itemconfigure(form_window, width=event.width))
        self.connection_widgets = []
        self.route_widgets = []
        ttk.Label(side, text="CONNECTION", style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        for label, variable in (("Drone ID", self.drone_id), ("Drone IP", self.esp_ip)):
            ttk.Label(side, text=label, style="Muted.TLabel").pack(anchor="w")
            entry = ttk.Entry(side, textvariable=variable)
            entry.pack(fill="x", pady=(3, 8))
            self.connection_widgets.append(entry)
        self.connect_button = ttk.Button(side, text="Start demo" if self.demo_mode else "Connect",
                                         style="Accent.TButton", command=self._connect)
        self.connect_button.pack(fill="x", pady=(0, 16))

        ttk.Label(side, text="TRAJECTORY", style="Section.TLabel").pack(anchor="w")
        names = sorted(p.name for p in (REPO_ROOT / "trajectory").glob("*.csv"))
        picker = ttk.Combobox(side, textvariable=self.route_name, values=names)
        picker.pack(fill="x", pady=(8, 6))
        self.route_widgets.append((picker, "normal"))
        picker.bind("<<ComboboxSelected>>", lambda event: self._load_route())
        browse = ttk.Button(side, text="Browse CSV…", command=self._browse)
        browse.pack(fill="x")
        self.route_widgets.append((browse, "normal"))
        row = ttk.Frame(side)
        row.pack(fill="x", pady=(10, 3))
        ttk.Label(row, text="Sample interval (s)", style="Muted.TLabel").pack(side="left")
        interval = ttk.Combobox(row, width=7, textvariable=self.interval,
                                values=("0.05", "0.10", "0.20"))
        interval.pack(side="right")
        self.route_widgets.append((interval, "normal"))
        interval.bind("<<ComboboxSelected>>", lambda event: self._load_route())
        ttk.Label(side, text="0.20 s uploads fewer points", style="Muted.TLabel").pack(anchor="w")
        preview = ttk.Button(side, text="Preview / validate", command=self._load_route)
        preview.pack(fill="x", pady=(8, 6))
        self.route_widgets.append((preview, "normal"))
        ttk.Label(side, textvariable=self.route_text, wraplength=265, justify="left").pack(anchor="w", pady=(0, 12))
        takeoff_check = ttk.Checkbutton(side, text="Take off before trajectory", variable=self.takeoff_first)
        takeoff_check.pack(anchor="w")
        self.route_widgets.append((takeoff_check, "normal"))
        row = ttk.Frame(side)
        row.pack(fill="x", pady=5)
        ttk.Label(row, text="Takeoff wait (s)", style="Muted.TLabel").pack(side="left")
        wait_entry = ttk.Entry(row, textvariable=self.takeoff_wait, width=8)
        wait_entry.pack(side="right")
        self.route_widgets.append((wait_entry, "normal"))
        row = ttk.Frame(side)
        row.pack(fill="x", pady=(0, 8))
        ttk.Label(row, text="After trajectory", style="Muted.TLabel").pack(side="left")
        finish_picker = ttk.Combobox(row, textvariable=self.finish, values=("hold", "land"), width=8,
                                     state="readonly")
        finish_picker.pack(side="right")
        self.route_widgets.append((finish_picker, "readonly"))
        self.fly_button = ttk.Button(actions, text="Fly trajectory", style="Accent.TButton",
                                     command=lambda: self._launch(False))
        self.fly_button.pack(fill="x", pady=(3, 6))
        self.takeoff_button = ttk.Button(actions, text="Take off only", command=lambda: self._launch(True))
        self.takeoff_button.pack(fill="x")
        row = ttk.Frame(actions)
        row.pack(fill="x", pady=8)
        self.hold_button = ttk.Button(row, text="HOLD", command=lambda: self._interrupt(CMD_HOLD))
        self.hold_button.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.land_button = ttk.Button(row, text="LAND", style="Danger.TButton", command=lambda: self._interrupt(CMD_LAND))
        self.land_button.pack(side="left", fill="x", expand=True, padx=(4, 0))
        ttk.Separator(actions).pack(fill="x", pady=10)
        ttk.Button(actions, text="Save maps + camera", command=self._save_session).pack(fill="x", pady=(0, 6))
        ttk.Button(actions, text="Reset map origin", command=self._reset_map).pack(fill="x")

        content = ttk.Panedwindow(body, orient="vertical")
        content.pack(side="left", fill="both", expand=True)
        self.content = content
        self.maps_expanded = False
        top = ttk.Frame(content)
        self.camera_panel = top
        content.add(top, weight=2)
        top.columnconfigure(0, weight=3)
        top.columnconfigure(1, weight=2)
        top.rowconfigure(1, weight=1)
        ttk.Label(top, textvariable=self.camera_text, style="Section.TLabel").grid(row=0, column=0, sticky="w", pady=(0, 5))
        self.camera = tk.Label(top, text="Connect to view the drone camera", bg=PANEL, fg=MUTED)
        self.camera.grid(row=1, column=0, sticky="nsew", padx=(0, 14))
        info = ttk.Frame(top)
        info.grid(row=0, column=1, rowspan=2, sticky="nsew")
        ttk.Label(info, text="DRONE STATUS", style="Section.TLabel").pack(anchor="w", pady=(0, 5))
        for label, variable in self.fields.items():
            row = ttk.Frame(info)
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=label, style="Muted.TLabel").pack(side="left")
            ttk.Label(row, textvariable=variable).pack(side="right")
        ttk.Label(info, text="VISIBLE TAGS", style="Section.TLabel").pack(anchor="w", pady=(8, 5))
        self.tags = ttk.Treeview(info, columns=("id", "distance", "forward", "right"), show="headings", height=2)
        for key, title in (("id", "ID"), ("distance", "Range m"), ("forward", "Fwd m"), ("right", "Right m")):
            self.tags.heading(key, text=title)
            self.tags.column(key, width=62, anchor="center", stretch=True)
        self.tags.pack(fill="both", expand=True)

        middle = ttk.Frame(content)
        content.add(middle, weight=7)
        map_header = ttk.Frame(middle)
        map_header.pack(fill="x", pady=(6, 0))
        ttk.Label(map_header, text="LIVE VIEW + PERMANENT MAP", style="Section.TLabel").pack(side="left")
        self.expand_maps_button = ttk.Button(map_header, text="Expand maps", command=self._toggle_maps)
        self.expand_maps_button.pack(side="right")
        ttk.Label(middle, text="Click a fixed-map tag for X / Y coordinates (m).", style="Muted.TLabel").pack(anchor="w")
        self.figure = Figure(figsize=(11, 5), dpi=100, layout="constrained", facecolor=BG)
        self.canvas = FigureCanvasTkAgg(self.figure, master=middle)
        self.plot = TagPlot(self.figure, show=False, process_events=False)
        self.figure.suptitle("")
        self.planned_line, = self.plot.map_ax.plot([], [], "--", color="#baa5ff", label="CSV preview", alpha=.8)
        self.plot.update_map_legend()
        self.plan_anchor = (0.0, 0.0)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, middle, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(fill="x")
        toolbar.configure(bg=BG)
        self._style_maps()

        lower = ttk.Frame(content)
        self.log_panel = lower
        content.add(lower, weight=1)
        ttk.Label(lower, textvariable=self.stage_text, style="Accent.TLabel").pack(anchor="w", pady=(5, 5))
        self.logs = scrolledtext.ScrolledText(lower, height=4, state="disabled", wrap="word",
                                             bg=PANEL, fg=MUTED, insertbackground=TEXT,
                                             relief="flat", font=("Consolas", 9))
        self.logs.pack(fill="both", expand=True)
        self.logs.tag_configure("ERROR", foreground=RED)
        self.logs.tag_configure("WARNING", foreground="#f4c879")
        self._set_buttons(False, False)
        root.after_idle(self._size_map_panes)

    def _size_map_panes(self):
        self.root.update_idletasks()
        height = self.content.winfo_height()
        self.content.sashpos(0, min(self.camera_panel.winfo_reqheight(), int(height * .42)))
        self.content.sashpos(1, height - 90)

    def _toggle_maps(self):
        if self.maps_expanded:
            self.content.insert(0, self.camera_panel, weight=2)
            self.content.add(self.log_panel, weight=1)
            self.root.after_idle(self._restore_map_panes)
        else:
            self.saved_map_sashes = (self.content.sashpos(0), self.content.sashpos(1))
            self.content.forget(self.camera_panel)
            self.content.forget(self.log_panel)
        self.maps_expanded = not self.maps_expanded
        self.expand_maps_button.configure(text="Restore layout" if self.maps_expanded else "Expand maps")

    def _restore_map_panes(self):
        for index, position in enumerate(self.saved_map_sashes):
            self.content.sashpos(index, position)

    def _style_maps(self):
        for ax in (self.plot.position_ax, self.plot.map_ax):
            ax.set_facecolor(PANEL)
            ax.tick_params(colors=MUTED, labelsize=8)
            ax.xaxis.label.set_color(MUTED)
            ax.yaxis.label.set_color(MUTED)
            ax.title.set_color(TEXT)
            ax.title.set_fontsize(10)
            for spine in ax.spines.values():
                spine.set_color(EDGE)
            legend = ax.get_legend()
            if legend:
                legend.get_frame().set_facecolor(PANEL)
                legend.get_frame().set_edgecolor(EDGE)
                for label in legend.get_texts():
                    label.set_color(TEXT)
        self.plot.position_ax.lines[0].set_color(TEXT)
        self.plot.drone_marker.set_color(TEXT)
        self.plot.tag_coordinates.set_color(TEXT)
        self.plot.tag_coordinates.get_bbox_patch().set_facecolor(PANEL)
        self.plot.tag_coordinates.get_bbox_patch().set_edgecolor(ACCENT)

    def _set_buttons(self, live, busy):
        for widget, normal_state in self.route_widgets:
            widget.configure(state="disabled" if busy or self.disconnecting else normal_state)
        can_fly = live and not busy and not self.demo_mode and not self.disconnecting
        for button in (self.takeoff_button, self.fly_button):
            button.configure(state="normal" if can_fly else "disabled")
        for button in (self.hold_button, self.land_button):
            button.configure(state="normal" if self.session and not self.demo_mode and not self.disconnecting else "disabled")

    def _browse(self):
        path = filedialog.askopenfilename(title="Choose trajectory", initialdir=REPO_ROOT / "trajectory",
                                           filetypes=(("Trajectory CSV", "*.csv"), ("All files", "*.*")))
        if path:
            self.route_name.set(path)
            self._load_route()

    def _load_route(self):
        try:
            path = Path(self.route_name.get())
            if not path.is_absolute():
                path = REPO_ROOT / "trajectory" / path
            self.route = Route.load(path, float(self.interval.get()))
            self.plan_anchor = (0.0, 0.0)
            z = self.route.points[:, 2]
            self.route_text.set(f"{self.route.duration:.1f} s  ·  {len(self.route.points)} points\n"
                                f"Peak {self.route.peak_speed:.3f} m/s\nZ offset {z.min():+.2f} to {z.max():+.2f} m")
            self._draw_route()
        except (OSError, ValueError) as exc:
            self.route = None
            self.route_text.set(str(exc))
            if hasattr(self, "planned_line"):
                self.planned_line.set_data([], [])
            log.error("Cannot load CSV: %s", exc)

    def _draw_route(self):
        if self.route is None:
            return
        heading = self.plot.map.origin[2] if self.plot.map.origin is not None else 0
        c, s = math.cos(heading), math.sin(heading)
        north, east = self.route.points[:, 0], self.route.points[:, 1]
        self.planned_line.set_data(-s * north + c * east + self.plan_anchor[1],
                                   c * north + s * east + self.plan_anchor[0])
        self.canvas.draw_idle()

    def _connect(self):
        if self.session is not None:
            self._begin_disconnect()
            return
        try:
            config = self.config or str(REPO_ROOT / "laptop" / "setup.yaml")
            session = DroneSession(int(self.drone_id.get()), self.esp_ip.get(),
                                   config=config, demo=self.demo_mode)
            session.start()
            self.session = session
            self.plot.map = StartMap()
            self.plot.current.clear()
            self.last_image = None
            self.last_frame_number = -1
            self.connect_button.configure(text="Disconnect")
            for widget in self.connection_widgets:
                widget.configure(state="disabled")
            self.stage_text.set("Demo running" if self.demo_mode else "Waiting for drone telemetry")
            self.camera_text.set("Waiting for camera")
            log.info("%s session for drone %d at %s", "Demo" if self.demo_mode else "Connected",
                     session.drone_id, session.esp_ip)
        except Exception as exc:
            log.error("Connection failed: %s", exc)
            messagebox.showerror("Connection failed", str(exc), parent=self.root)

    def _launch(self, takeoff_only):
        if self.session is None or self.demo_mode:
            return
        if not takeoff_only:
            self._load_route()
            if self.route is None:
                return
        takeoff = takeoff_only or self.takeoff_first.get()
        try:
            self.session.feedback.packet()
            wait = float(self.takeoff_wait.get())
            if not math.isfinite(wait) or not 0 <= wait < 40:
                raise ValueError("Takeoff wait must be between 0 and 40 seconds")
            detail = ("Take off and hover" if takeoff_only else
                      f"Fly {self.route.path.name}\n{self.route.duration:.1f} s, {len(self.route.points)} points\n"
                      f"Takeoff first: {'yes' if takeoff else 'no — start from hover'}\nFinish: {self.finish.get().upper()}")
            if not messagebox.askyesno("Confirm flight", f"Drone {self.session.drone_id} · {self.session.esp_ip}\n\n{detail}\n\nSend flight commands?",
                                       parent=self.root, default="no"):
                return
            if self.plot.map.origin is not None and self.plot.map.poses:
                self.plan_anchor = self.plot.map.relative_position(*self.plot.map.poses[-1][1][:2])
            self._draw_route()
            self.session.launch(None if takeoff_only else self.route, takeoff=takeoff,
                                takeoff_wait=wait, finish=self.finish.get())
            self.stage_text.set("Starting operation…")
            self._set_buttons(True, True)
        except Exception as exc:
            log.error("Cannot start operation: %s", exc)
            messagebox.showerror("Cannot start operation", str(exc), parent=self.root)

    def _interrupt(self, command):
        if self.session is None or self.demo_mode:
            return
        try:
            self.session.interrupt(command)
            log.info("%s requested for drone %d", "LAND" if command == CMD_LAND else "HOLD", self.session.drone_id)
        except Exception as exc:
            log.error("Command failed: %s", exc)
            messagebox.showerror("Command failed", str(exc), parent=self.root)

    def _reset_map(self):
        self.plot.map.reset()
        self.plan_anchor = (0.0, 0.0)
        self._draw_route()
        log.info("Map reset at the latest drone position")

    def _display_frame(self, bundle):
        frame, gray, tags, processed_s = bundle
        self.plot.update(tags, processed_s, frame_s=frame.first_rx - self.session.started - frame.age_ms / 1000)
        view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for det in tags:
            draw_tag(view, det, 1.0)
        self.last_image = view
        image = Image.fromarray(cv2.cvtColor(view, cv2.COLOR_BGR2RGB))
        image.thumbnail((max(240, self.camera.winfo_width()), max(180, self.camera.winfo_height())), Image.Resampling.LANCZOS)
        self.photo = ImageTk.PhotoImage(image)
        self.camera.configure(image=self.photo, text="")
        prefix = "DEMO CAMERA" if self.demo_mode else "LIVE CAMERA"
        self.camera_text.set(f"{prefix}  ·  frame {frame.frame_id}  ·  {len(tags)} tags")
        self.tags.delete(*self.tags.get_children())
        for tag_id, body in sorted(self.plot.current.items()):
            self.tags.insert("", "end", values=(tag_id, f"{math.dist((0, 0, 0), body):.2f}", f"{body[0]:+.2f}", f"{body[1]:+.2f}"))

    def _status(self, packet, age):
        self.fields["Navigation"].set(packet.nav_state_name)
        self.fields["Position N / E"].set(f"{packet.ned_x:+.2f} / {packet.ned_y:+.2f} m")
        self.fields["Heading"].set(f"{math.degrees(packet.heading_rad):.0f}°")
        self.fields["Telemetry"].set(f"{age:.1f} s old" + (" · STALE" if age > 1 else ""))
        if packet.flight_status_version == 1:
            if not packet.px4_link_ok:
                self.fields["Telemetry"].set(f"{age:.1f} s old · PX4 STALE")
            height = packet.ground_z - packet.local_z
            self.fields["Height"].set(f"{height:.2f} m" if math.isfinite(height) else "Unavailable")
            mode = "OFFBOARD" if packet.px4_custom_main_mode == 6 else f"Mode {packet.px4_custom_main_mode}"
            self.fields["Armed / mode"].set(f"{'Armed' if packet.px4_armed else 'Disarmed'} · {mode}")
        else:
            self.fields["Height"].set("Unavailable")
            self.fields["Armed / mode"].set("Unavailable · legacy")

    def _poll(self):
        if self.destroyed:
            return
        try:
            if self.session is not None and not self.disconnecting:
                packet, received, samples, number, bundle = self.session.snapshot()
                for sample, elapsed in samples:
                    self.plot.map.update_pose(sample, elapsed)
                now = time.monotonic()
                live = received is not None and now - received <= 1
                self.link_text.set("DEMO" if self.demo_mode else ("LIVE" if live else "WAITING / STALE"))
                if packet is not None:
                    self._status(packet, now - received)
                if bundle is not None and number != self.last_frame_number:
                    self.last_frame_number = number
                    self._display_frame(bundle)
                if bundle is not None and now - self.session.started - bundle[3] > 1:
                    self.camera_text.set("Camera stalled — last image retained")
                self.plot.refresh(now - self.session.started)
                self._draw_route()
                control_ready = live and (packet.flight_status_version != 1 or packet.px4_link_ok)
                self._set_buttons(control_ready, self.session.busy)
                for _ in range(100):
                    try:
                        kind, value = self.session.events.get_nowait()
                    except queue.Empty:
                        break
                    if kind == "stage":
                        self.stage_text.set(value)
                    elif kind == "operation_done":
                        if value == "takeoff":
                            self.takeoff_first.set(False)
                        self.stage_text.set("Takeoff wait complete; holding" if value == "takeoff" else "Flight operation complete")
                    elif kind == "operation_error":
                        self.stage_text.set(f"Operation failed: {value}")
                    elif kind == "camera_status":
                        self.camera_text.set(value)
            if self.disconnecting and self._disconnect_finished.is_set():
                self._finish_disconnect()
                if self.destroyed:
                    return
            for _ in range(100):
                try:
                    level, text = self._log_queue.get_nowait()
                except queue.Empty:
                    break
                self.logs.configure(state="normal")
                self.logs.insert("end", text + "\n", level)
                if int(self.logs.index("end-1c").split(".")[0]) > 1000:
                    self.logs.delete("1.0", "100.0")
                self.logs.see("end")
                self.logs.configure(state="disabled")
        except Exception as exc:
            self.display_errors = (self.display_errors + [str(exc)])[-20:]
            log.exception("Display update failed")
        if not self.destroyed:
            self.root.after(100, self._poll)

    def _save_session(self, quiet=False):
        if self.smoke_test:
            return
        if self.last_image is None and not self.plot.map.drone_path and not self.plot.map.tags:
            if not quiet:
                log.info("Nothing to save yet")
            return
        try:
            stamp = datetime.now(SGT).strftime("%Y%m%d_%H%M%S_%f")
            destination = Path(__file__).resolve().parent / "recordings" / stamp
            destination.mkdir(parents=True)
            self.figure.savefig(destination / "maps.png", dpi=150, facecolor=BG)
            if self.last_image is not None:
                if not cv2.imwrite(str(destination / "camera.png"), self.last_image):
                    raise OSError("Camera image could not be written")
            data = {"demo": self.demo_mode, "drone_id": self.drone_id.get(),
                    "drone_ip": self.esp_ip.get(), "origin": self.plot.map.origin,
                    "tags_forward_right_m": self.plot.map.tags,
                    "path_forward_right_m": [list(point) if all(math.isfinite(v) for v in point) else None
                                              for point in self.plot.map.drone_path]}
            (destination / "map.json").write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
            (destination / "log.txt").write_text(self.logs.get("1.0", "end"), encoding="utf-8")
            log.info("Saved camera, maps, coordinates and log to %s", destination)
            self.stage_text.set(f"Saved session: {destination.name}")
        except Exception as exc:
            log.error("Save failed: %s", exc)
            if not quiet:
                messagebox.showerror("Save failed", str(exc), parent=self.root)

    def _begin_disconnect(self):
        if self.session is None or self.disconnecting:
            return
        if self.session.ever_controlled and not self.demo_mode:
            choice = messagebox.askyesnocancel("Close flight session",
                "Request LAND before disconnecting?\n\nYes: LAND and disconnect\nNo: HOLD and disconnect\nCancel: keep the app connected",
                parent=self.root)
            if choice is None:
                self.closing = False
                return
            try:
                self.session.interrupt(CMD_LAND if choice else CMD_HOLD)
            except Exception as exc:
                log.error("Disconnect command failed: %s", exc)
                self.closing = False
                messagebox.showerror("Still connected", str(exc), parent=self.root)
                return
        self._save_session(quiet=True)
        self.disconnecting = True
        self.connect_button.configure(state="disabled")
        self._set_buttons(False, False)
        self.stage_text.set("Disconnecting…")
        self._disconnect_finished.clear()
        session = self.session

        def stop():
            try:
                session.close()
            except Exception as exc:
                self._disconnect_error = str(exc)
            finally:
                self._disconnect_finished.set()

        threading.Thread(target=stop, daemon=True).start()

    def _finish_disconnect(self):
        self.disconnecting = False
        self.connect_button.configure(state="normal")
        if self._disconnect_error is not None:
            self.stage_text.set("Disconnect failed: " + self._disconnect_error)
            self._disconnect_error = None
            self.closing = False
            return
        self.session = None
        self.link_text.set("OFFLINE")
        self.stage_text.set("Disconnected · map retained")
        self.camera_text.set("Camera offline · last image retained")
        self.connect_button.configure(text="Start demo" if self.demo_mode else "Connect")
        for widget in self.connection_widgets:
            widget.configure(state="normal")
        self._set_buttons(False, False)
        if self.closing:
            self._destroy()

    def _on_close(self):
        self.closing = True
        if self.session is not None:
            self._begin_disconnect()
        else:
            self._destroy()

    def _destroy(self):
        self.destroyed = True
        logging.getLogger().removeHandler(self._log_handler)
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone-id", type=int, default=4)
    parser.add_argument("--esp-ip", default=None)
    parser.add_argument("--config", default=None, help="Optional fleet setup YAML")
    parser.add_argument("--demo", action="store_true", help="Simulated data; no networking or flight commands")
    parser.add_argument("--smoke-test", action="store_true", help="Hidden startup check; auto-closes without flight commands")
    args = parser.parse_args()
    if not 0 <= args.drone_id <= 30:
        parser.error("Drone ID must be between 0 and 30")
    root = tk.Tk()
    if args.smoke_test:
        root.withdraw()
    app = DroneDesk(root, drone_id=args.drone_id,
                    esp_ip=args.esp_ip or f"192.168.1.{200 + args.drone_id}",
                    config=args.config, demo=args.demo, smoke_test=args.smoke_test)
    failures = []
    if args.smoke_test:
        def check_map_layout():
            try:
                app._toggle_maps()
                root.update_idletasks()
                if len(app.content.panes()) != 1:
                    failures.append("Map expansion did not hide the other panels")
                app._toggle_maps()
                root.update_idletasks()
                if len(app.content.panes()) != 3:
                    failures.append("Map layout did not restore all panels")
            except Exception as exc:
                failures.append(f"Map layout check failed: {exc}")

        root.after(1800, check_map_layout)

        def finish_check():
            failures.extend(app.display_errors)
            if app.route is None:
                failures.append("Default trajectory did not load")
            if args.demo and (app.last_image is None or not app.plot.map.tags):
                failures.append("Simulated camera/tags did not reach the UI")
            app._on_close()
        root.after(2500, finish_check)
    root.mainloop()
    if failures:
        raise SystemExit("Startup check failed: " + "; ".join(failures))
    if args.smoke_test:
        print("Startup check passed" + (": simulated camera, tags, telemetry and maps" if args.demo else ""))


if __name__ == "__main__":
    main()

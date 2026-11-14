#!/usr/bin/env python3
"""Numbered terminal UI for connection, flight commands, trajectories and tags."""

from __future__ import annotations

import csv
import ipaddress
import logging
import math
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time


def find_project_root() -> Path:
    """Find the checkout containing laptop/, trajectory/ and drone_app/."""
    for candidate in Path(__file__).resolve().parents:
        if ((candidate / "laptop").is_dir()
                and (candidate / "trajectory").is_dir()
                and (candidate / "drone_app").is_dir()):
            return candidate
    raise RuntimeError("Cannot find the project root")


PROJECT_ROOT = find_project_root()
LAPTOP_DIR = PROJECT_ROOT / "laptop"
DRONE_APP_DIR = PROJECT_ROOT / "drone_app"
TRAJECTORY_DIR = Path(__file__).resolve().parent.parent / "trajectory"
if not TRAJECTORY_DIR.is_dir():
    # Also works while this folder is staged under drone_app/ before installation.
    TRAJECTORY_DIR = PROJECT_ROOT / "trajectory"

for dependency in (str(PROJECT_ROOT), str(LAPTOP_DIR)):
    if dependency not in sys.path:
        sys.path.insert(0, dependency)

from drone_app.flight import (  # noqa: E402
    FlightCancelled,
    FlightControl,
    FlightFeedback,
    FlightOptions,
    load_trajectory,
    run_flight,
)
from drone_app.transport import CommsNode  # noqa: E402
from protocol import (  # noqa: E402
    CMD_GOTO,
    CMD_HOLD,
    CMD_LAND,
    TRAJ_MAX_PTS,
    CommandPacket,
    build_goto_yaw,
)


TELEMETRY_LOG_PORT = 5011
TELEM_PORT = 5005
CMD_PORT = 5006


def ask_number(prompt: str, minimum: int, maximum: int) -> int:
    while True:
        value = input(prompt).strip()
        try:
            number = int(value)
        except ValueError:
            print(f"Enter a number from {minimum} to {maximum}.")
            continue
        if minimum <= number <= maximum:
            return number
        print(f"Enter a number from {minimum} to {maximum}.")


def ask_float(prompt: str) -> float:
    while True:
        try:
            value = float(input(prompt).strip())
        except ValueError:
            print("Enter a finite number.")
            continue
        if math.isfinite(value):
            return value
        print("Enter a finite number.")


def infer_csv_dt(path: Path) -> float:
    """Use the CSV's own first positive time step, with a 0.05 s fallback."""
    times: list[float] = []
    with path.open(newline="", encoding="utf-8-sig") as source:
        for line in source:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            cells = next(csv.reader([line]))
            if cells[0].strip().lower() == "t":
                continue
            try:
                times.append(float(cells[0]))
            except (ValueError, IndexError):
                return 0.05
            if len(times) == 2:
                break
    if len(times) == 2:
        dt = times[1] - times[0]
        if math.isfinite(dt) and 0.001 <= dt <= 1.0:
            return dt
    return 0.05


def fit_csv_dt(path: Path) -> float:
    """The CSV's own step, coarsened only as far as needed to fit TRAJ_MAX_PTS.

    A 0.01 s min-snap export or a long spiral would otherwise exceed the
    firmware's point limit; the result is a whole number of milliseconds.
    """
    dt = infer_csv_dt(path)
    first = last = None
    with path.open(newline="", encoding="utf-8-sig") as source:
        for line in source:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            try:
                t = float(next(csv.reader([line]))[0])
            except (ValueError, IndexError):
                continue  # header row; load_trajectory reports real errors
            first = t if first is None else first
            last = t
    if first is None or not math.isfinite(last - first):
        return dt
    needed = math.ceil(round((last - first) * 1000 / (TRAJ_MAX_PTS - 1), 6)) / 1000
    return min(1.0, max(dt, needed))


class TerminalController:
    def __init__(self, drone_id: int = 4, esp_ip: str = "192.168.1.204"):
        self.drone_id = drone_id
        self.esp_ip = esp_ip
        self.comms: CommsNode | None = None
        self.feedback = FlightFeedback(drone_id)
        self.latest = None
        self.latest_received: float | None = None
        self.route_path: Path | None = None
        self.route_points = None
        self.route_dt_ms = 50
        self.route_duration = 0.0
        self.route_peak_speed = 0.0
        self.route_has_yaw = False
        self.control: FlightControl | None = None
        self.operation: threading.Thread | None = None
        self.map_process: subprocess.Popen | None = None
        self.sim_process: subprocess.Popen | None = None
        self.log_process: subprocess.Popen | None = None
        self.log_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._print_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        """Whether the terminal owns the UDP listener (not drone presence)."""
        return self.comms is not None

    @property
    def connection_state(self) -> str:
        """Operator-facing state based on telemetry from the selected drone."""
        if self.comms is None:
            return "OFFLINE"
        if self.latest_received is None:
            return "LISTENING"
        age = time.monotonic() - self.latest_received
        return "CONNECTED" if age <= self.feedback.stale_timeout else "STALE"

    @property
    def busy(self) -> bool:
        return self.operation is not None and self.operation.is_alive()

    def say(self, text: str):
        with self._print_lock:
            print(f"\n[{time.strftime('%H:%M:%S')}] {text}", flush=True)

    def connect(self):
        if self.connected:
            print("Already connected.")
            return
        config = PROJECT_ROOT / "laptop" / "setup.yaml"
        self.feedback = FlightFeedback(self.drone_id)
        self.latest = None
        self.latest_received = None
        node = CommsNode(TELEM_PORT, CMD_PORT, str(config), reuse_address=False)
        node.on_telemetry(self._on_telemetry)
        node.start()
        self.comms = node
        print(f"Listening for drone {self.drone_id} at {self.esp_ip} on UDP {TELEM_PORT}.")

    def _on_telemetry(self, packet, source_ip: str):
        if packet.drone_id != self.drone_id or source_ip != self.esp_ip:
            return
        self.feedback.update(packet, source_ip)
        self.latest = packet
        self.latest_received = time.monotonic()
        armed = "?" if packet.px4_armed is None else ("ARMED" if packet.px4_armed else "DISARMED")
        phase = "?" if packet.mission_phase is None else str(packet.mission_phase)
        line = (f"{time.strftime('%H:%M:%S')} | drone {packet.drone_id} | {armed} | "
                f"N {packet.ned_x:+.2f} E {packet.ned_y:+.2f} | "
                f"heading {math.degrees(packet.heading_rad):.0f} deg | "
                f"nav {packet.nav_state_name} | phase {phase} | tag {packet.tag_id}")
        try:
            self.log_sender.sendto(line.encode("utf-8"), ("127.0.0.1", TELEMETRY_LOG_PORT))
        except OSError:
            pass

    def send_command(self, drone_id: int, command: CommandPacket) -> bool:
        if self.comms is None:
            raise ConnectionError("Connect first")
        if drone_id != self.drone_id:
            raise ConnectionError("Command target does not match the selected drone")
        if self.comms.known_drones().get(drone_id) != self.esp_ip:
            raise ConnectionError("Selected drone ID/IP has not been verified by telemetry")
        return self.comms.send_command(drone_id, command)

    def send_raw(self, drone_id: int, data: bytes) -> bool:
        if self.comms is None:
            raise ConnectionError("Connect first")
        if drone_id != self.drone_id or self.comms.known_drones().get(drone_id) != self.esp_ip:
            raise ConnectionError("Selected drone ID/IP has not been verified by telemetry")
        return self.comms.send_raw(drone_id, data)

    def select_trajectory(self):
        files = sorted(TRAJECTORY_DIR.glob("*.csv"), key=lambda p: p.name.lower())
        if not files:
            print(f"No CSV files found in {TRAJECTORY_DIR}")
            return
        print(f"\nTrajectories in {TRAJECTORY_DIR}:")
        for index, path in enumerate(files, 1):
            print(f"  {index}. {path.name}")
        print("  0. Cancel")
        choice = ask_number("Choose trajectory: ", 0, len(files))
        if choice == 0:
            return
        path = files[choice - 1]
        dt = fit_csv_dt(path)
        if dt > infer_csv_dt(path) + 1e-9:
            print(f"Note: {path.name} exceeds {TRAJ_MAX_PTS} points at its own step; "
                  f"resampling at {dt:.3f} s.")
        points = load_trajectory(str(path), dt)
        peak = float(max(math.dist(a[:3], b[:3]) for a, b in zip(points, points[1:])) / dt)
        has_yaw = points.ndim == 2 and points.shape[1] == 4
        self.route_path = path
        self.route_points = points
        self.route_dt_ms = round(dt * 1000)
        self.route_duration = (len(points) - 1) * dt
        self.route_peak_speed = peak
        self.route_has_yaw = has_yaw
        yaw_label = ", explicit yaw" if has_yaw else ""
        print(f"Selected {path.name}: {len(points)} points, {self.route_duration:.1f} s, "
              f"dt {dt:.3f} s, peak {peak:.3f} m/s{yaw_label}")

    def _start_flight(self, points, takeoff: bool, finish: str):
        if not self.connected:
            print("Connect first.")
            return
        if self.busy:
            print("A flight operation is already active.")
            return
        try:
            self.feedback.packet()
        except Exception as exc:
            print(f"Cannot start: {exc}")
            return

        self.open_telemetry_log()
        self.control = FlightControl()
        options = FlightOptions(
            self.drone_id,
            self.route_dt_ms if points is not None else 50,
            takeoff,
            12.0,
            40.0,
            finish,
        )

        def run():
            try:
                run_flight(self, self.feedback, points, options,
                           control=self.control, on_progress=self.say)
                self.say("Flight operation complete")
            except FlightCancelled:
                self.say("Flight operation stopped by HOLD/LAND")
            except Exception as exc:
                self.say(f"Flight operation failed: {exc}")

        self.operation = threading.Thread(target=run, name="terminal-flight", daemon=True)
        self.operation.start()

    def takeoff(self):
        if input(f"Type TAKEOFF-{self.drone_id} to confirm: ").strip() != f"TAKEOFF-{self.drone_id}":
            print("Cancelled.")
            return
        self._start_flight(None, takeoff=True, finish="hold")

    def fly(self):
        if self.route_points is None or self.route_path is None:
            print("Select a trajectory first.")
            return
        print(f"{self.route_path.name}: {self.route_duration:.1f} s, "
              f"{len(self.route_points)} points, peak {self.route_peak_speed:.3f} m/s"
              f"{', explicit yaw' if self.route_has_yaw else ''}")
        takeoff = ask_number("Take off first? 1=Yes, 2=No (already hovering): ", 1, 2) == 1
        finish = "land" if ask_number("Finish action: 1=LAND, 2=HOLD: ", 1, 2) == 1 else "hold"
        if input(f"Type TRAJ-{self.drone_id} to confirm: ").strip() != f"TRAJ-{self.drone_id}":
            print("Cancelled.")
            return
        self._start_flight(self.route_points, takeoff=takeoff, finish=finish)

    def interrupt(self, command: int):
        if not self.connected:
            print("Connect first.")
            return
        control = self.control or FlightControl()
        self.control = control
        sent = False
        error = None
        for attempt in range(3):
            try:
                sent = control.interrupt(self, self.drone_id, CommandPacket(command)) or sent
            except Exception as exc:
                error = exc
            if attempt < 2:
                time.sleep(0.06)
        action = "LAND" if command == CMD_LAND else "HOLD"
        print(f"{action} requested." if sent else f"{action} failed: {error or 'not sent'}")

    def goto(self):
        if not self.connected:
            print("Connect first.")
            return
        x = ask_float("Goal N/x (m): ")
        y = ask_float("Goal E/y (m): ")
        yaw_text = input("Yaw deg (0=North, clockwise; blank = face travel): ").strip()
        try:
            if yaw_text:
                yaw_deg = float(yaw_text)
                if not math.isfinite(yaw_deg):
                    raise ValueError("yaw must be finite")
                sent = self.send_raw(self.drone_id,
                                     build_goto_yaw(x, y, math.radians(yaw_deg)))
                label = f"GOTO ({x:+.2f}, {y:+.2f}) yaw {yaw_deg:+.1f} deg"
            else:
                sent = self.send_command(self.drone_id, CommandPacket(CMD_GOTO, x, y))
                label = f"GOTO ({x:+.2f}, {y:+.2f})"
            print(f"{label} sent." if sent else f"{label} was not sent.")
        except Exception as exc:
            print(f"GOTO failed: {exc}")

    def simulate_trajectory(self):
        """Open the real-time 2D simulation in its own process (no drone needed)."""
        if self.route_path is None:
            print("Select a trajectory first.")
            return
        if self.sim_process is not None and self.sim_process.poll() is None:
            print("The trajectory simulation window is already open.")
            return
        command = [sys.executable, "-m", "drone_terminal.trajectory_sim",
                   str(self.route_path)]
        self.sim_process = subprocess.Popen(command, cwd=PROJECT_ROOT)
        print(f"Opened real-time simulation for {self.route_path.name}.")

    def open_map(self):
        if not self.connected:
            print("Connect first so telemetry can be mirrored to the map.")
            return
        if self.map_process is not None and self.map_process.poll() is None:
            print("The tag_stream.py camera/map windows are already open.")
            return
        output = PROJECT_ROOT / "drone_terminal" / (
            f"tag_stream_plot_{time.strftime('%Y%m%d_%H%M%S')}.png")
        command = [
            sys.executable,
            str(LAPTOP_DIR / "tag_stream.py"),
            "--esp-ip", self.esp_ip,
            "--telem-port", "5010",
            "--plot-output", str(output),
            "--map-span", "4.0",
        ]
        self.map_process = subprocess.Popen(command, cwd=PROJECT_ROOT)
        print("Opened tag_stream.py camera and permanent-map windows.")
        print("The map receives mirrored telemetry from this terminal app on UDP 5010.")

    def open_telemetry_log(self):
        if self.log_process is not None and self.log_process.poll() is None:
            return
        command = [sys.executable, "-m", "drone_terminal.telemetry_log",
                   "--port", str(TELEMETRY_LOG_PORT)]
        kwargs = {"cwd": PROJECT_ROOT}
        if sys.platform == "win32":
            kwargs["creationflags"] = subprocess.CREATE_NEW_CONSOLE
        self.log_process = subprocess.Popen(command, **kwargs)
        print("Opened a separate live telemetry terminal.")

    def status(self):
        print(f"Drone: {self.drone_id} at {self.esp_ip}")
        print(f"Connection: {self.connection_state.lower()}")
        print(f"Operation: {'active' if self.busy else 'idle'}")
        print(f"Trajectory: {self.route_path.name if self.route_path else 'none'}")
        if self.latest is None or self.latest_received is None:
            print("Telemetry: none received")
            return
        packet = self.latest
        age = time.monotonic() - self.latest_received
        armed = "unavailable" if packet.px4_armed is None else ("armed" if packet.px4_armed else "disarmed")
        print(f"Telemetry: {age:.1f} s old, {armed}, nav {packet.nav_state_name}")
        print(f"Position: N {packet.ned_x:+.2f}, E {packet.ned_y:+.2f} m, "
              f"heading {math.degrees(packet.heading_rad):.0f} deg, tag {packet.tag_id}")

    def disconnect(self):
        if self.busy:
            print("Use HOLD or LAND and wait for the active operation to stop first.")
            return False
        if self.comms is not None:
            self.comms.stop()
            self.comms = None
        self.latest = None
        self.latest_received = None
        print("Disconnected.")
        return True

    def close(self):
        if self.busy:
            print("An operation is active. Requesting HOLD before exit.")
            self.interrupt(CMD_HOLD)
            if self.operation is not None:
                self.operation.join(timeout=3.0)
        if self.comms is not None:
            self.comms.stop()
            self.comms = None
        # Leave tag_stream.py open so its map remains available for inspection
        # and it can save normally when the user closes that window.
        if self.log_process is not None and self.log_process.poll() is None:
            self.log_process.terminate()
        self.log_sender.close()


def print_menu(controller: TerminalController):
    route = controller.route_path.name if controller.route_path else "none"
    if controller.route_path and controller.route_has_yaw:
        route += " [yaw]"
    state = controller.connection_state
    print("\n" + "=" * 62)
    print(f"DRONE TERMINAL | {state} | drone {controller.drone_id} | route: {route}")
    print("=" * 62)
    print("  1. CONNECT")
    print("  2. SELECT TRAJECTORY")
    print("  3. TAKEOFF AND HOLD")
    print("  4. FLY SELECTED TRAJECTORY")
    print("  5. GOTO COMMAND")
    print("  6. HOLD")
    print("  7. LAND")
    print("  8. OPEN TAG CAMERA + PERMANENT MAP")
    print("  9. OPEN TELEMETRY LOG TERMINAL")
    print(" 10. STATUS")
    print(" 11. DISCONNECT")
    print(" 12. SIMULATE SELECTED TRAJECTORY")
    print("  0. EXIT")


def main():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s | %(levelname)s | %(message)s",
                        datefmt="%H:%M:%S")
    print("ESP32/PX4 Drone Terminal")
    drone_id = ask_number("Drone ID [0-30] (usually 4): ", 0, 30)
    default_ip = f"192.168.1.{200 + drone_id}"
    esp_ip = input(f"Drone IP [{default_ip}]: ").strip() or default_ip
    try:
        esp_ip = str(ipaddress.ip_address(esp_ip))
    except ValueError:
        raise SystemExit(f"Invalid IP address: {esp_ip}")

    controller = TerminalController(drone_id, esp_ip)
    actions = {
        1: controller.connect,
        2: controller.select_trajectory,
        3: controller.takeoff,
        4: controller.fly,
        5: controller.goto,
        6: lambda: controller.interrupt(CMD_HOLD),
        7: lambda: controller.interrupt(CMD_LAND),
        8: controller.open_map,
        9: controller.open_telemetry_log,
        10: controller.status,
        11: controller.disconnect,
        12: controller.simulate_trajectory,
    }
    try:
        while True:
            print_menu(controller)
            choice = ask_number("Choose: ", 0, 12)
            if choice == 0:
                break
            try:
                actions[choice]()
            except (ConnectionError, OSError, ValueError) as exc:
                print(f"Action failed: {exc}")
    except (KeyboardInterrupt, EOFError):
        print("\nExiting...")
    finally:
        controller.close()


if __name__ == "__main__":
    main()


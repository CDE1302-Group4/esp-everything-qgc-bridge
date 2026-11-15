#!/usr/bin/env python3
"""Build the GIFs and images used in drone_terminal/README.md.

    python drone_terminal/docs/make_readme_media.py

Terminal GIFs replay real sessions saved in docs/recordings/ (absolute paths
shortened to <repo>); keyboard input is typed back in at each prompt.  The
simulator GIF and result image are rendered from an actual simulated flight,
so they need the same setup as the simulator (gcc + requirements).
"""

from __future__ import annotations

import io
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "laptop")]

import matplotlib  # noqa: E402

matplotlib.use("Agg")
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

DOCS = Path(__file__).resolve().parent
REC = DOCS / "recordings"

BG, FG, DIM, PROMPT, INPUT = (24, 26, 33), (220, 223, 228), (130, 136, 150), (98, 200, 120), (255, 214, 102)
COLS, ROWS = 100, 30
FONT_SIZE = 15


def _font():
    for name in ("consola.ttf", "Menlo.ttc", "DejaVuSansMono.ttf", "LiberationMono-Regular.ttf"):
        try:
            return ImageFont.truetype(name, FONT_SIZE)
        except OSError:
            continue
    return ImageFont.load_default()


FONT = _font()
CW = int(FONT.getlength("M")) or 9
LH = FONT_SIZE + 5
PAD = 14


class Terminal:
    """Tiny terminal emulator that records frames."""

    def __init__(self, title: str):
        self.title = title
        self.lines: list[list[tuple[str, tuple]]] = [[]]
        self.frames: list[Image.Image] = []
        self.durations: list[int] = []

    def write(self, text: str, colour=FG):
        for i, part in enumerate(text.split("\n")):
            if i:
                self.lines.append([])
            if part:
                self.lines[-1].append((part, colour))

    def frame(self, ms: int):
        w, h = PAD * 2 + CW * COLS, PAD * 2 + LH * (ROWS + 1)
        img = Image.new("RGB", (w, h), BG)
        d = ImageDraw.Draw(img)
        d.rectangle((0, 0, w, LH + 6), fill=(40, 43, 54))
        for i, c in enumerate(((255, 95, 86), (255, 189, 46), (39, 201, 63))):
            d.ellipse((10 + i * 18, 7, 22 + i * 18, 19), fill=c)
        d.text((w // 2, 4), self.title, fill=DIM, font=FONT, anchor="ma")
        y = LH + 12
        for line in self.lines[-ROWS:]:
            x = PAD
            used = 0
            for text, colour in line:
                room = COLS - used
                if room <= 0:
                    break
                shown = text if len(text) <= room else text[:room - 1] + "…"
                d.text((x, y), shown, fill=colour, font=FONT)
                x += CW * len(shown)
                used += len(shown)
            y += LH
        self.frames.append(img)
        self.durations.append(ms)

    def type(self, text: str, colour=INPUT, per_char=55):
        for ch in text:
            self.write(ch, colour)
            self.frame(per_char)

    def command(self, cmd: str):
        self.write("$ ", PROMPT)
        self.frame(400)
        self.type(cmd)
        self.write("\n")
        self.frame(350)

    def save(self, path: Path):
        self.frames[-1:] = [self.frames[-1]]
        self.durations[-1] = 3500
        self.frames[0].save(path, save_all=True, append_images=self.frames[1:],
                            duration=self.durations, loop=0, optimize=True)
        print(f"wrote {path.relative_to(ROOT)} ({path.stat().st_size // 1024} KB, "
              f"{len(self.frames)} frames)")


def colour_for(line: str):
    if line.startswith("[firmware]"):
        return (130, 190, 255)
    if line.startswith("[sim]"):
        return PROMPT
    if line.startswith("Requirement already satisfied"):
        return DIM
    if set(line.strip()) == {"="}:
        return DIM
    return FG


def install_gif():
    """gcc check, requirements install, first simulator run (firmware compile)."""
    term = Terminal("install + first run")
    blocks = re.split(r"^\$ (.*)$", (REC / "install_out.txt").read_text(encoding="utf-8"),
                      flags=re.M)
    sim = re.split(r"^\$ (.*)$", (REC / "sim_out.txt").read_text(encoding="utf-8"), flags=re.M)
    pairs = list(zip(blocks[1::2], blocks[2::2])) + list(zip(sim[1::2], sim[2::2]))
    for cmd, out in pairs:
        term.command(cmd.strip())
        lines = [l for l in out.strip("\n").splitlines()]
        if len(lines) > 8:   # pip: show the start and the end of its output
            lines = lines[:5] + [f"... ({len(lines) - 7} more lines)"] + lines[-2:]
        for line in lines:
            term.write(line + "\n", colour_for(line))
            term.frame(900 if line.startswith("[firmware] compiling") else 160)
        term.frame(900)
    term.save(DOCS / "install.gif")


def menu_gif():
    """Drone Terminal menu: select a route, status, open the simulator, exit."""
    term = Terminal("python -m drone_terminal")
    term.command("python -m drone_terminal")
    out = (REC / "menu_out.txt").read_text(encoding="utf-8")
    answers = (REC / "menu_input.txt").read_text(encoding="utf-8").split("\n")
    prompt = re.compile(r"(Drone ID \[0-30\] \(usually 4\): |Drone IP \[[^\]]*\]: |"
                        r"Choose trajectory: |Choose: )")
    parts = prompt.split(out)
    for i, chunk in enumerate(parts):
        if i % 2 == 0:
            for j, line in enumerate(chunk.split("\n")):
                if j:
                    term.write("\n")
                if line:
                    term.write(line, colour_for(line))
                if j % 3 == 2:
                    term.frame(60)
            term.frame(500)
        else:
            term.write(chunk, PROMPT)
            term.frame(500)
            answer = answers.pop(0) if answers else ""
            term.type(answer if answer else "⏎", per_char=180)
            term.write("\n")
            term.frame(250)
    term.save(DOCS / "terminal_menu.gif")


def simulator_media():
    """A real simulated flight: live window GIF + the result window PNG."""
    from drone_terminal.trajectory_sim import Plan, Simulation

    path = (ROOT / "trajectory" / "flying_area_search_5p4x4p5_yaw_center_slow.csv").resolve()
    sim = Simulation(path, Plan(path, 0.0), 1.0, False, 2, 1.0)
    sim.fig.set_size_inches(11, 8.6)
    f = sim.flight
    times = [0.5, 1.5, 2.5, 3.2, 4.0] + [5 + k * (f.duration - 6) / 70 for k in range(71)]
    frames = []
    for t in times:
        sim.t = t
        sim.last_clock = time.monotonic()   # freeze the playback clock at t
        sim._update(0)
        buf = io.BytesIO()
        sim.fig.savefig(buf, format="png", dpi=78, bbox_inches="tight", pad_inches=0.1)
        frames.append(Image.open(buf).convert("RGB").quantize(colors=96, method=2))
    out = DOCS / "simulator.gif"
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=[500] * 5 + [140] * 70 + [3000], loop=0, optimize=True)
    print(f"wrote {out.relative_to(ROOT)} ({out.stat().st_size // 1024} KB, {len(frames)} frames)")

    summary = sim._show_summary()
    out = DOCS / "result_window.png"
    summary.savefig(out, dpi=90)
    print(f"wrote {out.relative_to(ROOT)}")


if __name__ == "__main__":
    install_gif()
    menu_gif()
    simulator_media()

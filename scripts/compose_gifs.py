#!/usr/bin/env python3
"""Compose the README GIF frames from record_mode takes: panels, overlays, captions, live numbers.

    python3 scripts/compose_gifs.py --raw <dir> --out <dir> [--fps 10] [--only hero ...]

<raw> holds the record_mode scenes (hero/{correct,wrong}, tracking/{pid,pid_ff}, jogging/jog); for
each GIF this writes <out>/<gif>/NNNNN.png, 640 px wide, ready for ffmpeg (scripts/record_gifs.sh).

Every number drawn comes from the recorded take (take.json), none is typed in here: the push force
and the payload error from the take's metadata, the tool height from the recorded joint states
(forward kinematics in the recorder), the tracking error from the controller's reference vs the
measured joint positions, and the jog numbers from the jog controller's ~/jog_state. The force
arrow and the tool-point traces are 3D points projected through the world camera's pose and
intrinsics, read from worlds/bench_camera.sdf (pinhole model, no distortion).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDF = os.path.join(REPO, "src", "ur5e_teaching_bringup", "worlds", "bench_camera.sdf")
WIDTH = 640
FONT_DIR = "/usr/share/fonts/truetype/dejavu"
MINUS = "−"

BG = (20, 20, 20)
WHITE = (240, 240, 240)
GREY = (175, 175, 175)
ARROW = (225, 45, 35)
TRACE = (255, 205, 0)
PHASE_COLOURS = [(40, 170, 235), (245, 135, 25)]
OK_GREEN = (70, 185, 90)


# ---- drawing helpers ----------------------------------------------------------------------------
def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = os.path.join(FONT_DIR, "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default(size=size)


def signed(value: float, fmt: str = ".0f") -> str:
    text = format(value, fmt)
    if float(text) == 0.0:
        text = format(0.0, fmt)
    return text.replace("-", MINUS) if text.startswith("-") else "+" + text


def text_outlined(draw, xy, text, fnt, fill=WHITE, outline=(0, 0, 0), width=2, anchor="la"):
    draw.text(xy, text, font=fnt, fill=fill, anchor=anchor, stroke_width=width, stroke_fill=outline)


def caption_bar(text: str, width: int = WIDTH) -> Image.Image:
    """Dark bar with the caption, the largest font size (17 down to 13) that fits on one line,
    else wrapped onto two lines."""
    probe = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    for size in range(17, 12, -1):
        fnt = font(size)
        if probe.textlength(text, font=fnt) <= width - 20:
            lines = [text]
            break
    else:
        fnt = font(15)
        words, lines, line = text.split(), [], ""
        for word in words:
            trial = f"{line} {word}".strip()
            if probe.textlength(trial, font=fnt) > width - 20 and line:
                lines.append(line)
                line = word
            else:
                line = trial
        lines.append(line)
    line_h = fnt.size + 6
    bar = Image.new("RGB", (width, 12 + line_h * len(lines)), BG)
    draw = ImageDraw.Draw(bar)
    for i, line in enumerate(lines):
        draw.text((10, 7 + i * line_h), line, font=fnt, fill=WHITE)
    return bar


def arrow(draw, tail, head, colour=ARROW, width=6, head_len=16):
    tail, head = np.asarray(tail, float), np.asarray(head, float)
    d = head - tail
    n = np.linalg.norm(d)
    if n < 1.0:
        return
    d /= n
    side = np.array([-d[1], d[0]])
    base = head - d * head_len
    tri = [tuple(head), tuple(base + side * head_len * 0.55), tuple(base - side * head_len * 0.55)]
    draw.line([tuple(tail), tuple(base)], fill=(0, 0, 0), width=width + 4)
    draw.polygon(tri, fill=(0, 0, 0), outline=(0, 0, 0), width=3)
    draw.line([tuple(tail), tuple(base)], fill=colour, width=width)
    draw.polygon(tri, fill=colour)


def translucent_box(img: Image.Image, box, alpha: int = 170, radius: int = 6) -> None:
    """Darken `box` of an RGBA image in place, so text on it stays legible over any background."""
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    ImageDraw.Draw(overlay).rounded_rectangle(box, radius=radius, fill=(*BG, alpha))
    img.alpha_composite(overlay)


# ---- camera -------------------------------------------------------------------------------------
def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y),
                              math.sin(y))  # fmt: skip
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def pose_matrix(element) -> np.ndarray:
    t = np.eye(4)
    if element is not None and element.text:
        x, y, z, r, p, yaw = (float(v) for v in element.text.split())
        t[:3, :3] = rpy_matrix(r, p, yaw)
        t[:3, 3] = (x, y, z)
    return t


class Camera:
    """Pinhole model of the world camera in bench_camera.sdf (Gazebo: x forward, y left, z up)."""

    def __init__(self, sdf_path: str = SDF):
        model = ET.parse(sdf_path).getroot().find(".//model[@name='world_camera']")
        link = model.find("link")
        sensor = link.find("sensor")
        world_t_cam = pose_matrix(model.find("pose")) @ pose_matrix(link.find("pose"))
        world_t_cam = world_t_cam @ pose_matrix(sensor.find("pose"))
        cam = sensor.find("camera")
        self.width = int(cam.find("image/width").text)
        self.height = int(cam.find("image/height").text)
        hfov = float(cam.find("horizontal_fov").text)
        self.f = self.width / 2.0 / math.tan(hfov / 2.0)
        self.rotation = world_t_cam[:3, :3]
        self.origin = world_t_cam[:3, 3]

    def project(self, p) -> np.ndarray:
        c = self.rotation.T @ (np.asarray(p, float) - self.origin)
        return np.array(
            [self.width / 2.0 - self.f * c[1] / c[0], self.height / 2.0 - self.f * c[2] / c[0]]
        )


# ---- takes --------------------------------------------------------------------------------------
class Take:
    def __init__(self, path: str):
        self.path = path
        with open(os.path.join(path, "take.json")) as f:
            data = json.load(f)
        self.meta = data["meta"]
        self.frames = data["frames"]
        self.trace = np.array(data["trace"]) if data["trace"] else np.zeros((0, 4))
        self.series = data["series"]
        self.frame_t = np.array([fr["t"] for fr in self.frames])

    def frame_at(self, t: float) -> dict:
        return self.frames[int(np.argmin(np.abs(self.frame_t - t)))]

    def image(self, frame: dict) -> Image.Image:
        return Image.open(os.path.join(self.path, "frames", frame["file"])).convert("RGB")

    def series_at(self, name: str, key: str, t: float):
        s = self.series[name]
        k = max(int(np.searchsorted(np.asarray(s["t"]), t, side="right")) - 1, 0)
        return s[key][k]


def timeline(takes: list[Take], fps: float) -> np.ndarray:
    t_start = max(tk.meta["t_start"] for tk in takes)
    t_end = min(min(tk.meta["t_end"], tk.frame_t[-1]) for tk in takes)
    return np.arange(t_start, t_end + 1e-9, 1.0 / fps)


def crop_box(camera: Camera, takes, times, aspect: float, margin=42.0, extra=(), min_w=0.0):
    """Pixel box around every projected link point over the clip, grown to `aspect` (h / w) and
    to at least `min_w` pixels wide, kept inside the image."""
    pts = []
    for tk in takes:
        for t in times:
            fr = tk.frame_at(t)
            for name, p in fr["points"].items():
                if name != "tool_axis" and (name != "payload_bottom" or "payload" in extra):
                    pts.append(camera.project(p))
    pts = np.array(pts)
    x0, y0 = pts.min(axis=0) - margin
    x1, y1 = pts.max(axis=0) + margin
    if x1 - x0 < min_w:
        x0, x1 = (x0 + x1 - min_w) / 2, (x0 + x1 + min_w) / 2
    w, h = x1 - x0, y1 - y0
    if h / w < aspect:
        grow = w * aspect - h
        y0, y1 = y0 - grow / 2, y1 + grow / 2
    else:
        grow = h / aspect - w
        x0, x1 = x0 - grow / 2, x1 + grow / 2
    w, h = min(x1 - x0, camera.width), min(y1 - y0, camera.height)
    x0 = min(max(x0, 0.0), camera.width - w)
    y0 = min(max(y0, 0.0), camera.height - h)
    return x0, y0, w, h


class Panel:
    """A crop of the camera image scaled to `width`; maps world points into panel pixels."""

    def __init__(self, camera: Camera, box, width: int):
        self.camera = camera
        self.x0, self.y0, self.w, self.h = box
        self.scale = width / self.w
        self.size = (width, int(round(self.h * self.scale)))

    def render(self, img: Image.Image) -> Image.Image:
        box = (self.x0, self.y0, self.x0 + self.w, self.y0 + self.h)
        return img.resize(self.size, Image.Resampling.LANCZOS, box=box)

    def px(self, p) -> tuple:
        u, v = self.camera.project(p)
        return ((u - self.x0) * self.scale, (v - self.y0) * self.scale)


def stack(*parts: Image.Image) -> Image.Image:
    out = Image.new("RGB", (WIDTH, sum(p.height for p in parts)), BG)
    y = 0
    for p in parts:
        out.paste(p, (0, y))
        y += p.height
    return out


def side_by_side(left: Image.Image, right: Image.Image, gap: int = 4) -> Image.Image:
    out = Image.new("RGB", (WIDTH, max(left.height, right.height)), BG)
    out.paste(left, (0, 0))
    out.paste(right, (WIDTH - right.width, 0))
    return out


PANEL_W = (WIDTH - 4) // 2


def tile(head: str, picture: Image.Image, strip_h: int) -> tuple[Image.Image, Image.Image]:
    """One panel: header strip, camera crop, and a dark strip below it for the live numbers (kept
    off the picture so they never cover the arm). Returns the tile and the empty strip; draw on
    the strip, then finish_tile() pastes it in."""
    head_h = 26
    img = Image.new("RGB", (picture.width, head_h + picture.height + strip_h), BG)
    img.paste(picture, (0, head_h))
    draw = ImageDraw.Draw(img)
    draw.text((8, 4), head, font=font(15, bold=True), fill=WHITE)
    strip = Image.new("RGB", (picture.width, strip_h), BG)
    return img, strip


def finish_tile(img: Image.Image, strip: Image.Image) -> Image.Image:
    img.paste(strip, (0, img.height - strip.height))
    return img


def save(img: Image.Image, out_dir: str, k: int) -> None:
    img.save(os.path.join(out_dir, f"{k:05d}.png"))


# ---- hero: hand guiding --------------------------------------------------------------------------
def compose_hero(raw: str, out: str, fps: float, camera: Camera) -> str:
    takes = [Take(os.path.join(raw, "hero", n)) for n in ("correct", "wrong")]
    times = timeline(takes, fps)
    panel = Panel(camera, crop_box(camera, takes, times, 1.0, extra=("payload",)), PANEL_W)
    force = np.asarray(takes[0].meta["push_force_n"], float)
    force_n = float(np.linalg.norm(force))
    error_kg = takes[1].meta["actual_payload_kg"] - takes[1].meta["model_payload_kg"]
    headers = ["correct payload model", f"{error_kg:g} kg model error"]
    caption = (
        f"pushed with {force_n:g} N; correct payload model (left) vs "
        f"{error_kg:g} kg model error (right)"
    )
    bar = caption_bar(caption)
    big, small, label = font(26, bold=True), font(13), font(15, bold=True)
    push_s = takes[0].meta["push_s"]
    arrow_len_m = 0.28
    for k, t in enumerate(times):
        tiles = []
        for tk, head in zip(takes, headers, strict=True):
            fr = tk.frame_at(t)
            ref = tk.frame_at(0.0)["points"]["tool0"]
            picture = panel.render(tk.image(fr))
            draw = ImageDraw.Draw(picture)
            # Tool-point path since hand guiding took over.
            path = [panel.px(p[1:]) for p in tk.trace if 0.0 <= p[0] <= fr["t"]]
            if len(path) > 1:
                draw.line(path, fill=(0, 0, 0), width=5, joint="curve")
                draw.line(path, fill=TRACE, width=3, joint="curve")
            push = np.asarray(fr["push_force"], float)
            if np.linalg.norm(push) > 0.0:
                p = np.asarray(fr["points"]["wrist_3_link"])  # where ApplyLinkWrench pushes
                d = push / np.linalg.norm(push)
                tail, tip = panel.px(p - d * arrow_len_m), panel.px(p - d * 0.02)
                arrow(draw, tail, tip)
                text_outlined(draw, (tail[0], tail[1] - 8), f"{np.linalg.norm(push):g} N", label,
                              anchor="md")  # fmt: skip
            img, strip = tile(head, picture, 58)
            sd = ImageDraw.Draw(strip)
            dz_mm = 1e3 * (fr["points"]["tool0"][2] - ref[2]) if fr["t"] >= 0.0 else 0.0
            sd.text((10, 2), "tool height change", font=small, fill=GREY)
            sd.text((10, 18), f"{signed(dz_mm)} mm", font=big, fill=WHITE)
            if fr["t"] < 0.0:
                state = "held still"
            elif fr["t"] < push_s:
                state = f"pushed, {force_n:g} N"
            else:
                state = "let go"
            sd.text((PANEL_W - 10, 24), state, font=label, fill=WHITE, anchor="ra")
            tiles.append(finish_tile(img, strip))
        save(stack(side_by_side(*tiles), bar), out, k)
    return caption


# ---- tracking: PID alone vs PID + feedforward ----------------------------------------------------
def compose_tracking(raw: str, out: str, fps: float, camera: Camera) -> str:
    takes = [Take(os.path.join(raw, "tracking", n)) for n in ("pid", "pid_ff")]
    times = timeline(takes, fps)
    panel = Panel(camera, crop_box(camera, takes, times, 0.9), PANEL_W)
    headers = ["PID alone", "PID + feedforward"]
    caption = "same path, same gains; feedforward removes most of the error"
    bar = caption_bar(caption)
    series = []
    for tk in takes:
        s = tk.series["tracking"]
        st, se = np.asarray(s["t"]), 1e3 * np.asarray(s["worst_abs_err_rad"])
        keep = (st >= times[0]) & (st <= times[-1])
        series.append((st[keep], se[keep]))
    y_max = max(float(se.max()) for _, se in series)
    big, small = font(30, bold=True), font(13)
    for k, t in enumerate(times):
        tiles = []
        for tk, head, (st, se) in zip(takes, headers, series, strict=True):
            img, strip = tile(head, panel.render(tk.image(tk.frame_at(t))), 104)
            sd = ImageDraw.Draw(strip)
            now = float(se[max(int(np.searchsorted(st, t, side="right")) - 1, 0)])
            sd.text((10, 2), "worst-joint error now", font=small, fill=GREY)
            sd.text((10, 17), f"{now:.2f} mrad", font=big, fill=WHITE)
            # Error history so far, both panels on the same scale.
            x0, x1, y0, y1 = 10, PANEL_W - 10, 60, 98
            sd.text((x1, 2), f"history, 0 to {y_max:.0f} mrad", font=small, fill=GREY,
                    anchor="ra")  # fmt: skip
            sd.line([(x0, y1), (x1, y1)], fill=GREY, width=1)
            sel = st <= t
            if sel.sum() > 1:
                xs = x0 + (st[sel] - times[0]) / (times[-1] - times[0]) * (x1 - x0)
                ys = y1 - se[sel] / y_max * (y1 - y0)
                sd.line(list(zip(xs, ys, strict=True)), fill=TRACE, width=2)
            tiles.append(finish_tile(img, strip))
        save(stack(side_by_side(*tiles), bar), out, k)
    return caption


# ---- jogging: straight line, then towards the wrist singularity ---------------------------------
def compose_jogging(raw: str, out: str, fps: float, camera: Camera) -> list[str]:
    tk = Take(os.path.join(raw, "jogging", "jog"))
    times = timeline([tk], fps)
    # Zoom in on the arm (at most 640 / 460 = 1.4x, so the camera image stays sharp enough).
    panel = Panel(camera, crop_box(camera, [tk], times, 0.75, margin=50.0, min_w=460.0), WIDTH)
    meta = tk.meta
    speed_mm = 1e3 * meta["speed_mps"]
    phases = meta["phases"]
    w0 = meta["manipulability_threshold"]
    names = [ph["name"].replace("-", MINUS) for ph in phases]
    captions = [
        f"1/2  jog along {names[0]} at {speed_mm:.0f} mm/s: the tool point traces a straight line",
        f"2/2  jog along {names[1]} towards the wrist singularity: the damping keeps the "
        "joints slow, so the tool slows down and drifts off the line",
    ]
    bars = [caption_bar(c) for c in captions]
    bar_h = max(b.height for b in bars)  # same frame height in both phases
    padded = []
    for b in bars:
        p = Image.new("RGB", (WIDTH, bar_h), BG)
        p.paste(b, (0, 0))
        padded.append(p)
    bars = padded
    jog = tk.series["jog"]
    jt = np.asarray(jog["t"])
    big, mid, small = font(24, bold=True), font(15, bold=True), font(13)
    starts = []  # tool point where each phase began: its commanded line starts there
    for ph in phases:
        seg = tk.trace[tk.trace[:, 0] >= ph["t0"]]
        starts.append(seg[0, 1:] if len(seg) else tk.trace[-1, 1:])
    for k, t in enumerate(times):
        fr = tk.frame_at(t)
        img = panel.render(tk.image(fr))
        draw = ImageDraw.Draw(img)
        active = next((i for i, ph in enumerate(phases) if ph["t0"] <= t < ph["t1"]), None)
        current = active if active is not None else (0 if t < phases[0]["t0"] else 1)
        for i, ph in enumerate(phases):
            if t < ph["t0"]:
                continue
            seg = tk.trace[(tk.trace[:, 0] >= ph["t0"]) & (tk.trace[:, 0] <= min(t, ph["t1"]))]
            length = meta["speed_mps"] * (ph["t1"] - ph["t0"])
            a = np.array(panel.px(starts[i]))
            b = np.array(panel.px(starts[i] + np.asarray(ph["direction"]) * length))
            for s in np.arange(0.0, 1.0, 0.04):  # dashed: the commanded straight line
                draw.line([tuple(a + (b - a) * s), tuple(a + (b - a) * (s + 0.02))], fill=WHITE,
                          width=2)  # fmt: skip
            path = [panel.px(p[1:]) for p in seg]
            if len(path) > 1:
                draw.line(path, fill=(0, 0, 0), width=6, joint="curve")
                draw.line(path, fill=PHASE_COLOURS[i], width=4, joint="curve")
            for p in seg[:: max(1, int(round(0.5 / 0.02)))]:  # a dot every 0.5 s
                x, y = panel.px(p[1:])
                draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill=PHASE_COLOURS[i], outline=(0, 0, 0))
        # Live numbers, in a strip below the picture: ~/jog_state, the joint states and the trace.
        j = max(int(np.searchsorted(jt, t, side="right")) - 1, 0)
        w = jog["manipulability"][j]
        lam = jog["damping"][j]
        v_meas = 1e3 * float(np.linalg.norm(jog["v_meas"][j]))
        commanding = float(np.linalg.norm(jog["v_cmd"][j])) > 0.0
        fastest = max(abs(v) for v in fr["qd"])
        upto = tk.trace[tk.trace[:, 0] <= t]
        off_mm = 0.0
        if len(upto) and t >= phases[0]["t0"]:
            rel = upto[-1, 1:] - starts[current]
            d = np.asarray(phases[current]["direction"], float)
            off_mm = 1e3 * float(np.linalg.norm(rel - (rel @ d) * d))
        strip = Image.new("RGB", (WIDTH, 92), BG)
        sd = ImageDraw.Draw(strip)
        cmd = f"{speed_mm:.0f} mm/s along {names[current]}" if commanding else "none (stopped)"
        sd.text((10, 6), f"jog command: {cmd}", font=mid, fill=PHASE_COLOURS[current])
        colour = OK_GREEN if lam <= 0.0 else PHASE_COLOURS[1]
        status = "damping off" if lam <= 0.0 else f"damping on, {lam:.3f}"
        sd.text((WIDTH - 10, 6), status, font=mid, fill=colour, anchor="ra")
        for x, label, value, fnt in (
            (10, "tool speed", f"{v_meas:.0f} mm/s", big),
            (140, "off the line", f"{off_mm:.0f} mm", big),
            (262, "fastest joint", f"{fastest:.2f} rad/s", big),
        ):
            sd.text((x, 32), label, font=small, fill=GREY)
            sd.text((x, 48), value, font=fnt, fill=WHITE)
        # Manipulability gauge (empty at a singularity) with the damping threshold w0 marked.
        gx0, gx1, gy = 412, WIDTH - 10, 54
        g_max = 2.0 * w0
        sd.text((gx0, 32), f"manipulability {w:.3f}", font=small, fill=GREY)
        sd.rectangle((gx0, gy, gx1, gy + 10), outline=GREY)
        sd.rectangle((gx0, gy, gx0 + (gx1 - gx0) * min(w / g_max, 1.0), gy + 10), fill=colour)
        tx = gx0 + (gx1 - gx0) * w0 / g_max
        sd.line([(tx, gy - 4), (tx, gy + 14)], fill=WHITE, width=2)
        sd.text((gx1, gy + 16), f"damping below {w0:g}", font=small, fill=GREY, anchor="ra")
        save(stack(img, strip, bars[current]), out, k)
    return captions


COMPOSERS = {"hero": compose_hero, "tracking": compose_tracking, "jogging": compose_jogging}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--raw", required=True, help="record_mode output (scene directories)")
    parser.add_argument("--out", required=True, help="frames go to <out>/<gif>/NNNNN.png")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--only", nargs="+", choices=list(COMPOSERS), default=list(COMPOSERS))
    args = parser.parse_args(argv)
    camera = Camera()
    captions = {}
    for name in args.only:
        out = os.path.join(args.out, name)
        os.makedirs(out, exist_ok=True)
        captions[name] = COMPOSERS[name](args.raw, out, args.fps, camera)
        print(f"[compose_gifs] {name}: {len(os.listdir(out))} frames, caption: {captions[name]}")
    with open(os.path.join(args.out, "captions.json"), "w") as f:
        json.dump(captions, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())

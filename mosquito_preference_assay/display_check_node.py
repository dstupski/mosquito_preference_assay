#!/usr/bin/env python3
"""Put a test pattern on the display the config says the assay will use, so
you can confirm the projector is the one that lights up -- and that the
stimuli will land where the arena expects -- without running an experiment.

    ros2 launch mosquito_preference_assay display_check.launch.py
    ros2 launch mosquito_preference_assay display_check.launch.py \\
        params_file:=/abs/path/to/my_assay_params.yaml

It reads the SAME `config/assay_params.yaml` the assay reads, and hands the
display settings to `assay.settings()` -- the very function the real sketch
uses to pick a screen. So this is not a separate implementation that might
agree with the assay by luck; if the test pattern lands on the projector, the
stimulus will too.

What the pattern shows, and what each part is for:

  * corner brackets at the screen edges -- if one is missing or cut off, the
    projector is overscanning or the resolution is wrong;
  * the two stimulus circles, drawn at the diameter and the left/right centres
    the experiment file (plus any `*_center_px` override) resolves to, so you
    can check they sit where the arena wants them BEFORE running an animal;
  * a live frame counter and measured frame rate, which proves the sketch is
    actually rendering on that screen rather than showing a frozen window;
  * which display it actually opened on, next to which one was requested.

ALIGNMENT. A dashed rectangle marks the PROJECTION SURFACE -- the part of the
projector's output that actually falls on the surface you care about, which is
usually not the whole frame. Drag it over the real illuminated area, then place
the circles inside it. The two circles can be dragged to line them up with the
arena, and the positions saved -- so the rig is aligned by eye against the real thing
rather than by guessing pixel coordinates:

    drag either circle   move it
    drag inside the rect move the projection surface
    drag a rect corner   resize it
    [ and ]              shrink / grow both circles
    s                    save the current geometry to `out_file` and print it
    r                    reset to what the config says
    q or ESC             close

Saving writes a params snippet that can be pasted into `assay_params.yaml`
(or passed as its own params file), so the alignment you just did by hand is
what the next run uses.

Parameters (the display ones are shared with stimulus_publisher, so the same
params file drives both):
    experiment_file   string  ""       geometry comes from this; "" = built-in default
    fullscreen        bool    False    True for the mosquito-facing display
    monitor           string  ""       "" primary | "N" that display | "span"
    window_pos        string  ""       windowed only, "x,y" on the virtual desktop
    window_w/h         int     1200/800
    left_center_px / right_center_px  string  ""  override the experiment's centres
    duration_sec      double  0.0      0 = stay up until closed
    surface_px        string  ""       "x0,y0,x1,y1" starting rectangle;
                                       "" = a centred box inset from the screen
    live_file         string  ""       the file the ASSAY reads back; "" =
                                       config/display_geometry.local.yaml,
                                       "none" to not write it
    out_file          string  ""       dated archive `s` also writes:
                                         ""          -> ./<YYYYMMDD>_display_config.yaml
                                         a directory -> <dir>/<YYYYMMDD>_display_config.yaml
                                         a .yaml path -> exactly that
"""

import datetime
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

from . import assay
from .experiment import Experiment, ExperimentError
from .stimulus_publisher_node import _resolve_experiment_file

INK = 20
ACCENT = "#d95f02"


def _resolve_out_file(spec):
    """Where `s` writes. Date-stamped by default, because an alignment is a
    measurement of the rig on a particular day -- after the projector is
    bumped, the old numbers are wrong and you want both files to still exist.

    "" -> ./<date>_display_config.yaml · a directory -> that name inside it ·
    anything else -> used verbatim."""
    name = f"{datetime.date.today():%Y%m%d}_display_config.yaml"
    spec = (spec or "").strip()
    if not spec:
        return str(Path.cwd() / name)
    path = Path(spec).expanduser()
    if path.is_dir() or spec.endswith("/"):
        return str(path / name)
    return str(path)


def _resolve_live_file(spec):
    """The file the assay reads back. Defaults to the package's
    config/display_geometry.local.yaml -- gitignored, and layered over the
    params file by every launch file. "" or "none" disables writing it."""
    spec = (spec or "").strip()
    if spec.lower() == "none":
        return ""
    if spec:
        return str(Path(spec).expanduser())
    try:
        from .config_paths import resolve_config
        shipped = resolve_config("assay_params.yaml")
        return str(Path(shipped).resolve().parent / "display_geometry.local.yaml")
    except Exception:                                   # noqa: BLE001
        return "display_geometry.local.yaml"


def _rect_param(node, name):
    raw = str(node.declare_parameter(name, "").value).strip()
    if not raw:
        return None
    try:
        x0, y0, x1, y1 = (float(v) for v in raw.split(","))
    except ValueError:
        raise ValueError(f"{name} must be 'x0,y0,x1,y1' pixels, got {raw!r}") from None
    return [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]


def _center_param(node, name):
    raw = str(node.declare_parameter(name, "").value).strip()
    if not raw:
        return None
    try:
        x, y = (float(v) for v in raw.split(","))
    except ValueError:
        raise ValueError(f"{name} must be 'x,y' pixels, got {raw!r}") from None
    return [x, y]


class DisplayCheck(Node):

    def __init__(self):
        super().__init__("display_check")

        experiment_file = str(self.declare_parameter("experiment_file", "").value).strip()
        fullscreen = bool(self.declare_parameter("fullscreen", False).value)
        monitor = str(self.declare_parameter("monitor", "").value).strip() or None
        window_w = int(self.declare_parameter("window_w", 1200).value)
        window_h = int(self.declare_parameter("window_h", 800).value)
        self.duration_sec = float(self.declare_parameter("duration_sec", 0.0).value)
        self.out_file = _resolve_out_file(
            str(self.declare_parameter("out_file", "").value))
        self.surface_px = _rect_param(self, "surface_px")
        self.live_file = _resolve_live_file(
            str(self.declare_parameter("live_file", "").value))
        window_pos = _center_param(self, "window_pos")
        left_center = _center_param(self, "left_center_px")
        right_center = _center_param(self, "right_center_px")

        # the same lookup stimulus_publisher does, so a bare name resolves
        # against the installed experiments/ folder exactly as it would there
        path = _resolve_experiment_file(experiment_file)
        self.experiment = Experiment.from_file(path) if path else Experiment.default()

        # the same configuration the real sketch runs on
        assay.configure(
            fullscreen=fullscreen, monitor=monitor, window_pos=window_pos,
            window_w=window_w, window_h=window_h, experiment=self.experiment,
            left_center_px=left_center, right_center_px=right_center,
        )
        self.requested = monitor
        self.get_logger().info(
            f"display check: fullscreen={fullscreen}, monitor={monitor!r}, "
            f"experiment={self.experiment.name!r} "
            f"(circle {self.experiment.circle_diameter_px} px)"
        )
        self.get_logger().info(f"'s' will save to {self.out_file}")


def run_pattern(node):
    """Draw the pattern. settings() is assay's own, so the screen is chosen by
    exactly the code the assay uses."""
    import py5

    experiment = node.experiment
    state = {"start": None, "display": None, "left": None, "right": None,
             "surface": None, "grab": None,
             "diameter": float(experiment.circle_diameter_px), "dragging": None,
             "saved": ""}

    def reset_geometry():
        state["left"] = list(assay._resolve_center(
            "left", experiment, py5.width, py5.height))
        state["right"] = list(assay._resolve_center(
            "right", experiment, py5.width, py5.height))
        state["diameter"] = float(experiment.circle_diameter_px)
        if node.surface_px:
            state["surface"] = list(node.surface_px)
        else:
            inset_x, inset_y = py5.width * 0.12, py5.height * 0.12
            state["surface"] = [inset_x, inset_y,
                                py5.width - inset_x, py5.height - inset_y]

    def setup():
        py5.frame_rate(60)
        py5.text_align(py5.CENTER, py5.CENTER)
        if not assay._cfg["fullscreen"] and assay._cfg["window_pos"]:
            x, y = assay._cfg["window_pos"]
            py5.window_move(int(x), int(y))
        state["display"] = assay._which_display()
        state["start"] = py5.millis()
        reset_geometry()
        where = state["display"]
        if where and "index" in where:
            node.get_logger().info(
                f"opened on display {where['index']}: {where['width']}x{where['height']} "
                f"at +{where['x']}+{where['y']} ({where['id']})")
        else:
            node.get_logger().info(f"opened: {where}")

    def draw():
        width, height = py5.width, py5.height
        py5.background(experiment.background_gray)

        _corner_brackets(py5, width, height)
        _surface_rect(py5, state["surface"],
                      held=str(state["dragging"] or "").startswith("surface"))
        for slot, label in (("left", "LEFT"), ("right", "RIGHT")):
            _stimulus_outline(py5, state[slot], state["diameter"], label,
                              held=state["dragging"] == slot)

        _readout(py5, node, state, width, height)

        if node.duration_sec > 0 and (py5.millis() - state["start"]) / 1000.0 >= node.duration_sec:
            py5.exit_sketch()

    def mouse_pressed():
        mx, my = py5.mouse_x, py5.mouse_y
        x0, y0, x1, y1 = state["surface"]
        # corners first: they are small targets sitting on the rectangle's
        # edge, so anything else would shadow them
        for index, (cx, cy) in enumerate(((x0, y0), (x1, y0), (x0, y1), (x1, y1))):
            if py5.dist(mx, my, cx, cy) <= 22:
                state["dragging"] = f"surface-corner-{index}"
                return
        radius = state["diameter"] / 2
        for slot in ("left", "right"):
            cx, cy = state[slot]
            if py5.dist(mx, my, cx, cy) <= radius:
                state["dragging"] = slot
                return
        if x0 <= mx <= x1 and y0 <= my <= y1:
            state["dragging"] = "surface-move"
            state["grab"] = (mx - x0, my - y0, x1 - x0, y1 - y0)
            return
        state["dragging"] = None

    def mouse_dragged():
        holding = state["dragging"]
        if not holding:
            return
        mx, my = float(py5.mouse_x), float(py5.mouse_y)
        if holding in ("left", "right"):
            state[holding] = [mx, my]
        elif holding == "surface-move":
            off_x, off_y, w, h = state["grab"]
            state["surface"] = [mx - off_x, my - off_y, mx - off_x + w, my - off_y + h]
        elif holding.startswith("surface-corner-"):
            index = int(holding.rsplit("-", 1)[1])
            x0, y0, x1, y1 = state["surface"]
            if index in (0, 2):
                x0 = mx
            else:
                x1 = mx
            if index in (0, 1):
                y0 = my
            else:
                y1 = my
            state["surface"] = [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]

    def mouse_released():
        holding = state["dragging"]
        if holding in ("left", "right"):
            cx, cy = state[holding]
            node.get_logger().info(f"{holding}_center_px = {cx:.0f},{cy:.0f}")
        elif holding:
            x0, y0, x1, y1 = state["surface"]
            node.get_logger().info(
                f"surface_px = {x0:.0f},{y0:.0f},{x1:.0f},{y1:.0f}")
        state["dragging"] = None

    def key_pressed():
        key = str(py5.key).lower()
        if key in ("q", "\x1b"):
            py5.exit_sketch()
        elif key == "r":
            reset_geometry()
            state["saved"] = "reset to the config"
            node.get_logger().info("reset to the configured geometry")
        elif key == "[":
            state["diameter"] = max(10.0, state["diameter"] - 4)
        elif key == "]":
            state["diameter"] += 4
        elif key == "s":
            state["saved"] = _save(node, state)

    py5.run_sketch(sketch_functions={
        "settings": assay.settings, "setup": setup, "draw": draw,
        "key_pressed": key_pressed, "mouse_pressed": mouse_pressed,
        "mouse_dragged": mouse_dragged, "mouse_released": mouse_released},
        block=True)


def _save(node, state):
    """Write the dragged geometry twice.

    `live_file` is the one the assay actually reads -- every launch file layers
    it over the params file, so pressing `s` is the whole workflow: align, save,
    launch, and the circles are where you put them at the size you set. It is
    gitignored, so a pull cannot move your arena.

    `out_file` is a dated archive of the same numbers. Alignment is a
    measurement of the rig on a day; when the projector is next knocked the old
    values are wrong but you still want to know what they were."""
    left, right = state["left"], state["right"]
    x0, y0, x1, y1 = state["surface"]
    text = (
        "# Written by display_check after aligning against the arena by hand.\n"
        "# Paste into config/assay_params.yaml, or pass with --params-file.\n"
        "/**:\n"
        "  ros__parameters:\n"
        f"    left_center_px: \"{left[0]:.0f},{left[1]:.0f}\"\n"
        f"    right_center_px: \"{right[0]:.0f},{right[1]:.0f}\"\n"
        "\n"
        "    # The part of the projector's output that lands on the surface of\n"
        "    # interest. display_check reads this back so the rectangle starts\n"
        "    # where you left it. NOTE the assay does not consume it yet -- the\n"
        "    # stimulus centres above are absolute screen pixels, and it is on\n"
        "    # you to keep them inside this rectangle.\n"
        f"    surface_px: \"{x0:.0f},{y0:.0f},{x1:.0f},{y1:.0f}\"\n"
        "\n"
        "    # Overrides the experiment file's display.circle_diameter_px.\n"
        "    # The angular size the animal sees is the scientific variable;\n"
        "    # the pixels that achieve it depend on this rig's throw distance,\n"
        "    # which is why it belongs here and not in the experiment.\n"
        f"    circle_diameter_px: {state['diameter']:.1f}\n"
    )
    written = []
    for path in (node.live_file, node.out_file):
        if not path:
            continue
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w") as handle:
                handle.write(text)
            written.append(path)
        except OSError as exc:
            node.get_logger().error(f"could not write {path}: {exc}")
    if not written:
        return "could not write anything"
    node.get_logger().info(
        "saved:\n  " + "\n  ".join(written) + f"\n\n{text}")
    return "saved - the assay will use this on its next launch"


def _corner_brackets(py5, width, height):
    """If a bracket is clipped, the screen is overscanning or mis-sized."""
    arm, inset, weight = min(width, height) * 0.06, 8, 4
    py5.stroke(INK)
    py5.stroke_weight(weight)
    py5.no_fill()
    for x, y, dx, dy in ((inset, inset, 1, 1), (width - inset, inset, -1, 1),
                         (inset, height - inset, 1, -1), (width - inset, height - inset, -1, -1)):
        py5.line(x, y, x + arm * dx, y)
        py5.line(x, y, x, y + arm * dy)
    py5.stroke_weight(1)
    py5.line(width / 2, height / 2 - arm, width / 2, height / 2 + arm)
    py5.line(width / 2 - arm, height / 2, width / 2 + arm, height / 2)


def _surface_rect(py5, rect, held=False):
    """The part of the projector's output that lands on the surface of
    interest -- usually not the whole frame, so the stimuli have to be placed
    inside it rather than relative to the screen."""
    x0, y0, x1, y1 = rect
    py5.no_fill()
    py5.stroke(INK)
    py5.stroke_weight(3 if held else 2)
    dash = 14
    for x in _dashes(x0, x1, dash):
        py5.line(x[0], y0, x[1], y0)
        py5.line(x[0], y1, x[1], y1)
    for y in _dashes(y0, y1, dash):
        py5.line(x0, y[0], x0, y[1])
        py5.line(x1, y[0], x1, y[1])

    py5.no_stroke()
    py5.fill(INK)
    for cx, cy in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)):
        py5.ellipse(cx, cy, 13, 13)
    # both labels ride the TOP edge: below the rectangle they collide with the
    # key map whenever the surface is dragged near the bottom of the screen
    py5.text_size(14)
    py5.text("projection surface", (x0 + x1) / 2, y0 - 26)
    py5.text_size(12)
    py5.text(f"{x0:.0f},{y0:.0f}  to  {x1:.0f},{y1:.0f}"
             f"   ({x1 - x0:.0f} x {y1 - y0:.0f} px)",
             (x0 + x1) / 2, y0 - 9)


def _dashes(start, end, dash):
    """Dash spans across [start, end], so the rectangle reads as a guide
    rather than as something being displayed to the animal."""
    spans, position = [], start
    while position < end:
        spans.append((position, min(position + dash, end)))
        position += dash * 2
    return spans


def _stimulus_outline(py5, centre, diameter, label, held=False):
    """Where a stimulus will actually be drawn -- check it against the arena."""
    cx, cy = centre
    py5.no_fill()
    py5.stroke(INK)
    py5.stroke_weight(5 if held else 3)
    py5.ellipse(cx, cy, diameter, diameter)
    py5.stroke_weight(1)
    py5.line(cx - diameter / 2, cy, cx + diameter / 2, cy)
    py5.line(cx, cy - diameter / 2, cx, cy + diameter / 2)
    py5.no_stroke()
    py5.fill(INK)
    py5.text_size(max(14, diameter * 0.13))
    py5.text(label, cx, cy - diameter * 0.66)
    py5.text_size(max(11, diameter * 0.085))
    py5.text(f"{cx:.0f}, {cy:.0f} px   d={diameter:.0f}", cx, cy + diameter * 0.66)


def _readout(py5, node, state, width, height):
    where = state["display"] or {}
    if "index" in where:
        line = (f"display {where['index']}  -  {where['width']}x{where['height']} "
                f"at +{where['x']}+{where['y']}  ({where['id']})")
    elif where.get("spanning"):
        line = f"spanning {where.get('attached_displays', '?')} displays"
    else:
        line = f"windowed  -  {width}x{height}"
    requested = node.requested if node.requested else "(primary)"

    py5.no_stroke()
    py5.fill(INK)
    py5.text_size(max(15, height * 0.022))
    py5.text(line, width / 2, height * 0.12)
    py5.text_size(max(12, height * 0.016))
    py5.text(f"requested monitor:={requested}      sketch surface {width}x{height}",
             width / 2, height * 0.165)
    py5.text(f"frame {py5.frame_count}   {py5.get_frame_rate():.0f} fps",
             width / 2, height * 0.86)
    py5.text("drag circles / the surface rect  ·  corners resize  ·  "
             "[ ] size  ·  s save  ·  r reset  ·  q quit",
             width / 2, height * 0.91)
    if state.get("saved"):
        py5.text(state["saved"], width / 2, height * 0.955)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DisplayCheck()
    except (ExperimentError, ValueError) as exc:
        print(f"[display_check] {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(2)
    try:
        run_pattern(node)
    except ValueError as exc:            # an impossible `monitor`, reported clearly
        print(f"[display_check] {exc}", file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(2)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

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

ALIGNMENT. The two circles can be dragged to line them up with the arena, and
the positions saved -- so the rig is aligned by eye against the real thing
rather than by guessing pixel coordinates:

    drag either circle   move it
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
    out_file          string  display_alignment.yaml   where `s` saves to
"""

import sys

import rclpy
from rclpy.node import Node

from . import assay
from .experiment import Experiment, ExperimentError
from .stimulus_publisher_node import _resolve_experiment_file

INK = 20
ACCENT = "#d95f02"


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
        self.out_file = str(self.declare_parameter(
            "out_file", "display_alignment.yaml").value)
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


def run_pattern(node):
    """Draw the pattern. settings() is assay's own, so the screen is chosen by
    exactly the code the assay uses."""
    import py5

    experiment = node.experiment
    state = {"start": None, "display": None, "left": None, "right": None,
             "diameter": float(experiment.circle_diameter_px), "dragging": None,
             "saved": ""}

    def reset_geometry():
        state["left"] = list(assay._resolve_center(
            "left", experiment, py5.width, py5.height))
        state["right"] = list(assay._resolve_center(
            "right", experiment, py5.width, py5.height))
        state["diameter"] = float(experiment.circle_diameter_px)

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
        for slot, label in (("left", "LEFT"), ("right", "RIGHT")):
            _stimulus_outline(py5, state[slot], state["diameter"], label,
                              held=state["dragging"] == slot)

        _readout(py5, node, state, width, height)

        if node.duration_sec > 0 and (py5.millis() - state["start"]) / 1000.0 >= node.duration_sec:
            py5.exit_sketch()

    def mouse_pressed():
        radius = state["diameter"] / 2
        for slot in ("left", "right"):
            cx, cy = state[slot]
            if py5.dist(py5.mouse_x, py5.mouse_y, cx, cy) <= radius:
                state["dragging"] = slot
                return
        state["dragging"] = None

    def mouse_dragged():
        if state["dragging"]:
            state[state["dragging"]] = [float(py5.mouse_x), float(py5.mouse_y)]

    def mouse_released():
        if state["dragging"]:
            cx, cy = state[state["dragging"]]
            node.get_logger().info(
                f"{state['dragging']}_center_px = {cx:.0f},{cy:.0f}")
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
    """Write the dragged geometry as a params snippet that can be pasted into
    assay_params.yaml, or passed straight back as its own params file."""
    left, right = state["left"], state["right"]
    text = (
        "# Written by display_check after aligning against the arena by hand.\n"
        "# Paste into config/assay_params.yaml, or pass with --params-file.\n"
        "/**:\n"
        "  ros__parameters:\n"
        f"    left_center_px: \"{left[0]:.0f},{left[1]:.0f}\"\n"
        f"    right_center_px: \"{right[0]:.0f},{right[1]:.0f}\"\n"
        "\n"
        f"# circle_diameter_px is experiment geometry, so it belongs in the\n"
        f"# experiment file's display: block, not here:\n"
        f"#   display:\n"
        f"#     circle_diameter_px: {state['diameter']:.0f}\n"
    )
    try:
        with open(node.out_file, "w") as handle:
            handle.write(text)
    except OSError as exc:
        node.get_logger().error(f"could not write {node.out_file}: {exc}")
        return f"could not write {node.out_file}"
    node.get_logger().info(f"saved to {node.out_file}:\n{text}")
    return f"saved to {node.out_file}"


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
    py5.text("drag a circle to align  ·  [ ] size  ·  s save  ·  r reset  ·  q quit",
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

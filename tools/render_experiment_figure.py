#!/usr/bin/env python3
"""Render a slide graphic explaining the assay: the camera view with the
detection that fires the trial, beside what the mosquito is shown, plus the
design in words.

    python3 tools/render_experiment_figure.py \\
        --session /path/to/session --out experiment.mp4

The camera half is real footage run through the real `detection.py`; the
display half is the real stimulus classes drawing the pair the experiment
file draws. Nothing here is a mock-up, so the graphic cannot drift from what
the rig does.

The narrative comes out of the data rather than being staged: the animal is
tracked from the first frame, and the trial fires when it has been inside the
trigger zone for `consecutive_frames` frames -- which in the bundled footage
takes about two seconds, so the "armed, waiting" phase is genuinely waiting.

    --out x.mp4   video (needs ffmpeg) -- the phases play out in time
    --out x.png   a single still at the moment of detection, for a static slide

Options:
    --session DIR        directory holding cam_a/ and cam_b/     (required)
    --camera cam_a       which camera's view to show
    --experiment PATH    experiment file (default: experiments/ten_stimulus_panel.yaml)
    --left / --right     stimulus names from that file; default picks a
                         visually contrasting pair
    --trigger-zone       "x0,y0,x1,y1" px, the region that fires the trial
    --consecutive N      frames inside the zone before it fires (default 3)
    --trial-sec          how long to show the trial (default 15)
    --pre-sec            seconds of ARMED to show before the trigger (default 2)
    --fps                playback rate (default 30)
    --roi                detection ROI (default: the tuned cam_a arena ROI)
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, Rectangle  # noqa: E402

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from mosquito_preference_assay import assay  # noqa: E402

assay._ensure_java_home()  # before anything that imports py5 (Experiment does)

from mosquito_preference_assay.detection import find_candidates, parse_roi  # noqa: E402
from mosquito_preference_assay.experiment import Experiment  # noqa: E402

DEFAULT_ROI = "340,40,1260,1070"
DEFAULT_ZONE = "550,481,750,681"
# the scoring zones: the volume in front of each stimulus. Which side the
# animal spends more time in is what eventually decides the trial.
DEFAULT_LEFT_ZONE = "380,300,680,750"
DEFAULT_RIGHT_ZONE = "920,300,1220,750"
ARMED, DETECTED, RUNNING = "armed", "detected", "running"

INK = "#1f2933"
MUTED = "#7b8794"
ACCENT = "#d95f02"
GOOD = "#22a06b"
LEFT = "#2563eb"
RIGHT = "#7c3aed"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--camera", default="cam_a")
    parser.add_argument("--experiment",
                        default=str(PACKAGE_ROOT / "experiments" / "ten_stimulus_panel.yaml"))
    parser.add_argument("--left", default="")
    parser.add_argument("--right", default="")
    parser.add_argument("--out", default="experiment.mp4")
    parser.add_argument("--trigger-zone", default=DEFAULT_ZONE)
    parser.add_argument("--left-zone", default=DEFAULT_LEFT_ZONE)
    parser.add_argument("--right-zone", default=DEFAULT_RIGHT_ZONE)
    parser.add_argument("--consecutive", type=int, default=3)
    parser.add_argument("--trial-sec", type=float, default=15.0)
    parser.add_argument("--pre-sec", type=float, default=2.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--roi", default=DEFAULT_ROI)
    parser.add_argument("--title", default="Two-choice visual preference assay")
    return parser.parse_args()


def track(session, camera, roi, limit):
    """Per-frame centroid, exactly as tracker_node does it: the first frame is
    the background, every later frame is differenced against it."""
    frames = sorted((Path(session) / camera).glob("*.bmp"))[:limit]
    if len(frames) < 2:
        raise SystemExit(f"need at least 2 frames in {session}/{camera}")
    kwargs = dict(diff_threshold=25, min_area=4.0, max_area=5000.0, morph_kernel=3)
    background = cv2.imread(str(frames[0]), cv2.IMREAD_UNCHANGED)
    out = []
    for path in frames[1:]:
        gray = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        found = find_candidates(gray, background, roi=roi, **kwargs)
        out.append({"gray": gray,
                    "xy": (found[0]["cx"], found[0]["cy"]) if found else None})
    return out


def find_trigger(records, zone, consecutive):
    """First frame with `consecutive` consecutive detections inside the zone --
    the same debounce mosquito_detector_node applies."""
    x0, y0, x1, y1 = zone
    run = 0
    for index, record in enumerate(records):
        xy = record["xy"]
        inside = xy is not None and x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1
        run = run + 1 if inside else 0
        if run >= consecutive:
            return index
    return None


def render_display_frames(experiment_path, left_name, right_name, n_frames, fps, out_dir):
    """Draw the left/right display with the REAL stimulus classes, in py5."""
    import py5
    import random as _random
    from mosquito_preference_assay.param_spec import resolve_params
    from mosquito_preference_assay.stimulus_types import build_stimulus

    experiment = Experiment.from_file(experiment_path)
    rng = _random.Random(0)
    width, height = 900, 560
    diameter = experiment.circle_diameter_px
    state = {"i": 0, "left": None, "right": None}
    out_dir.mkdir(parents=True, exist_ok=True)
    for old in out_dir.glob("d_*.png"):
        old.unlink()

    def settings():
        py5.size(width, height)

    def setup():
        py5.frame_rate(1000)
        py5.noise_seed(0)
        py5.random_seed(0)
        for slot, name in (("left", left_name), ("right", right_name)):
            spec = experiment.stimuli[name]
            state[slot] = build_stimulus(
                spec.type, diameter, resolve_params(spec.params, rng), rng)

    def draw():
        if state["i"] >= n_frames:
            py5.exit_sketch()
            return
        t = state["i"] / fps
        py5.background(experiment.background_gray)
        state["left"].display(width * 0.27, height / 2, t)
        state["right"].display(width * 0.73, height / 2, t)
        py5.save_frame(str(out_dir / f"d_{state['i']:05d}.png"))
        state["i"] += 1

    py5.run_sketch(sketch_functions={"settings": settings, "setup": setup, "draw": draw},
                   block=True)
    return experiment


def main():
    args = parse_args()
    roi = parse_roi(args.roi)
    zone = parse_roi(args.trigger_zone)
    experiment = Experiment.from_file(args.experiment)

    left_name, right_name = args.left, args.right
    if not left_name or not right_name:
        # a contrasting default: something moving against the static control
        names = list(experiment.stimuli)
        left_name = left_name or ("static_black" if "static_black" in names else names[0])
        right_name = right_name or (
            "telescope_outward" if "telescope_outward" in names else names[-1])
    for name in (left_name, right_name):
        if name not in experiment.stimuli:
            raise SystemExit(f"{name!r} is not in {args.experiment}; "
                             f"available: {', '.join(experiment.stimuli)}")

    pre_frames = int(args.pre_sec * args.fps)
    trial_frames = int(args.trial_sec * args.fps)

    print(f"tracking {args.camera} ...")
    records = track(args.session, args.camera, roi, limit=1 + pre_frames + trial_frames + 60)
    trigger = find_trigger(records, zone, args.consecutive)
    if trigger is None:
        raise SystemExit("the animal never entered the trigger zone -- widen "
                         "--trigger-zone or use a different session")
    print(f"  trigger at frame {trigger} "
          f"({trigger / args.fps:.1f}s in), after {args.consecutive} frames in the zone")

    start = max(0, trigger - pre_frames)
    records = records[start:trigger + trial_frames]
    trigger_index = trigger - start

    frames_dir = Path(args.out).resolve().parent / "_exp_frames"
    print(f"drawing the display ({left_name} vs {right_name}) ...")
    render_display_frames(args.experiment, left_name, right_name,
                          len(records) - trigger_index, args.fps, frames_dir)

    compose(records, trigger_index, frames_dir, roi, zone, experiment,
            left_name, right_name, args)


def compose(records, trigger_index, frames_dir, roi, zone, experiment,
            left_name, right_name, args):
    still = Path(args.out).suffix.lower() == ".png"
    x0, y0, x1, y1 = roi
    step = 2

    fig = plt.figure(figsize=(16, 9), dpi=120)
    fig.patch.set_facecolor("white")
    grid = fig.add_gridspec(3, 2, height_ratios=[1, 0.10, 0.17], width_ratios=[1, 1],
                            left=0.035, right=0.965, top=0.845, bottom=0.045,
                            wspace=0.09, hspace=0.05)
    ax_cam = fig.add_subplot(grid[0, 0])
    ax_disp = fig.add_subplot(grid[0, 1])
    ax_time = fig.add_subplot(grid[1, :])
    ax_text = fig.add_subplot(grid[2, :])
    for ax in (ax_time, ax_text):
        ax.axis("off")

    fig.text(0.035, 0.955, args.title, fontsize=23, color=INK, weight="bold",
             va="top")
    fig.text(0.035, 0.907,
             "a detection in the arena triggers one trial: two stimuli, left and right",
             fontsize=13.5, color=MUTED, va="top")

    cam_view = records[0]["gray"][y0:y1:step, x0:x1:step]
    im_cam = ax_cam.imshow(cam_view, cmap="gray", vmin=0, vmax=255)
    ax_cam.set_xticks([])
    ax_cam.set_yticks([])
    ax_cam.set_title(f"arena camera ({args.camera})", fontsize=13, color=INK, pad=8)
    ax_cam.add_patch(Rectangle(((zone[0] - x0) / step, (zone[1] - y0) / step),
                               (zone[2] - zone[0]) / step, (zone[3] - zone[1]) / step,
                               fill=False, ec=ACCENT, lw=2.0, ls="--"))
    ax_cam.text((zone[0] - x0) / step + 6, (zone[1] - y0) / step - 10,
                "trigger zone", color=ACCENT, fontsize=11, weight="bold")
    score_zones = [(parse_roi(args.left_zone), LEFT, "in front of\nleft stimulus"),
                   (parse_roi(args.right_zone), RIGHT, "in front of\nright stimulus")]
    zone_patches = []
    for (zx0, zy0, zx1, zy1), colour, label in score_zones:
        patch = Rectangle(((zx0 - x0) / step, (zy0 - y0) / step),
                          (zx1 - zx0) / step, (zy1 - zy0) / step,
                          fill=True, fc=colour, ec=colour, lw=2.0, alpha=0.10)
        ax_cam.add_patch(patch)
        zone_patches.append(patch)
        ax_cam.text(((zx0 + zx1) / 2 - x0) / step, (zy1 - y0) / step + 12,
                    label, color=colour, fontsize=10.5, ha="center", va="top",
                    weight="bold", linespacing=1.25)

    marker, = ax_cam.plot([], [], "o", mfc="none", mec="#ffd400", mew=2.5, ms=20)
    badge = ax_cam.text(0.02, 0.975, "", transform=ax_cam.transAxes, va="top",
                        fontsize=13, weight="bold", color="white",
                        bbox=dict(boxstyle="round,pad=0.4", fc=MUTED, ec="none"))

    display_paths = sorted(frames_dir.glob("d_*.png"))
    blank = np.full_like(np.asarray(plt.imread(str(display_paths[0]))),
                         experiment.background_gray / 255.0)
    im_disp = ax_disp.imshow(blank)
    ax_disp.set_xticks([])
    ax_disp.set_yticks([])
    ax_disp.set_title("what the mosquito is shown", fontsize=13, color=INK, pad=8)
    for frac, label in ((0.27, "left stimulus"), (0.73, "right stimulus")):
        ax_disp.text(frac, -0.045, label, transform=ax_disp.transAxes, ha="center",
                     va="top", fontsize=13, color=INK, weight="bold")
    disp_note = ax_disp.text(0.5, 0.5, "", transform=ax_disp.transAxes, ha="center",
                             va="center", fontsize=15, color=MUTED, style="italic")

    arrow = FancyArrowPatch((0.487, 0.60), (0.513, 0.60), transform=fig.transFigure,
                            arrowstyle="-|>", mutation_scale=26, lw=2.5,
                            color=ACCENT, alpha=0.0)
    fig.patches.append(arrow)

    _design_panel(ax_text, experiment, left_name, right_name, args)
    timeline = _timeline(ax_time, args, len(records), trigger_index)

    def side_of(xy):
        """Which scoring zone the animal is in, if any."""
        if xy is None:
            return None
        for index, ((zx0, zy0, zx1, zy1), _colour, _label) in enumerate(score_zones):
            if zx0 <= xy[0] <= zx1 and zy0 <= xy[1] <= zy1:
                return index
        return None

    # precomputed, so a single-frame still reports the same numbers a video does
    dwell = []
    totals = [0, 0]
    for index, record in enumerate(records):
        if index >= trigger_index:
            side = side_of(record["xy"])
            if side is not None:
                totals[side] += 1
        dwell.append((totals[0] / args.fps, totals[1] / args.fps))

    # inside the image, over the empty floor of the arena, so it cannot
    # collide with the timeline underneath the panel
    tally = ax_cam.text(0.5, 0.045, "", transform=ax_cam.transAxes, ha="center",
                        va="bottom", fontsize=12.5, color=INK,
                        bbox=dict(boxstyle="round,pad=0.35", fc="white",
                                  ec="#d3d8de", alpha=0.92))

    def update(i):
        record = records[i]
        im_cam.set_data(record["gray"][y0:y1:step, x0:x1:step])
        side = side_of(record["xy"])
        if record["xy"]:
            marker.set_data([(record["xy"][0] - x0) / step],
                            [(record["xy"][1] - y0) / step])
            marker.set_mec(score_zones[side][1] if side is not None else "#ffd400")
            marker.set_mew(3.5 if side is not None else 2.5)
        else:
            marker.set_data([], [])
        for index, patch in enumerate(zone_patches):
            patch.set_alpha(0.28 if side == index else
                            (0.10 if i >= trigger_index else 0.05))

        left_s, right_s = dwell[i]
        if i >= trigger_index:
            tally.set_text(f"time in front:   left {left_s:4.1f} s      "
                           f"right {right_s:4.1f} s")
        else:
            tally.set_text("time in front:   scoring starts at the trigger")

        if i < trigger_index:
            phase, colour, label = ARMED, MUTED, "ARMED — waiting for a mosquito"
            im_disp.set_data(blank)
            disp_note.set_text("background only\n(no stimuli yet)")
            arrow.set_alpha(0.0)
        else:
            j = i - trigger_index
            phase = DETECTED if j < int(0.7 * args.fps) else RUNNING
            colour = ACCENT if phase is DETECTED else GOOD
            label = ("MOSQUITO DETECTED → trial starts"
                     if phase is DETECTED else
                     f"TRIAL RUNNING — {j / args.fps:4.1f} / {args.trial_sec:.0f} s")
            im_disp.set_data(plt.imread(str(display_paths[min(j, len(display_paths) - 1)])))
            disp_note.set_text("")
            arrow.set_alpha(1.0 if phase is DETECTED else 0.35)
        badge.set_text(label)
        badge.get_bbox_patch().set_facecolor(colour)
        timeline(i)

    out = Path(args.out).resolve()
    if still:
        update(trigger_index + int(0.3 * args.fps))
        fig.savefig(out, facecolor="white")
        print(f"done -- {out}")
    else:
        video_dir = out.parent / "_exp_video"
        video_dir.mkdir(parents=True, exist_ok=True)
        for old in video_dir.glob("v_*.png"):
            old.unlink()
        print(f"compositing {len(records)} frames ...")
        for i in range(len(records)):
            update(i)
            fig.savefig(video_dir / f"v_{i:05d}.png", facecolor="white")
            if (i + 1) % 100 == 0:
                print(f"  {i + 1}/{len(records)}", flush=True)
        _encode(video_dir, out, args.fps)
        shutil.rmtree(video_dir, ignore_errors=True)
    plt.close(fig)
    shutil.rmtree(frames_dir, ignore_errors=True)


def _design_panel(ax, experiment, left_name, right_name, args):
    """The four steps, as one centred row. Positions come from measuring the
    rendered text rather than being guessed, so the row stays centred whatever
    the wording and font size are."""
    steps = [
        "mosquito triggers detection",
        "two random stimuli, one left and one right",
        f"{args.trial_sec:.0f} s trials",
        "winner stimulus determined",
    ]
    size, arrow, gap = 15, "\u2192", 0.012
    figure = ax.figure
    figure.canvas.draw()
    renderer = figure.canvas.get_renderer()

    def width_in_axes(text_object):
        box = text_object.get_window_extent(renderer=renderer)
        inverse = ax.transAxes.inverted()
        return (inverse.transform((box.width, 0)) - inverse.transform((0, 0)))[0]

    pieces = []
    for index, step in enumerate(steps):
        if index:
            pieces.append(ax.text(0, 0.62, arrow, transform=ax.transAxes, fontsize=18,
                                  color=ACCENT, va="center", weight="bold"))
        pieces.append(ax.text(0, 0.62, step, transform=ax.transAxes, fontsize=size,
                              color=INK, va="center"))

    widths = [width_in_axes(piece) for piece in pieces]
    x = (1.0 - (sum(widths) + gap * 2 * (len(steps) - 1))) / 2.0
    for piece, width in zip(pieces, widths):
        piece.set_x(x)
        x += width + (gap if piece.get_text() == arrow else gap)


def _timeline(ax, args, n_frames, trigger_index):
    y = 0.34
    ax.plot([0.006, 0.994], [y, y], transform=ax.transAxes, color="#d3d8de", lw=3.5,
            solid_capstyle="round", clip_on=False)
    frac = trigger_index / max(1, n_frames - 1)
    x_trigger = 0.006 + frac * 0.988
    ax.plot([x_trigger], [y], "o", transform=ax.transAxes, color=ACCENT, ms=10,
            clip_on=False, zorder=3)
    ax.text(0.006, y + 0.30, "armed", transform=ax.transAxes, fontsize=11, color=MUTED)
    ax.text(x_trigger, y + 0.30, "detection", transform=ax.transAxes, fontsize=11,
            color=ACCENT, ha="center", weight="bold")
    ax.text(0.994, y + 0.30, f"trial ends ({args.trial_sec:.0f} s)",
            transform=ax.transAxes, fontsize=11, color=MUTED, ha="right")
    head, = ax.plot([0.006], [y], "o", transform=ax.transAxes, color=INK, ms=8,
                    clip_on=False, zorder=4)

    def set_position(i):
        head.set_data([0.012 + (i / max(1, n_frames - 1)) * 0.976], [y])
    return set_position


def _encode(frames_dir, out, fps):
    print(f"encoding {out} ...")
    subprocess.run([
        "ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps),
        "-i", str(frames_dir / "v_%05d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
        "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", str(out),
    ], check=True)
    print(f"done -- {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()

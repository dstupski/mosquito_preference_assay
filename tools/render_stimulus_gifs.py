#!/usr/bin/env python3
"""Render an animated GIF of every stimulus in an experiment file -- for talks,
posters, and checking at a glance that a stimulus does what its name says.

The frames come from the REAL stimulus classes running in a real py5 sketch,
driven from the experiment YAML, so a GIF cannot drift from what the assay
actually puts on screen. Change a parameter in the YAML, re-run this, and the
GIF changes with it.

    python3 tools/render_stimulus_gifs.py \\
        --experiment experiments/ten_stimulus_panel.yaml --out-dir /tmp/gifs

Needs py5 (so a Java 17 JVM) and a display, same as the assay itself, plus
Pillow to assemble the GIFs. Renders offscreen-ish: a sketch window does open,
but nothing depends on it being visible or focused.

Options:
    --experiment PATH   experiment YAML (required)
    --out-dir DIR       where the GIFs go (default: ./stimulus_gifs)
    --seconds FLOAT     duration of each GIF (default 3.0)
    --fps INT           frames per second (default 20)
    --size INT          canvas edge in px; default = circle diameter + 80
    --only NAME [...]   render just these stimuli, by their YAML names
    --keep-frames       leave the intermediate PNGs on disk
"""

import argparse
import random
import shutil
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from mosquito_preference_assay import assay  # noqa: E402
from mosquito_preference_assay.experiment import Experiment  # noqa: E402
from mosquito_preference_assay.param_spec import resolve_params  # noqa: E402
from mosquito_preference_assay.stimulus_types import build_stimulus  # noqa: E402

assay._ensure_java_home()  # py5 needs a Java 17 JVM; same fallback the assay uses

import py5  # noqa: E402  -- must come after JAVA_HOME is settled


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True)
    parser.add_argument("--out-dir", default="stimulus_gifs")
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--size", type=int, default=0)
    parser.add_argument("--only", nargs="+", default=None)
    parser.add_argument("--keep-frames", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    experiment = Experiment.from_file(args.experiment)
    rng = random.Random(0)  # fixed: re-rendering gives byte-identical GIFs

    names = list(experiment.stimuli)
    if args.only:
        unknown = [n for n in args.only if n not in experiment.stimuli]
        if unknown:
            raise SystemExit(
                f"not in {args.experiment}: {', '.join(unknown)}\n"
                f"available: {', '.join(names)}")
        names = list(args.only)

    diameter = experiment.circle_diameter_px
    background_gray = experiment.background_gray
    canvas = args.size or int(diameter + 80)
    frames_per_gif = max(1, int(round(args.seconds * args.fps)))

    out_dir = Path(args.out_dir).resolve()
    frames_dir = out_dir / "_frames"
    out_dir.mkdir(parents=True, exist_ok=True)
    frames_dir.mkdir(parents=True, exist_ok=True)

    print(f"rendering {len(names)} stimuli from {args.experiment}")
    print(f"  {canvas}x{canvas} px, circle {diameter} px, "
          f"{args.seconds}s at {args.fps} fps = {frames_per_gif} frames each")

    state = {"index": 0, "frame": 0, "stimulus": None}

    def settings():
        py5.size(canvas, canvas)

    def setup():
        py5.frame_rate(1000)  # render as fast as the machine allows
        py5.noise_seed(0)
        py5.random_seed(0)

    def draw():
        if state["index"] >= len(names):
            py5.exit_sketch()
            return

        name = names[state["index"]]
        if state["stimulus"] is None:
            spec = experiment.stimuli[name]
            state["stimulus"] = build_stimulus(
                spec.type, diameter, resolve_params(spec.params, rng), rng)
            (frames_dir / name).mkdir(parents=True, exist_ok=True)
            print(f"  [{state['index'] + 1}/{len(names)}] {name} "
                  f"({spec.type})", flush=True)

        t = state["frame"] / args.fps
        py5.background(background_gray)
        state["stimulus"].display(canvas / 2, canvas / 2, t)
        py5.save_frame(str(frames_dir / name / f"f_{state['frame']:04d}.png"))

        state["frame"] += 1
        if state["frame"] >= frames_per_gif:
            state["index"] += 1
            state["frame"] = 0
            state["stimulus"] = None

    py5.run_sketch(sketch_functions={"settings": settings, "setup": setup, "draw": draw},
                   block=True)

    assemble(names, frames_dir, out_dir, args.fps)
    if not args.keep_frames:
        shutil.rmtree(frames_dir, ignore_errors=True)
    print(f"\ndone -- {len(names)} GIFs in {out_dir}")


def assemble(names, frames_dir, out_dir, fps):
    from PIL import Image

    print("\nassembling GIFs:")
    for name in names:
        paths = sorted((frames_dir / name).glob("f_*.png"))
        if not paths:
            print(f"  {name}: no frames rendered -- skipped")
            continue
        frames = [Image.open(p).convert("RGB") for p in paths]
        gif_path = out_dir / f"{name}.gif"
        frames[0].save(
            gif_path, save_all=True, append_images=frames[1:],
            duration=int(round(1000.0 / fps)), loop=0, optimize=True,
        )
        size_kb = gif_path.stat().st_size / 1024
        print(f"  {gif_path.name:<26} {len(frames):>3} frames  {size_kb:>7.0f} KB")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Render a presentation video: both camera feeds side by side with their
detections marked, next to the 3D flight path building up in real time.

    python3 tools/render_tracking_video.py \\
        --session /path/to/session \\
        --checkerboard-file /path/Checkerboard_<date>.npy \\
        --plumbline-file /path/Plumbline_<date>.npy \\
        --out flight.mp4

Uses the same detection.py and triangulator_node math the live pipeline runs,
frame for frame, so the positions and reprojection errors shown are the ones
the pipeline produces. It is rendered offline rather than screen-captured from
a live run, which is what makes it frame-accurate: nothing is dropped because
a node fell behind, and the playback rate is whatever reads best rather than
whatever the machine managed on the day.

Axis limits default to the full extent of the triangulated flight (plus a
margin) so the box is fixed for the whole video and nothing drifts; pass
--xlim/--ylim/--zlim to pin a shared box across several sessions instead.

Options:
    --session DIR          directory holding cam_a/ and cam_b/   (required)
    --checkerboard-file / --plumbline-file                        (required)
    --out PATH             .mp4 (needs ffmpeg) or .gif           (default flight.mp4)
    --fps INT              playback frame rate (default 30)
    --roi-a / --roi-b      "x0,y0,x1,y1" detection ROIs
    --max-reprojection-error-px FLOAT   drop worse triangulations (default 3.0)
    --trail INT            3D points kept visible (default 0 = the whole flight)
    --downscale INT        camera-frame shrink factor for rendering (default 2)
    --limit INT            only render the first N frames (for a quick look)
    --xlim / --ylim / --zlim  "min,max" to pin an axis
"""

import argparse
import sys
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")  # render to file; no display needed
import matplotlib.pyplot as plt  # noqa: E402

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from mosquito_preference_assay.detection import find_candidates, parse_roi  # noqa: E402
from mosquito_preference_assay.triangulator_node import (  # noqa: E402
    _load_checkerboard, _load_plumbline, triangulate_point,
)

DEFAULT_ROI_A = "340,40,1260,1070"
DEFAULT_ROI_B = "350,20,1370,1070"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--checkerboard-file", required=True)
    parser.add_argument("--plumbline-file", required=True)
    parser.add_argument("--out", default="flight.mp4")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--roi-a", default=DEFAULT_ROI_A)
    parser.add_argument("--roi-b", default=DEFAULT_ROI_B)
    parser.add_argument("--max-reprojection-error-px", type=float, default=3.0)
    parser.add_argument("--trail", type=int, default=0)
    parser.add_argument("--downscale", type=int, default=2)
    parser.add_argument("--full-frame", action="store_true",
                        help="show the whole camera frame instead of cropping to the "
                             "ROI (the ROI is essentially the arena, so cropping "
                             "makes the mosquito bigger and drops the surround)")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--xlim", default="")
    parser.add_argument("--ylim", default="")
    parser.add_argument("--zlim", default="")
    return parser.parse_args()


def _range(text):
    text = (text or "").strip()
    if not text:
        return None
    lo, hi = (float(v) for v in text.split(","))
    return lo, hi


def detect_session(session, roi_a, roi_b, limit):
    """Per-frame centroids for both cameras, the way tracker_node does it:
    first frame becomes the background, every later frame is differenced
    against it."""
    frames_a = sorted((session / "cam_a").glob("*.bmp"))
    frames_b = sorted((session / "cam_b").glob("*.bmp"))
    n = min(len(frames_a), len(frames_b))
    if limit:
        n = min(n, limit)
    if n < 2:
        raise SystemExit(f"need at least 2 frame pairs in {session}")

    kwargs = dict(diff_threshold=25, min_area=4.0, max_area=5000.0, morph_kernel=3)
    background_a = cv2.imread(str(frames_a[0]), cv2.IMREAD_UNCHANGED)
    background_b = cv2.imread(str(frames_b[0]), cv2.IMREAD_UNCHANGED)

    records = []
    for i in range(1, n):
        gray_a = cv2.imread(str(frames_a[i]), cv2.IMREAD_UNCHANGED)
        gray_b = cv2.imread(str(frames_b[i]), cv2.IMREAD_UNCHANGED)
        cand_a = find_candidates(gray_a, background_a, roi=roi_a, **kwargs)
        cand_b = find_candidates(gray_b, background_b, roi=roi_b, **kwargs)
        records.append({
            "index": i,
            "gray_a": gray_a, "gray_b": gray_b,
            "a": (cand_a[0]["cx"], cand_a[0]["cy"]) if cand_a else None,
            "b": (cand_b[0]["cx"], cand_b[0]["cy"]) if cand_b else None,
        })
        if i % 100 == 0:
            print(f"  detected {i}/{n - 1}", flush=True)
    return records


def main():
    args = parse_args()
    session = Path(args.session).resolve()
    roi_a, roi_b = parse_roi(args.roi_a), parse_roi(args.roi_b)
    calibration = _load_checkerboard(args.checkerboard_file)
    plumbline = _load_plumbline(args.plumbline_file)

    print(f"session: {session.name}")
    records = detect_session(session, roi_a, roi_b, args.limit)

    # triangulate every frame that has both cameras, applying the same
    # reprojection-error gate the triangulator node applies
    n_pairs = n_dropped = 0
    for record in records:
        record["xyz"] = None
        record["error"] = None
        if record["a"] is None or record["b"] is None:
            continue
        n_pairs += 1
        xyz, error = triangulate_point(
            calibration, plumbline, record["a"][0], record["a"][1],
            record["b"][0], record["b"][1])
        record["error"] = error
        if args.max_reprojection_error_px > 0 and error > args.max_reprojection_error_px:
            n_dropped += 1
            continue
        record["xyz"] = xyz

    good = np.array([r["xyz"] for r in records if r["xyz"] is not None])
    errors = np.array([r["error"] for r in records if r["error"] is not None])
    print(f"  frames: {len(records)}, both cameras: {n_pairs}, "
          f"3D points: {len(good)}, dropped on reprojection error: {n_dropped}")
    print(f"  reprojection error: median {np.median(errors):.2f} px, "
          f"p90 {np.percentile(errors, 90):.2f} px")

    limits = []
    for axis, override in enumerate((args.xlim, args.ylim, args.zlim)):
        explicit = _range(override)
        if explicit:
            limits.append(explicit)
        else:
            lo, hi = good[:, axis].min(), good[:, axis].max()
            pad = (hi - lo) * 0.12 or 1.0
            limits.append((lo - pad, hi + pad))

    render(records, good, limits, args, session.name)


def render(records, good, limits, args, session_name):
    step = args.downscale
    roi_a, roi_b = parse_roi(args.roi_a), parse_roi(args.roi_b)
    # cropping to the ROI is cropping to the arena: it drops the equipment and
    # dark surround outside it, so the animal fills more of the panel
    crop_a = None if args.full_frame else roi_a
    crop_b = None if args.full_frame else roi_b

    def view(gray, crop):
        if crop is not None:
            x0, y0, x1, y1 = crop
            gray = gray[y0:y1, x0:x1]
        return gray[::step, ::step]

    def marker_xy(point, crop):
        x, y = point
        if crop is not None:
            x, y = x - crop[0], y - crop[1]
        return x / step, y / step

    shape_a = view(records[0]["gray_a"], crop_a).shape
    shape_b = view(records[0]["gray_b"], crop_b).shape

    fig = plt.figure(figsize=(16.5, 6.0), dpi=120)
    grid = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.45], wspace=0.06,
                            left=0.012, right=0.985, top=0.88, bottom=0.03)
    ax_a = fig.add_subplot(grid[0])
    ax_b = fig.add_subplot(grid[1])
    ax_3d = fig.add_subplot(grid[2], projection="3d")

    image_a = ax_a.imshow(np.zeros(shape_a, np.uint8), cmap="gray",
                          vmin=0, vmax=255, interpolation="nearest")
    image_b = ax_b.imshow(np.zeros(shape_b, np.uint8), cmap="gray",
                          vmin=0, vmax=255, interpolation="nearest")
    for ax, label in ((ax_a, "camera A"), (ax_b, "camera B")):
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(label, fontsize=12, pad=6)

    marker_a, = ax_a.plot([], [], "o", mfc="none", mec="#ffd400", mew=2.0, ms=17)
    marker_b, = ax_b.plot([], [], "o", mfc="none", mec="#ffd400", mew=2.0, ms=17)
    if args.full_frame:
        for ax, roi in ((ax_a, roi_a), (ax_b, roi_b)):
            if roi:
                x0, y0, x1, y1 = (v / step for v in roi)
                ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                                           ec="#22c55e", lw=1.0, alpha=0.8))
    text_a = ax_a.text(0.02, 0.97, "", transform=ax_a.transAxes, va="top",
                       fontsize=10, color="#ffd400")
    text_b = ax_b.text(0.02, 0.97, "", transform=ax_b.transAxes, va="top",
                       fontsize=10, color="#ffd400")

    ax_3d.set_xlim3d(*limits[0])
    ax_3d.set_ylim3d(*limits[1])
    ax_3d.set_zlim3d(*limits[2])
    ax_3d.set_box_aspect([lim[1] - lim[0] for lim in limits])
    ax_3d.set_xlabel("x (mm)", labelpad=2)
    ax_3d.set_ylabel("y (mm)", labelpad=2)
    ax_3d.set_zlabel("z (mm)", labelpad=2)
    ax_3d.tick_params(labelsize=8)
    ax_3d.set_title("3D flight path", fontsize=12, pad=6)
    trail, = ax_3d.plot([], [], [], "-", color="#2563eb", lw=1.2)
    head, = ax_3d.plot([], [], [], "o", color="#dc2626", ms=7)

    header = fig.text(0.5, 0.965, "", ha="center", fontsize=13)

    xs, ys, zs = [], [], []
    frames_dir = Path(args.out).resolve().parent / "_video_frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    for old in frames_dir.glob("v_*.png"):
        old.unlink()

    print(f"rendering {len(records)} frames ...")
    for i, record in enumerate(records):
        image_a.set_data(view(record["gray_a"], crop_a))
        image_b.set_data(view(record["gray_b"], crop_b))

        for point, marker, text, crop in (
            (record["a"], marker_a, text_a, crop_a),
            (record["b"], marker_b, text_b, crop_b),
        ):
            if point is None:
                marker.set_data([], [])
                text.set_text("no detection")
                text.set_color("#ef4444")
            else:
                mx, my = marker_xy(point, crop)
                marker.set_data([mx], [my])
                text.set_text(f"({point[0]:.0f}, {point[1]:.0f}) px")
                text.set_color("#ffd400")

        if record["xyz"] is not None:
            xs.append(record["xyz"][0])
            ys.append(record["xyz"][1])
            zs.append(record["xyz"][2])
            if args.trail:
                xs, ys, zs = xs[-args.trail:], ys[-args.trail:], zs[-args.trail:]
        if xs:
            trail.set_data(xs, ys)
            trail.set_3d_properties(zs)
            head.set_data([xs[-1]], [ys[-1]])
            head.set_3d_properties([zs[-1]])

        seconds = record["index"] / args.fps
        error = record["error"]
        error_text = f"reprojection {error:.2f} px" if error is not None else "—"
        header.set_text(
            f"{session_name}   frame {record['index']}   t = {seconds:5.2f} s   "
            f"{len(xs)} 3D points   {error_text}")

        fig.savefig(frames_dir / f"v_{i:05d}.png", facecolor="white")
        if (i + 1) % 100 == 0:
            print(f"  rendered {i + 1}/{len(records)}", flush=True)

    plt.close(fig)
    encode(frames_dir, args)


def encode(frames_dir, args):
    import subprocess

    out = Path(args.out).resolve()
    print(f"\nencoding {out} ...")
    if out.suffix.lower() == ".gif":
        from PIL import Image
        paths = sorted(frames_dir.glob("v_*.png"))
        images = [Image.open(p).convert("RGB") for p in paths]
        images[0].save(out, save_all=True, append_images=images[1:],
                       duration=int(round(1000 / args.fps)), loop=0, optimize=True)
    else:
        subprocess.run([
            "ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps),
            "-i", str(frames_dir / "v_%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
            # even dimensions, required by yuv420p
            "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            str(out),
        ], check=True)

    for path in frames_dir.glob("v_*.png"):
        path.unlink()
    frames_dir.rmdir()
    print(f"done -- {out}  ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()

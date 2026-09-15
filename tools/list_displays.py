#!/usr/bin/env python3
"""List the displays this machine has, numbered the way the assay's `monitor`
parameter numbers them -- so you can tell which index is the projector.

    python3 tools/list_displays.py
    python3 tools/list_displays.py --identify        # flash the index on each
    python3 tools/list_displays.py --identify 2      # ...on display 2 only

`monitor` is passed to Processing's full_screen(N), which indexes the Java AWT
screen-device list, 1-based. That is NOT guaranteed to match the order xrandr
prints, or the order the displays are arranged in your desktop settings, so
this reads the AWT list directly and cross-references xrandr by resolution and
position to give each index its output name.

When a projector and a monitor share a resolution the cross-reference cannot
tell them apart -- that is what --identify is for: it opens a fullscreen panel
showing the index on each display in turn, so you confirm by looking.

Needs py5 (so a Java 17 JVM) and a display, same as the assay itself.
"""

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from mosquito_preference_assay import assay  # noqa: E402

assay._ensure_java_home()  # py5 needs a Java 17 JVM; same fallback the assay uses

import py5  # noqa: E402  -- must come after JAVA_HOME is settled


def awt_displays():
    """The AWT screen devices, in the order full_screen(N) indexes them."""
    from jpype import JClass

    environment = JClass("java.awt.GraphicsEnvironment").getLocalGraphicsEnvironment()
    displays = []
    for index, device in enumerate(environment.getScreenDevices(), start=1):
        mode = device.getDisplayMode()
        bounds = device.getDefaultConfiguration().getBounds()
        displays.append({
            "index": index,
            "id": str(device.getIDstring()),
            "width": int(mode.getWidth()),
            "height": int(mode.getHeight()),
            "refresh_hz": int(mode.getRefreshRate()),
            "x": int(bounds.x),
            "y": int(bounds.y),
        })
    return displays


def xrandr_monitors():
    """xrandr's view, keyed by (width, height, x, y) so it can be matched to
    the AWT list. Empty if xrandr is unavailable (non-X11, or not installed)."""
    if not shutil.which("xrandr"):
        return {}
    try:
        output = subprocess.run(["xrandr", "--listmonitors"], check=True,
                                capture_output=True, text=True).stdout
    except (subprocess.CalledProcessError, OSError):
        return {}

    monitors = {}
    # e.g. " 0: +*HDMI-0 1920/598x1080/336+0+0  HDMI-0"
    pattern = re.compile(r"^\s*\d+:\s+(\+?)(\*?)(\S+)\s+(\d+)\S*x(\d+)\S*\+(\d+)\+(\d+)")
    for line in output.splitlines():
        match = pattern.match(line)
        if not match:
            continue
        _, primary, name, width, height, x, y = match.groups()
        monitors[(int(width), int(height), int(x), int(y))] = {
            "name": name.lstrip("+*"),
            "primary": primary == "*" or "*" in line.split(":")[1][:3],
        }
    return monitors


def describe(displays, monitors):
    lines = ["Displays, numbered as the assay's `monitor` parameter numbers them:", ""]
    lines.append("  %-9s %-12s %-12s %-10s %s"
                 % ("monitor:=", "resolution", "position", "awt id", "output"))
    for display in displays:
        key = (display["width"], display["height"], display["x"], display["y"])
        match = monitors.get(key)
        output = match["name"] if match else "(no xrandr match)"
        if match and match.get("primary"):
            output += "  [primary]"
        lines.append("  %-9s %-12s %-12s %-10s %s" % (
            display["index"],
            f"{display['width']}x{display['height']}",
            f"+{display['x']}+{display['y']}",
            display["id"],
            output,
        ))
    lines += [
        "",
        "Use it as:  ros2 run mosquito_preference_assay stimulus_publisher --ros-args \\",
        "                -p fullscreen:=true -p monitor:=<N>",
        "",
        "`monitor:=\"\"` uses the primary display, `monitor:=span` spans them all.",
    ]
    if len(displays) > 1:
        widths = {(d["width"], d["height"]) for d in displays}
        if len(widths) < len(displays):
            lines.append("")
            lines.append("NOTE: two displays share a resolution, so the output names above "
                         "may be ambiguous -- run with --identify to confirm by eye.")
    return "\n".join(lines)


def identify(displays, only, seconds):
    """Open a fullscreen panel on each display in turn showing its index."""
    targets = [d for d in displays if only in (None, d["index"])]
    if not targets:
        raise SystemExit(f"no display {only}; there are {len(displays)}")

    for display in targets:
        print(f"  showing '{display['index']}' on display {display['index']} "
              f"({display['width']}x{display['height']}) for {seconds}s ...", flush=True)
        _flash(display, seconds)


def _flash(display, seconds):
    state = {"frames": 0}

    def settings():
        py5.full_screen(display["index"])

    def setup():
        py5.frame_rate(30)
        py5.text_align(py5.CENTER, py5.CENTER)

    def draw():
        py5.background(20)
        py5.fill(255)
        py5.text_size(min(py5.width, py5.height) * 0.45)
        py5.text(str(display["index"]), py5.width / 2, py5.height / 2)
        py5.fill(180)
        py5.text_size(min(py5.width, py5.height) * 0.05)
        py5.text(f"monitor:={display['index']}   "
                 f"{display['width']}x{display['height']}   {display['id']}",
                 py5.width / 2, py5.height * 0.88)
        state["frames"] += 1
        if state["frames"] >= seconds * 30:
            py5.exit_sketch()

    py5.run_sketch(sketch_functions={"settings": settings, "setup": setup, "draw": draw},
                   block=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--identify", nargs="?", const=-1, type=int, default=None,
                        help="flash the index fullscreen on every display, or on "
                             "just the one given")
    parser.add_argument("--seconds", type=float, default=4.0,
                        help="how long to show each panel (default 4)")
    args = parser.parse_args()

    displays = awt_displays()
    if not displays:
        raise SystemExit("no displays found -- is there an X display available?")
    print(describe(displays, xrandr_monitors()))

    if args.identify is not None:
        only = None if args.identify == -1 else args.identify
        print("\nidentifying:")
        identify(displays, only, args.seconds)
        print("done")


if __name__ == "__main__":
    main()

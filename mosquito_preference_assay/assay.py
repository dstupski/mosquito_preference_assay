"""Mosquito visual preference assay (py5 / pythonic Processing).

Two circular markers, left and right; each trial presents one *condition* (a
{left, right} pairing drawn from the experiment definition) for its duration,
then advances. What the pool of stimuli is, which pairings can occur, the
ordering and the timing all come from an :class:`~.experiment.Experiment`
(loaded from a YAML file, or the built-in default).

This module owns the sketch (``settings`` / ``setup`` / ``draw`` /
``key_pressed``) and, at all times, a thread-safe snapshot of exactly what is
on screen -- ``current_state()`` / ``current_state_json()``. The ROS node
imports it, registers ``set_trial_change_callback`` and polls
``current_state_json()`` on a timer.

Run standalone, no ROS (uses the built-in default experiment):

    python -m mosquito_preference_assay.assay
    ros2 run mosquito_preference_assay assay

Keys while running:
    d     toggle the debug label/timer overlay (hide before a real trial)
    n     skip to the next trial immediately
    esc   quit
"""

import json
import os
import random
import threading
import time
import uuid


def _ensure_java_home():
    """py5 needs a Java 17 JVM. If JAVA_HOME is unset, fall back to a JDK we
    can find on disk (the Processing 4 bundle, or one py5 installed itself)
    rather than letting py5 pick up an incompatible system Java.

    The portable fix, if none of these exist, is:  py5-install-jdk
    """
    if os.environ.get("JAVA_HOME"):
        return
    from glob import glob
    candidates = [
        os.path.expanduser("~/Applications/Processing/lib/app/resources/jdk"),
        *sorted(glob(os.path.expanduser("~/.cache/py5/**/jdk*"), recursive=True)),
        *sorted(glob(os.path.expanduser("~/.cache/py5/**/*jdk*/"), recursive=True)),
    ]
    for cand in candidates:
        if os.path.exists(os.path.join(cand, "bin", "java")):
            os.environ["JAVA_HOME"] = cand
            return


_ensure_java_home()

import py5  # noqa: E402  (must follow _ensure_java_home)

from .experiment import Experiment  # noqa: E402
from .stimulus_types import build_stimulus  # noqa: E402

STATE_SCHEMA = "mosquito_preference_assay/stimulus_state/3"
INFO_SCHEMA = "mosquito_preference_assay/experiment_info/1"

# --- configuration: set via configure() BEFORE run(); never mutated after ---
_cfg = {
    "window_w": 1200,
    "window_h": 800,
    "fullscreen": False,
    # Which display to play on:
    #   None / 0        -> primary (or the OS default)
    #   int N (1-based) -> that monitor / projector  (fullscreen only)
    #   "span"          -> span all displays          (fullscreen only)
    "monitor": None,
    # Windowed-mode placement, [x, y] px on the virtual desktop, or None. Use
    # this to push a window onto a second monitor without going fullscreen.
    "window_pos": None,
    "master_seed": None,       # None -> draw a random one at setup() and log it
    "show_debug": True,
    # "auto"     -> start trials immediately at setup() (default)
    # "triggered" -> open ARMED (blank), begin only when start_run() is called
    "start_mode": "auto",
    "experiment": None,        # an Experiment instance; None -> Experiment.default()
    # Optional overrides of the experiment's display.*_center_px, handy while
    # aligning the window to the arena without editing the experiment file.
    "left_center_px": None,    # [x, y] or None
    "right_center_px": None,   # [x, y] or None
}

_on_trial_change = None       # optional callback(state_dict), called from the sketch thread

# --- runtime state, guarded by _lock (read from both the sketch thread and,
#     in the ROS node, the executor thread) ---
_lock = threading.Lock()
_rt = {
    "experiment": None,
    "master_seed": None,
    "noise_seed": None,
    "master_rng": None,
    "scheduler": None,
    "trial_id": -1,
    "trial_seed": None,
    "trial_uuid": None,
    "trial_duration_sec": 0.0,
    "condition_name": "",
    "condition_ordered": False,
    "trial_start_wall": 0.0,
    "trial_start_monotonic": 0.0,
    "left": None,          # Stimulus instance
    "right": None,         # Stimulus instance
    "left_name": "",
    "right_name": "",
    "geometry": {},
    "show_debug": True,
    "phase": "armed",     # "armed" | "running" | "complete"
    "run_id": 0,          # increments each start_run(); 0 while never triggered
    "start_pending": False,  # set by start_run(), consumed by draw() on the sketch thread
    "complete": False,
    "started": False,
}


def configure(**kwargs):
    """Override defaults before run(). Unknown keys raise, to catch typos."""
    unknown = set(kwargs) - set(_cfg)
    if unknown:
        raise KeyError(f"unknown assay config key(s): {sorted(unknown)}")
    _cfg.update(kwargs)


def set_trial_change_callback(fn):
    """Register fn(state_dict); called once, from the sketch thread, each time
    a new trial starts (including the first). Pass None to clear."""
    global _on_trial_change
    _on_trial_change = fn


# --------------------------------------------------------------------------- #
# sketch
# --------------------------------------------------------------------------- #
def settings():
    # size() / full_screen() must run here, before the sketch surface exists.
    if _cfg["fullscreen"]:
        mon = _cfg["monitor"]
        if mon in (None, 0, "", "0"):
            py5.full_screen()
        elif str(mon).lower() == "span":
            py5.full_screen(py5.SPAN)
        else:
            py5.full_screen(int(mon))
    else:
        py5.size(_cfg["window_w"], _cfg["window_h"])


def setup():
    py5.frame_rate(60)
    py5.text_align(py5.CENTER)

    if not _cfg["fullscreen"] and _cfg["window_pos"]:
        x, y = _cfg["window_pos"]
        py5.window_move(int(x), int(y))

    experiment = _cfg["experiment"] or Experiment.default()

    seed = _cfg["master_seed"]
    if seed is None:
        seed = random.SystemRandom().randrange(2 ** 32)
    noise_seed = seed & 0xFFFFFFFF
    py5.noise_seed(noise_seed)   # make jitter's Perlin walk reproducible too

    master_rng = random.Random(seed)

    triggered = _cfg["start_mode"] == "triggered"

    with _lock:
        _rt["experiment"] = experiment
        _rt["master_seed"] = seed
        _rt["noise_seed"] = noise_seed
        _rt["master_rng"] = master_rng
        _rt["scheduler"] = experiment.scheduler(master_rng)
        _rt["show_debug"] = _cfg["show_debug"]
        _rt["started"] = True
        _rt["phase"] = "armed" if triggered else "running"
        _rt["run_id"] = 0 if triggered else 1

    if experiment.mode == "sample":
        desc = f"mode=sample pool={experiment.pool}"
    else:
        desc = f"mode=pairs, {len(experiment.conditions)} conditions"
    print(f"[assay] experiment {experiment.name!r} ({desc}, sha1 {experiment.sha1})")
    print(f"[assay] master_seed={seed}  "
          f"(reproduce this run with configure(master_seed={seed}))")

    if triggered:
        print("[assay] ARMED -- waiting for start_run() trigger")
    else:
        _start_new_trial()


def draw():
    with _lock:
        experiment = _rt["experiment"]
        left = _rt["left"]
        right = _rt["right"]
        start_monotonic = _rt["trial_start_monotonic"]
        duration = _rt["trial_duration_sec"]
        show_debug = _rt["show_debug"]
        phase = _rt["phase"]
        start_pending = _rt["start_pending"]

    py5.background(experiment.background_gray if experiment else 128)

    if phase == "armed":
        if show_debug:
            py5.fill(0)
            py5.text_size(16)
            py5.text("waiting for trigger", py5.width / 2, py5.height / 2)
        return

    if start_pending:
        # a trigger arrived on another thread; build the first trial here, on
        # the sketch thread (py5 object creation must not happen off it), then
        # render it from the next frame.
        with _lock:
            _rt["start_pending"] = False
        _start_new_trial()
        return

    if phase == "complete" or left is None:
        if show_debug:
            py5.fill(0)
            py5.text_size(16)
            py5.text("experiment complete", py5.width / 2, py5.height / 2)
        return

    t = time.monotonic() - start_monotonic
    if t >= duration:
        _start_new_trial()
        t = 0.0
        with _lock:
            left = _rt["left"]
            right = _rt["right"]
            phase = _rt["phase"]
        if phase != "running" or left is None:
            return

    w, h = py5.width, py5.height
    left_c = _resolve_center("left", experiment, w, h)
    right_c = _resolve_center("right", experiment, w, h)

    with _lock:
        _rt["geometry"] = {
            "window_w": w,
            "window_h": h,
            "fullscreen": bool(_cfg["fullscreen"]),
            "monitor": _cfg["monitor"],
            "circle_diameter_px": experiment.circle_diameter_px,
            "left_center_px": [left_c[0], left_c[1]],
            "right_center_px": [right_c[0], right_c[1]],
        }

    left.display(left_c[0], left_c[1], t)
    right.display(right_c[0], right_c[1], t)

    if show_debug:
        _draw_debug_overlay(t, duration, left_c, right_c)


def _resolve_center(slot, experiment, w, h):
    override = _cfg[f"{slot}_center_px"]
    if override:
        return (float(override[0]), float(override[1]))
    from_exp = getattr(experiment, f"{slot}_center_px")
    if from_exp:
        return (float(from_exp[0]), float(from_exp[1]))
    frac = 0.25 if slot == "left" else 0.75
    return (w * frac, h / 2)


def key_pressed():
    k = str(py5.key).lower()
    if k == "d":
        with _lock:
            _rt["show_debug"] = not _rt["show_debug"]
    elif k == "n":
        with _lock:
            running = _rt["phase"] == "running"
        if running:            # only advances an in-progress run
            _start_new_trial()


def run(block=True):
    """Start the py5 sketch. Must be called from the main thread.

    block=True  -> return only when the sketch window closes (standalone use).
    block=False -> return immediately, sketch runs on its own thread (the ROS
                   node uses this so the main thread can run rclpy.spin).

    The sketch functions are passed explicitly rather than left to py5's
    caller-namespace introspection, which does not survive being wrapped in a
    module / console_script entry point.
    """
    import py5_tools
    if not py5_tools.jvm.is_jvm_running():
        # -Xrs: stop the JVM from installing its own SIGINT/SIGTERM handlers,
        # so Ctrl-C reaches Python (and rclpy) and the node shuts down.
        py5_tools.jvm.add_options("-Xrs")

    py5.run_sketch(
        block=block,
        sketch_functions={
            "settings": settings,
            "setup": setup,
            "draw": draw,
            "key_pressed": key_pressed,
        },
    )


def sketch_running():
    """True while the sketch window is open. (Not named is_running: py5
    reserves that name as one of its dynamic variables and strips a module
    attribute that shadows it.)"""
    try:
        return bool(py5.is_running)
    except Exception:  # noqa: BLE001
        return False


def request_stop():
    """Ask the sketch to close. Safe to call from any thread or a signal
    handler; unblocks a running run()."""
    try:
        py5.exit_sketch()
    except Exception:  # noqa: BLE001 - best effort during shutdown
        pass


def start_run():
    """Trigger: leave ARMED and begin trials. No-op if already running or the
    experiment is finished. Thread-safe -- the actual first trial is built on
    the sketch thread next frame (py5 objects must not be created off it)."""
    with _lock:
        if _rt["phase"] != "armed":
            return
        _rt["run_id"] += 1
        _rt["phase"] = "running"
        _rt["start_pending"] = True
        run_id = _rt["run_id"]
    print(f"[assay] trigger -> run {run_id}")


def abort_run():
    """Trigger: stop the current run and return to ARMED (no-op if not
    running). Thread-safe."""
    with _lock:
        if _rt["phase"] != "running":
            return
        _rt["phase"] = "armed"
        _rt["start_pending"] = False
        _rt["left"] = None
        _rt["right"] = None
    print("[assay] run aborted -> ARMED")


def phase():
    with _lock:
        return _rt["phase"]


def _notify_state():
    cb = _on_trial_change
    if cb is not None:
        try:
            cb(current_state())
        except Exception as exc:                       # noqa: BLE001
            print(f"[assay] state callback raised: {exc!r}")


def main():
    """Standalone entry point (no ROS): built-in default experiment."""
    configure()
    run()


# --------------------------------------------------------------------------- #
# trial transitions + state snapshot
# --------------------------------------------------------------------------- #
def _start_new_trial():
    with _lock:
        experiment = _rt["experiment"]
        master_rng = _rt["master_rng"]
        scheduler = _rt["scheduler"]
        trial_id = _rt["trial_id"] + 1

    draw = scheduler.next_trial()
    if draw is None:
        with _lock:
            _rt["complete"] = True
            _rt["phase"] = "complete"
        print("[assay] experiment complete")
        _notify_state()   # publish one final message before the node shuts down
        return

    # One RNG per trial, seeded from the master stream: given the draw (which
    # stimuli, which sides), the trial's visuals replay from trial_seed alone.
    trial_seed = master_rng.randrange(2 ** 32)
    trial_rng = random.Random(trial_seed)

    plan = experiment.realize(draw, trial_rng)
    d = experiment.circle_diameter_px
    left = build_stimulus(plan.left_type, d, plan.left_params, trial_rng)
    right = build_stimulus(plan.right_type, d, plan.right_params, trial_rng)

    now_wall = time.time()
    now_monotonic = time.monotonic()
    trial_uuid = str(uuid.uuid4())

    with _lock:
        _rt.update(
            trial_id=trial_id,
            trial_seed=trial_seed,
            trial_uuid=trial_uuid,
            trial_duration_sec=plan.duration_sec,
            condition_name=plan.condition_name,
            condition_ordered=plan.condition_ordered,
            left=left,
            right=right,
            left_name=plan.left_name,
            right_name=plan.right_name,
            trial_start_wall=now_wall,
            trial_start_monotonic=now_monotonic,
        )

    print(f"[assay] trial {trial_id}: {plan.condition_name}  "
          f"LEFT={plan.left_name} RIGHT={plan.right_name}  "
          f"{plan.duration_sec:.1f}s  (trial_seed={trial_seed})")

    cb = _on_trial_change
    if cb is not None:
        state = current_state()
        try:
            cb(state)
        except Exception as exc:                       # noqa: BLE001
            print(f"[assay] trial-change callback raised: {exc!r}")


def experiment_info():
    """Static, per-run metadata. Publish once, latched, on ~/experiment_info --
    it never changes during a run, so it is kept out of every stimulus_state
    message. Available as soon as configure() has been called."""
    experiment = _cfg["experiment"] or Experiment.default()
    seed = _cfg["master_seed"]
    return {
        "schema": INFO_SCHEMA,
        "stamp_wall": time.time(),
        "start_mode": _cfg["start_mode"],
        "master_seed": seed,
        "noise_seed": None if seed is None else seed & 0xFFFFFFFF,
        "experiment": experiment.summary(),
    }


def experiment_info_json():
    return json.dumps(experiment_info(), separators=(",", ":"))


def current_state():
    """A JSON snapshot of what is on screen right now, or None before setup().

    In the "armed" phase it is just schema/stamp/phase/run_id. Otherwise it is
    the trial: ids + seed, the condition, geometry, and each side's name / type
    / resolved params. Static run metadata (experiment, seeds) lives on
    ~/experiment_info, not here.
    """
    with _lock:
        experiment = _rt["experiment"]
        phase_now = _rt["phase"]
        run_id = _rt["run_id"]
        if experiment is None:
            return None

        state = {
            "schema": STATE_SCHEMA,
            "stamp_wall": time.time(),
            "phase": phase_now,
            "run_id": run_id,
        }
        left = _rt["left"]
        right = _rt["right"]
        if left is None or right is None:
            return state

        state.update({
            "trial_id": _rt["trial_id"],
            "trial_seed": _rt["trial_seed"],
            "trial_uuid": _rt["trial_uuid"],
            "condition": {
                # name: grouping key for the pairing. ordered=False -> "a|b"
                # (sorted, side-independent); the sides this trial are in
                # left/right below. ordered=True -> "a->b", sides fixed.
                "name": _rt["condition_name"],
                "ordered": _rt["condition_ordered"],
            },
            "trial_start_wall": _rt["trial_start_wall"],
            "trial_duration_sec": _rt["trial_duration_sec"],
            "geometry": dict(_rt["geometry"]),
            "left": {
                "name": _rt["left_name"],
                "type": left.type_name,
                "params": left.params(),
            },
            "right": {
                "name": _rt["right_name"],
                "type": right.type_name,
                "params": right.params(),
            },
        })
        start_monotonic = _rt["trial_start_monotonic"]

    state["elapsed_sec"] = max(0.0, time.monotonic() - start_monotonic)
    return state


def to_json(state):
    return json.dumps(state, separators=(",", ":"))


def current_state_json():
    state = current_state()
    return None if state is None else to_json(state)


def experiment_complete():
    with _lock:
        return bool(_rt["complete"])


# --------------------------------------------------------------------------- #
# debug overlay
# --------------------------------------------------------------------------- #
def _draw_debug_overlay(t, duration, left_c, right_c):
    with _lock:
        experiment = _rt["experiment"]
        left_name = _rt["left_name"]
        right_name = _rt["right_name"]
        trial_id = _rt["trial_id"]
        condition_name = _rt["condition_name"]
    half_d = (experiment.circle_diameter_px if experiment else 200) / 2
    py5.fill(0)
    py5.text_size(16)
    py5.text(left_name, left_c[0], left_c[1] + half_d + 30)
    py5.text(right_name, right_c[0], right_c[1] + half_d + 30)
    remaining = duration - t
    py5.text(f"trial {trial_id}  [{condition_name}]  {remaining:0.1f}s left   "
             f"(d: labels, n: next)", py5.width / 2, 30)


if __name__ == "__main__":
    main()

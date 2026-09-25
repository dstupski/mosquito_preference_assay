# mosquito_preference_assay

A two-choice visual preference assay for mosquitoes. Two circular markers, one
**left** and one **right**; each trial shows a pairing of stimuli for a set
duration, then advances. What the stimuli are, how each trial's pair is chosen,
and the timing all come from a **YAML experiment file**. It runs as a **ROS 2
node** that publishes, at all times, a complete JSON description of exactly
what is on screen — so a whole session is reconstructable from a rosbag.

Written in [py5](https://py5coding.org/) (Processing for Python) for the
graphics; ROS 2 Humble for the plumbing.

---

## Contents

**Start here** — [how it works](#how-it-works) · [install](#setup) ·
[**setting up a rig, start to finish**](#setting-up-a-rig-start-to-finish)

**Running the assay** — [setting up a new experiment](#setting-up-a-new-experiment-step-by-step) ·
[writing an experiment](#writing-an-experiment) ·
[the stimulus display](#the-stimulus-display) ·
[deploying to another rig](#deploying-to-another-rig) ·
[custom params file](#launching-with-your-own-params-file) ·
[where does this setting go?](#where-does-this-setting-go) ·
[**running the real experiment**](#running-the-real-experiment) ·
[the trigger zone](#setting-the-trigger-region-for-a-new-rig) ·
[triggering](#triggering) · [the rig workflow](#detector-armed-capture--the-rig-workflow)

**Tracking the animal** — [detecting a mosquito](#detecting-a-mosquito) ·
[stereo tracking](#real-time-stereo-tracking) ·
[3D triangulation](#3d-triangulation) ·
[benchmarking and latency](#benchmarking-and-latency)

**Reference** — [ROS parameters](#ros-parameters) ·
[published messages](#published-messages) ·
[reproducing a run](#reproducing-a-run-offline) ·
[adding a marker behavior](#adding-a-new-marker-behavior) ·
[development](#development) · [license](#license--citing)

---

## How it works

**A run is one trial.** The experiment YAML has two layers plus a duration:

| Layer | Where | What it does |
|---|---|---|
| **Stimulus pool** | `stimuli:` | Named, reusable stimulus definitions — a `type` (one of the built-in marker behaviors) plus `params`. A param can be a fixed value **or** a random spec resolved per trial. |
| **Conditions** | `conditions:` | How the trial's `{left, right}` pair is chosen. `mode: sample` (default) draws **two distinct** stimuli from the pool at random; `mode: pairs` picks one entry from a `pairs:` list. |
| | `duration_sec:` | How long the trial runs (a number, or a `{uniform: [...]}` spec). |

The **marker behaviors** are code (`stimuli.py`); the YAML only *composes
instances* of them:

| `type` | Behavior |
|---|---|
| `blank` | Nothing drawn at all — an empty side, for a stimulus-vs-nothing trial |
| `static_dark` | Plain dark circle, no motion — baseline / control |
| `jitter` | Dark circle whose position wanders smoothly (Perlin noise) |
| `moving_grating` | Black/white stripes drifting across the circle (optomotor-style) |
| `split_grating` | The circle halved, each half's stripes drifting the *opposite* way — converging on the midline or streaming out of it |
| `telescope` | Concentric rings expanding outward — tunnel effect |

**Direction is the sign of the speed.** `speed_px_per_sec` negative runs a
`telescope` inward (rings contracting) instead of outward, and reverses a
`moving_grating` along its `angle_deg` axis — with `angle_deg: 90`, positive
drifts down and negative drifts up. `split_grating` takes an explicit
`direction: inward | outward` and `axis: horizontal | vertical` instead,
since "which way" there means two things at once.

The **ROS node** (`stimulus_publisher`) runs the sketch and publishes: the
static run metadata once on `~/experiment_info`, and the trial state on
`~/stimulus_state` (continuously) + `~/trial_start`. It plays immediately
(`start_mode: auto`) or opens **ARMED** and waits for a `std_msgs/Bool`
trigger (`start_mode: triggered`, or an experiment with a `trigger:` block).
`phase` goes `armed → running → complete`; then a finite run's node exits.

**Reproducibility:** one integer `master_seed` replays the run — the draw, the
sides, every resolved parameter. It's logged at startup and in
`~/experiment_info`.

### Repository layout

Nodes are grouped by what they are for. Every one is a console script, so
`ros2 run mosquito_preference_assay <name>`.

Which command do you want? Most people need the first group only:

| If you want to | Run |
|---|---|
| see the stimuli, no ROS | `preview` |
| check the display / align the circles | `display_check` |
| rehearse a trial, no camera | `trigger_display_test.launch.py` |
| run an animal | `triggered_assay.launch.py` |
| tune the detector | `mosquito_detector` + `detector.launch.py` |
| everything else below | tracking and diagnostics — not needed to run the assay |

**The assay** — the stimulus display and its trial logic

| | |
|---|---|
| `stimuli.py` · `stimulus_types.py` | the marker behaviors, and the type registry |
| `param_spec.py` · `experiment.py` | literal-or-random params; load/validate the YAML and draw a trial |
| `assay.py` | the py5 sketch, display selection, thread-safe `current_state()` |
| `stimulus_publisher` | the ROS node that runs the sketch and publishes what is on screen |
| `display_check` | test pattern on the configured display, drag-to-align |
| `test_trigger` | bench helper: fire the trigger by hand |
| `snapshot_supervisor` | flushes a `--snapshot-mode` bag at trial start / end |

**Tracking** — camera in, 3D position out

| | |
|---|---|
| `detection.py` | background-subtraction blob detection (ported from test_videos_particle_tracking) |
| `mosquito_detector` | watches a feed, publishes detection events — the trigger |
| `tracker` | real-time 2D tracking, **one instance per camera** |
| `stereo_sync` | pairs two `tracker` outputs by timestamp |
| `triangulator` | a stereo pair → a real 3D position in mm, via the rig calibration |
| `trajectory_plotter` | live 3D plot of a position stream |

**Standing in for hardware, and measuring it**

| | |
|---|---|
| `frame_source.py` · `video_publisher` · `dual_video_publisher` | replay footage as one or two synchronized camera feeds |
| `synthetic_trajectory_publisher` | a made-up 3D track, for testing the plotter |
| `benchmark` · `pipeline_monitor` | per-component cost; live per-stage lag, rate and yield |

**Everything else**

```
experiments/    two_choice_default · control_vs_grating · single_trigger
                ten_stimulus_panel · sippell_retest_experiment
config/         assay_params.yaml (the assay) · detector_params.yaml (the detector)
launch/         assay · detector · display_check · trigger_display_test
                triggered_assay (the rig workflow) · tracking_benchmark
tools/          list_displays
test/           unit + lint tests
```

---

## Setup

Assumes **ROS 2 is already installed** and you can `source
/opt/ros/$ROS_DISTRO/setup.bash`. Developed and CI-tested on **Humble**;
Iron / Jazzy / Rolling should work unchanged (see *dependency reference* for
the two version-sensitive spots). A **display is required** — the stimulus is
a real window, there is no headless mode.

```bash
# 0. ROS-side packages (most are in ros-<distro>-desktop already)
sudo apt install ros-$ROS_DISTRO-cv-bridge      # for mosquito_detector / video_publisher

# 1. a colcon workspace (skip if you already have one)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws

# 2. clone
git clone https://github.com/dstupski/mosquito_preference_assay.git src/mosquito_preference_assay

# 3. Python deps -- into the SAME interpreter ROS uses (usually /usr/bin/python3).
#    py5 is not in rosdep; the numpy pin avoids an ABI clash with apt matplotlib.
python3 -m pip install --user py5 "numpy<2"

# 4. Java 17 for py5 (one-time). Skip if you already have the Processing 4
#    bundle at ~/Applications/Processing; otherwise let py5 fetch its own JDK:
python3 -m py5_tools.tools.install_jdk -j 17     # installed as `py5-install-jdk` too

# 5. build + source
colcon build --packages-select mosquito_preference_assay
source install/setup.bash             # add this line to ~/.bashrc to make it stick

# 6. verify -- opens a window, plays one 15 s trial of the built-in default,
#    publishes nothing, exits:
ros2 run mosquito_preference_assay assay
```

If step 6 shows two circles and prints `[assay] trial 0: ...`, you're set —
go to [setting up a rig](#setting-up-a-rig-start-to-finish). If `import py5` fails, see **`JAVA_HOME`**
below.

### Dependency reference

| Dependency | Version | Comes from | Notes |
|---|---|---|---|
| **ROS 2** | Humble+ | apt (`ros-<distro>-desktop`) | `rclpy`, `std_msgs`, `sensor_msgs`, `launch`, `rosbag2`, `ament_index_python` all included |
| **PyYAML** | any | ships with ROS 2 (`rclpy` dep) | experiment-file parsing |
| **py5** | ≥ 0.10 | `pip install --user py5` | the sketch. **Not in rosdep** — install into the interpreter ROS uses |
| **numpy** | **< 2** | `pip install --user "numpy<2"` | on Ubuntu 22.04, py5 pulls numpy 2 which is ABI-incompatible with the apt `python3-matplotlib` (`_ARRAY_API not found` spam; the sketch still runs). py5 is fine on 1.26. On Ubuntu 24.04 / Jazzy the stack is numpy-2-native — this pin may not be needed. |
| **Java** | 17 | Processing 4 bundle, or `py5-install-jdk` | py5 needs a Java 17 JVM |
| **cv_bridge**, **OpenCV** | any | apt `ros-<distro>-cv-bridge` (in `-desktop`) | camera/tracking nodes only — not needed for the assay itself |
| **message_filters** | any | apt `ros-<distro>-message-filters` (in `-desktop`) | `stereo_sync` only |
| **geometry_msgs** | any | apt (in `-desktop`) | `tracker` / `stereo_sync` only |
| **matplotlib** | any | apt `python3-matplotlib` (often already present — see the numpy note above) | `trajectory_plotter` only |
| **rosbag2_interfaces** | any | apt (with `rosbag2`, in `-desktop`) | `snapshot_supervisor` only — the `/rosbag2_recorder/snapshot` service |
| a display | — | — | windowed / fullscreen sketch; also `trajectory_plotter`'s GUI |

**`JAVA_HOME`** — on import, `assay.py` sets it (if unset) to the first of:
`~/Applications/Processing/lib/app/resources/jdk`, or a JDK under
`~/.cache/py5/` that `py5-install-jdk` created. If neither exists, export it
yourself: `export JAVA_HOME=/path/to/jdk-17`.

**Older / newer ROS:** the only version-sensitive code is
`rclpy.try_shutdown()` (Humble+) and `rclpy.executors.ExternalShutdownException`
(Galactic+), used in each node's `main()`. Foxy would need those two swapped
for `rclpy.shutdown()` + a bare `KeyboardInterrupt` catch.

There is no `requirements.txt` — the apt half and the pip half live in
different places, and the one `pip install --user` line above is the whole
Python story. A venv with `--system-site-packages` (for `rclpy`) is tidier if
you deploy to several rigs.

---

## Setting up a rig, start to finish

Six steps from a fresh clone to running an animal. Each one leaves something
you can check, so you find problems at the step that caused them.

**1. Build, once, so your edits take effect without rebuilding.**

```bash
cd ~/ros2_ws
colcon build --packages-select mosquito_preference_assay --symlink-install
source install/setup.bash          # add to ~/.bashrc to make it stick
```

**2. Make your own config.** `config/assay_params.yaml` is tracked boilerplate
that `git pull` can change. Copy it — a `.local.yaml` is gitignored and is
preferred automatically, so nothing else needs to change:

```bash
cd src/mosquito_preference_assay
cp config/assay_params.yaml config/assay_params.local.yaml
```

Copy the **whole** file, not a few lines: a params file *replaces* rather than
merges, so anything missing falls back to a built-in default.

**3. Find your projector's number.** It is the Java screen order, which matches
neither `xrandr` nor Ubuntu's Settings:

```bash
python3 tools/list_displays.py                 # lists them
python3 tools/list_displays.py --identify      # flashes the number on each
```

Put the answer in `config/assay_params.local.yaml`:

```yaml
fullscreen: true
monitor: "2"        # a string
```

*Check:* re-run `list_displays.py` and confirm the number matches the screen
you mean.

**4. Align the circles to the arena.**

```bash
ros2 launch mosquito_preference_assay display_check.launch.py
```

Drag the **midpoint** to place the pair, drag a circle to set the separation,
`[`/`]` to resize, then **`s`** to save.

**That is the whole handoff — there is nothing to copy or configure.** `s`
writes `config/display_geometry.local.yaml`, and every launch layers it over
the params file, so the next experiment comes up with your circles at your
size. Each launch prints where it got them:

```
display calibration: .../config/display_geometry.local.yaml
```

`s` also writes a dated archive, so `display_config:=config/20260923_display_config.yaml`
goes back to an earlier alignment.

*Check:* the four corner brackets are all visible. A missing one means the
projector is overscanning, and every position you just set is shifted.

**5. Rehearse a whole trial — no camera, no animal.**

```bash
ros2 launch mosquito_preference_assay trigger_display_test.launch.py \
    experiment_file:=sippell_retest_experiment
```

It comes up ARMED, fires its own trigger after a few seconds, runs the trial,
and closes the bag exactly as a real run would.

*Check:* the circles are where you put them, and the bag is complete:

```bash
ros2 bag info display_test_*        # metadata.yaml present = it closed cleanly
```

**6. Run an animal.**

```bash
ros2 launch mosquito_preference_assay triggered_assay.launch.py \
    experiment_file:=sippell_retest_experiment \
    bag_dir:=~/data/sippell_retest_experiment/animal_01
```

One launch = one animal = one bag. Before the first real animal, set the
detector's trigger region — see
[setting the trigger region](#setting-the-trigger-region-for-a-new-rig),
because it ships as the whole frame and will fire on reflections.

### Running a different setup

The file named `config/assay_params.local.yaml` is always the one used. To keep
several and switch between them, name them and pass the one you want:

```bash
ros2 launch mosquito_preference_assay triggered_assay.launch.py \
    params_file:=config/rig_b.local.yaml experiment_file:=sippell_retest_experiment
```

Anything matching `config/*.local.yaml` is gitignored, so named setups never
end up in a commit.

### Just looking at stimuli

No ROS, no recording — draws a trial and nothing else:

```bash
ros2 run mosquito_preference_assay preview     # `assay` is the same command
```

Keys: `d` debug overlay · `n` a fresh trial · `esc` quit.

---

## Writing an experiment

### Setting up a new experiment, step by step

**1. Start from a file that already works.** Don't write one from scratch —
copy the closest existing experiment and edit it.

```bash
cd ~/ros2_ws/src/mosquito_preference_assay/experiments
cp ten_stimulus_panel.yaml looming_vs_static.yaml
```

Name the file after the **question**, not the stimuli — `looming_vs_static`
tells you why the experiment exists; `experiment_3` does not.

**2. Edit three things**, in this order:

| In the file | Ask yourself |
|---|---|
| `stimuli:` | what am I showing? Each entry is a named recipe you can reuse |
| `conditions:` | how is each trial's pair chosen? `mode: sample` draws two at random; `mode: pairs` fixes the comparisons |
| `duration_sec:` | how long does one animal see it? |

Change **one thing at a time** between stimuli you intend to compare. The two
jitter levels in `ten_stimulus_panel.yaml` are the pattern to copy: identical
`seed_x`/`seed_y`, different amplitude, so the only difference is the one you
are asking about.

**3. Check it loads.** Malformed files fail loudly with a specific message, so
this catches typos before the rig is involved:

```bash
ros2 run mosquito_preference_assay stimulus_publisher --ros-args \
    -p experiment_file:=looming_vs_static
```

**4. Look at the stimuli** before the rig is involved — cheaper than
discovering mid-session that a "looming" stimulus recedes. The no-ROS preview
draws a trial from your file and nothing else; `n` redraws a fresh pair:

```bash
ros2 run mosquito_preference_assay assay
```

**5. Rehearse the whole trial on the rig**, with no animal — the circles in
place, the trigger firing, the bag closing:

```bash
ros2 launch mosquito_preference_assay trigger_display_test.launch.py \
    params_file:=~/rig/assay_params.yaml \
    experiment_file:=looming_vs_static fullscreen:=true monitor:=2
```

**6. Run it for real**, one animal per launch:

```bash
ros2 launch mosquito_preference_assay triggered_assay.launch.py \
    params_file:=~/rig/assay_params.yaml \
    experiment_file:=looming_vs_static fullscreen:=true monitor:=2 \
    bag_dir:=~/data/looming_vs_static/animal_07
```

#### Once you have collected data, treat the file as frozen

Every run records the experiment's **sha1** in `~/experiment_info`. That is
what lets you prove two animals saw the same thing. Editing a file after
collecting data silently breaks that: the name stays the same, the sha1
changes, and nothing warns you that animals 1-6 and animals 7-12 are no longer
comparable.

So don't edit an experiment you have data for. Copy it to a new name —
`looming_vs_static_v2.yaml`, or better something that says what changed — and
run that. Old bags keep pointing at the old definition, which is the whole
point.

Keep experiment files **in the repo and committed**. They are the record of
what you did, they should be identical on every machine, and one of them plus
its `master_seed` is enough to replay a run exactly.

`experiments/two_choice_default.yaml` is the fully-commented reference. The
shape:

```yaml
schema: mosquito_preference_assay/experiment/1
name: my_experiment

# POOL — named stimulus specs. Any param is a literal OR a random spec
# resolved per trial from the trial seed:
#   {uniform: [lo,hi]}  {randint: [lo,hi]}  {choice: [...]}  {normal: [mu,sd]}
stimuli:
  control:      {type: static_dark,    params: {fill_gray: 20}}
  wander:       {type: jitter,         params: {amplitude_px: 20, noise_speed: 1.2}}
  grating:      {type: moving_grating, params: {period_px: 24,
                                                speed_px_per_sec: {uniform: [20, 80]},
                                                angle_deg: {uniform: [0, 360]}}}
  tunnel:       {type: telescope,      params: {ring_spacing_px: 18, speed_px_per_sec: 50}}

# CONDITIONS — how the trial picks its {left, right} pair.
conditions:
  mode: sample                       # sample (default) | pairs
  pool: [control, wander, grating, tunnel]   # subset of `stimuli`; default = all
  # weights: {grating: 2, control: 1}         # sample mode — bias the draw (default: equal)
  # --- mode: pairs: pick one entry at random; {a,b} randomizes sides, {left,right} fixes them ---
  # pairs:
  #   - {a: control, b: grating}
  #   - {left: control, right: tunnel}

duration_sec: 15.0                    # trial length in seconds (literal, or {uniform: [25,35]})

# Optional. Its presence makes this a triggered experiment: the node opens
# ARMED and waits for a std_msgs/Bool before the trial runs. Omit to play now.
trigger:
  topic: /arena/mosquito_present     # your tracking node publishes true = go
  # node: arena                      # shorthand for topic: /arena/trigger

display:
  circle_diameter_px: 200
  left_center_px: null               # null -> auto (w*0.25, h/2)
  right_center_px: null              # null -> auto (w*0.75, h/2)
  background_gray: 128
```

### `mode: sample` (default)

Draw **two distinct** stimuli from `pool` at random (no replacement) — first
drawn → right, second → left. `weights:` biases the draw. This is the standard
preference-assay design.

### `mode: pairs`

Give a `pairs:` list; one entry is picked at random for the trial. `{a: X, b: Y}`
randomizes which side each lands on; `{left: X, right: Y}` fixes them. Use it for
a control-vs-treatment design (see `experiments/control_vs_grating.yaml`).

### The ten-stimulus panel

`experiments/ten_stimulus_panel.yaml` is a ready-made pool covering the main
motion axes, with every parameter fixed (no random specs) so a given stimulus
looks identical in every trial it appears in:

| Name | Type | What it does |
|---|---|---|
| `blank` | `blank` | nothing drawn — the empty control |
| `static_black` | `static_dark` | motionless dark circle |
| `jitter_small` | `jitter` | wanders, amplitude 30 px |
| `jitter_large` | `jitter` | **same path, mirrored**, amplitude 80 px (2.7x) |
| `telescope_inward` | `telescope` | rings contracting toward the center |
| `telescope_outward` | `telescope` | rings expanding — the looming direction |
| `grating_down` | `moving_grating` | whole-field stripes drifting down |
| `grating_up` | `moving_grating` | the same drifting up |
| `grating_inward` | `split_grating` | halves converge on the vertical midline |
| `grating_outward` | `split_grating` | halves stream out to left and right |

The two jitter levels pin `seed_x`/`seed_y` to the *same* values, so they trace
the same underlying path and differ in amplitude alone — a one-variable
manipulation, and the reason they read as one motion at two sizes rather than
two unrelated wanders. `jitter_large` additionally sets `mirror_x`/`mirror_y`:
shared seeds *without* mirroring make the pair move in perfect lockstep when
they appear together, which reads as one object seen twice rather than two
things to choose between. Mirrored, they stay matched in speed, excursion and
timing while moving independently. Delete the seed lines from both to get
fully independent per-trial wander back.

The panel is drawn on a **white** background (`background_gray: 255`), and
every patterned stimulus sets `color_a_gray: 255` so its light phase is the
same white — otherwise the default 240 leaves each patterned circle on a
faintly grey disc with a visible rim. Set them back to 240 if you *want*
the disc boundary visible.

At ±80 px the large jitter needs a GIF canvas bigger than the default
(diameter + 80) or the circle clips the edge — render the panel with
`--size 420`. On screen there is far more room: the left and right
positions sit a quarter screen-width apart.

It uses `mode: sample` (two distinct stimuli drawn per trial, so every pairing
is sampled across enough animals); the file's header comment shows the
`mode: pairs` block to paste in for a control-versus-each design instead.

### Choosing an experiment at launch

```bash
-p experiment_file:=control_vs_grating     # a name -> experiments/<name>.yaml
-p experiment_file:=/abs/path/to/my.yaml    # or a path
-p experiment_file:=""                      # the built-in default
```

Malformed definitions fail at startup with a specific message — unknown
`type`, unknown param name, a stimulus referenced in `conditions` that isn't
defined, a bad random spec, `mode: sample` with < 2 pool entries, `mode: pairs`
with no `pairs:` list, and so on.

---

## The stimulus display

The stimulus display is selected with two ROS params, so the same build runs on
a laptop screen or a projector without edits:

| Param | Meaning |
|---|---|
| `fullscreen` | `true` for the mosquito-facing display |
| `monitor` | `""` primary · `N` that display (1-based) · `span` all of them |
| `window_pos` | windowed only: `"x,y"` on the virtual desktop, e.g. `"1920,0"` |

**Finding which `N` the projector is.** `monitor` is handed to Processing's
`full_screen(N)`, which indexes the Java AWT screen-device list — *not*
necessarily the order xrandr prints, and *not* the numbers Ubuntu's Settings →
Displays panel shows. Do not guess from those; ask directly:

```bash
python3 tools/list_displays.py
```

```
  monitor:= resolution   position     awt id     output
  1         1920x1080    +0+0         :0.0       HDMI-0  [primary]
  2         2560x1440    +1920+0      :0.1       DP-0
```

When two displays share a resolution the output names can't disambiguate them,
so confirm by eye — this opens a fullscreen panel showing the index on each
screen in turn:

```bash
python3 tools/list_displays.py --identify        # every display
python3 tools/list_displays.py --identify 2      # just this one
```

**Cross-checking against Ubuntu's own view.** The `output` column above comes
from `xrandr`, which is how you tie an index to a physical connector:

```bash
xrandr --listmonitors        # one line per active monitor, with position
xrandr | grep " connected"   # connector names, resolutions, physical size
```

```
 0: +*HDMI-0 1920/598x1080/336+0+0  HDMI-0      <- the * marks the primary
 1: +DP-0 2560/697x1440/392+1920+0  DP-0
```

Match a row to `list_displays.py` by **resolution and position** — `DP-0` at
`+1920+0` is the `monitor:=2` line. The connector name tells you which cable:
`HDMI-0`, `DP-0`, `DP-1`, and so on. Ubuntu's **Settings → Displays** shows the
same monitors with an *Identify* button that flashes a number on each screen,
which is useful for working out which physical panel is which — but those
numbers are Ubuntu's own and do **not** map to `monitor:`. Only
`list_displays.py --identify` shows the number this package wants.

> **Wayland.** All of the above needs an **X11** session. Under Wayland,
> `xrandr` reports a single logical output and Java runs through XWayland,
> where fullscreen-on-a-chosen-display is unreliable. Check with
> `echo $XDG_SESSION_TYPE` — if it prints `wayland`, log out and pick
> "Ubuntu on Xorg" from the gear icon on the login screen.

Then put the number in `config/assay_params.yaml`:

```yaml
/**:
  ros__parameters:
    fullscreen: true
    monitor: "2"      # a STRING; "" = primary, "span" = all displays
```

A `monitor` that doesn't exist now fails at startup with the list of what does,
instead of a Java traceback several frames later. Whichever display it ends up
on is recorded in every `stimulus_state` / `trial_start` message under
`geometry.display` (index, resolution, position), so a bag says which physical
screen the animal was actually shown:

```json
"display": {"index": 2, "id": ":0.1", "width": 2560, "height": 1440, "x": 1920, "y": 0}
```

**Check it, and align it to the arena.** `display_check` puts a test pattern
on whichever display the config selects, so you can confirm the projector is
the one that lights up before running an animal:

```bash
ros2 launch mosquito_preference_assay display_check.launch.py
ros2 launch mosquito_preference_assay display_check.launch.py fullscreen:=true monitor:=2
```

It loads the same `config/assay_params.yaml` the assay loads and hands the
settings to `assay.settings()` — the very function the real sketch uses to
pick a screen — so it is not a parallel implementation that might agree by
luck. The pattern shows:

| On screen | What it tells you |
|---|---|
| corner brackets | a clipped or missing one means the projector is overscanning, or the resolution is wrong |
| the two stimulus circles | exactly where the experiment's geometry will put them, at the configured diameter |
| live frame counter + fps | the sketch is really rendering on that screen, not a frozen window |
| display index, resolution, position | which screen it *actually* opened on, beside the one requested |

**Aligning, and pointing the experiment at the result.** Press `s` and
`display_check` writes the geometry twice: `config/display_geometry.local.yaml`
(gitignored, and what every launch file layers over the params file by
default) and a dated archive. So the workflow is align → `s` → launch, with
the circles where you put them, at the size you set:

```bash
ros2 launch mosquito_preference_assay triggered_assay.launch.py \
    experiment_file:=sippell_retest_experiment                     # uses the current one

ros2 launch mosquito_preference_assay triggered_assay.launch.py \
    experiment_file:=sippell_retest_experiment \
    display_config:=config/20260923_display_config.yaml   # or a specific one
```

`display_config:=` takes any file `s` wrote, so keep every dated calibration
and point at whichever you want. It is layered *after* the params file, so it
wins. `circle_diameter_px` is in there as a real parameter: the angular size
the animal sees is the scientific variable, and the pixels that achieve it
depend on this rig's throw distance — so it overrides the experiment's
diameter rather than living in the experiment file.

**Aligning.** The two circles are treated as one object — a **midpoint** and a
**separation** — because that is the rig's real constraint: they sit at the
same height, equidistant from centre. You cannot accidentally leave them at
different heights or off-centre, because there is no way to express it.

| | |
|---|---|
| drag the **midpoint** | move the pair, staying level and equidistant |
| drag **either circle** | set the separation (mirrored on the far side) and the shared height |
| `,` / `.` | closer together / further apart |
| `[` / `]` | shrink / grow both circles |
| drag the dashed rect, or a corner | move / resize the projection surface |
| `s` · `r` · `q` | save · reset to the config · quit |

The dashed rectangle is the **projection surface** — the part of the
projector's output that actually lands on the surface you care about, which is
usually not the whole frame. Put it over the real illuminated area, then place
the circles inside it.

The saved midpoint and half-separation are rounded to whole pixels *before*
the two centres are derived, so the written values are exactly symmetric.
Rounding each centre independently leaves them a pixel apart whenever the
separation is odd — negligible optically, but a built-in left/right asymmetry
is the one bias a two-choice assay should not ship with.

**Saving** writes a **date-stamped** file, `<YYYYMMDD>_display_config.yaml` —
an alignment is a measurement of the rig on a particular day, so when the
projector is next bumped the old numbers are wrong but still on disk. Set
`out_file` to a **directory** to collect them somewhere (the rig config folder
is the natural home), or to a `.yaml` path to name one yourself:

```bash
ros2 launch mosquito_preference_assay display_check.launch.py \
    fullscreen:=true monitor:=2 out_file:=~/rig
```

The startup log prints the absolute path it will save to, and saving prints it
again — the default is relative to wherever you launched from, which is easy
to lose. The file is a valid params file, so paste it into `assay_params.yaml`
or pass it straight back with `--params-file`.

```yaml
/**:
  ros__parameters:
    left_center_px: "500,600"
    right_center_px: "1920,720"
    surface_px: "510,390,2323,1317"
```

`display_check` reads `surface_px` back, so the rectangle starts where you
left it. **The assay does not consume it yet** — stimulus centres are absolute
screen pixels, so keeping them inside the rectangle is currently down to you.

**On a second monitor.** Opening fullscreen on another display is verified:
with `monitor:=2` here the sketch window measures 2560x1440 at +1920+0, the
right screen, and the pattern draws correctly on it. The *dragging* has only
been exercised by hand on the primary display — attempts to verify it on the
second display with synthetic pointer input (`xdotool`) gave inconsistent
results, which looks like warping the pointer rather than a real defect, since
py5 updates `mouse_x`/`mouse_y` from motion events. If a drag on the projector
ever moves the wrong circle or lands off-target, that is worth reporting
rather than working around.

**Stop the projector blanking.** An idle X session will blank the screen and
DPMS will power it down mid-experiment. On the rig machine:

```bash
xset s off; xset s noblank; xset -dpms
```


---

## Deploying to another rig

Split configuration by **what the value belongs to** — the science, the
machine, or the session. The test: *if I moved this experiment to another rig,
should this value travel with it?*

| Belongs to | What | Where |
|---|---|---|
| **The science** | `experiments/*.yaml` — stimulus pool, conditions, duration, circle diameter | in the repo, committed, identical everywhere |
| **The rig** | display / `monitor`, stimulus centres, detector ROI, camera topics, calibration | `config/*.yaml` to start — see below |
| **The session** | bag paths, `master_seed`, one-off overrides | the command line |

Stimulus definitions travel — they *are* the manipulation. A monitor index and
a pixel centre describe a room, and must not.

**Copy the boilerplate to a `.local` file and edit that.** `config/*.yaml` is
tracked, so it is both the fully-commented boilerplate a fresh clone gets and
a file `git pull` can rewrite. A `.local.yaml` beside it is gitignored and
**takes precedence automatically** — every launch file prefers one when it
exists, so no command changes:

```bash
cp config/assay_params.yaml config/assay_params.local.yaml
# edit config/assay_params.local.yaml -- monitor, fullscreen, circle centres
```

A pull can then never touch the config you are actually running on, which is
what you want when you are on site mid-session. A fresh clone has no `.local`
file and simply uses the tracked boilerplate, so nothing is broken before you
make one. The same works for `detector_params.yaml`.

Copy the **whole** file rather than writing a short one: `params_file:=`
replaces rather than merges, so anything you leave out falls back to the
node's hardcoded default rather than to the boilerplate. Every parameter the
node accepts is in there for that reason.

**Split it out when a second rig appears.** The moment two machines each want
their own `monitor` and circle centres, one tracked file cannot hold both —
they will fight on every `git pull`. That is the signal to move to a per-rig
file outside the repo, passed with `params_file:=`:

```
~/rig/                                   # only once you have more than one rig
  assay_params.yaml                      # copied from config/, then edited
  detector_params.yaml
  20260921_display_config.yaml           # alignments collect here, date-stamped
  calibration/Checkerboard_2025_April_10.npy
              Plumbline_2025_April_10.npy
```

```bash
ros2 launch mosquito_preference_assay triggered_assay.launch.py \
    params_file:=~/rig/assay_params.yaml
ros2 launch mosquito_preference_assay display_check.launch.py \
    fullscreen:=true monitor:=2 out_file:=~/rig
```

Version that directory. Alignment values are *data*: "which centres were in use
on 14 May" is something you will want when interpreting results, and both
calibration and alignment need redoing whenever a projector or camera is
physically bumped.

### Rehearsing a trial with no camera and no mosquito

Before an animal is anywhere near the rig, check the whole path at once:

```bash
ros2 launch mosquito_preference_assay trigger_display_test.launch.py \
    params_file:=~/rig/assay_params.yaml \
    experiment_file:=ten_stimulus_panel \
    fullscreen:=true monitor:=2
```

The node comes up ARMED, `test_trigger` fires the trigger itself after
`delay_sec` — standing in for the detector, so no camera is involved — the
trial runs for its `duration_sec`, the node exits, and that exit closes the
bag exactly as on a real run. In one go you see whether:

- the circles land where your config says, on the screen you meant
- your experiment file draws the trial you expect
- the trigger path works end to end
- the bag records **and finalizes**, with the trial inside it

The two config paths are separate because they answer different questions:
`params_file` is **this rig** (which screen, where the circles sit) and
`experiment_file` is **the science** (which stimuli, how the pair is drawn,
how long). `fullscreen` / `monitor` override the params file for a one-off, so
you can rehearse on the projector without editing anything.

Afterwards:

```bash
ros2 bag info <bag_dir>     # metadata.yaml present = it closed cleanly
```

Other arguments: `delay_sec` (4.0) how long ARMED before firing, `repeat_sec`
(>0 to watch several trials), `bag_dir`, `record:=false` for display only.

Like `triggered_assay.launch.py`, this file **forces** the trigger topic and
type on both ends rather than trusting the experiment file's `trigger:` block
to match — a display rehearsal that silently never fires would be worse than
no rehearsal.

### Building so your edits take effect immediately

Build once with `--symlink-install` and you stop rebuilding after every edit:

```bash
colcon build --packages-select mosquito_preference_assay --symlink-install
source install/setup.bash
```

The install space then symlinks through the build space to your **source
tree**, so these are live the moment you save:

| Edit | Live? |
|---|---|
| any `.py` under `mosquito_preference_assay/` | ✅ next time the node starts |
| `experiments/*.yaml`, `config/*.yaml`, `launch/*.py` | ✅ next launch |
| anything under `tools/` | ✅ always — never installed, you run it from source |
| **`setup.py` (a new entry point) or `package.xml`** | ❌ rebuild |

Nothing is live *within* a running process: the sketch reads its experiment
once at startup, so "live" means the next `ros2 run` / `ros2 launch`, not
mid-trial.

**Switching an existing workspace over, and why it is worth doing:** colcon
never *removes* files from the install space, so a plain build leaves deleted
files behind indefinitely. This workspace still had two experiment files that
were deleted from the source months earlier, which meant
`experiment_file:=grating_speed_sweep` worked here and would fail on any fresh
clone. Clear the package out as you switch:

```bash
rm -rf build/mosquito_preference_assay install/mosquito_preference_assay
colcon build --packages-select mosquito_preference_assay --symlink-install
```

### Launching with your own params file

**You only need this once your config lives somewhere other than
`config/assay_params.yaml`** — that file is every launch file's default, so
while you are editing it in place, none of the commands below need
`params_file:=` at all.

Every launch file that runs the sketch or the detector takes `params_file:=`.
Pass it on the command line rather than editing the launch file's default:

```bash
# one animal, the full rig workflow
ros2 launch mosquito_preference_assay triggered_assay.launch.py \
    params_file:=~/rig/assay_params.yaml \
    experiment_file:=sippell_retest_experiment

# rehearse the same thing with no camera
ros2 launch mosquito_preference_assay trigger_display_test.launch.py \
    params_file:=~/rig/assay_params.yaml \
    experiment_file:=sippell_retest_experiment

# check the display and align the circles
ros2 launch mosquito_preference_assay display_check.launch.py \
    params_file:=~/rig/assay_params.yaml out_file:=~/rig

# the plain assay, no trigger
ros2 launch mosquito_preference_assay assay.launch.py \
    params_file:=~/rig/assay_params.yaml

# the detector on its own (its own params file, not the assay's)
ros2 launch mosquito_preference_assay detector.launch.py \
    params_file:=~/rig/detector_params.yaml
```

Or with `ros2 run`, which takes the file directly:

```bash
ros2 run mosquito_preference_assay stimulus_publisher --ros-args \
    --params-file ~/rig/assay_params.yaml
```

| Launch file | `params_file:=` |
|---|---|
| `assay`, `detector`, `display_check`, `trigger_display_test`, `triggered_assay` | ✅ |
| `tracking_benchmark` | ❌ — use `ros2 run` with `--params-file`, or the individual arguments |

**Arguments beat the file, but only when you give them.** `fullscreen`,
`monitor`, `experiment_file`, `master_seed` are all unset-means-leave-alone, so
this rehearses on the projector without touching your saved config:

```bash
ros2 launch mosquito_preference_assay trigger_display_test.launch.py \
    params_file:=~/rig/assay_params.yaml fullscreen:=true monitor:=2
```

**Tired of typing it?** An alias is the right place for it — not the launch
file:

```bash
alias mpa-rig='ros2 launch mosquito_preference_assay triggered_assay.launch.py params_file:=~/rig/assay_params.yaml'
```

### `params_file:=` replaces, it does not merge

A launch file passes **one** params file. Anything you leave out falls back to
the node's **hardcoded** default — *not* to `config/assay_params.yaml`, which
is not read at all once you pass your own. There is one place those disagree:

| | node default | `config/assay_params.yaml` |
|---|---|---|
| `experiment_file` | `""` → the built-in default experiment | `two_choice_default` |

So a minimal rig file containing only `monitor: "2"` would silently run a
*different experiment*. **Copy `config/assay_params.yaml` and edit the copy**
rather than writing a short one from scratch.

### Pulling updates without losing your rig files

**Editing `config/*.yaml` in place is fine while you are the only one
committing** — your changes are just commits like any other, and they follow
you to the next machine. It stops being fine the moment a second rig edits the
same tracked file: then every `git pull` is a merge conflict with calibration
values inside it, and the fix is the per-rig file above. Untracked files pull
cleanly; modified tracked files do not.

Your `*_display_config.yaml` files are safe from `git pull` — it does not touch
untracked files. The command that *would* have deleted them is `git clean -fd`,
so they are now in `.gitignore`, which both keeps `git status` quiet and makes
`clean -fd` skip them. `git clean -fdx` still removes them: `-x` deliberately
includes ignored files.

Keeping the rig directory outside the repo avoids all of this, which is the
real reason to do it.

---

## Where does this setting go?

Five files, each answering a different question. When you are not sure which
one a setting belongs in, ask **what would have to change for this value to be
wrong** — a different question, a different room, or a different afternoon.

| File | Holds | Changes when |
|---|---|---|
| `experiments/<name>.yaml` | the **science**: stimuli, how the pair is drawn, trial length | you ask a different question |
| `config/assay_params.yaml` | the **display node**: which screen, window, trigger wiring | you move to a different rig |
| `config/detector_params.yaml` | the **detector**: camera topic, trigger region, debounce | the camera or arena framing moves |
| `config/tracking_params.yaml` | the **3D pipeline**: per-camera ROIs, sync, calibration paths | the cameras or calibration change |
| `config/display_geometry.local.yaml` | the **calibration**: circle centres and diameter — written by `display_check`, not edited by hand | you realign the projector |

Copy any `config/*.yaml` to `config/*.local.yaml` and edit that: the `.local`
copy is gitignored and wins automatically, so a `git pull` can never move your
rig mid-session.

### The settings people hunt for

| Setting | Lives in | |
|---|---|---|
| **trial length** | `experiments/<name>.yaml` | `duration_sec` |
| **which stimuli, how paired** | `experiments/<name>.yaml` | `stimuli:` / `conditions:` |
| circle **size** and **position** | `config/display_geometry.local.yaml` | written by `display_check`; overrides the experiment's `display:` block |
| which **screen** / projector | `config/assay_params.yaml` | `fullscreen`, `monitor` |
| **trigger region** in the frame | `config/detector_params.yaml` | `roi` — ships as `""`, meaning the whole frame |
| which **camera** fires the trigger | `config/detector_params.yaml` | `image_topic` |
| **how many frames** before a detection counts | `config/detector_params.yaml` | `consecutive_frames` (3) |
| **minimum gap between triggers** | `config/detector_params.yaml` | `cooldown_sec` (10) |
| blob size / sensitivity | `config/detector_params.yaml` | `diff_threshold`, `min_area_px`, `max_area_px` |
| per-camera **tracking ROIs** | `config/tracking_params.yaml` | `tracker_a` / `tracker_b` → `roi` |
| **calibration** `.npy` paths | `config/tracking_params.yaml` | `triangulator` → `checkerboard_file`, `plumbline_file` |
| stereo **pairing tolerance** | `config/tracking_params.yaml` | `stereo_sync` → `sync_slop_sec` |
| 3D **quality gate** | `config/tracking_params.yaml` | `triangulator` → `max_reprojection_error_px` |

### Delays — there are four, and they are unrelated

This is the one that catches people, because they sound alike:

| Delay | Where | What it actually is |
|---|---|---|
| `detector_delay` | **launch argument** (`triggered_assay`) | seconds before the *detector starts*, so it cannot fire at a sketch that is not armed yet. Not a property of detection |
| `delay_sec` | **launch argument** (`trigger_display_test`) | how long the rehearsal sits ARMED before firing its own fake trigger. Rehearsal only |
| `consecutive_frames` | `config/detector_params.yaml` | frames in a row a blob must be seen before it counts as a detection — the real debounce |
| `cooldown_sec` | `config/detector_params.yaml` | minimum gap between one trigger and the next |
| `exit_grace_sec` | `config/assay_params.yaml` | seconds the node stays alive after the trial so trailing messages reach the bag |

None of them is the lag between the trigger and the stimuli appearing. That is
not configurable — it is processing time, measured at **~2 ms** from trigger
message to stimuli drawn, or **~14 ms** end to end from the detector first
seeing the animal.

### Things that belong on the command line, not in a file

Per-run choices: `bag_dir`, `experiment_file`, `master_seed`, and one-off
overrides like `fullscreen:=true monitor:=2`. Every launch argument is
unset-means-leave-the-file-alone, so overriding for one run never disturbs a
saved config.

---

## Triggering

The node opens **ARMED** (blank screen) whenever the experiment file has a
`trigger:` block (or `-p start_mode:=triggered`). It then waits on the trigger
topic for either:

- **`std_msgs/Bool`** (default) — `true` starts the run, `false` aborts back
  to ARMED. What `test_trigger` (below) speaks.
- **`std_msgs/String`** (`trigger.msg_type: string`) — any message received
  starts the run (no abort). What `mosquito_detector_node` (below) speaks —
  its detection-event JSON doubles as the trigger, so the same message both
  fires the trial and gets recorded.

**Which topic / type:** the experiment file's `trigger.topic` / `trigger.msg_type`
(or `trigger.node` → `/<node>/trigger`); the `trigger_topic` / `trigger_msg_type`
ROS params override them. Both are recorded in `~/experiment_info`.

**When a trigger is refused.** Two cases, both logged as warnings rather than
passing silently, since each means a detected animal did *not* get a trial:

- **while a trial is already running** — the trial in progress is left alone.
- **before the sketch has finished starting up.** Opening the JVM and the
  window takes seconds, and a trigger landing in that window cannot run a
  trial. Start whatever produces triggers *after* the sketch is up —
  `triggered_assay.launch.py` does this with `detector_delay`.

### `test_trigger` — fire the trigger on command

A bench helper that publishes the `std_msgs/Bool` trigger, so you can drive a
triggered run without the real tracking nodes.

```bash
# interactive — run from a terminal, one line per action:
ros2 run mosquito_preference_assay test_trigger --ros-args -p topic:=/arena/mosquito_present
#   [Enter] (or t / go)   -> publish true   (start the run)
#   a[Enter] (or f / stop) -> publish false  (abort back to ARMED)
#   q[Enter]               -> quit

# timed — for scripts or inside a launch file (no terminal needed):
ros2 run mosquito_preference_assay test_trigger --ros-args \
    -p topic:=/arena/mosquito_present -p mode:=timer -p delay_sec:=3.0
```

| Param | Default | Meaning |
|---|---|---|
| `topic` | `/stimulus_publisher/trigger` | topic to publish the `std_msgs/Bool` on — set it to match the experiment's `trigger.topic` |
| `mode` | `keypress` | `keypress` (read stdin) or `timer` |
| `delay_sec` | `2.0` | `timer` — fire `true` once, this long after startup |
| `repeat_sec` | `0.0` | `timer` — if `> 0`, keep firing `true` every this many seconds |

Equivalent one-liner without the node:
`ros2 topic pub --once /arena/mosquito_present std_msgs/msg/Bool "{data: true}"`.

### Detector-armed capture — the rig workflow

`triggered_assay.launch.py` is the whole rig in one command: the display comes up **ARMED on the projector before the animal is
introduced**, and the stimuli appear only when a mosquito is found. The point
is *when* the costs are paid — booting the JVM and opening a fullscreen window
takes seconds, and that happens at launch, not at detection.

```bash
ros2 launch mosquito_preference_assay triggered_assay.launch.py \
    fullscreen:=true monitor:=2
```

```
launch ──> recorder up (discovered, so nothing is missed later)
      ├──> stimulus_publisher opens on the projector, sits ARMED drawing
      │      only the background
      └──> mosquito_detector starts after detector_delay and watches
                    │
  mosquito detected ──> detection_event IS the trigger ──> stimuli appear
                    │
       15 s later ──> trial completes ──> node exits ──> bag finalized
```

**Measured on real footage: 14 ms from the detector seeing the mosquito to the
stimuli being on screen** — about one frame at 60 fps. (Compare the
`detection_event`'s `stamp_wall` with `trial_start`'s `trial_start_wall` in the
bag to re-measure on your rig.)

Bagging is unchanged: the node still exits when its trial ends, which still
fires `OnProcessExit → Shutdown`, which still SIGINTs the recorder so
`metadata.yaml` is written. One launch = one animal = one bag.

| Launch arg | Default | |
|---|---|---|
| `fullscreen` / `monitor` | `false` / `""` | the projector — see [the stimulus display](#the-stimulus-display) |
| `detector_params` | `config/detector_params.yaml` | detector tuning |
| `image_topic` | `""` | `""` leaves the params file authoritative |
| `trigger_topic` | `/arena/mosquito_present` | the launch file forces **both** ends onto this, rather than trusting two config files to agree |
| `detector_delay` | `4.0` | seconds before the detector starts |
| `record_mode` | `continuous` | `continuous` \| `snapshot` \| `none` |
| `record_all` | `true` | `-a` (includes camera feeds) vs assay + detection only |
| `experiment_file` / `bag_dir` / `master_seed` | | as above |

**`record_mode`.** `continuous` writes from launch — simple, no discovery race,
and the only mode that can take raw camera video. `snapshot` runs the recorder
in `--snapshot-mode`, buffering in RAM and writing only when the trigger fires,
so waiting for an animal costs nothing on disk; `snapshot_supervisor` calls
`/rosbag2_recorder/snapshot` at trial start and at completion. That buffer is
bounded by `--max-cache-size` (100 MiB default), which is ample for the assay
and tracking topics and **far too small for raw camera feeds**:

| Camera rate | Two 1440×1080 feeds | 15 s trial |
|---|---|---|
| 200 fps | 622 MB/s | 9.3 GB |
| 100 fps | 311 MB/s | 4.7 GB |
| 60 fps | 187 MB/s | 2.8 GB |
| 30 fps | 93 MB/s | 1.4 GB |

So: `snapshot` for the small topics, `continuous` when you want the video.

A detection that arrives before the sketch is armed **cannot** run a trial.
That used to fail silently — the animal was lost and nothing said so. The node
now refuses such a trigger with a `TRIGGER REFUSED` warning, and
`detector_delay` is what stops it arising in the first place.

---

## Running the real experiment

`arena_experiment.launch.py` is the one you run with an animal in the arena.
It assumes your stereo camera package is **already publishing** — it starts no
cameras.

```bash
ros2 launch mosquito_preference_assay arena_experiment.launch.py \
    experiment_file:=sippell_retest_experiment \
    save_dir:=/data/mosquito/2026-09-25
```

Check the cameras first. A launch against silent cameras comes up armed and
simply waits forever:

```bash
ros2 topic hz /cam_sync/cam0/image_raw
```

### What happens

```
launch ──> recorder up, writing every topic EXCEPT the camera feeds
      ├──> stimulus_publisher opens on the projector and sits ARMED
      └──> mosquito_detector watches the trigger zone, HOLDING FIRE
                    │
   sketch reports armed ──> detector goes live
                    │
   mosquito detected ──> stimuli appear (that same message is the trigger)
                    ├──> a second bag starts, for the video
        duration ───> trial ends ──> sketch exits ──> both bags finalized
```

### Where it saves

`save_dir` is the parent; each run gets its own timestamped folder:

```
/data/mosquito/2026-09-25/
    trial_20260925_094717/
        assay/    everything but the video — up from launch, no gap
        video/    the two camera feeds — starts at the trigger
```

`run_name:=blackfly_m3` changes the prefix. Both bags carry the same
timestamps, so they align on playback.

**Two bags, because video cannot be treated like the other topics.** Two
1440×1080 feeds at 200 fps is ~620 MB/s: recording from launch would cost
~2.2 TB per hour of waiting for an animal, and buffering 15 s of it in RAM
(what `--snapshot-mode` does) would need ~9.3 GB. So the video recorder is
spawned at the trigger instead. **Budget ~9 GB per trial at 200 fps** — a
measured 18 s test at 30 fps wrote 3.3 GB.

The cost of starting at the trigger is **0.16 s** before the first frame lands
(rosbag2 subscribing to an already-live topic) — about 30 frames at 200 fps.
The assay topics have no such gap: the detection event, the stimulus
definitions and the trial timing come from the recorder that has been up since
launch.

### The detector holds fire until the display is armed

The sketch takes a few seconds to boot its JVM and open the window. A
detection arriving in that window **cannot** run a trial, and before this was
handled the animal's first approach was refused and the trial ran on whatever
it did after the 10 s cooldown instead.

So the detector watches the sketch's own `stimulus_state` and stays silent
until it reports `armed` — the fact itself, rather than a guessed
`detector_delay`. A real run looks like this:

```
mosquito_detector: holding fire (assay phase=None) -- 100 detections suppressed
mosquito_detector: assay phase: armed
mosquito_detector: mosquito_detected at (211,926) area=1112     ← 10 ms later
stimulus_publisher: [run 1] trial 0 (running): LEFT=jitter_small RIGHT=blank
```

It also suppresses detections *during* a trial, which previously relied on
`cooldown_sec` being long enough. Set `arm_topic: ""` in `detector_params` to
disable the gate.

### Which camera detects

Normally you do not set this at launch. `trigger_roi` saves the camera
alongside the zone, and this launch layers that file over `detector_params`,
so **drawing the zone on cam0 is what points detection at cam0**. For one run:
`detection_camera:=cam1`.

Every resolved choice is printed at startup, so a wrong file is visible
immediately rather than after the session:

```
run folder      : /data/mosquito/2026-09-25/trial_20260925_094717
  assay bag     : .../assay
  video bag     : .../video  (starts at the trigger)
display calib   : .../config/display_geometry.local.yaml
trigger zone    : .../config/trigger_roi.local.yaml
detection camera: /cam_sync/cam0/image_raw
video topics    : /cam_sync/cam0/image_raw, /cam_sync/cam1/image_raw
```

### Arguments

| | | |
|---|---|---|
| `save_dir` | *cwd* | parent directory for the run folder |
| `run_name` | `trial` | run folder prefix; a timestamp is appended |
| `experiment_file` | `sippell_retest_experiment` | |
| `cam0_topic` / `cam1_topic` | `/cam_sync/cam{0,1}/image_raw` | |
| `detection_camera` | *from the zone file* | `cam0` \| `cam1` \| a topic |
| `trigger_config` | *`trigger_roi.local.yaml`* | a specific saved zone |
| `params_file` | `config/assay_params.yaml` | prefers `*.local.yaml` |
| `display_config` | *`display_geometry.local.yaml`* | a specific alignment |
| `monitor` / `fullscreen` | *from params file* | |
| `detector_delay` | `4.0` | backstop; the arming gate is the real guard |
| `record_video` | `true` | `false` = assay topics only |
| `record` | `true` | `false` = dry run, no bags |
| `master_seed` | *from params file* | |

### Which launch file to use

| | |
|---|---|
| `arena_experiment` | **the real run** — two cameras, video recorded from the trigger |
| `triggered_assay` | one camera, simpler recording; the original single-feed workflow |
| `trigger_display_test` | no camera at all — rehearse a trial with a fake trigger |
| `display_check` / `trigger_roi` | calibration: the projector, and the camera |

---

## Detecting a mosquito

`mosquito_detector_node` watches a camera feed, and when something
mosquito-sized moves into a region, publishes a detection-event message —
which (via `trigger.msg_type: string`, above) is also what arms the trial.

**Algorithm:** background subtraction against a static reference frame
(captured once, from the first image received), restricted to `roi`,
blob-area filtered — the same approach and parameter names as
`test_videos_particle_tracking`'s
`detection.py` (ported into `detection.py` here so this package stays
self-contained; frame-to-frame track linking isn't needed for a live
trigger). A detection fires once `consecutive_frames` frames in a row have a
qualifying blob, at most once per `cooldown_sec`.

### Configure it: `config/detector_params.yaml`

Everything the detector needs — which image topic, ROI, thresholds, timing —
is a ROS parameter, all spelled out and documented in
**`config/detector_params.yaml`**. Copy it, edit for your rig, and:

```bash
ros2 launch mosquito_preference_assay detector.launch.py            # uses the shipped default
ros2 launch mosquito_preference_assay detector.launch.py \
    params_file:=/abs/path/to/my_detector.yaml                       # your copy
# or without launch:
ros2 run mosquito_preference_assay mosquito_detector --ros-args \
    --params-file /abs/path/to/my_detector.yaml
```

| Param | Default | Meaning |
|---|---|---|
| `image_topic` | `/camera/image_raw` | `sensor_msgs/Image` input |
| `image_qos` | `reliable` | `reliable` or `sensor_data` (best-effort — matches most camera drivers) |
| `topic` | `/arena/mosquito_present` | detection-event output (`std_msgs/String` JSON) — point the assay's `trigger.topic` here |
| `roi` | `""` | `"x0,y0,x1,y1"` px, exclusive; `""` = whole frame |
| `diff_threshold` | `25` | pixel intensity diff vs. background to count as foreground |
| `min_area_px` / `max_area_px` | `4.0` / `5000.0` | blob area bounds (rejects noise speckle and large intruders) |
| `morph_kernel` | `3` | open/close kernel size (px) cleaning up the mask |
| `consecutive_frames` | `3` | frames with a qualifying blob required before firing |
| `cooldown_sec` | `10.0` | minimum gap between fired events |
| `publish_debug_image` | `false` | also publish an annotated `~/debug_image` (ROI + detected box) — view with `ros2 run rqt_image_view rqt_image_view` |

One-off overrides still work: `-p roi:=340,40,1260,1070` on the `ros2 run`
line, or `-p` after the params file.

Tuning a new rig: set `roi: ""` and `publish_debug_image: true`, watch
`~/debug_image`, and narrow `roi` to exclude anything static that isn't the
arena (equipment, lights, reflections) — `find_candidates()` picks the
*largest* blob, so a static bright spot outside the ROI can otherwise win
over the mosquito.

### Setting the trigger region for a new rig

**Two things decide what fires a trial, and both live in
`config/detector_params.yaml`:**

| | |
|---|---|
| `image_topic` | *which camera*. The trigger is **single-camera**, even though tracking uses two — only this feed can start a trial |
| `roi` | *where in that camera's frame*. `"x0,y0,x1,y1"`, x1/y1 exclusive. This is the trigger region |

`roi` ships as `""`, meaning **the whole frame**. That is rarely what you want:
equipment, indicator lights, mesh edges and reflections are all mosquito-sized
blobs as far as the detector is concerned, and any of them can start a trial.
Setting it is part of commissioning a rig, not an optimisation.

**Drawing it: `trigger_roi`.** Point it at the live camera and drag the box
onto the arena. It is the `display_check` of the camera side.

```bash
ros2 launch mosquito_preference_assay trigger_roi.launch.py
ros2 launch mosquito_preference_assay trigger_roi.launch.py camera:=cam1
```

| | |
|---|---|
| drag inside the box | move the zone |
| drag a corner | resize |
| drag on empty image | draw a new zone |
| arrows (or `w`/`a`/`e`/`x`) | nudge 1 px |
| `[` `]` | shrink / grow about the centre |
| `d` | detection overlay on/off |
| `b` | re-capture the background frame |
| `f` | reset to the whole frame |
| `s` | **save** |

Everything outside the zone is dimmed, so the region the detector is blind to
is visible rather than inferred. The overlay runs the detector's own code with
the detector's own thresholds: a blob boxed **green** would fire a trial, a
**grey** one is seen and ignored. The counter reads
`blobs: 1 inside (would fire), 3 outside (ignored)` — tune until the animal is
the only thing inside.

Press `b` after anything in the scene changes permanently (a moved lamp, a
repositioned feeder); the background frame is captured once and everything
different from it is foreground.

**`s` is the whole handoff.** It writes `config/trigger_roi.local.yaml`, which
every launch layers over `detector_params` automatically — nothing to copy:

```yaml
/**:
  ros__parameters:
    image_topic: "/cam_sync/cam0/image_raw"
    roi: "412,300,980,835"
```

Note that **`image_topic` is saved with the zone**. A pixel box only means
something on the camera it was drawn on, so choosing the camera in the tool is
what points detection at that camera — the two facts cannot drift apart. It
also writes a dated archive (`save_dir:=/data/rig` to choose where), and
`trigger_config:=<path>` runs an older one.

Then check the consequence rather than the picture:

```bash
ros2 topic echo /arena/mosquito_present
```

With no animal in the arena this should stay silent. Anything arriving is a
false trigger, and on the rig it would burn an animal's trial on a reflection.


**Starting numbers.** For the arena in the bundled footage the tracking ROIs
are `340,40,1260,1070` (cam_a) and `350,20,1370,1070` (cam_b) — they exclude
the equipment stand and its indicator lights in the bottom-left, which
otherwise capture the largest-blob heuristic. They are a reasonable first
guess for a similar framing, but they are *that* rig's numbers: re-derive
after any camera move.

**Related knobs in the same file:** `consecutive_frames` (3) is how many
frames in a row must contain a blob before it counts, which suppresses
single-frame noise; `cooldown_sec` (10) is the minimum gap between triggers.
Together they decide how twitchy the trigger is, and neither substitutes for a
correct `roi`.

### JSON schema `mosquito_preference_assay/detection_event/1`

```json
{"schema":"mosquito_preference_assay/detection_event/1","stamp_wall":1788555546.93,
 "event":"mosquito_detected","position_px":[386.18,550.40],"bbox_px":[371,538,30,25],
 "area_px":92.0,"consecutive_frames":2,"roi_px":[340,40,1260,1070],
 "image_topic":"/camera/image_raw","frame_stamp":1788555546.83}
```
(A real detection, from the arena footage under `test_videos_particle_tracking/data/raw/`.)

### Testing without a camera: `video_publisher`

Plays a video file, or a directory of frame images, as a pseudo camera feed —
so the whole pipeline (`video_publisher` → `mosquito_detector` →
`stimulus_publisher`) runs on real or recorded footage with no hardware.

```bash
# a directory of frames, e.g. a real session from test_videos_particle_tracking:
ros2 run mosquito_preference_assay video_publisher --ros-args \
    -p source:=/path/to/test_videos_particle_tracking/data/raw/<session>/cam_a \
    -p topic:=/camera/image_raw -p rate_hz:=30 -p loop:=false

# a video file:
ros2 run mosquito_preference_assay video_publisher --ros-args \
    -p source:=/path/to/clip.mp4 -p rate_hz:=20 -p loop:=true
```

| Param | Default | Meaning |
|---|---|---|
| `source` | *(required)* | video file path, or a directory of `.bmp`/`.png`/`.jpg` frames (sorted by filename) |
| `topic` | `/camera/image_raw` | |
| `rate_hz` | `20.0` | |
| `loop` | `true` | restart from the beginning when the source runs out |
| `frame_id` | `camera` | image `header.frame_id` |

**`rate_hz` is a target, not a guarantee** — each tick decodes a full frame
off disk, so the achieved rate can top out below the request; check with
`ros2 topic hz /camera/image_raw`. `FrameSource` reads frame-directory sources
with `IMREAD_UNCHANGED` rather than forcing a color decode, so a genuinely
grayscale source (mono machine-vision cameras, e.g. Basler `ac*m*`, publish
`mono8` — not upconverted to `bgr8` and converted back downstream). Tested
requesting `200.0` (to emulate the real rig) against the `cam_a` session
above: it settled around **~188 Hz** — close to the request; disk decode is
no longer the bottleneck once the redundant color round-trip is gone (it was
capped around 125 Hz at a 140 Hz request before that fix). The detector still
fires correctly at whatever rate frames actually arrive; which frame index it
lands on to fire can shift between runs since `consecutive_frames` debounces
against the real arrival rate.

Full pipeline, end to end, on real footage:

```bash
# 1. pseudo camera feed
ros2 run mosquito_preference_assay video_publisher --ros-args \
    -p source:=.../data/raw/<session>/cam_a -p rate_hz:=30 -p loop:=false &

# 2. detector (edit config/detector_params.yaml for real use; here just set the roi)
ros2 launch mosquito_preference_assay detector.launch.py &
#    ...or: ros2 run mosquito_preference_assay mosquito_detector --ros-args -p roi:=340,40,1260,1070 &

# 3. the assay, armed, listening for the detector's events, recording to a bag
#    (triggered_assay.launch.py does steps 2 and 3 together -- see "the rig workflow")
ros2 launch mosquito_preference_assay assay.launch.py \
    --ros-args -p start_mode:=triggered \
    -p trigger_topic:=/arena/mosquito_present -p trigger_msg_type:=string
```

### Two synchronized cameras: `dual_video_publisher`

For a stereo rig, `dual_video_publisher` plays two sources from **one shared
timer** — frame *N* of A and frame *N* of B are published together with the
identical ROS timestamp every tick, the way a hardware-triggered stereo pair
would arrive. (For one camera, use `video_publisher` instead.)

```bash
ros2 run mosquito_preference_assay dual_video_publisher --ros-args \
    -p source_a:=.../data/raw/<session>/cam_a -p source_b:=.../data/raw/<session>/cam_b \
    -p topic_a:=/cam_a/image_raw -p topic_b:=/cam_b/image_raw \
    -p rate_hz:=200 -p loop:=false
```

| Param | Default | Meaning |
|---|---|---|
| `source_a` / `source_b` | *(required)* | video file or frame directory, one per camera |
| `topic_a` / `topic_b` | `/cam_a/image_raw` / `/cam_b/image_raw` | |
| `rate_hz` | `20.0` | |
| `loop` | `true` | if **either** source runs out, restart **both** together — never let them drift onto mismatched frame indices |
| `frame_id_a` / `frame_id_b` | `cam_a` / `cam_b` | |

---

## Real-time stereo tracking

First step toward feeding two cameras' 2D detections into a 3D calibration:
run 2D blob tracking on each camera continuously (not the debounced trigger
`mosquito_detector` fires occasionally), and pair the two streams by
timestamp into one synchronized output.

**Two single-camera `tracker` nodes, not one node that owns both cameras** —
run the same node twice, like `video_publisher`:

```bash
ros2 run mosquito_preference_assay tracker --ros-args \
    -p image_topic:=/cam_a/image_raw -p topic:=/tracking/cam_a/position \
    -p roi:=340,40,1260,1070 -p frame_id:=cam_a &
ros2 run mosquito_preference_assay tracker --ros-args \
    -p image_topic:=/cam_b/image_raw -p topic:=/tracking/cam_b/position \
    -p roi:=340,40,1260,1070 -p frame_id:=cam_b &
ros2 run mosquito_preference_assay stereo_sync
```

| Why split | |
|---|---|
| Parallelism | two OS processes → real separate cores, instead of camera A and B sharing one Python callback |
| Modularity | run / tune one camera without the other — same node, run twice |
| Resilience | camera A keeps publishing even if camera B stalls |
| Where sync lives | pairing belongs with whoever needs paired data (eventually: 3D triangulation) — not baked into the tracker |

**`tracker`** (one per camera) — same background-subtraction detection as
`mosquito_detector` (`detection.py`), but publishes **every** frame with a
qualifying blob instead of a debounced trigger. Output is
`geometry_msgs/PointStamped`: `point.x` / `point.y` are the pixel position,
**`point.z` is the blob *area*** (not a real z — reused so the message
carries a `header.stamp` for `message_filters` to synchronize on, without a
custom message type). Nothing is published for a frame with no qualifying
blob — "not detected" is the absence of a message.

| Param | Default | Meaning |
|---|---|---|
| `image_topic` | `/camera/image_raw` | `sensor_msgs/Image` input |
| `image_qos` | `reliable` | `reliable` or `sensor_data` |
| `topic` | `~/position` | `geometry_msgs/PointStamped` output |
| `frame_id` | `camera` | point `header.frame_id` |
| `roi`, `diff_threshold`, `min_area_px` / `max_area_px`, `morph_kernel` | same as `mosquito_detector` | detection tuning |
| `log_every_n` | `200` | log the achieved detection rate every N (`0` disables) |
| `publish_debug_image` | `False` | publish `~/debug_image`: the frame with the ROI box and the accepted blob drawn on it — for eyeballing *when and where* detection is good. Off by default (costs a color conversion + encode per frame) |

To watch both cameras' detections live, turn it on for each tracker and open
them with `ros2 run rqt_image_view rqt_image_view /tracker_a/debug_image`.
Frames where the two cameras circle *different* objects are exactly the ones
the triangulator's reprojection-error filter rejects.

**`stereo_sync`** — pairs two `tracker` outputs by `header.stamp`
(`message_filters.ApproximateTimeSynchronizer`) and republishes them as one
`std_msgs/String` JSON message — the same shape a combined tracker would have
produced, so a downstream 3D step doesn't care that tracking is two processes.
A pair only comes out when **both** cameras detected something at (about) the
same instant, which is exactly what triangulation needs.

| Param | Default | Meaning |
|---|---|---|
| `topic_a` / `topic_b` | `/tracking/cam_a/position` / `/tracking/cam_b/position` | the two `tracker` outputs |
| `topic` | `/tracking/stereo_track` | combined output (`std_msgs/String` JSON) |
| `sync_slop_sec` | `0.05` | max stamp difference to count as one instant |
| `queue_size` | `100` | buffered messages per side awaiting a match |
| `log_every_n` | `200` | log the achieved synchronized-pair rate every N (`0` disables) |

### JSON schema `mosquito_preference_assay/stereo_track/1`

```json
{"schema":"mosquito_preference_assay/stereo_track/1","stamp_wall":1789148349.56,
 "frame_stamp":1789148349.54,
 "a":{"detected":true,"x":386.20,"y":550.10,"area":93.0},
 "b":{"detected":true,"x":414.48,"y":549.85,"area":81.0}}
```
(A real synchronized pair from the arena footage — two viewpoints on the same
physical mosquito, ready to feed a 3D calibration.)

### Throughput

On 1440×1080 footage with a real ROI, `tracker` ×2 → `stereo_sync` keeps pace
at **~150–165 Hz with no dropped pairs**; camera decode tops out around
188 Hz, which is the real ceiling.

Two design points behind that, both worth keeping if you modify the pipeline:
the trackers are **separate nodes** (a single combined node fell behind and
permanently lost frames once `message_filters`' sync queue overflowed), and
`detection.py` crops to the ROI **before** the diff/threshold/morphology
rather than masking afterwards, which is worth ~6.5× per frame.

## Tested: real footage, 200 fps target

`dual_video_publisher` → two `tracker`s → `stereo_sync`, on the real `cam_a`/
`cam_b` session, `rate_hz:=200`, real ROI:

| | Rate | 776 real frame-pairs |
|---|---|---|
| Camera decode (after the grayscale fix) | ~188 Hz | — |
| Split tracking (`tracker` ×2 + `stereo_sync`) | **~150–165 Hz**, kept pace | **all 776, zero drops** |

An earlier single combined-node design was tried first and discarded: it
fell behind at ~90–125 Hz and **permanently lost frames** once
`message_filters`' sync queue overflowed (stalled at 500/776, no further
output even after a 30 s wait) — the split design above is what actually
works at this rate, not just an incremental tweak.

`detection.py` crops to the ROI **before** the diff/threshold/morphology
rather than working full-frame and masking afterwards, which is worth ~6.5×
per frame (7.8 ms → 1.2 ms under load at 1440×1080 with the arena ROI) and
lifts the per-camera ceiling from ~128 Hz to ~836 Hz. Verified to produce
identical centroids on a full real session.

### Watching a trajectory live: `trajectory_plotter`

A live matplotlib 3D plot of a position stream — for watching a trajectory
get computed instead of reading numbers off a topic echo. It's a plain
visualizer (subscribes only, never publishes), aimed at whatever eventually
publishes real x/y/z from the 3D calibration; for now, test it against a
made-up trajectory with `synthetic_trajectory_publisher`:

```bash
ros2 run mosquito_preference_assay synthetic_trajectory_publisher --ros-args \
    -p pattern:=lissajous
ros2 run mosquito_preference_assay trajectory_plotter
```

Both read/write `geometry_msgs/PointStamped` on `/tracking/position_3d` by
default — real `x`/`y`/`z`, no field-reuse hack needed since there are three
real dimensions to fill.

**The axis box holds still.** Each axis grows to fit the full trajectory seen
so far and then stops — it does *not* rescale to whichever points currently
happen to be in the trailing window, which would make it visibly resize every
redraw as the trail slides. Give `xlim`/`ylim`/`zlim` (`"min,max"`) to pin an
axis to a known range (e.g. the real arena size) from the very first frame
instead of growing into it.

| Param | Default | Meaning |
|---|---|---|
| `topic` | `/tracking/position_3d` | `geometry_msgs/PointStamped` input |
| `max_points` | `500` | trailing window kept/drawn (`<=0` = unbounded) |
| `redraw_hz` | `15.0` | plot refresh rate — decoupled from the message rate |
| `xlim` / `ylim` / `zlim` | `""` | `"min,max"` to fix an axis (e.g. the arena); `""` = grow-to-fit then hold |
| `equal_aspect` | `True` | one unit is the same length on all three axes, so fixed arena limits draw the arena's true shape rather than a cube |
| `title` | `mosquito_preference_assay -- 3D trajectory` | window title |

`synthetic_trajectory_publisher` params: `pattern` (`lissajous` default /
`helix` / `random_walk`), `rate_hz` (30.0), `period_sec` (8.0, lissajous/helix),
`scale_xy` / `scale_z` / `z_offset` (extent + center), `step_std` /`seed`
(random_walk), `topic`, `frame_id`.

---

## 3D triangulation

`triangulator` turns each synchronized stereo pair into a real 3D position,
completing the chain:

```
dual_video_publisher ─┬─> tracker (cam_a) ─┐
                      └─> tracker (cam_b) ─┴─> stereo_sync ─> triangulator ─> trajectory_plotter
```

```bash
ros2 run mosquito_preference_assay triangulator --ros-args \
    -p checkerboard_file:=/path/Checkerboard_<date>.npy \
    -p plumbline_file:=/path/Plumbline_<date>.npy \
    -p max_reprojection_error_px:=3.0
```

Output is `geometry_msgs/PointStamped` on `/tracking/position_3d` — x/y/z in
**mm** in the calibration's world frame, carrying the original camera-frame
stamp. `trajectory_plotter` subscribes to exactly this by default, so the two
compose with no arguments.

Per pair: undistort → `cv2.triangulatePoints` → axis remap → plumbline
rotation → reprojection error.

| Param | Default | Meaning |
|---|---|---|
| `topic` | `/tracking/stereo_track` | `stereo_track/1` JSON input |
| `output_topic` | `/tracking/position_3d` | `geometry_msgs/PointStamped` output, mm |
| `checkerboard_file` | — | **required**, intrinsics + projection matrices |
| `plumbline_file` | — | **required**, 3×3 rotation into the gravity-aligned frame |
| `frame_id` | `arena` | output `header.frame_id` |
| `max_reprojection_error_px` | `0.0` | drop triangulations worse than this; `0` = keep all |
| `log_every_n` | `200` | log rate + last reprojection error every N |

**Calibration is rig-specific and is not shipped with this package.** The
`Checkerboard` array is 27×5, read as `[0:3]` K0 · `[3:6]` K1 · `[12:15]` P0 ·
`[15:18]` P1 · `[18:19]`/`[19:20]` distortion.

**Camera order matters.** `topic_a` must be the camera the calibration calls
cam0. Getting it backwards still triangulates, just badly — an order of
magnitude worse reprojection error. If errors look high, try the swap before
blaming the calibration.

**Reprojection error is the per-frame quality signal.** It separates good
frames from frames where the two cameras locked onto *different* objects,
which is what happens with more than one animal in the arena. A blown-up p90
with a sane median means mismatched targets, not bad calibration;
`max_reprojection_error_px:=3.0` filters them out.

---

## Benchmarking and latency

Two tools for checking a machine keeps up — run them after moving to new
hardware, before trusting it with live cameras.

### `benchmark` — per-component cost, no messaging involved

```bash
ros2 run mosquito_preference_assay benchmark --ros-args \
    -p frames:=/path/to/session/cam_a -p roi:=340,40,1260,1070 \
    -p checkerboard_file:=/path/Checkerboard_<date>.npy \
    -p plumbline_file:=/path/Plumbline_<date>.npy
```

Times each stage on real frames and reports the frame rate ceiling each one
implies, so when the live pipeline misses a target you know what to blame.
Sample (idle workstation, 1440×1080 mono):

| stage | ms/frame | Hz ceiling | |
|---|---|---|---|
| decode frame file | 0.63 | 1577 | replay only |
| detect, full frame | 1.55 | 643 | 1.56 Mpx |
| detect, ROI | 0.68 | 1477 | 0.95 Mpx (61% of frame) |
| cv_bridge encode | 1.33 | 752 | **does not shrink with the ROI** |
| cv_bridge decode | 0.26 | 3910 | likewise |
| triangulate one pair | 0.018 | 56094 | per pair, not per camera |

Run it on an otherwise idle machine — the same detection measured 0.68 ms
idle and 1.17 ms with a live pipeline running alongside.

### `pipeline_monitor` — live per-stage lag and yield

```bash
ros2 run mosquito_preference_assay pipeline_monitor
```

Every stage forwards the *original* camera-frame stamp, so for each one
`lag = wall clock - frame stamp` is the true age of that data. Comparing lag
across stages localizes where latency accumulates; comparing counts shows
where frames are lost:

```
=== pipeline report (4.0 s window) ===
stage                 msgs   rate Hz  lag med     p90     p99     max
cam_a position         767     191.7      4.3    16.8    26.5    27.1
cam_b position         705     176.2     54.1    58.7    60.8    63.1
stereo pairs           705     176.2     54.9    59.2    61.2    63.8
3D positions           705     176.2     55.4    59.8    61.8    64.7
yield: pairs/cam_a 91.9%, 3D/pairs 100.0%
```

Read that as: cam_b's tracker is 50 ms behind cam_a's, and since a pair can
only complete once the slower camera arrives, **the whole pipeline inherits
the slowest camera's lag**. Each report is also published as JSON on
`~/report` (schema `mosquito_preference_assay/pipeline_report/1`), so a bag
of a run carries its own timings.

| Param | Default | Meaning |
|---|---|---|
| `report_period_sec` | `5.0` | reporting interval (`0` = only on shutdown) |
| `watch_images` | `False` | also measure the image topics — costs full-frame bandwidth and can skew what you are measuring |
| `publish_report` | `True` | publish each report as JSON on `~/report` |
| `image_qos` | `sensor_data` | QoS for the image topics when `watch_images` is on |

Two caveats worth knowing: **lag is wall clock minus stamp**, so it only
means anything if whatever stamped the frames shares this machine's clock
(same box is fine; a camera host elsewhere needs PTP/chrony or you are
measuring clock offset). And leave `watch_images` off unless you need input
accounting — receiving every full frame is real bandwidth.

### Where the tracking configuration lives

`config/tracking_params.yaml` — the same boilerplate-plus-`.local` pattern as
the assay:

```bash
cp config/tracking_params.yaml config/tracking_params.local.yaml
# edit the .local one: ROIs, and the calibration paths
```

It is keyed **per node**, not with the `/**` wildcard, because `tracker_a` and
`tracker_b` are two instances of the same executable that must *not* share
settings — each has its own camera and its own ROI.

| Node | What lives there |
|---|---|
| `tracker_a` / `tracker_b` | camera topic, output topic, `frame_id`, **ROI**, `image_qos`, detection tuning |
| `stereo_sync` | the two input topics, `sync_slop_sec`, queue size |
| `triangulator` | **`checkerboard_file` / `plumbline_file`**, output topic, `max_reprojection_error_px` |
| `pipeline_monitor` | reporting interval, `watch_images` |

Setting the calibration paths there means you stop passing them on every
command. Launch arguments still override for a one-off, and an unset argument
leaves the file alone:

```bash
ros2 launch mosquito_preference_assay tracking_benchmark.launch.py \
    session:=/path/to/session rate_hz:=200.0        # everything from the config

ros2 launch mosquito_preference_assay tracking_benchmark.launch.py \
    session:=/path/to/session image_qos:=sensor_data roi_a:=400,100,1200,1000
```

With no calibration set anywhere it says so and runs tracking only, with
stereo pairs as the final stage, rather than failing obscurely.

### One command for the whole thing

```bash
ros2 launch mosquito_preference_assay tracking_benchmark.launch.py \
    session:=/path/to/session rate_hz:=200.0 \
    checkerboard_file:=/path/Checkerboard_<date>.npy \
    plumbline_file:=/path/Plumbline_<date>.npy
```

Brings up the full pipeline against recorded footage with the monitor
attached. `session` must contain `cam_a/` and `cam_b/`. Omit the calibration
arguments to benchmark tracking only. Useful arguments: `rate_hz`, `loop`,
`roi_a`/`roi_b`, `image_qos`, `max_reprojection_error_px`, `plot:=true` for
the live 3D view, `watch_images:=true` for input-rate accounting.

### Lag, and the `image_qos` trade-off

End-to-end, image published → 3D point delivered:

| Config | Median lag | Frames kept |
|---|---|---|
| 200 Hz, `reliable` | 56 ms | all |
| 200 Hz, `sensor_data` | **7.2 ms** | ~85% |
| 150 Hz, `reliable` | 16 ms | all |
| 150 Hz, `sensor_data` | **6.9 ms** | ~95% |

**This is a real trade, not a tuning knob.** `reliable` queues (depth 10)
rather than dropping, so every frame is processed but lag grows to roughly
*queue depth × frame interval* under load — 56 ms is about 11 frames at
200 Hz. `sensor_data` (best-effort) always works on the newest frame and
discards stale ones: ~7 ms, at the cost of frames when saturated.

Use `sensor_data` for closed-loop triggering, where freshness wins.
Use `reliable` when recording a complete trajectory.

Shrinking the ROI is **not** a lever for lag: there is a floor of a few ms
even at 200×200 px, because the full frame is still encoded, shipped and
decoded regardless. Cropping at the *camera* shrinks payload, encode,
transport and detection together; keeping full frames from crossing a process
boundary at all (detection inside the camera node, or intra-process
composition) removes the transport term entirely.

---|---|---|---|
| 200 Hz, `reliable` | 56 ms | 758 | 181 Hz |
| 200 Hz, `sensor_data` | **7.2 ms** | 646 | 181 Hz |
| 150 Hz, `sensor_data` | **6.9 ms** | 716 | 150 Hz |
| 150 Hz, `reliable` | 16 ms | **758** | 150 Hz |

**The `image_qos` choice is a real trade, not a tuning knob.** `reliable`
queues (depth 10) rather than dropping, so every frame is processed but lag
grows to roughly *queue depth × frame interval* under load — 56 ms ≈ 11
frames at 200 Hz. `sensor_data` (best-effort) always works on the newest
frame and discards stale ones: ~7 ms, at the cost of ~15% of frames when
saturated. Use `sensor_data` for closed-loop triggering where freshness
wins, `reliable` when recording a complete trajectory.

Two things that are *not* levers, both measured: shrinking the ROI only
touches the detection term (there is a ~3.6 ms floor even at 200×200 px,
because the full frame is still encoded, shipped and decoded), and the ROI is
already close to the flight envelope — detections span 850×787 px inside the
920×1030 `cam_a` ROI, so trimming further starts clipping real flight near
the arena walls. Cropping at the *camera* shrinks payload, encode, transport
and detection together; keeping full frames from crossing a process boundary
at all (detection in the camera node, or intra-process composition) removes
the transport term entirely.

---

## ROS parameters

Operational only — experiment *design* lives in the experiment YAML.
`config/assay_params.yaml` holds the defaults; override with `-p name:=value`
or a `params_file`.

| Param | Default | Notes |
|---|---|---|
| `experiment_file` | `"two_choice_default"` | name (→ `experiments/`), path, or `""` for the built-in default |
| `start_mode` | `""` | `""` derive from the experiment's `trigger:` block · `"auto"` play immediately · `"triggered"` open ARMED |
| `trigger_topic` | `""` | `""` use the experiment's `trigger.topic`, else `~/trigger` |
| `trigger_msg_type` | `""` | `""` use the experiment's `trigger.msg_type`, else `bool`. `bool` = `std_msgs/Bool` (true start, false abort); `string` = `std_msgs/String` (any message starts) |
| `master_seed` | `-1` | `-1` → random (logged + in `experiment_info`); `≥0` → reproducible |
| `fullscreen` | `false` | `true` for the mosquito-facing display |
| `monitor` | `""` | `""` primary · `"2"` that display (1-indexed) · `"span"` all. Fullscreen only. An index that doesn't exist fails at startup listing the ones that do — see [the stimulus display](#the-stimulus-display) for finding the projector's number |
| `window_pos` | `""` | windowed only — place the sketch at `"x,y"` px |
| `window_w` / `window_h` | `1200` / `800` | ignored when `fullscreen` |
| `left_center_px` / `right_center_px` | `""` | `"x,y"` px override of the experiment's `display.*_center_px` |
| `heartbeat_hz` | `10.0` | `stimulus_state` re-publish rate |
| `exit_grace_sec` | `2.0` | stay up this long after a finite experiment completes |
| `show_debug` | `false` | on-screen labels/timer overlay. **Off**: it draws text on the mosquito-facing display — "waiting for trigger" while armed, and the stimulus names under each circle during a trial. Press `d` to toggle it while setting up |

Screen selection is a ROS param (rig-specific) not an experiment-file field, so
an experiment YAML stays portable between rigs.

---

## Published messages

Three topics, all `std_msgs/String` carrying one JSON object, all **latched**
(reliable, transient_local, keep_last(1) — a subscriber or `ros2 bag record`
that starts late immediately gets the current value). Names relative to the
node (`/stimulus_publisher/…`).

| Topic | When | Contents |
|---|---|---|
| `~/experiment_info` | **once**, at startup | static run metadata — see below |
| `~/stimulus_state` | at trial start, at `heartbeat_hz`, and on phase changes | the trial state |
| `~/trial_start` | at trial start and on `phase: "complete"` | same object as `stimulus_state` |

`ros2 topic echo` truncates long strings — use `--full-length`, or:

```bash
ros2 topic echo --field data /stimulus_publisher/stimulus_state \
  | python3 -c 'import sys,json;[print(json.dumps(json.loads(l),indent=2)) for l in sys.stdin if l.strip()]'
```

### `~/experiment_info` — schema `mosquito_preference_assay/experiment_info/1`

Everything that's constant for the whole run, so it stays out of every
`stimulus_state` message.

```json
{"schema":"mosquito_preference_assay/experiment_info/1",
 "stamp_wall": 1788458991.09,
 "start_mode": "triggered",
 "master_seed": 99, "noise_seed": 99,
 "experiment": {"name":"single_trigger","file":".../single_trigger.yaml",
                "sha1":"712444465f1a","n_stimuli":4,"mode":"sample",
                "pool":["static_dark","jitter","telescope","moving_grating"],
                "duration_sec":15.0,"circle_diameter_px":200,
                "trigger_topic":"/arena/mosquito_present","weights":null}}
```

`mode: pairs` replaces `weights` with `pairs` (the list of pairing names).
`master_seed` replays the run. `sha1` changes if you edit the experiment YAML,
so recordings are distinguishable.

### `~/stimulus_state` — schema `mosquito_preference_assay/stimulus_state/3`

**Armed** (triggered mode, before the trigger) — just:

```json
{"schema":"…/3","stamp_wall":…,"phase":"armed","run_id":0}
```

**Running / complete:**

```json
{
  "schema": "mosquito_preference_assay/stimulus_state/3",
  "stamp_wall": 1788459002.90,           // time.time() at publish
  "phase": "running",                    // running | complete
  "run_id": 1,                           // increments per trigger
  "trial_id": 0,                         // monotonic from 0
  "trial_seed": 767950141,               // + the condition -> replays this trial
  "trial_uuid": "573d1acf-…",
  "condition": {"name": "jitter|moving_grating", "ordered": false},
  "trial_start_wall": 1788459000.95,
  "trial_duration_sec": 3.0,             // resolved value
  "elapsed_sec": 1.95,                   // since trial_start_wall
  "geometry": {"window_w":2560,"window_h":1440,"fullscreen":true,"monitor":"2",
               "display":{"index":2,"id":":0.1","width":2560,"height":1440,
                          "x":1920,"y":0},
               "circle_diameter_px":160,
               "left_center_px":[640.0,720.0],"right_center_px":[1920.0,720.0]},
  "left":  {"name":"jitter","type":"jitter",
            "params":{"fill_gray":20,"amplitude_px":20,"noise_speed":1.2,
                      "seed_x":490.67,"seed_y":154.37}},
  "right": {"name":"moving_grating","type":"moving_grating",
            "params":{"period_px":24,"speed_px_per_sec":40,"angle_deg":150.47,
                      "color_a_gray":240,"color_b_gray":20}}
}
```

- **`left` / `right`** are the authoritative placement — `name` is the pool
  entry, `type` is the marker behavior (they differ when the YAML gives a
  custom name). `params` are all resolved concrete values.
- **`geometry.display`** is the screen the sketch *actually* opened on
  (`index` is the value `monitor` takes), as opposed to `monitor`, which is
  only what was asked for — so a bag records which physical display the animal
  was shown. It reads `{"windowed": true, …}` in windowed mode and
  `{"spanning": true, …}` for `monitor: span`. Present from the very first
  message, including `trial_start`.
- **`condition.name`** is a grouping key, *not* placement — `ordered: false` →
  `"a|b"` sorted, side-independent (the sides this trial are in `left`/`right`);
  `ordered: true` → `"a->b"`, sides fixed by the pairing.
- Per-type `params`: `static_dark` → `fill_gray`; `jitter` → `fill_gray,
  amplitude_px, noise_speed, seed_x, seed_y`; `moving_grating` → `period_px,
  speed_px_per_sec, angle_deg, color_a_gray, color_b_gray`; `telescope` →
  `ring_spacing_px, speed_px_per_sec, color_a_gray, color_b_gray`.

---

## Reproducing a run offline

`master_seed` (in `experiment_info`) replays the run — the draw, the sides,
every resolved parameter. `left`/`right` in `stimulus_state` give the placement
and resolved params directly, and each animation phase is closed-form in
`elapsed_sec` (grating: `(t·speed) % period`; telescope: `(t·speed) %
(2·spacing)`; jitter: `py5.noise()` under the recorded `noise_seed` +
`seed_x/seed_y`).

---

## Adding a new marker behavior

1. Subclass `Stimulus` in `stimuli.py`: `__init__(self, diameter_px, **params)`
   with defaults, `display(self, cx, cy, t)`, and `_params()` returning every
   parameter (all resolved values must appear here — that's what the message
   publishes).
2. Register it in `STIMULUS_TYPES` in `stimulus_types.py`. If it has internal
   animation state that should be RNG-seeded (like jitter's noise offsets), add
   it to `_RNG_FILLED`.
3. Reference it from an experiment YAML's `stimuli:`.

---

## Development

```bash
# unit tests (param_spec/detection are pure; experiment needs py5 + a JDK on
# PATH/JAVA_HOME; detection needs opencv)
cd ~/ros2_ws/src/mosquito_preference_assay
python3 -m pytest test/

# lint
python3 -m flake8 mosquito_preference_assay/
python3 -m pydocstyle mosquito_preference_assay/    # pep257

# or via colcon
cd ~/ros2_ws && colcon test --packages-select mosquito_preference_assay
```

CI (`.github/workflows/ci.yml`) runs flake8 + pep257 + `colcon build` on
Humble on every push.

---

## License & citing

MIT — see [LICENSE](LICENSE). © 2026 David Stupski, Riffell Lab, University of
Washington.

If you use this in a publication, please cite the repository
(`https://github.com/dstupski/mosquito_preference_assay`).


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

- [How it works](#how-it-works)
- [Setup](#setup) · [dependency reference](#dependency-reference)
- [Quick start](#quick-start)
- [Writing an experiment](#writing-an-experiment)
- [Triggering](#triggering) · [`test_trigger`](#test_trigger--fire-the-trigger-on-command)
- [Detecting a mosquito](#detecting-a-mosquito) · [`video_publisher`](#testing-without-a-camera-video_publisher) · [`dual_video_publisher`](#two-synchronized-cameras-dual_video_publisher)
- [Real-time stereo tracking](#real-time-stereo-tracking) — `tracker` + `stereo_sync`
- [Watching a trajectory live](#watching-a-trajectory-live-trajectory_plotter) — `trajectory_plotter`
- [3D triangulation](#3d-triangulation) — `triangulator`
- [Benchmarking and latency](#benchmarking-and-latency) — `benchmark` + `pipeline_monitor`
- [ROS parameters](#ros-parameters)
- [Published messages](#published-messages)
- [Reproducing a run offline](#reproducing-a-run-offline)
- [Adding a new marker behavior](#adding-a-new-marker-behavior)
- [Development](#development)
- [License & citing](#license--citing)

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
| `static_dark` | Plain dark circle, no motion — baseline / control |
| `jitter` | Dark circle whose position wanders smoothly (Perlin noise) |
| `moving_grating` | Black/white stripes drifting across the circle (optomotor-style) |
| `telescope` | Concentric rings expanding outward — tunnel effect |

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

```
mosquito_preference_assay/
  stimuli.py                   marker behaviors (Stimulus subclasses)
  stimulus_types.py            type registry + build_stimulus()
  param_spec.py                literal-or-random parameter resolution
  experiment.py                load/validate the YAML; the trial draw
  assay.py                     the py5 sketch + thread-safe current_state()
  stimulus_publisher_node.py   the ROS 2 node
  test_trigger_node.py         bench helper: publish the Bool trigger on command
  detection.py                 background-subtraction blob detection (ported
                                from test_videos_particle_tracking)
  mosquito_detector_node.py    watches a camera feed, publishes detection events
  frame_source.py               shared video-file / frame-directory reader
  video_publisher_node.py      plays a video / frame directory as a pseudo camera feed
  dual_video_publisher_node.py  two SYNCHRONIZED pseudo camera feeds (one shared timer)
  tracker_node.py               real-time 2D blob tracking, one camera per instance
  stereo_sync_node.py           pairs two tracker_node outputs by timestamp
  trajectory_plotter_node.py    live matplotlib 3D plot of a position stream
  synthetic_trajectory_publisher_node.py  made-up 3D trajectory, for testing the plotter
  triangulator_node.py          stereo pair -> real 3D position (mm), via the rig calibration
  pipeline_monitor_node.py      live per-stage lag / rate / yield reporting
  benchmark_node.py             per-component timings: what this machine can do
experiments/                   experiment definitions (installed to share/)
  two_choice_default.yaml       random draw of 2 markers (also the built-in default)
  control_vs_grating.yaml       mode: pairs — control vs a random grating band
  single_trigger.yaml           triggered 15 s trial, then everything concludes
config/
  assay_params.yaml             operational ROS params for stimulus_publisher
  detector_params.yaml          ROS params for mosquito_detector
launch/
  assay.launch.py               stimulus_publisher + its params file
  detector.launch.py            mosquito_detector + its params file
  triggered_capture.launch.py   stimulus_publisher (triggered) + ros2 bag record + auto-shutdown
  tracking_benchmark.launch.py  whole tracking pipeline + pipeline_monitor, on recorded footage
test/                          unit + lint tests
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
go to [Quick start](#quick-start). If `import py5` fails, see **`JAVA_HOME`**
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

## Quick start

```bash
# play immediately, built-in default experiment (random draw of 2 markers / trial)
ros2 launch mosquito_preference_assay assay.launch.py

# a specific experiment, fullscreen on projector 2
ros2 run mosquito_preference_assay stimulus_publisher --ros-args \
    -p experiment_file:=control_vs_grating -p fullscreen:=true -p monitor:=2

# record alongside your other topics
ros2 bag record /stimulus_publisher/stimulus_state /stimulus_publisher/trial_start
```

The trial runs for its `duration_sec`, then `phase` becomes `complete` and the
node exits (Ctrl-C or closing the window also stop it).

**No-ROS preview** (built-in default, just to eyeball the stimuli — publishes
nothing):

```bash
ros2 run mosquito_preference_assay assay
```

Keys while running: `d` toggle the debug overlay · `n` draw a fresh trial · `esc` quit.

---

## Writing an experiment

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

### Single-run capture

`triggered_capture.launch.py` brings the node up ARMED next to `ros2 bag
record`. The trigger plays one trial (15 s with `single_trigger`), then the
node exits, which emits a launch `Shutdown` — SIGINT to the recorder, bag
finalized. One launch = one animal = one bag. Re-arm for the next = relaunch.

```bash
ros2 launch mosquito_preference_assay triggered_capture.launch.py
```

| Launch arg | Default | |
|---|---|---|
| `experiment_file` | `single_trigger` | name or path |
| `trigger_topic` | `""` | override the experiment's `trigger:` topic |
| `trigger_msg_type` | `""` | override the experiment's `trigger.msg_type` (`bool` \| `string`) |
| `bag_dir` | `./mpa_<timestamp>` | output dir (must not already exist) |
| `record_all` | `true` | `true` → `ros2 bag record -a` (captures cameras / trigger too); `false` → assay topics only |
| `fullscreen` / `monitor` / `master_seed` | | passed to the node |

The node's `exit_grace_sec` (default 2 s) keeps it alive briefly after the
trial so the trailing `phase: "complete"` messages land in the bag.

---

## Detecting a mosquito

`mosquito_detector_node` watches a camera feed, and when something
mosquito-sized moves into a region, publishes a detection-event message —
which (via `trigger.msg_type: string`, above) is also what arms the trial.

**Algorithm:** background subtraction against a static reference frame
(captured once, from the first image received), restricted to `roi`,
blob-area filtered — the same approach and parameter names as
[`test_videos_particle_tracking`](../test_videos_particle_tracking)'s
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
ros2 launch mosquito_preference_assay triggered_capture.launch.py \
    trigger_topic:=/arena/mosquito_present trigger_msg_type:=string
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

### Tested: real footage, 200 fps target

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
    -p checkerboard_file:=/path/Checkerboard_2025_April_10.npy \
    -p plumbline_file:=/path/Plumbline_2025_April_10.npy \
    -p max_reprojection_error_px:=3.0
```

Output is `geometry_msgs/PointStamped` on `/tracking/position_3d` — real
x/y/z in **mm**, in the calibration's world frame, carrying the original
camera-frame stamp. `trajectory_plotter` subscribes to exactly this by
default, so the two compose with no arguments.

The math (undistort → `cv2.triangulatePoints` → axis remap → plumbline
rotation → reprojection error) is a direct port of `triangulate()` from
`validate_mosquito_centroid_tracking_vid_output.py` in the calibration set,
run per message instead of over a recorded array — verified to reproduce it
to 6e-11 mm.

| Param | Default | Meaning |
|---|---|---|
| `topic` | `/tracking/stereo_track` | `stereo_track/1` JSON input |
| `output_topic` | `/tracking/position_3d` | `geometry_msgs/PointStamped` output, mm |
| `checkerboard_file` | — | **required**, path to `Checkerboard_<date>.npy` |
| `plumbline_file` | — | **required**, path to `Plumbline_<date>.npy` |
| `frame_id` | `arena` | output `header.frame_id` |
| `max_reprojection_error_px` | `0.0` | drop triangulations worse than this; `0` = keep all |
| `log_every_n` | `200` | log rate + last reprojection error every N |

**Calibration is rig-specific and is not shipped with this package** — point
the two parameters at your rig's pair. Expected layout of the 27×5
`Checkerboard` array (rows 6–11 and 20–26 exist but are unused, matching the
reference script): `[0:3]` K0 · `[3:6]` K1 · `[12:15]` P0 · `[15:18]` P1 ·
`[18:19]`/`[19:20]` distortion. `Plumbline` is 3×3, rotating the triangulated
point into the gravity-aligned world frame.

**Camera order matters.** `topic_a` must be the camera the calibration calls
cam0. Getting it backwards still triangulates, but badly — on real footage
the correct order gave a **0.25 px** median reprojection error versus
**2.34 px** swapped, and 97% vs 70% of frames under 3 px. If your errors look
high, try the swap before blaming the calibration.

**Reprojection error is your per-frame quality signal.** It cleanly separates
good frames from frames where the two cameras locked onto *different* objects
— which is exactly what happens with two mosquitoes in the arena. Measured
across five recorded sessions against one calibration:

| session | median | p90 | ≤ 3 px | note |
|---|---|---|---|---|
| A (single mosquito) | 0.25 px | 0.59 | 97.3% | |
| B (single mosquito) | 0.54 px | 0.86 | 99.2% | |
| C | 2.50 px | **150.95** | 50.7% | two mosquitoes in the arena |
| D | 0.88 px | **233.78** | 53.0% | likewise |
| E | 1.47 px | 1.64 | 96.4% | recorded a day *before* the calibration |

A blown-up p90 with a sane median means mismatched targets, not bad
calibration. `max_reprojection_error_px:=3.0` filters them out.

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

### Measured: where the time goes at 200 fps

End-to-end (image published → 3D point delivered), 776 real frame pairs:

| Config | Median lag | Points (of 758) | Throughput |
|---|---|---|---|
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
| `monitor` | `""` | `""` primary · `"2"` that display (1-indexed) · `"span"` all. Fullscreen only. |
| `window_pos` | `""` | windowed only — place the sketch at `"x,y"` px |
| `window_w` / `window_h` | `1200` / `800` | ignored when `fullscreen` |
| `left_center_px` / `right_center_px` | `""` | `"x,y"` px override of the experiment's `display.*_center_px` |
| `heartbeat_hz` | `10.0` | `stimulus_state` re-publish rate |
| `exit_grace_sec` | `2.0` | stay up this long after a finite experiment completes |
| `show_debug` | `true` | on-screen overlay |

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
  "geometry": {"window_w":1200,"window_h":800,"fullscreen":false,"monitor":null,
               "circle_diameter_px":160,
               "left_center_px":[300.0,400.0],"right_center_px":[900.0,400.0]},
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

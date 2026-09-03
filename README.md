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
- [Dependencies](#dependencies)
- [Install](#install)
- [Quick start](#quick-start)
- [Writing an experiment](#writing-an-experiment)
- [Triggering](#triggering) · [`test_trigger`](#test_trigger--fire-the-trigger-on-command)
- [ROS parameters](#ros-parameters)
- [Published messages](#published-messages)
- [Reproducing a session offline](#reproducing-a-session-offline)
- [Adding a new marker behaviour](#adding-a-new-marker-behaviour)
- [Development](#development)
- [License & citing](#license--citing)

---

## How it works

Three layers, top to bottom:

| Layer | Where | What it does |
|---|---|---|
| **Stimulus pool** | `stimuli:` in the experiment YAML | Named, reusable stimulus definitions — a `type` (one of the built-in marker behaviours) plus `params`. A param can be a fixed value **or** a random spec resolved per trial. |
| **Conditions** | `conditions:` | How each trial's `{left, right}` pair is chosen. `mode: sample` (default) draws two distinct stimuli from the pool at random each trial; `mode: pairs` cycles a fixed set of pairings. |
| **Schedule** | `schedule:` | Trial-sequence controls. `max_trials` caps the run (**defaults to 1 when a `trigger:` block is present** — one trigger, one trial). `order` / `loop` matter only for `mode: pairs` free-running sessions. |

The **marker behaviours** are code (`stimuli.py`); the YAML only *composes
instances* of them:

| `type` | Behaviour |
|---|---|
| `static_dark` | Plain dark circle, no motion — baseline / control |
| `jitter` | Dark circle whose position wanders smoothly (Perlin noise) |
| `moving_grating` | Black/white stripes drifting across the circle (optomotor-style) |
| `telescope` | Concentric rings expanding outward — tunnel effect |

The **ROS node** (`stimulus_publisher`) runs the sketch and publishes: the
static run metadata once on `~/experiment_info`, and the current trial on
`~/stimulus_state` (continuously) + `~/trial_start` (per trial). It can start
playing immediately (`start_mode: auto`) or wait ARMED for a trigger message
(`start_mode: triggered`).

**Reproducibility:** one integer `master_seed` replays the whole run (every
draw, every side, every parameter); it is logged at startup and published in
`~/experiment_info`.

### Repository layout

```
mosquito_preference_assay/
  stimuli.py                   marker behaviours (Stimulus subclasses)
  stimulus_types.py            type registry + build_stimulus()
  param_spec.py                literal-or-random parameter resolution
  experiment.py                load/validate the YAML, conditions, scheduler
  assay.py                     the py5 sketch + thread-safe current_state()
  stimulus_publisher_node.py   the ROS 2 node
  test_trigger_node.py         bench helper: publish the Bool trigger on command
experiments/                   experiment definitions (installed to share/)
  two_choice_default.yaml       random-draw default (also the built-in default)
  grating_speed_sweep.yaml      fixed pairings, random-range params, finite run
  single_trigger.yaml           one triggered 15 s trial, then everything concludes
config/assay_params.yaml       operational ROS params
launch/
  assay.launch.py               node + params file
  triggered_capture.launch.py   node (triggered) + ros2 bag record + auto-shutdown
test/                          unit + lint tests
```

---

## Dependencies

| Dependency | Version | Comes from | Notes |
|---|---|---|---|
| **ROS 2** | Humble | apt (`ros-humble-desktop`) | `rclpy`, `std_msgs`, `launch`, `ros2bag`, `ament_index_python` all included |
| **PyYAML** | any | ships with ROS 2 (`rclpy` dep) | experiment-file parsing |
| **py5** | ≥ 0.10 | `pip install --user py5` | the sketch. **Not in rosdep** — must be installed into the interpreter ROS uses (`/usr/bin/python3`) |
| **numpy** | **< 2** | `pip install --user "numpy<2"` | py5 pulls numpy 2, which is ABI-incompatible with the apt `python3-matplotlib` (harmless `_ARRAY_API not found` spam otherwise). py5 runs fine on 1.26. |
| **Java** | 17 | Processing 4 bundle, or `py5-install-jdk` | py5 needs a Java 17 JVM. `assay.py` auto-sets `JAVA_HOME` if it's unset and can find one (see below). |
| a display | — | — | this is a windowed/fullscreen sketch; there is no headless mode |

**`JAVA_HOME`** — on import, `assay.py` sets it (if unset) to the first of:
`~/Applications/Processing/lib/app/resources/jdk`, or a JDK under
`~/.cache/py5/` that `py5-install-jdk` created. If none exist, export it
yourself before launching.

There is no `requirements.txt` because the ROS/apt half and the pip half live
in different places; the two `pip install --user` lines above are the whole
Python story. A dedicated venv with `--system-site-packages` (for `rclpy`) is
the tidier option if you deploy this to several rigs.

---

## Install

```bash
# 1. Python deps into the ROS interpreter
python3 -m pip install --user py5 "numpy<2"
python3 -c 'import py5; print("py5", py5.__version__)'   # sanity (needs JAVA_HOME or a findable JDK)

# 2. clone into a workspace and build
cd ~/ros2_ws/src
git clone https://github.com/dstupski/mosquito_preference_assay.git
cd ~/ros2_ws
colcon build --packages-select mosquito_preference_assay
source install/setup.bash
```

---

## Quick start

```bash
# play immediately, built-in default experiment (random draw of 2 markers / trial)
ros2 launch mosquito_preference_assay assay.launch.py

# a specific experiment, fullscreen on projector 2
ros2 run mosquito_preference_assay stimulus_publisher --ros-args \
    -p experiment_file:=grating_speed_sweep -p fullscreen:=true -p monitor:=2

# record alongside your other topics
ros2 bag record /stimulus_publisher/stimulus_state /stimulus_publisher/trial_start
```

Stop with Ctrl-C or by closing the sketch window. A finite experiment
(`schedule.max_trials`, or `mode: pairs` + `loop: false`) shuts the node down
when it finishes.

**No-ROS preview** (built-in default, just to eyeball the stimuli — publishes
nothing):

```bash
ros2 run mosquito_preference_assay assay
```

Keys while running: `d` toggle the debug overlay · `n` next trial · `esc` quit.

---

## Writing an experiment

`experiments/two_choice_default.yaml` is the fully-commented reference. The
shape:

```yaml
schema: mosquito_preference_assay/experiment/1
name: my_experiment

# 1. POOL — named stimulus specs. Any param is a literal OR a random spec
#    resolved per trial from the trial seed:
#      {uniform: [lo,hi]}  {randint: [lo,hi]}  {choice: [...]}  {normal: [mu,sd]}
stimuli:
  control:      {type: static_dark,    params: {fill_gray: 20}}
  wander:       {type: jitter,         params: {amplitude_px: 20, noise_speed: 1.2}}
  grating:      {type: moving_grating, params: {period_px: 24,
                                                speed_px_per_sec: {uniform: [20, 80]},
                                                angle_deg: {uniform: [0, 360]}}}
  tunnel:       {type: telescope,      params: {ring_spacing_px: 18, speed_px_per_sec: 50}}

# 2. CONDITIONS — how each trial picks its {left, right} pair.
conditions:
  mode: sample                       # sample (default) | pairs
  pool: [control, wander, grating, tunnel]   # subset of `stimuli`; default = all
  # weights: {grating: 2, control: 1}         # optional, sample mode — bias the draw

  # --- mode: pairs only ---
  # generate: all_pairs              # all_pairs | all_ordered_pairs | none
  # allow_same: false                # include X-vs-X pairs
  # explicit: [{left: control, right: grating}]   # + hand-listed pairs (fixed sides)
  # exclude:  [{a: wander, b: tunnel}]            # - drop pairs

# 3. SCHEDULE — omit it entirely for a triggered experiment (one trigger = one
#    trial). Only needed to cap a batch, or to shape a mode: pairs run.
schedule:
  max_trials: null                   # cap the run; defaults to 1 if `trigger:` is set
  order: shuffle                     # mode: pairs only — shuffle | sequential | random
  loop: true                         # mode: pairs only — false -> one pass then "complete"
  reshuffle_each_loop: true

duration_sec: 15.0                    # trial length — the single knob (literal, or {uniform: [25,35]})

# Optional. Its presence makes this a triggered experiment: the node opens
# ARMED, and each trigger fires one trial. Omit it to play immediately.
trigger:
  topic: /arena/mosquito_present     # a std_msgs/Bool your tracking node publishes; true = go
  # node: arena                      # shorthand for topic: /arena/trigger

display:
  circle_diameter_px: 200
  left_center_px: null               # null -> auto (w*0.25, h/2)
  right_center_px: null              # null -> auto (w*0.75, h/2)
  background_gray: 128
```

### `mode: sample` (default)

Each trial: draw **two distinct** stimuli from `pool` at random (no
replacement) — first drawn → right, second → left. `weights:` biases the draw.
Coverage of pairings is by chance. This is the standard preference-assay
design.

### `mode: pairs`

Build a **fixed set** of pairings — `generate: all_pairs` (every distinct
unordered pair), `all_ordered_pairs` (left/right fixed), or `none` — then add
`explicit:` pairs and drop `exclude:` pairs. `schedule.order` walks the set;
unordered pairs get their sides coin-flipped each trial for counterbalancing.
Use this for balanced factorial designs or a hand-picked pairing list.

### Choosing an experiment at launch

```bash
-p experiment_file:=grating_speed_sweep     # a name -> experiments/<name>.yaml
-p experiment_file:=/abs/path/to/my.yaml    # or a path
-p experiment_file:=""                      # the built-in default
```

Malformed definitions fail at startup with a specific message — unknown
`type`, unknown param name, a stimulus referenced in `conditions` that isn't
defined, a bad random spec, `mode: sample` with < 2 pool entries, and so on.

---

## Triggering

The node opens **ARMED** (blank screen) whenever the experiment file has a
`trigger:` block (or `-p start_mode:=triggered`). It then waits for a
`std_msgs/Bool` on the trigger topic: **`true` = start the run**, `false` =
abort back to ARMED.

**Which topic:** the experiment file's `trigger.topic` (or `trigger.node` →
`/<node>/trigger`); the `trigger_topic` ROS param overrides it. The resolved
topic is recorded in `~/experiment_info`.

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
finalised. One launch = one animal = one bag. Re-arm for the next = relaunch.

```bash
ros2 launch mosquito_preference_assay triggered_capture.launch.py
```

| Launch arg | Default | |
|---|---|---|
| `experiment_file` | `single_trigger` | name or path |
| `trigger_topic` | `""` | override the experiment's `trigger:` topic |
| `bag_dir` | `./mpa_<timestamp>` | output dir (must not already exist) |
| `record_all` | `true` | `true` → `ros2 bag record -a` (captures cameras / trigger too); `false` → assay topics only |
| `fullscreen` / `monitor` / `master_seed` | | passed to the node |

The node's `exit_grace_sec` (default 2 s) keeps it alive briefly after the
trial so the trailing `phase: "complete"` messages land in the bag.

The trigger is `std_msgs/Bool` — if your source uses a different type, change
the subscription in `stimulus_publisher_node.py` (`_on_trigger`).

---

## ROS parameters

Operational only — experiment *design* lives in the experiment YAML.
`config/assay_params.yaml` holds the defaults; override with `-p name:=value`
or a `params_file`.

| Param | Default | Notes |
|---|---|---|
| `experiment_file` | `"two_choice_default"` | name (→ `experiments/`), path, or `""` for the built-in default |
| `start_mode` | `"auto"` | `"auto"` play immediately · `"triggered"` open ARMED |
| `trigger_topic` | `"~/trigger"` | `std_msgs/Bool` — `true` start, `false` abort |
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
| `~/stimulus_state` | every trial change + `heartbeat_hz` + phase changes | the current trial |
| `~/trial_start` | once per new trial (and on `phase: "complete"`) | same object as `stimulus_state` |

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
                "duration_sec":15.0,"circle_diameter_px":160,
                "max_trials":1,"weights":null}}
```

`master_seed` replays the whole run. `sha1` changes if you edit the experiment
YAML, so recordings are distinguishable.

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
  entry, `type` is the marker behaviour (they differ when the YAML gives a
  custom name). `params` are all resolved concrete values.
- **`condition.name`** is a grouping key, *not* placement — `ordered: false` →
  `"a|b"` sorted, side-independent (the sides this trial are in `left`/`right`);
  `ordered: true` → `"a->b"`, sides fixed by the pairing.
- Per-type `params`: `static_dark` → `fill_gray`; `jitter` → `fill_gray,
  amplitude_px, noise_speed, seed_x, seed_y`; `moving_grating` → `period_px,
  speed_px_per_sec, angle_deg, color_a_gray, color_b_gray`; `telescope` →
  `ring_spacing_px, speed_px_per_sec, color_a_gray, color_b_gray`.

---

## Reproducing a session offline

`master_seed` (in `experiment_info`) replays the entire run — every random
draw, every side assignment, every resolved parameter. For a single trial:
`left`/`right` give the placement and resolved params directly, and each
animation phase is closed-form in `elapsed_sec` (grating: `(t·speed) % period`;
telescope: `(t·speed) % (2·spacing)`; jitter: `py5.noise()` under the recorded
`noise_seed` + `seed_x/seed_y`).

---

## Adding a new marker behaviour

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
# unit tests (param_spec is pure; experiment needs py5 + a JDK on PATH/JAVA_HOME)
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

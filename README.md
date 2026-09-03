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
- [Triggered single-run capture](#triggered-single-run-capture)
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
| **Schedule** | `schedule:` | `max_trials` cap; for `mode: pairs`, the order (`shuffle` / `sequential` / `random`) and looping. |

The **marker behaviours** are code (`stimuli.py`); the YAML only *composes
instances* of them:

| `type` | Behaviour |
|---|---|
| `static_dark` | Plain dark circle, no motion — baseline / control |
| `jitter` | Dark circle whose position wanders smoothly (Perlin noise) |
| `moving_grating` | Black/white stripes drifting across the circle (optomotor-style) |
| `telescope` | Concentric rings expanding outward — tunnel effect |

The **ROS node** (`stimulus_publisher`) runs the sketch and publishes the
current state on `~/stimulus_state` (continuously) and `~/trial_start` (per
trial). It can start playing immediately (`start_mode: auto`) or wait ARMED for
a trigger message (`start_mode: triggered`).

**Reproducibility:** one integer `master_seed` replays the whole run (every
draw, every side, every parameter); it is logged at startup and included in
every message.

### Repository layout

```
mosquito_preference_assay/
  stimuli.py                   marker behaviours (Stimulus subclasses)
  stimulus_types.py            type registry + build_stimulus()
  param_spec.py                literal-or-random parameter resolution
  experiment.py                load/validate the YAML, conditions, scheduler
  assay.py                     the py5 sketch + thread-safe current_state()
  stimulus_publisher_node.py   the ROS 2 node
experiments/                   experiment definitions (installed to share/)
  two_choice_default.yaml       random-draw default (also the built-in default)
  grating_speed_sweep.yaml      fixed pairings, random-range params, finite run
  single_trigger_15s.yaml       one 15 s trial per trigger
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

# 3. SCHEDULE
schedule:
  max_trials: null                   # integer to cap the run
  # mode: pairs only:
  order: shuffle                     # shuffle | sequential | random
  loop: true                         # false -> one pass then "complete"
  reshuffle_each_loop: true

trial:
  duration_sec: 30.0                 # literal or {uniform: [25, 35]}

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

## Triggered single-run capture

`triggered_capture.launch.py` brings up the node **ARMED** (blank screen) next
to `ros2 bag record`. A `std_msgs/Bool` `{data: true}` on the trigger topic
plays one trial (15 s with `single_trigger_15s`); the node then exits, which
emits a launch `Shutdown`, which SIGINTs the recorder so the bag is finalised
and closed.

```bash
ros2 launch mosquito_preference_assay triggered_capture.launch.py

# from your trigger source, or by hand:
ros2 topic pub --once /stimulus_publisher/trigger std_msgs/msg/Bool "{data: true}"
```

| Launch arg | Default | |
|---|---|---|
| `experiment_file` | `single_trigger_15s` | name or path |
| `trigger_topic` | `/stimulus_publisher/trigger` | |
| `bag_dir` | `./mpa_<timestamp>` | output dir (must not already exist) |
| `record_all` | `true` | `true` → `ros2 bag record -a`; `false` → assay + trigger topics only |
| `fullscreen` / `monitor` / `master_seed` | | passed to the node |

The node's `exit_grace_sec` (default 2 s) keeps it alive briefly after the
trial so the trailing `phase: "complete"` messages land in the bag.

To wait for a trigger *without* the bag/launch machinery:
`-p start_mode:=triggered -p trigger_topic:=/your/topic`. The trigger is
`std_msgs/Bool` (`true` = start, `false` = abort to ARMED) — if your source
uses a different type, change the subscription in `stimulus_publisher_node.py`
(`_on_trigger`).

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
| `master_seed` | `-1` | `-1` → random (logged, in every message); `≥0` → reproducible |
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

Both topics are `std_msgs/String` carrying one JSON object. Names are relative
to the node (`/stimulus_publisher/…`).

| Topic | When | QoS |
|---|---|---|
| `~/stimulus_state` | every trial change + `heartbeat_hz` + phase changes | reliable, transient_local, keep_last(1) — **latched** |
| `~/trial_start` | once per new trial (and on `phase: "complete"`) | same |

Latching means a subscriber or `ros2 bag record` that starts mid-session
immediately gets the current state.

`ros2 topic echo` truncates long strings — use `--full-length`, or:

```bash
ros2 topic echo --field data /stimulus_publisher/stimulus_state \
  | python3 -c 'import sys,json;[print(json.dumps(json.loads(l),indent=2)) for l in sys.stdin if l.strip()]'
```

### JSON schema `mosquito_preference_assay/stimulus_state/2`

| Key | Meaning |
|---|---|
| `schema` | `"mosquito_preference_assay/stimulus_state/2"` |
| `stamp_wall` | `time.time()` at publish |
| `phase` | `"armed"` (triggered mode, pre-trigger) · `"running"` · `"complete"` |
| `run_id` | increments per trigger; `0` before the first (`auto` mode starts at `1`) |
| `experiment` | `{name, file, sha1, n_stimuli, mode, pool, max_trials, …}` — identifies the definition |
| `master_seed`, `noise_seed` | session seeds (constant all run) |
| `trial_id` | monotonic, from 0 |
| `trial_seed` | given the draw, fully determines the trial's visuals |
| `trial_uuid` | uuid4 |
| `condition_name` | grouping key for the pairing. `condition_ordered: false` → `"a\|b"` (sorted, side-independent); `true` → `"a->b"` (left→right fixed) |
| `condition_ordered` | bool |
| `trial_start_wall`, `trial_duration_sec`, `elapsed_sec` | timing (`trial_duration_sec` is the per-trial resolved value) |
| `geometry` | `window_w/h`, `fullscreen`, `monitor`, `circle_diameter_px`, `left_center_px [x,y]`, `right_center_px [x,y]` |
| **`left_name` / `right_name`** | **authoritative** — the pool name actually on each side this trial |
| `left` / `right` | full descriptor of that side: `type`, `uuid`, `diameter_px`, `slot`, `center_px`, **+ every resolved param** |

**To know what's on which side, read `left_name` / `right_name` (or
`left`/`right`), never `condition_name`** — for an unordered pairing the name
is deliberately side-independent so trials of the same pairing group together.

While `phase == "armed"` the message is short: `schema`, `stamp_wall`,
`phase`, `run_id`, `experiment`, `master_seed`, `noise_seed` — no trial fields.
Check `phase` first.

Per-type params in `left`/`right`: `static_dark` → `fill_gray`; `jitter` →
`fill_gray, amplitude_px, noise_speed, seed_x, seed_y`; `moving_grating` →
`period_px, speed_px_per_sec, angle_deg, color_a_gray, color_b_gray`;
`telescope` → `ring_spacing_px, speed_px_per_sec, color_a_gray, color_b_gray`.

---

## Reproducing a session offline

`master_seed` replays the entire run — every random draw, every side
assignment, every resolved parameter. For a single trial: `left_name` /
`right_name` give the placement, `trial_seed` + `random.Random(trial_seed)`
replays the resolved params (`left` / `right` already carry the resolved
values), and each animation phase is closed-form in `elapsed_sec`
(grating: `(t·speed) % period`; telescope: `(t·speed) % (2·spacing)`; jitter:
`py5.noise()` under the recorded `noise_seed` + `seed_x/seed_y`).

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

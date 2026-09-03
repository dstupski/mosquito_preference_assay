"""Load and interpret an experiment definition.

An experiment YAML has three layers:

  stimuli:     named, reusable stimulus specs (type + params; params may be
               literals or random specs -- see param_spec.py)
  conditions:  how each trial's {left, right} pairing is chosen --
                 mode: sample  (default)  each trial draws TWO DISTINCT
                               stimuli from `pool` at random (no replacement);
                               first drawn -> right, second -> left.
                 mode: pairs              cycle through a FIXED set of pairings
                               built from `generate` (all_pairs /
                               all_ordered_pairs / none) + `explicit` -
                               `exclude`, ordered by `schedule`.
  schedule:    trial-sequence controls -- max_trials (both modes); order /
               loop / reshuffle_each_loop (pairs mode only).

plus a top-level `duration_sec` (the single trial-length knob) and a `display`
block (circle diameter, centres, background).

Randomness is split so a session replays from the master seed and a single
trial replays from its trial seed:

  * the Scheduler consumes the *master* RNG (the sample draw / the pair order
    and per-trial side flip)
  * Experiment.realize() consumes the *trial* RNG (resolving random param
    specs, per-trial duration)
"""

import copy
import hashlib
import os
from collections import namedtuple

import yaml

from .param_spec import resolve_params, resolve_value, validate_params
from .stimulus_types import STIMULUS_TYPES, known_param_names

EXPERIMENT_SCHEMA = "mosquito_preference_assay/experiment/1"


class ExperimentError(ValueError):
    """A malformed or inconsistent experiment definition."""


TrialPlan = namedtuple(
    "TrialPlan",
    "condition_name condition_ordered left_name right_name left_type right_type "
    "left_params right_params duration_sec",
)

# What the Scheduler hands back each trial: the two stimulus names with sides
# already decided, plus a grouping key.
_Draw = namedtuple("_Draw", "left right name ordered")


class StimulusSpec:
    __slots__ = ("name", "type", "params")

    def __init__(self, name, type_, params):
        self.name = name
        self.type = type_
        self.params = params  # unresolved


class Condition:
    """A fixed {left, right} pairing (pairs mode only).

    ordered=True  -> ``a`` is always LEFT, ``b`` always RIGHT; name "a->b".
    ordered=False -> unordered pair; the side flip is per trial; name is the
                     two members sorted and joined with "|" -- a stable,
                     position-independent key for grouping trials.
    """

    __slots__ = ("a", "b", "ordered", "name")

    def __init__(self, a, b, ordered):
        self.a = a
        self.b = b
        self.ordered = ordered
        self.name = f"{a}->{b}" if ordered else "|".join(sorted((a, b)))

    def __repr__(self):
        return f"Condition({self.name})"


def _pair_ab(pair):
    if isinstance(pair, (list, tuple)) and len(pair) == 2:
        return pair[0], pair[1]
    if isinstance(pair, dict):
        if "left" in pair and "right" in pair:
            return pair["left"], pair["right"]
        if "a" in pair and "b" in pair:
            return pair["a"], pair["b"]
    raise ExperimentError(f"bad pair {pair!r}; use [a, b] or {{left: a, right: b}}")


# Built-in default: the four classic markers, each trial a fresh random draw of
# two distinct markers from the pool.
DEFAULT_DOC = {
    "schema": EXPERIMENT_SCHEMA,
    "name": "two_choice_default",
    "description": "Each trial: two distinct markers drawn at random from the pool.",
    "stimuli": {
        "static_dark": {"type": "static_dark", "params": {"fill_gray": 20}},
        "jitter": {"type": "jitter",
                   "params": {"fill_gray": 20, "amplitude_px": 20, "noise_speed": 1.2}},
        "moving_grating": {"type": "moving_grating",
                           "params": {"period_px": 24, "speed_px_per_sec": 40,
                                      "angle_deg": {"uniform": [0, 360]}}},
        "telescope": {"type": "telescope",
                      "params": {"ring_spacing_px": 18, "speed_px_per_sec": 50}},
    },
    "conditions": {"mode": "sample"},
    "schedule": {},
    "duration_sec": 15.0,
    "display": {"circle_diameter_px": 200, "background_gray": 128},
}


class Experiment:

    def __init__(self, doc, source="<default>"):
        if not isinstance(doc, dict):
            raise ExperimentError("experiment definition must be a mapping")
        self.source = source
        self.raw = copy.deepcopy(doc)
        self.sha1 = hashlib.sha1(
            yaml.safe_dump(doc, sort_keys=True).encode()
        ).hexdigest()[:12]
        self._parse(doc)

    # -- constructors ------------------------------------------------------- #
    @classmethod
    def default(cls):
        return cls(copy.deepcopy(DEFAULT_DOC), source="<built-in default>")

    @classmethod
    def from_file(cls, path):
        try:
            with open(path) as fh:
                doc = yaml.safe_load(fh)
        except OSError as exc:
            raise ExperimentError(f"cannot read experiment file {path!r}: {exc}") from exc
        except yaml.YAMLError as exc:
            raise ExperimentError(f"{path}: invalid YAML: {exc}") from exc
        if not isinstance(doc, dict):
            raise ExperimentError(f"{path}: top level must be a mapping")
        return cls(doc, source=os.path.abspath(path))

    # -- parsing / validation -------------------------------------------- #
    def _parse(self, doc):
        schema = doc.get("schema")
        if schema and schema != EXPERIMENT_SCHEMA:
            raise ExperimentError(
                f"unsupported experiment schema {schema!r} "
                f"(this build expects {EXPERIMENT_SCHEMA!r})"
            )
        self.name = doc.get("name", "unnamed")
        self.description = doc.get("description", "")

        self.stimuli = self._parse_stimuli(doc.get("stimuli") or {})
        self._parse_conditions(doc.get("conditions") or {})

        sched = doc.get("schedule") or {}
        self.order = sched.get("order", "shuffle")
        if self.order not in ("shuffle", "sequential", "random"):
            raise ExperimentError(
                f"schedule.order {self.order!r} must be shuffle | sequential | random"
            )
        self.loop = bool(sched.get("loop", True))
        self.reshuffle_each_loop = bool(sched.get("reshuffle_each_loop", True))
        self.max_trials = sched.get("max_trials")
        if self.max_trials is not None:
            self.max_trials = int(self.max_trials)

        # A single top-level `duration_sec` (a `trial: {duration_sec: ...}`
        # block is also accepted for older files). Default 15 s.
        self.duration_spec = doc.get("duration_sec")
        if self.duration_spec is None:
            self.duration_spec = (doc.get("trial") or {}).get("duration_sec", 15.0)
        try:
            validate_params({"duration_sec": self.duration_spec})
        except ValueError as exc:
            raise ExperimentError(str(exc)) from None

        disp = doc.get("display") or {}
        self.circle_diameter_px = int(disp.get("circle_diameter_px", 200))
        self.left_center_px = _center_or_none(
            disp.get("left_center_px"), "display.left_center_px")
        self.right_center_px = _center_or_none(
            disp.get("right_center_px"), "display.right_center_px")
        self.background_gray = int(disp.get("background_gray", 128))

        # Optional: what fires the trial. Its presence means this is a triggered
        # experiment (the node opens ARMED). A ROS param still overrides.
        #   trigger: {topic: /arena_tracking/mosquito_detected}
        #   trigger: {node: arena_tracking}     # -> /arena_tracking/trigger
        self.trigger_topic = None
        trig = doc.get("trigger")
        if trig is not None:
            if not isinstance(trig, dict):
                raise ExperimentError("trigger: must be a mapping with `topic` or `node`")
            if trig.get("topic"):
                self.trigger_topic = str(trig["topic"])
            elif trig.get("node"):
                self.trigger_topic = f"/{str(trig['node']).strip('/')}/trigger"
            else:
                raise ExperimentError("trigger: needs a `topic` or a `node`")

    def _parse_stimuli(self, raw):
        if not raw:
            raise ExperimentError("experiment defines no `stimuli`")
        out = {}
        for name, spec in raw.items():
            if not isinstance(spec, dict) or "type" not in spec:
                raise ExperimentError(
                    f"stimulus {name!r}: expected a mapping with a `type` key"
                )
            type_ = spec["type"]
            if type_ not in STIMULUS_TYPES:
                raise ExperimentError(
                    f"stimulus {name!r}: unknown type {type_!r}; "
                    f"known: {sorted(STIMULUS_TYPES)}"
                )
            params = spec.get("params") or {}
            unknown = set(params) - known_param_names(type_)
            if unknown:
                raise ExperimentError(
                    f"stimulus {name!r} (type {type_}): unknown param(s) "
                    f"{sorted(unknown)}; accepted: {sorted(known_param_names(type_))}"
                )
            try:
                validate_params(params)
            except ValueError as exc:
                raise ExperimentError(f"stimulus {name!r}: {exc}") from None
            out[name] = StimulusSpec(name, type_, params)
        return out

    def _parse_conditions(self, cdoc):
        self.mode = cdoc.get("mode", "sample")
        if self.mode not in ("sample", "pairs"):
            raise ExperimentError(f"conditions.mode {self.mode!r} must be sample | pairs")

        self.pool = cdoc.get("pool") or list(self.stimuli)
        for n in self.pool:
            if n not in self.stimuli:
                raise ExperimentError(f"conditions.pool: {n!r} is not a defined stimulus")

        self.weights = None
        self.conditions = []

        if self.mode == "sample":
            if len(self.pool) < 2:
                raise ExperimentError(
                    "conditions.mode=sample needs a pool of at least 2 stimuli"
                )
            weights = cdoc.get("weights")
            if weights:
                for n in weights:
                    if n not in self.pool:
                        raise ExperimentError(
                            f"conditions.weights: {n!r} is not in the pool"
                        )
                self.weights = {n: float(weights.get(n, 1)) for n in self.pool}
            return

        # mode == "pairs"
        self.conditions = self._build_pair_conditions(cdoc)
        if not self.conditions:
            raise ExperimentError("conditions.mode=pairs produced no pairings")

    def _build_pair_conditions(self, cdoc):
        gen = cdoc.get("generate", "all_pairs")
        if gen not in ("all_pairs", "all_ordered_pairs", "none"):
            raise ExperimentError(
                f"conditions.generate {gen!r} must be all_pairs | all_ordered_pairs | none"
            )
        allow_same = bool(cdoc.get("allow_same", False))
        conds = {}  # name -> Condition (dedup, preserves insertion order)

        def add(a, b, ordered):
            conds.setdefault(Condition(a, b, ordered).name, Condition(a, b, ordered))

        if gen in ("all_pairs", "all_ordered_pairs"):
            ordered = gen == "all_ordered_pairs"
            for i, a in enumerate(self.pool):
                for j, b in enumerate(self.pool):
                    if a == b and not allow_same:
                        continue
                    if not ordered and j < i:
                        continue
                    add(a, b, ordered)

        for pair in cdoc.get("explicit") or []:
            a, b = _pair_ab(pair)
            for n in (a, b):
                if n not in self.stimuli:
                    raise ExperimentError(
                        f"conditions.explicit: {n!r} is not a defined stimulus"
                    )
            add(a, b, ordered=True)

        for pair in cdoc.get("exclude") or []:
            a, b = _pair_ab(pair)
            for key in [k for k, c in conds.items() if {c.a, c.b} == {a, b}]:
                del conds[key]

        return list(conds.values())

    # -- per-trial ------------------------------------------------------- #
    def realize(self, draw, trial_rng):
        """Turn a Scheduler _Draw (sides already decided) into a TrialPlan,
        resolving random param specs and the per-trial duration."""
        left = self.stimuli[draw.left]
        right = self.stimuli[draw.right]
        left_params = resolve_params(left.params, trial_rng)
        right_params = resolve_params(right.params, trial_rng)
        duration = float(resolve_value(self.duration_spec, trial_rng))
        return TrialPlan(
            condition_name=draw.name,
            condition_ordered=draw.ordered,
            left_name=draw.left, right_name=draw.right,
            left_type=left.type, right_type=right.type,
            left_params=left_params, right_params=right_params,
            duration_sec=duration,
        )

    def scheduler(self, rng):
        return Scheduler(self, rng)

    def summary(self):
        out = {
            "name": self.name,
            "file": self.source,
            "sha1": self.sha1,
            "n_stimuli": len(self.stimuli),
            "mode": self.mode,
            "pool": list(self.pool),
            "duration_sec": self.duration_spec,
            "circle_diameter_px": self.circle_diameter_px,
            "max_trials": self.max_trials,
            "trigger_topic": self.trigger_topic,
        }
        if self.mode == "sample":
            out["weights"] = self.weights
        else:
            out["n_conditions"] = len(self.conditions)
            out["order"] = self.order
            out["loop"] = self.loop
        return out


def _center_or_none(value, label):
    if value is None:
        return None
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return [float(value[0]), float(value[1])]
    raise ExperimentError(f"{label}: expected [x, y] or null, got {value!r}")


class Scheduler:
    """Emits one _Draw per trial. Returns None when the run is complete
    (max_trials reached, or -- pairs mode -- the set is exhausted and
    loop=False)."""

    def __init__(self, experiment, rng):
        self.exp = experiment
        self.rng = rng
        self._queue = []
        self._passes = 0
        self._emitted = 0

    def next_trial(self):
        if self.exp.max_trials is not None and self._emitted >= self.exp.max_trials:
            return None

        if self.exp.mode == "sample":
            draw = self._sample_draw()
        else:
            draw = self._pairs_draw()
            if draw is None:
                return None

        self._emitted += 1
        return draw

    # -- sample mode -- #
    def _sample_draw(self):
        pool = self.exp.pool
        weights = self.exp.weights
        if weights:
            wl = [weights[n] for n in pool]
            first = self.rng.choices(pool, weights=wl, k=1)[0]
            rest = [n for n in pool if n != first]
            second = self.rng.choices(
                rest, weights=[weights[n] for n in rest], k=1
            )[0]
            right, left = first, second
        else:
            right, left = self.rng.sample(pool, 2)   # 2 distinct, uniform
        name = "|".join(sorted((left, right)))
        return _Draw(left=left, right=right, name=name, ordered=False)

    # -- pairs mode -- #
    def _refill(self):
        conds = list(self.exp.conditions)
        if self.exp.order == "shuffle" and (
            self._passes == 0 or self.exp.reshuffle_each_loop
        ):
            self.rng.shuffle(conds)
        self._queue = conds
        self._passes += 1

    def _pairs_draw(self):
        if self.exp.order == "random":
            cond = self.rng.choice(self.exp.conditions)
        else:
            if not self._queue:
                if self._passes >= 1 and not self.exp.loop:
                    return None
                self._refill()
            cond = self._queue.pop(0)

        if cond.ordered:
            left, right = cond.a, cond.b
        elif self.rng.random() < 0.5:
            left, right = cond.a, cond.b
        else:
            left, right = cond.b, cond.a
        return _Draw(left=left, right=right, name=cond.name, ordered=cond.ordered)

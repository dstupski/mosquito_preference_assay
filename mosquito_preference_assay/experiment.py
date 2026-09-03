"""Load and interpret an experiment definition.

An experiment is **one trial**. The YAML has:

  stimuli:     named, reusable stimulus specs (type + params; params may be
               literals or random specs -- see param_spec.py)
  conditions:  how the trial's {left, right} pair is chosen --
                 mode: sample  (default)  draw TWO DISTINCT stimuli from
                               `pool` at random (no replacement); first drawn
                               -> right, second -> left. `weights` biases it.
                 mode: pairs              pick one entry from `pairs` at random;
                               {a, b} randomises the sides, {left, right} fixes
                               them.
  duration_sec: trial length (a number, or a {uniform: [...]} spec)
  display:     circle diameter, marker centres, background
  trigger:     optional -- {topic: ...} or {node: ...}; its presence makes the
               node open ARMED and wait for a std_msgs/Bool before running.

Randomness is split so the run replays from the master seed and the trial's
visuals replay from its trial seed:

  * Experiment.draw() consumes the *master* RNG (which stimuli, which sides)
  * Experiment.realize() consumes the *trial* RNG (resolving random param
    specs and the per-trial duration)
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

# The two stimulus names with sides already decided, plus a grouping key.
_Draw = namedtuple("_Draw", "left right name ordered")


class StimulusSpec:
    __slots__ = ("name", "type", "params")

    def __init__(self, name, type_, params):
        self.name = name
        self.type = type_
        self.params = params  # unresolved


class Condition:
    """A {left, right} pairing (pairs mode).

    ordered=True  -> ``a`` is LEFT, ``b`` is RIGHT; name "a->b".
    ordered=False -> unordered; sides randomised per trial; name is the two
                     members sorted and joined with "|".
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
    """Return (a, b, ordered) for a `pairs:` entry: {left,right} fixes the
    sides, {a,b} (or [a,b]) randomises them."""
    if isinstance(pair, (list, tuple)) and len(pair) == 2:
        return pair[0], pair[1], False
    if isinstance(pair, dict):
        if "left" in pair and "right" in pair:
            return pair["left"], pair["right"], True
        if "a" in pair and "b" in pair:
            return pair["a"], pair["b"], False
    raise ExperimentError(
        f"bad pairs entry {pair!r}; use {{a: x, b: y}} or {{left: x, right: y}}"
    )


# Built-in default: two distinct markers drawn at random from the four classics.
DEFAULT_DOC = {
    "schema": EXPERIMENT_SCHEMA,
    "name": "two_choice_default",
    "description": "Two distinct markers drawn at random from the pool.",
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

        # A number, or a {uniform: [...]} / {choice: [...]} spec.
        self.duration_spec = doc.get("duration_sec", 15.0)
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
        #   trigger: {topic: /arena/mosquito_present}
        #   trigger: {node: arena}     # -> /arena/trigger
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
        self.pairs = []

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
        raw_pairs = cdoc.get("pairs") or []
        if not raw_pairs:
            raise ExperimentError("conditions.mode=pairs needs a non-empty `pairs` list")
        for entry in raw_pairs:
            a, b, ordered = _pair_ab(entry)
            for n in (a, b):
                if n not in self.stimuli:
                    raise ExperimentError(
                        f"conditions.pairs: {n!r} is not a defined stimulus"
                    )
            self.pairs.append(Condition(a, b, ordered))

    # -- the trial ----------------------------------------------------- #
    def draw(self, rng):
        """Choose the trial's two stimuli and their sides, using the master
        RNG. Returns a _Draw."""
        if self.mode == "sample":
            pool, weights = self.pool, self.weights
            if weights:
                first = rng.choices(pool, weights=[weights[n] for n in pool], k=1)[0]
                rest = [n for n in pool if n != first]
                second = rng.choices(
                    rest, weights=[weights[n] for n in rest], k=1
                )[0]
                right, left = first, second
            else:
                right, left = rng.sample(pool, 2)     # 2 distinct, uniform
            return _Draw(left=left, right=right,
                         name="|".join(sorted((left, right))), ordered=False)

        cond = rng.choice(self.pairs)
        if cond.ordered:
            left, right = cond.a, cond.b
        elif rng.random() < 0.5:
            left, right = cond.a, cond.b
        else:
            left, right = cond.b, cond.a
        return _Draw(left=left, right=right, name=cond.name, ordered=cond.ordered)

    def realize(self, draw, trial_rng):
        """Turn a _Draw (sides already decided) into a concrete TrialPlan,
        resolving random param specs and the trial duration."""
        left = self.stimuli[draw.left]
        right = self.stimuli[draw.right]
        return TrialPlan(
            condition_name=draw.name,
            condition_ordered=draw.ordered,
            left_name=draw.left, right_name=draw.right,
            left_type=left.type, right_type=right.type,
            left_params=resolve_params(left.params, trial_rng),
            right_params=resolve_params(right.params, trial_rng),
            duration_sec=float(resolve_value(self.duration_spec, trial_rng)),
        )

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
            "trigger_topic": self.trigger_topic,
        }
        if self.mode == "sample":
            out["weights"] = self.weights
        else:
            out["pairs"] = [c.name for c in self.pairs]
        return out


def _center_or_none(value, label):
    if value is None:
        return None
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return [float(value[0]), float(value[1])]
    raise ExperimentError(f"{label}: expected [x, y] or null, got {value!r}")

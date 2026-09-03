"""Experiment parsing / conditions / scheduler.

Importing experiment.py pulls in py5 (via stimulus_types -> stimuli), which
needs a Java 17 JVM; skipped cleanly if that isn't available.
"""

import random

import pytest

pytest.importorskip("yaml")
try:
    from mosquito_preference_assay.experiment import Experiment, ExperimentError
except Exception as exc:  # pragma: no cover - CI without py5/JDK
    pytest.skip(f"experiment.py unavailable ({exc})", allow_module_level=True)


def _doc(**over):
    doc = {
        "schema": "mosquito_preference_assay/experiment/1",
        "name": "t",
        "stimuli": {
            "a": {"type": "static_dark", "params": {"fill_gray": 20}},
            "b": {"type": "telescope", "params": {"speed_px_per_sec": 50}},
            "c": {"type": "jitter"},
        },
        "conditions": {"mode": "sample"},
        "duration_sec": 5.0,
    }
    doc.update(over)
    return doc


def test_default_experiment_is_sample_mode():
    e = Experiment.default()
    assert e.mode == "sample"
    assert len(e.pool) == 4


def test_sample_draw_is_two_distinct_from_pool():
    e = Experiment(_doc())
    s = e.scheduler(random.Random(0))
    for _ in range(200):
        d = s.next_trial()
        assert d.left != d.right
        assert {d.left, d.right} <= set(e.pool)
        assert d.ordered is False


def _draw_sequence(doc, seed, n):
    s = Experiment(doc).scheduler(random.Random(seed))
    out = []
    for _ in range(n):
        d = s.next_trial()
        out.append((d.left, d.right))
    return out


def test_sample_reproducible_from_seed():
    assert _draw_sequence(_doc(), 9, 20) == _draw_sequence(_doc(), 9, 20)


def test_max_trials_ends_the_run():
    e = Experiment(_doc(schedule={"max_trials": 3}))
    s = e.scheduler(random.Random(1))
    got = [s.next_trial() for _ in range(5)]
    assert [g is not None for g in got] == [True, True, True, False, False]


def test_sample_needs_two_in_pool():
    with pytest.raises(ExperimentError):
        Experiment(_doc(conditions={"mode": "sample", "pool": ["a"]}))


def test_pairs_all_pairs_count():
    e = Experiment(_doc(conditions={"mode": "pairs", "generate": "all_pairs"}))
    assert e.mode == "pairs"
    assert len(e.conditions) == 3   # C(3,2)


def test_pairs_explicit_only_and_finite():
    e = Experiment(_doc(
        conditions={"mode": "pairs", "generate": "none",
                    "explicit": [{"left": "a", "right": "b"}]},
        schedule={"order": "sequential", "loop": False},
    ))
    s = e.scheduler(random.Random(0))
    d = s.next_trial()
    assert (d.left, d.right, d.ordered) == ("a", "b", True)
    assert s.next_trial() is None   # one pair, no loop


def test_unknown_type_rejected():
    with pytest.raises(ExperimentError):
        Experiment(_doc(stimuli={"a": {"type": "nope"}}))


def test_unknown_param_rejected():
    with pytest.raises(ExperimentError):
        Experiment(_doc(stimuli={"a": {"type": "static_dark", "params": {"bogus": 1}},
                                 "b": {"type": "telescope"}}))


def test_condition_references_undefined_stimulus():
    with pytest.raises(ExperimentError):
        Experiment(_doc(conditions={"mode": "pairs", "generate": "none",
                                    "explicit": [{"left": "a", "right": "zzz"}]}))


def test_trigger_block_absent_by_default():
    assert Experiment(_doc()).trigger_topic is None


def test_trigger_topic_and_node_shorthand():
    assert Experiment(_doc(trigger={"topic": "/arena/go"})).trigger_topic == "/arena/go"
    assert Experiment(_doc(trigger={"node": "arena"})).trigger_topic == "/arena/trigger"


def test_trigger_block_needs_topic_or_node():
    with pytest.raises(ExperimentError):
        Experiment(_doc(trigger={}))


def test_trigger_defaults_max_trials_to_one():
    e = Experiment(_doc(trigger={"topic": "/go"}))
    assert e.max_trials == 1
    s = e.scheduler(random.Random(0))
    assert s.next_trial() is not None
    assert s.next_trial() is None            # exactly one trial


def test_trigger_max_trials_explicit_wins():
    e = Experiment(_doc(trigger={"topic": "/go"}, schedule={"max_trials": 4}))
    assert e.max_trials == 4

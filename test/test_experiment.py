"""Experiment parsing / conditions / draw().

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


def _draws(doc, seed, n):
    e = Experiment(doc)
    rng = random.Random(seed)
    return [tuple(d[:3]) for d in (e.draw(rng) for _ in range(n))]  # (left,right,name)


def test_default_experiment_is_sample_mode():
    e = Experiment.default()
    assert e.mode == "sample"
    assert len(e.pool) == 4


def test_sample_draw_is_two_distinct_from_pool():
    e = Experiment(_doc())
    rng = random.Random(0)
    for _ in range(200):
        d = e.draw(rng)
        assert d.left != d.right
        assert {d.left, d.right} <= set(e.pool)
        assert d.ordered is False


def test_sample_reproducible_from_seed():
    assert _draws(_doc(), 9, 20) == _draws(_doc(), 9, 20)


def test_sample_needs_two_in_pool():
    with pytest.raises(ExperimentError):
        Experiment(_doc(conditions={"mode": "sample", "pool": ["a"]}))


def test_weights_bias_the_draw():
    e = Experiment(_doc(conditions={"mode": "sample", "weights": {"a": 20, "b": 1, "c": 1}}))
    rng = random.Random(0)
    seen = [n for _ in range(2000) for n in e.draw(rng)[:2]]
    assert seen.count("a") > seen.count("b") * 1.3
    assert seen.count("a") > seen.count("c") * 1.3


def test_pairs_mode_picks_one_pairing():
    e = Experiment(_doc(conditions={
        "mode": "pairs",
        "pairs": [{"a": "a", "b": "b"}, {"left": "a", "right": "c"}],
    }))
    assert e.mode == "pairs"
    assert set(e.summary()["pairs"]) == {"a|b", "a->c"}
    rng = random.Random(1)
    for _ in range(50):
        d = e.draw(rng)
        assert {d.left, d.right} in ({"a", "b"}, {"a", "c"})


def test_pairs_fixed_sides():
    fixed = Experiment(_doc(conditions={"mode": "pairs", "pairs": [{"left": "a", "right": "b"}]}))
    rng = random.Random(0)
    for _ in range(20):
        d = fixed.draw(rng)
        assert (d.left, d.right) == ("a", "b")


def test_pairs_mode_needs_pairs_list():
    with pytest.raises(ExperimentError):
        Experiment(_doc(conditions={"mode": "pairs"}))


def test_unknown_type_rejected():
    with pytest.raises(ExperimentError):
        Experiment(_doc(stimuli={"a": {"type": "nope"}}))


def test_unknown_param_rejected():
    with pytest.raises(ExperimentError):
        Experiment(_doc(stimuli={"a": {"type": "static_dark", "params": {"bogus": 1}},
                                 "b": {"type": "telescope"}}))


def test_pair_references_undefined_stimulus():
    with pytest.raises(ExperimentError):
        Experiment(_doc(conditions={"mode": "pairs", "pairs": [{"a": "a", "b": "zzz"}]}))


def test_trigger_block_absent_by_default():
    assert Experiment(_doc()).trigger_topic is None


def test_trigger_topic_and_node_shorthand():
    assert Experiment(_doc(trigger={"topic": "/arena/go"})).trigger_topic == "/arena/go"
    assert Experiment(_doc(trigger={"node": "arena"})).trigger_topic == "/arena/trigger"


def test_trigger_msg_type_defaults_bool_and_can_be_string():
    assert Experiment(_doc(trigger={"topic": "/go"})).trigger_msg_type == "bool"
    e = Experiment(_doc(trigger={"topic": "/go", "msg_type": "string"}))
    assert e.trigger_msg_type == "string"
    assert e.summary()["trigger_msg_type"] == "string"


def test_trigger_msg_type_rejects_bad_value():
    with pytest.raises(ExperimentError):
        Experiment(_doc(trigger={"topic": "/go", "msg_type": "int32"}))


def test_trigger_block_needs_topic_or_node():
    with pytest.raises(ExperimentError):
        Experiment(_doc(trigger={}))


def test_realize_resolves_random_params_and_duration():
    e = Experiment(_doc(
        stimuli={"a": {"type": "moving_grating",
                       "params": {"speed_px_per_sec": {"uniform": [10, 20]}}},
                 "b": {"type": "static_dark"}},
        duration_sec={"uniform": [3, 4]},
    ))
    plan = e.realize(e.draw(random.Random(0)), random.Random(0))
    assert 3.0 <= plan.duration_sec <= 4.0
    params = plan.left_params if plan.left_name == "a" else plan.right_params
    assert 10.0 <= params["speed_px_per_sec"] <= 20.0

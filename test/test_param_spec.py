"""param_spec has no py5 dependency, so this runs anywhere."""

import random

import pytest

from mosquito_preference_assay import param_spec


def test_literal_passes_through():
    rng = random.Random(0)
    assert param_spec.resolve_value(24, rng) == 24
    assert param_spec.resolve_value([1, 2], rng) == [1, 2]
    assert param_spec.resolve_value("red", rng) == "red"


def test_uniform_in_range_and_reproducible():
    a = param_spec.resolve_value({"uniform": [10, 20]}, random.Random(42))
    b = param_spec.resolve_value({"uniform": [10, 20]}, random.Random(42))
    assert a == b
    assert 10 <= a < 20


def test_randint_inclusive():
    vals = {param_spec.resolve_value({"randint": [1, 3]}, random.Random(i)) for i in range(50)}
    assert vals <= {1, 2, 3}


def test_choice():
    v = param_spec.resolve_value({"choice": ["a", "b", "c"]}, random.Random(1))
    assert v in ("a", "b", "c")


def test_resolve_params_returns_new_dict_of_concrete_values():
    src = {"speed": {"uniform": [0, 1]}, "period": 24}
    out = param_spec.resolve_params(src, random.Random(7))
    assert set(out) == {"speed", "period"}
    assert out["period"] == 24
    assert isinstance(out["speed"], float)
    assert src["speed"] == {"uniform": [0, 1]}   # unchanged


@pytest.mark.parametrize("bad", [
    {"x": {"uniform": [1]}},
    {"x": {"randint": [1, 2, 3]}},
    {"x": {"choice": []}},
    {"x": {"bogus": [1, 2]}},
])
def test_validate_rejects_malformed_specs(bad):
    with pytest.raises(ValueError):
        param_spec.validate_params(bad)


def test_validate_accepts_good_specs():
    param_spec.validate_params({
        "a": 1,
        "b": {"uniform": [0, 1]},
        "c": {"choice": [1, 2]},
        "d": {"normal": [0, 1]},
    })

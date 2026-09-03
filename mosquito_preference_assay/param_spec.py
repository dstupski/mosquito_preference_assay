"""Resolve stimulus parameters that may be literals *or* random specs.

A parameter value in an experiment YAML is either a plain value (``24``,
``[1, 2]``, ``"red"``) or a one-key dict naming a distribution, drawn from the
per-trial RNG so the resolved value is reproducible from the trial seed:

    speed_px_per_sec: {uniform: [20, 80]}     # float in [20, 80)
    n_rings:          {randint: [3, 8]}       # int in [3, 8]  (inclusive)
    angle_deg:        {choice: [0, 45, 90]}   # one of the listed values
    contrast:         {normal: [0.5, 0.1]}    # gaussian(mean, std)

``resolve_params`` returns a new dict of concrete, JSON-serializable values;
that dict is what gets recorded in the published message.
"""

_SPEC_KINDS = ("uniform", "randint", "choice", "normal")


def is_random_spec(value):
    return (
        isinstance(value, dict)
        and len(value) == 1
        and next(iter(value)) in _SPEC_KINDS
    )


def resolve_value(value, rng):
    """Resolve one value against ``rng`` (a random.Random). Non-specs pass
    through unchanged."""
    if not is_random_spec(value):
        return value

    (kind, arg), = value.items()
    if kind == "uniform":
        lo, hi = arg
        return rng.uniform(lo, hi)
    if kind == "randint":
        lo, hi = arg
        return rng.randint(lo, hi)
    if kind == "choice":
        if not arg:
            raise ValueError("{choice: []} needs a non-empty list")
        return rng.choice(list(arg))
    if kind == "normal":
        mean, std = arg
        return rng.gauss(mean, std)
    raise ValueError(f"unknown random spec {kind!r}")


def resolve_params(params, rng):
    """Resolve every value in a params dict. Returns a new dict."""
    return {key: resolve_value(val, rng) for key, val in (params or {}).items()}


def validate_params(params):
    """Raise ValueError on a malformed random spec (called at load time so
    typos surface before a run rather than mid-experiment)."""
    for key, val in (params or {}).items():
        if not isinstance(val, dict) or len(val) != 1:
            continue
        kind, arg = next(iter(val.items()))
        if kind not in _SPEC_KINDS:
            raise ValueError(
                f"param {key!r}: {kind!r} is not a random spec "
                f"(expected one of {list(_SPEC_KINDS)})"
            )
        if kind == "choice":
            if not (isinstance(arg, (list, tuple)) and arg):
                raise ValueError(f"param {key!r}: {{choice: ...}} needs a non-empty list")
        elif not (isinstance(arg, (list, tuple)) and len(arg) == 2):
            raise ValueError(f"param {key!r}: {{{kind}: ...}} needs exactly [a, b]")

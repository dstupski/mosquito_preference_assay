"""Registry of stimulus *types* (the visual behaviors defined in stimuli.py).

An experiment YAML composes named *instances* of these types; this module just
maps a ``type`` string to its class and builds one from a resolved params dict.
"""

from .stimuli import (
    JitterStimulus,
    MovingGratingStimulus,
    StaticDarkStimulus,
    TelescopeStimulus,
)

STIMULUS_TYPES = {
    "static_dark": StaticDarkStimulus,
    "jitter": JitterStimulus,
    "moving_grating": MovingGratingStimulus,
    "telescope": TelescopeStimulus,
}

# Params that are internal animation state rather than experiment knobs: if the
# YAML doesn't set them, fill from the per-trial RNG over the given range so the
# trial stays reproducible from its seed.
_RNG_FILLED = {
    "jitter": {"seed_x": (0.0, 1000.0), "seed_y": (0.0, 1000.0)},
}


def build_stimulus(type_name, diameter_px, resolved_params, rng):
    """Instantiate one stimulus. ``resolved_params`` must already be concrete
    (see param_spec.resolve_params). Raises ValueError with a helpful message
    on an unknown type or bad param name."""
    try:
        cls = STIMULUS_TYPES[type_name]
    except KeyError:
        raise ValueError(
            f"unknown stimulus type {type_name!r}; "
            f"known types: {sorted(STIMULUS_TYPES)}"
        ) from None

    params = dict(resolved_params or {})
    for name, (lo, hi) in _RNG_FILLED.get(type_name, {}).items():
        params.setdefault(name, rng.uniform(lo, hi))

    try:
        return cls(diameter_px=diameter_px, **params)
    except TypeError as exc:
        raise ValueError(f"bad params for stimulus type {type_name!r}: {exc}") from exc


def known_param_names(type_name):
    """The accepted param names for a type (for load-time validation)."""
    import inspect
    cls = STIMULUS_TYPES[type_name]
    sig = inspect.signature(cls.__init__)
    return {
        p for p in sig.parameters
        if p not in ("self", "diameter_px")
    }

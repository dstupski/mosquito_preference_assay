"""Where a launch file should look for its params.

A rig's config has to survive `git pull` untouched -- discovering during an
experiment that a pull rewrote your monitor index or circle centres is exactly
the wrong time. But the tracked `config/*.yaml` cannot simply be ignored:
git only ignores files it is not already tracking, and untracking them would
leave a fresh clone with no config at all and every launch default pointing at
a missing file.

So: a `.local.yaml` beside the shipped one wins when it exists.

    config/assay_params.yaml         tracked. The defaults, and what a clone gets.
    config/assay_params.local.yaml   gitignored. Yours. Never touched by a pull.

Nothing to set up -- a clone with no local file uses the tracked one, and the
moment you create one it takes over, with no change to any command.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

PACKAGE = "mosquito_preference_assay"


def local_variant(path):
    """`foo.yaml` -> `foo.local.yaml`."""
    base, ext = os.path.splitext(path)
    return f"{base}.local{ext}"


DEFAULT_DISPLAY_CONFIG = "display_geometry.local.yaml"
DEFAULT_TRIGGER_CONFIG = "trigger_roi.local.yaml"


def _config_dir():
    try:
        shipped = os.path.join(
            get_package_share_directory(PACKAGE), "config", "assay_params.yaml")
    except PackageNotFoundError:
        return None
    return os.path.dirname(os.path.realpath(shipped))


def resolve_display_config(spec=""):
    """The calibration file to layer over the params file, or None.

    `spec` is the `display_config` launch argument: a path to a specific
    calibration, so you can keep every dated file display_check writes and
    point at whichever one you want --

        display_config:=config/20260923_display_config.yaml

    Empty falls back to config/display_geometry.local.yaml, the file `s`
    keeps current, and None when that does not exist either (a fresh clone),
    in which case the experiment's own geometry applies.

    Launch files layer it AFTER the params file, so it wins: with multiple
    params files, later ones override earlier ones.
    """
    spec = (spec or "").strip()
    if spec:
        path = os.path.expanduser(spec)
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"display_config {spec!r} not found. Point it at a file "
                f"display_check wrote, or leave it empty to use the current "
                f"{DEFAULT_DISPLAY_CONFIG}.")
        return os.path.abspath(path)

    directory = _config_dir()
    if not directory:
        return None
    path = os.path.join(directory, DEFAULT_DISPLAY_CONFIG)
    return path if os.path.exists(path) else None


def resolve_trigger_config(spec=""):
    """The trigger-zone calibration to layer over the detector's params, or None.

    Exactly the display_config story, for the camera instead of the projector:
    `trigger_roi` writes this file when you press `s`, and every launch that
    starts a detector layers it AFTER detector_params so it wins.

    It carries `image_topic` as well as `roi`, deliberately. A box in pixel
    coordinates only means something on the camera it was drawn on, so the two
    travel together -- drawing the zone on cam0 is what points detection at
    cam0. Keeping them in separate files is how you end up aligned on one
    device and running on another.
    """
    spec = (spec or "").strip()
    if spec:
        path = os.path.expanduser(spec)
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"trigger_config {spec!r} not found. Point it at a file "
                f"trigger_roi wrote, or leave it empty to use the current "
                f"{DEFAULT_TRIGGER_CONFIG}.")
        return os.path.abspath(path)

    directory = _config_dir()
    if not directory:
        return None
    path = os.path.join(directory, DEFAULT_TRIGGER_CONFIG)
    return path if os.path.exists(path) else None


def detector_setting(key, *paths, node="mosquito_detector"):
    """Read one of the detector's settings the way the DETECTOR would see it,
    whatever its params file happens to be keyed by.

    ROS matches a params file's top-level key against the node's own name, so
    a file written as `mosquito_detector:` reaches the detector and nothing
    else. Any tool that wants to SHOW what the detector is using -- arena_view
    -- is a different node, and would silently see nothing. That looks like
    the setting being ignored when it is only being delivered elsewhere.

    Later paths win, matching how the launch files layer them. Returns None if
    no file carries the key.
    """
    import yaml

    found = None
    for path in paths:
        if not path or not os.path.exists(path):
            continue
        try:
            with open(path) as handle:
                doc = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError):
            continue
        if not isinstance(doc, dict):
            continue
        # "/**" applies to every node; the node's own name applies to it; a
        # single-key file is unambiguous whatever it is called.
        candidates = ["/**", node]
        if len(doc) == 1:
            candidates.append(next(iter(doc)))
        for candidate in candidates:
            params = (doc.get(candidate) or {}).get("ros__parameters", {})
            if isinstance(params, dict) and params.get(key) not in (None, ""):
                found = params[key]
    return found


def resolve_config(name):
    """Absolute path to the params file a launch file should default to:
    the `.local.yaml` next to it if that exists, otherwise the shipped one.

    `name` is a bare filename in the package's config/ directory, e.g.
    "assay_params.yaml".
    """
    try:
        shipped = os.path.join(get_package_share_directory(PACKAGE), "config", name)
    except PackageNotFoundError:               # not built/sourced; nothing to resolve
        return name

    # Follow the symlink a --symlink-install build leaves, so the .local file
    # is looked for in the SOURCE tree where you would actually create it,
    # not in the build directory.
    real = os.path.realpath(shipped)
    for candidate in (local_variant(real), local_variant(shipped)):
        if os.path.exists(candidate):
            return candidate
    return shipped

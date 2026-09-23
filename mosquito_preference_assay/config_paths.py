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

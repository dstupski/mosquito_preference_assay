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

"""Locate this package's source checkout from its install share directory.

No rclpy import. Nodes pass in get_package_share_directory(...) and get back
the writable source directory (where params/, share/ and maps/ live), so no
node hard-codes a workspace layout like /root/skuba_ws/src/skuba_navigation —
the checkout may be nested (src/skuba-newgen/skuba-navigation) or renamed.

Resolution order:
  1. $SKUBA_NAVIGATION_SRC, if set and it exists.
  2. The target of the installed package.xml symlink (colcon --symlink-install).
  3. A search of every ancestor's src/ for the package.xml naming this package.
Returns None when none of those find it; callers pick their own fallback.
"""

import os
import re
from pathlib import Path

ENV_VAR = "SKUBA_NAVIGATION_SRC"

_SKIP_DIRS = {"build", "install", "log", ".git", "__pycache__", "node_modules"}


def _declares_package(package_xml: Path, package_name: str) -> bool:
    try:
        text = package_xml.read_text(errors="ignore")
    except OSError:
        return False
    match = re.search(r"<name>\s*([^<\s]+)\s*</name>", text)
    return bool(match) and match.group(1) == package_name


def _search_src(src_dir: Path, package_name: str):
    for root, dirs, files in os.walk(src_dir):
        if "COLCON_IGNORE" in files:
            dirs[:] = []
            continue
        if "package.xml" in files and _declares_package(
            Path(root) / "package.xml", package_name
        ):
            return Path(root)
        dirs[:] = [d for d in dirs if d not in _SKIP_DIRS and not d.startswith(".")]
    return None


def find_package_source_dir(share_dir, package_name, env=None):
    env = os.environ if env is None else env

    override = env.get(ENV_VAR)
    if override and Path(override).expanduser().is_dir():
        return Path(override).expanduser().resolve()

    share_dir = Path(share_dir)
    installed_xml = share_dir / "package.xml"
    if installed_xml.is_symlink():
        source_dir = installed_xml.resolve().parent
        if _declares_package(source_dir / "package.xml", package_name):
            return source_dir

    for ancestor in share_dir.resolve().parents:
        src_dir = ancestor / "src"
        if src_dir.is_dir():
            found = _search_src(src_dir, package_name)
            if found is not None:
                return found
    return None

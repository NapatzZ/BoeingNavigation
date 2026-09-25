"""
Locate skuba_navigation's editable data files.

saved_pose.yaml and params/costmap_dynamic_config.yaml are written at runtime,
so the nodes want the copy in the source checkout, not the installed one.
They used to rebuild that path as <ws>/src/skuba_navigation/..., which breaks
as soon as the checkout folder has another name (e.g. `skuba-navigation`, as
`git clone` names it) or the workspace is laid out differently.

Resolve relative to this module instead: with `colcon build --symlink-install`
the installed .py files are symlinks into the checkout, so the resolved module
path leads back to it whatever the folder is called. A plain install has no
checkout to go back to, so fall back to the installed share directory.
"""

from pathlib import Path

PACKAGE_NAME = 'skuba_navigation'


def source_root():
    """Return the source checkout root (the dir with package.xml), or None."""
    root = Path(__file__).resolve().parents[1]
    return root if (root / 'package.xml').is_file() else None


def data_path(*parts):
    """
    Return the path to a package data file.

    e.g. data_path('share', 'saved_pose.yaml')
    """
    root = source_root()
    if root is None:
        # Imported lazily so the module stays importable without a ROS env
        # (unit tests).
        from ament_index_python.packages import get_package_share_directory
        root = Path(get_package_share_directory(PACKAGE_NAME))
    return root.joinpath(*parts)

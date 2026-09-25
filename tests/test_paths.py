"""paths.data_path() must find the checkout's files whatever it is named."""
import os
import shutil
import subprocess
import sys
from pathlib import Path

from skuba_navigation import paths

REPO = Path(__file__).resolve().parents[1]


def test_source_root_is_the_checkout():
    assert paths.source_root() == REPO


def test_data_path_points_into_the_checkout():
    expected = REPO / 'share' / 'saved_pose.yaml'
    assert paths.data_path('share', 'saved_pose.yaml') == expected
    assert paths.data_path('params', 'costmap_dynamic_config.yaml').is_file()


def test_folder_name_does_not_matter(tmp_path):
    # Copy the package into a folder with a different name, e.g. what
    # `git clone` produces, and resolve from there.
    other = tmp_path / 'skuba-navigation-renamed'
    shutil.copytree(REPO / 'skuba_navigation', other / 'skuba_navigation')
    shutil.copy(REPO / 'package.xml', other / 'package.xml')
    out = subprocess.check_output(
        [sys.executable, '-c',
         'from skuba_navigation import paths; '
         'print(paths.data_path("share", "saved_pose.yaml"))'],
        cwd=other, env={**os.environ, 'PYTHONPATH': str(other)}, text=True)
    assert Path(out.strip()) == other / 'share' / 'saved_pose.yaml'

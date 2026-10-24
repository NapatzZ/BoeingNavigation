"""Saved maps on disk (maps/<category>/<name>.{yaml,pgm}) for tests."""

import yaml

from skuba_navigation.core import map_editor as ed


def make_map(root, map_id="30_home/home", width=4, height=3, origin=(-1.0, -2.0, 0.0)):
    folder = (root / map_id).parent
    folder.mkdir(parents=True, exist_ok=True)
    name = (root / map_id).name
    pixels = bytearray(range(width * height))
    ed.write_pgm(folder / f"{name}.pgm", width, height, pixels)
    (folder / f"{name}.yaml").write_text(
        yaml.safe_dump(
            {
                "image": f"{name}.pgm",
                "mode": "trinary",
                "resolution": 0.05,
                "origin": list(origin),
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            }
        )
    )
    return pixels

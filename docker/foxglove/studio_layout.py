"""Turn a foxglove-sdk ``Layout`` into the JSON Foxglove Studio imports.

``Layout.to_json()`` emits ``{"version": 1, "content": ...}``, which Foxglove
Studio's **Layouts → Import from file** rejects ("This file is not recognized
as a Foxglove layout"). Studio imports the shape it exports itself: one
``configById`` entry per panel plus a binary ``layout`` mosaic tree, with tabs
as ``Tab!`` panels holding their own trees. This converts one into the other.
"""

import itertools
import json

# foxglove-sdk panelType -> Foxglove Studio panel id prefix.
PANEL_IDS = {"ThreeDee": "3D", "Log": "RosOut"}


def to_studio_json(layout):
    """Return ``layout`` as Foxglove Studio layout JSON (a string)."""
    config_by_id = {}
    counter = itertools.count(1)

    def panel_id(kind):
        return f"{kind}!skuba{next(counter)}"

    def mosaic(content):
        kind = content["type"]
        if kind == "panel":
            pid = panel_id(PANEL_IDS.get(content["panelType"], content["panelType"]))
            config = dict(content.get("config") or {})
            if content.get("title"):
                config["foxglovePanelTitle"] = content["title"]
            config_by_id[pid] = config
            return pid
        if kind == "tabs":
            pid = panel_id("Tab")
            config_by_id[pid] = {
                "activeTabIdx": content.get("selectedTabIndex", 0),
                "tabs": [
                    {"title": tab["title"], "layout": mosaic(tab["content"])}
                    for tab in content["tabs"]
                ],
            }
            return pid
        if kind == "split":
            return split(content["direction"], content["items"])
        raise ValueError(f"unknown layout node {kind!r}")

    def split(direction, items):
        # Studio's mosaic is binary: [a, b, c] -> a | (b | c).
        if len(items) == 1:
            return mosaic(items[0]["content"])
        total = sum(item.get("proportion", 1) for item in items)
        first = items[0].get("proportion", 1)
        return {
            "direction": direction,
            "first": mosaic(items[0]["content"]),
            "second": split(direction, items[1:]),
            "splitPercentage": round(100 * first / total, 2),
        }

    content = json.loads(layout.to_json())["content"]
    tree = mosaic(content)
    return json.dumps(
        {
            "configById": config_by_id,
            "globalVariables": {},
            "userNodes": {},
            "playbackConfig": {"speed": 1},
            "layout": tree,
        },
        indent=2,
    )

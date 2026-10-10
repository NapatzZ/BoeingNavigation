"""Regenerate skuba_task.json: one Foxglove layout to watch a whole task.

Every SKUBA container serves the same ws://<robot>:8765 (see the entrypoints),
so this one layout covers navigation, vision, manipulation, state and speech::

    pip install foxglove-sdk
    python docker/foxglove/generate_task_layout.py > docker/foxglove/skuba_task.json
"""

import foxglove.layouts as L

from generate_layout import cmd_vel, door, e_stop, log, three_d

# Navigation's 3D view, plus what vision sees placed in the map.
three_d.title = "Robot"
three_d.config.topics.update(
    {
        "/skuba/vision/object_footprints": L.BaseRendererRosMarkerTopicSettings(
            visible=True
        ),
        "/skuba/vision/trash_markers": L.BaseRendererRosMarkerTopicSettings(
            visible=True
        ),
    }
)


def image(title, topic):
    return L.ImagePanel(
        title=title,
        config=L.ImageConfig(image_mode=L.ImageModeConfig(image_topic=topic)),
    )


def raw(title, path):
    return L.RawMessagesPanel(title=title, config=L.RawMessagesConfig(topic_path=path))


camera = image("Camera", "/camera/camera/color/image_raw")

# Each vision node's debug image only publishes while that node's goal runs.
vision_debug = L.TabContainer(
    tabs=[
        L.TabItem(
            title="objects",
            content=image("objects", "/skuba/vision/object_debug_image"),
        ),
        L.TabItem(
            title="follow", content=image("follow", "/skuba/vision/follow_visual")
        ),
        L.TabItem(
            title="trash", content=image("trash", "/skuba/vision/trash_debug_image")
        ),
        L.TabItem(
            title="count", content=image("count", "/skuba/vision/count_debug_image")
        ),
    ]
)

status = L.SplitContainer(
    direction="column",
    items=[
        L.SplitItem(
            proportion=0.15,
            content=L.SplitContainer(
                direction="row",
                items=[
                    L.SplitItem(content=e_stop),
                    L.SplitItem(content=door),
                ],
            ),
        ),
        L.SplitItem(
            proportion=0.15,
            content=raw("Picking (manipulation)", "/skuba/manipulation/picking.data"),
        ),
        L.SplitItem(
            proportion=0.15,
            content=raw("Follow state (vision)", "/skuba/vision/follow_state.data"),
        ),
        # skuba_state, speak_node and listen_node report through /rosout.
        L.SplitItem(
            proportion=0.55,
            content=L.TabContainer(
                tabs=[
                    L.TabItem(title="rosout", content=log),
                    L.TabItem(title="cmd_vel", content=cmd_vel),
                    L.TabItem(
                        title="topics", content=L.TopicGraphPanel(title="Topic graph")
                    ),
                ]
            ),
        ),
    ],
)

layout = L.Layout(
    content=L.SplitContainer(
        direction="row",
        items=[
            L.SplitItem(proportion=0.5, content=three_d),
            L.SplitItem(
                proportion=0.27,
                content=L.SplitContainer(
                    direction="column",
                    items=[
                        L.SplitItem(proportion=0.5, content=camera),
                        L.SplitItem(proportion=0.5, content=vision_debug),
                    ],
                ),
            ),
            L.SplitItem(proportion=0.23, content=status),
        ],
    )
)

if __name__ == "__main__":
    from studio_layout import to_studio_json

    print(to_studio_json(layout))

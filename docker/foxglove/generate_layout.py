"""Regenerate skuba_navigation.json (the Foxglove Studio layout).

The layout JSON is an internal Foxglove format not meant for hand editing, so
change this script and re-run it instead::

    pip install foxglove-sdk
    python docker/foxglove/generate_layout.py > docker/foxglove/skuba_navigation.json
"""

import foxglove.layouts as L

# Frames and topics as configured in params/rcj_param.yaml + params/twist_mux.yaml.
three_d = L.ThreeDeePanel(
    title="Map",
    config=L.ThreeDeeConfig(
        fixed_frame="map",
        follow_tf="base_link",
        follow_mode="follow-pose",
        topics={
            "/robot_description": L.BaseRendererUrdfTopicSettings(visible=True),
            "/map": L.BaseRendererRosOccupancyGridTopicSettings(
                visible=True, color_mode="map", draw_behind=True
            ),
            "/global_costmap/costmap": L.BaseRendererRosOccupancyGridTopicSettings(
                visible=True, color_mode="costmap", alpha=0.4
            ),
            "/local_costmap/costmap": L.BaseRendererRosOccupancyGridTopicSettings(
                visible=True, color_mode="costmap", alpha=0.6
            ),
            "/local_costmap/published_footprint": L.BaseRendererRosPolygonTopicSettings(
                visible=True, color="#00ff00"
            ),
            "/scan": L.BaseRendererLaserScanTopicSettings(
                visible=True, point_size=3, color_mode="flat", flat_color="#ff3030"
            ),
            "/scan_filtered": L.BaseRendererLaserScanTopicSettings(
                visible=False, point_size=3, color_mode="flat", flat_color="#ffaa00"
            ),
            "/plan": L.BaseRendererPosesTopicSettings(
                visible=True,
                type="line",
                line_width=0.04,
                gradient=("#00b4ff", "#00b4ff"),
            ),
            "/local_plan": L.BaseRendererPosesTopicSettings(
                visible=True,
                type="line",
                line_width=0.04,
                gradient=("#ff00ff", "#ff00ff"),
            ),
            "/amcl_pose": L.BaseRendererPoseTopicSettings(
                visible=True, type="arrow", show_covariance=True
            ),
        },
        # The 3D panel's publish button: "Pose estimate" sets AMCL's initial
        # pose, "Pose" sends a Nav2 goal (bt_navigator listens on /goal_pose).
        publish=L.ThreeDeePublishSettings(
            type="pose_estimate",
            pose_estimate_topic="/initialpose",
            pose_topic="/goal_pose",
            point_topic="/clicked_point",
        ),
    ),
)

# twist_mux lock: emergency_stop latches /e_stop true and the base stops.
e_stop = L.IndicatorPanel(
    title="E-stop",
    config=L.IndicatorConfig(
        path="/e_stop.data",
        style="background",
        rules=[
            L.IndicatorRule(
                operator="=", raw_value="true", color="#d32f2f", label="E-STOP"
            )
        ],
        fallback_color="#2e7d32",
        fallback_label="clear",
    ),
)

door = L.IndicatorPanel(
    title="Door",
    config=L.IndicatorConfig(
        path="/skuba/nav/door_status.data",
        style="bulb",
        rules=[
            L.IndicatorRule(
                operator="=", raw_value="true", color="#2e7d32", label="open"
            )
        ],
        fallback_color="#9e9e9e",
        fallback_label="closed / off",
    ),
)

# cmd_vel_key is twist_mux's keyboard lane (priority 90): e_stop still wins.
teleop = L.TeleopPanel(
    title="Teleop",
    config=L.TeleopConfig(topic="/cmd_vel_key", publish_rate=10),
)

cmd_vel = L.PlotPanel(
    title="cmd_vel",
    config=L.PlotConfig(
        paths=[
            L.PlotSeries(value="/cmd_vel.linear.x", label="linear.x", enabled=True),
            L.PlotSeries(value="/cmd_vel.angular.z", label="angular.z", enabled=True),
            L.PlotSeries(
                value="/cmd_vel_nav.linear.x", label="nav linear.x", enabled=True
            ),
        ]
    ),
)

log = L.LogPanel(
    title="rosout",
    config=L.LogConfig(show_level=True, show_time=True, show_name=True),
)

side = L.SplitContainer(
    direction="column",
    items=[
        L.SplitItem(
            proportion=0.12,
            content=L.SplitContainer(
                direction="row",
                items=[
                    L.SplitItem(content=e_stop),
                    L.SplitItem(content=door),
                ],
            ),
        ),
        L.SplitItem(proportion=0.3, content=teleop),
        L.SplitItem(
            proportion=0.58,
            content=L.TabContainer(
                tabs=[
                    L.TabItem(title="rosout", content=log),
                    L.TabItem(title="cmd_vel", content=cmd_vel),
                ]
            ),
        ),
    ],
)

layout = L.Layout(
    content=L.SplitContainer(
        direction="row",
        items=[
            L.SplitItem(proportion=0.7, content=three_d),
            L.SplitItem(proportion=0.3, content=side),
        ],
    )
)

if __name__ == "__main__":
    print(layout.to_json())

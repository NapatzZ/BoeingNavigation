"""The running map's poses file: the map map_server is serving, else an override."""

from pathlib import Path
from typing import Callable, Optional

from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ..core.navigation.location_store import poses_path_for_map, source_map_path
from ..core.navigation.map_identity import MapShape, find_running_map
from .futures import wait_for_future

_SERVICE = "/map_server/get_parameters"
_TIMEOUT = 1.0  # s, for the service to appear and for it to answer


class RunningMap:
    def __init__(
        self,
        node: Node,
        callback_group: CallbackGroup,
        source_dir: Optional[Path],
        override: str = "",
        wait: Optional[Callable] = None,
    ):
        """
        `override` names the map yaml instead of asking map_server.

        `wait(future, timeout) -> bool` is for a node whose executor cannot run
        the reply while this blocks; the default suits a multi-threaded one.
        """
        self._source_dir = source_dir
        self._override = override
        self._wait = wait or (
            lambda future, t: wait_for_future(future, lambda: False, t)
        )
        self._client = node.create_client(
            GetParameters, _SERVICE, callback_group=callback_group
        )
        self._live: Optional[MapShape] = None
        self._found = None  # ((live, reported), map yaml): the last check
        # yaml_filename only changes when map_server restarts, and a restarted
        # map_server publishes /map again: ask once, then again after each /map.
        self._reported: Optional[Path] = None
        self._ask = True
        # map_server publishes /map latched, so the current map arrives at once.
        node.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
            callback_group=callback_group,
        )

    def poses_path(self) -> Optional[Path]:
        """The map's poses file; None when no map is loaded or it is unclear which."""
        map_yaml = self._override or self._running_yaml()
        if not map_yaml:
            return None
        return poses_path_for_map(source_map_path(map_yaml, self._source_dir))

    def _running_yaml(self) -> Optional[Path]:
        reported = self._map_server_yaml()
        live = self._live
        if reported is None or live is None:
            return reported
        key = (live, reported)
        if self._found is None or self._found[0] != key:
            maps_root = self._source_dir / "maps" if self._source_dir else None
            self._found = (key, find_running_map(live, reported, maps_root))
        return self._found[1]

    def _on_map(self, grid: OccupancyGrid) -> None:
        self._ask = True
        info = grid.info
        self._live = MapShape(
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
        )

    def _map_server_yaml(self) -> Optional[Path]:
        if self._ask or self._reported is None or not self._client.service_is_ready():
            self._ask = False  # before asking: a /map arriving meanwhile asks again
            self._reported = self._ask_map_server()
        return self._reported

    def _ask_map_server(self) -> Optional[Path]:
        if not self._client.wait_for_service(timeout_sec=_TIMEOUT):
            return None
        future = self._client.call_async(GetParameters.Request(names=["yaml_filename"]))
        if not self._wait(future, _TIMEOUT):
            return None
        values = future.result().values
        return (
            Path(values[0].string_value) if values and values[0].string_value else None
        )

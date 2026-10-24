"""The running map's poses file: what Nav2's map_server loaded, else an override."""

from pathlib import Path
from typing import Callable, Optional

from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from ..core.navigation.location_store import poses_path_for_map, source_map_path
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

    def poses_path(self) -> Optional[Path]:
        """The map's poses file; None when no map is loaded."""
        map_yaml = self._override or self._map_server_yaml()
        if not map_yaml:
            return None
        return poses_path_for_map(source_map_path(map_yaml, self._source_dir))

    def _map_server_yaml(self) -> Optional[Path]:
        if not self._client.wait_for_service(timeout_sec=_TIMEOUT):
            return None
        future = self._client.call_async(GetParameters.Request(names=["yaml_filename"]))
        if not self._wait(future, _TIMEOUT):
            return None
        values = future.result().values
        return (
            Path(values[0].string_value) if values and values[0].string_value else None
        )

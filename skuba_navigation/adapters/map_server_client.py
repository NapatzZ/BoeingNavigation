"""Which map Nav2's map_server is serving, asked on its parameter service."""

import threading
from pathlib import Path
from typing import Callable, Optional

from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node


class MapServerClient:
    def __init__(
        self,
        node: Node,
        callback_group: CallbackGroup,
        service: str,
        timeout: float,
        wait: Optional[Callable] = None,
    ):
        """`wait(future, timeout) -> bool` for a node whose executor cannot run
        the reply while this blocks; the default suits a multi-threaded one."""
        self._timeout = timeout
        self._wait = wait or _wait_for_thread
        self._client = node.create_client(
            GetParameters, service, callback_group=callback_group
        )

    def map_yaml(self) -> Optional[Path]:
        """The loaded map's yaml path; None when map_server is down or serves none."""
        if not self._client.wait_for_service(timeout_sec=self._timeout):
            return None
        future = self._client.call_async(GetParameters.Request(names=["yaml_filename"]))
        if not self._wait(future, self._timeout):
            return None
        values = future.result().values
        path = values[0].string_value if values else ""
        return Path(path) if path else None


def _wait_for_thread(future, timeout: float) -> bool:
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout)

#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool
from kobuki_ros_interfaces.msg import SensorState
from ..core.estop_level import EstopLevel

HEARTBEAT_S = 0.1  # 10 Hz


class EmergencyStopNode(Node):
    """
    Publish the live e-stop button level from /sensors/core.

    Reads digital_input bit 0 of /sensors/core (kobuki_ros_interfaces/SensorState,
    about 50 Hz). Set means the e-stop is pressed. Unlike the old event latch, the
    first reading is the real button state, so a button already held at start-up
    locks the base, and a released button unlocks it again without a restart.

    The level goes out on two topics, always with the same value:
      - the twist_mux e_stop LOCK (std_msgs/Bool). While it is True, twist_mux
        blocks every velocity input and the base receives no command (see
        params/twist_mux.yaml).
      - /skuba/nav/e_stop, for everything that is not twist_mux: skuba-state
        reads it before a run (waits for the release) and during a run (aborts).

    A change is published at once. In between, the level is repeated at 10 Hz as
    a heartbeat, so a late subscriber gets it and a reader can tell a released
    button from a dead node. Nothing is published before the first reading, or
    while /sensors/core has been silent for 1 s (the level is unknown).

    Parameters
    ----------
    lock_topic: str - twist_mux lock topic to raise. Default 'e_stop'.
    state_topic: str - the state for other packages. Default '/skuba/nav/e_stop'.

    """

    def __init__(self):
        super().__init__("emergency_stop")

        self.declare_parameter("lock_topic", "e_stop")
        lock_topic: str = self.get_parameter("lock_topic").value
        self.declare_parameter("state_topic", "/skuba/nav/e_stop")
        state_topic: str = self.get_parameter("state_topic").value

        self._est = EstopLevel()
        self._known = False

        # Transient-local so a late subscriber (twist_mux, skuba-state) still gets
        # the current level.
        lock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self._lock_pub = self.create_publisher(Bool, lock_topic, lock_qos)
        self._state_pub = self.create_publisher(Bool, state_topic, lock_qos)

        self._sub = self.create_subscription(
            SensorState, "/sensors/core", self._sensors_cb, qos_profile_sensor_data
        )
        self._timer = self.create_timer(HEARTBEAT_S, self._heartbeat)

        self.get_logger().info(
            f"[EmergencyStop] Ready. Reading /sensors/core digital_input bit 0. "
            f'Publishing the live level on "{lock_topic}" and "{state_topic}".'
        )

    # ------------------------------------------------------------------
    def _sensors_cb(self, msg: SensorState) -> None:
        if self._est.hear(msg.digital_input, time.monotonic()):
            self._publish_now()  # a change goes out at once, not at the next beat

    # ------------------------------------------------------------------
    def _heartbeat(self) -> None:
        level = self._est.level(time.monotonic())
        if level is None:
            if self._known:
                self._known = False
                self.get_logger().error(
                    "[EmergencyStop] /sensors/core is silent: e-stop level unknown."
                )
            return
        self._known = True
        self._publish(level)

    # ------------------------------------------------------------------
    def _publish_now(self) -> None:
        level = self._est.level(time.monotonic())
        if level is None:
            return
        self._known = True
        self._publish(level)
        if level:
            self.get_logger().warn(
                "[EmergencyStop] *** E-STOP PRESSED *** twist_mux lock raised."
            )
        else:
            self.get_logger().info("[EmergencyStop] E-stop released. Lock lowered.")

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        if self._est.level(time.monotonic()):
            # Keep the lock raised on shutdown - never silently release an e-stop.
            for _ in range(5):
                self._publish(True)
        super().destroy_node()

    # ------------------------------------------------------------------
    def _publish(self, pressed: bool) -> None:
        self._lock_pub.publish(Bool(data=pressed))
        self._state_pub.publish(Bool(data=pressed))


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import threading
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
from kobuki_ros_interfaces.msg import DigitalInputEvent, SensorState
from ..core.estop_latch import EstopLatch, is_pressed


class EmergencyStopNode(Node):
    """
    Latch the e-stop from /events/digital_input.

    Listens to /events/digital_input (kobuki_ros_interfaces/DigitalInputEvent).
    When pin 0 goes True (e-stop pressed):
      - Latches permanently (never releases, even if pin returns to False).
      - Raises the twist_mux e_stop LOCK by publishing std_msgs/Bool(data=True).
      - A dedicated thread keeps re-publishing True as fast as possible so the
        lock stays raised even if a message is dropped.

    The lock is arbitrated by twist_mux (see params/twist_mux.yaml): while it is
    True, twist_mux blocks every velocity input and the base receives no command,
    so the robot stops. This replaces the old approach of spamming zero Twist on
    cmd_vel directly — twist_mux now owns /cmd_vel.

    The same latched state also goes out on /skuba/nav/e_stop, for everything
    that is not twist_mux: skuba-state stops its tree when it reads True.

    The first value is the real button state, read once from /sensors/core
    (digital_input bit 0). The events only fire on a change, so a button already
    held when this node starts would otherwise be missed. Nothing is published
    until that first message arrives.

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

        self._latch = EstopLatch()
        self._shutdown = False
        self._seed_sub = None

        # Latched (transient-local) so twist_mux gets the current lock state even
        # if it subscribes after we start publishing.
        lock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        input_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._lock_pub = self.create_publisher(Bool, lock_topic, lock_qos)
        self._state_pub = self.create_publisher(Bool, state_topic, lock_qos)

        self._sub = self.create_subscription(
            DigitalInputEvent,
            "/events/digital_input",
            self._digital_input_cb,
            input_qos,
        )

        self._seed_sub = self.create_subscription(
            SensorState, "/sensors/core", self._seed_cb, qos_profile_sensor_data
        )

        # Dedicated thread: keeps the lock raised once latched.
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

        self.get_logger().info(
            f"[EmergencyStop] Ready. Listening on /events/digital_input pin[0]. "
            f'Will latch permanently and raise the twist_mux lock "{lock_topic}".'
        )

    # ------------------------------------------------------------------
    def _seed_cb(self, msg: SensorState) -> None:
        """Set the first value from the real button state, then stop listening."""
        if self._seed_sub is None:
            return  # a message that was already queued
        sub, self._seed_sub = self._seed_sub, None
        self.destroy_subscription(sub)
        if is_pressed(msg.digital_input):
            self._latch_pressed()
        elif not self._latch.is_active:  # an event may have latched first
            self._publish(False)

    # ------------------------------------------------------------------
    def _digital_input_cb(self, msg: DigitalInputEvent) -> None:
        if msg.values[0]:
            self._latch_pressed()

    # ------------------------------------------------------------------
    def _latch_pressed(self) -> None:
        if self._latch.trigger():
            self._publish(True)  # raise lock immediately
            self.get_logger().warn(
                "[EmergencyStop] *** E-STOP LATCHED *** twist_mux lock raised."
            )

    # ------------------------------------------------------------------
    def _publish_loop(self) -> None:
        # 10 Hz: without a sleep this loop pinned a whole CPU core, all the
        # time (and flooded e_stop once latched). twist_mux's e_stop lock has
        # no timeout, so the rate only needs to cover late subscribers.
        while not self._shutdown:
            if self._latch.is_active:
                self._publish(True)
            time.sleep(0.1)

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self._shutdown = True
        self._pub_thread.join(timeout=1.0)
        if self._latch.is_active:
            # Keep the lock raised on shutdown — never silently release an e-stop.
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

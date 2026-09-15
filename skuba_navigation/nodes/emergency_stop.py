#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
from kobuki_ros_interfaces.msg import DigitalInputEvent
from ..core.estop_latch import EstopLatch


class EmergencyStopNode(Node):
    """
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

    Parameters
    ----------
    lock_topic: str - twist_mux lock topic to raise. Default 'e_stop'.
    """

    def __init__(self):
        super().__init__('emergency_stop')

        self.declare_parameter('lock_topic', 'e_stop')
        lock_topic: str = self.get_parameter('lock_topic').value

        self._latch = EstopLatch()
        self._shutdown = False

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
        # Publish the initial "unlocked" state so twist_mux does not treat a
        # missing message as a lock.
        self._lock_pub.publish(Bool(data=False))

        self._sub = self.create_subscription(
            DigitalInputEvent,
            '/events/digital_input',
            self._digital_input_cb,
            input_qos,
        )

        # Dedicated thread: keeps the lock raised once latched.
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

        self.get_logger().info(
            f'[EmergencyStop] Ready. Listening on /events/digital_input pin[0]. '
            f'Will latch permanently and raise the twist_mux lock "{lock_topic}".'
        )

    # ------------------------------------------------------------------
    def _digital_input_cb(self, msg: DigitalInputEvent) -> None:
        if msg.values[0] and self._latch.trigger():
            self._lock_pub.publish(Bool(data=True))  # raise lock immediately
            self.get_logger().warn('[EmergencyStop] *** E-STOP LATCHED *** twist_mux lock raised.')

    # ------------------------------------------------------------------
    def _publish_loop(self) -> None:
        while not self._shutdown:
            if self._latch.is_active:
                self._lock_pub.publish(Bool(data=True))

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self._shutdown = True
        self._pub_thread.join(timeout=1.0)
        if self._latch.is_active:
            # Keep the lock raised on shutdown — never silently release an e-stop.
            for _ in range(5):
                self._lock_pub.publish(Bool(data=True))
        super().destroy_node()


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


if __name__ == '__main__':
    main()

"""rclpy implementations of core.navigation.ports (and msg conversions).

Adapters talk to ROS but hold no navigation logic and are not Nodes: they
are handed the node that owns them. Blocking calls here wait on futures that
other executor threads complete, so the owning node must spin in a
MultiThreadedExecutor.
"""

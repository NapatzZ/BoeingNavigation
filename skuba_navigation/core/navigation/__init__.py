"""Logic behind navigation_server. No rclpy / ROS message imports.

Layering (see README "navigation_server"):

  nodes/navigation_server_node.py   ROS glue: params, action servers, services
  adapters/                         rclpy implementations of the ports below
  core/navigation/                  this package - everything testable without ROS

    tasks.py           one function per action: the readable "what happens"
    navigation_job.py  send-to-Nav2 pipeline: retries, deadline, cancel, feedback
    motion.py          closed-loop rotate / drive-straight on cmd_vel
    standoff.py        where to park next to a TF frame and how to verify it
    location_store.py  saved named locations (saved_pose.yaml)
    validation.py      goal checks done before a goal is accepted
    cancellation.py    CancelToken + GoalArbiter (one base-motion goal at a time)
    retry_policy.py    Nav2 attempt outcome -> retry / finish decision
    ports.py           Protocols the adapters implement (Nav2, TF, cmd_vel)
    clock.py           Clock / Deadline / RateLimiter (injectable time)
    config.py          tunables; each field is also a ROS parameter
    types.py           Pose2D, ErrorCode, Phase, TaskFailure, TaskResult
"""

"""In-memory fakes of the navigation ports, driven by a simulated clock.

SimRobot is both the PoseSource and the BaseCommander: velocity commands are
integrated every time FakeClock.sleep() advances time, so the closed-loop
controllers in core/navigation/motion.py run against a (very) simple robot.
"""
import math

from skuba_navigation.core.geometry_utils import normalize_angle
from skuba_navigation.core.navigation.cancellation import CancelToken
from skuba_navigation.core.navigation.config import MotionConfig, NavigationConfig
from skuba_navigation.core.navigation.location_store import LocationStore
from skuba_navigation.core.navigation.motion import BaseMotion
from skuba_navigation.core.navigation.ports import Nav2Feedback, Nav2Status
from skuba_navigation.core.navigation.tasks import TaskContext
from skuba_navigation.core.navigation.types import ErrorCode, Pose2D, TaskFailure


class FakeClock:

    def __init__(self):
        self.t = 0.0
        self.listeners = []

    def now(self):
        return self.t

    def sleep(self, seconds, token=None):
        # Step in small increments so the robot is integrated finely.
        remaining = max(0.0, seconds)
        while remaining > 1e-9:
            if token is not None and token.cancelled:
                return
            dt = min(remaining, 0.01)
            self.t += dt
            remaining -= dt
            for listener in self.listeners:
                listener(dt)


class SimRobot:

    def __init__(self, clock, pose=Pose2D(0.0, 0.0, 0.0)):
        self.pose = pose
        self.linear = 0.0
        self.angular = 0.0
        self.frames = {}
        self.tf_down = False
        self.stops = 0
        self.commands = 0
        clock.listeners.append(self._integrate)

    # PoseSource
    def robot_pose(self, timeout=None):
        if self.tf_down:
            raise TaskFailure(ErrorCode.TF_LOOKUP_FAILED, 'tf down')
        return self.pose

    def frame_position(self, frame, timeout=None):
        if frame not in self.frames:
            raise TaskFailure(ErrorCode.TF_LOOKUP_FAILED, f'no frame {frame}')
        return self.frames[frame]

    # BaseCommander
    def command(self, linear, angular):
        self.commands += 1
        self.linear, self.angular = linear, angular

    def stop(self):
        self.stops += 1
        self.linear = self.angular = 0.0

    @property
    def moving(self):
        return self.linear != 0.0 or self.angular != 0.0

    def _integrate(self, dt):
        yaw = self.pose.yaw + self.angular * dt
        self.pose = Pose2D(self.pose.x + self.linear * math.cos(yaw) * dt,
                           self.pose.y + self.linear * math.sin(yaw) * dt,
                           normalize_angle(yaw))


class FakeNav2:
    """Scripted Nav2.

    `script` holds one entry per attempt: a Nav2Status to return right away,
    'arrive' (teleport to the goal, SUCCEEDED), 'abort_but_arrive' (teleport,
    ABORTED) or 'hang' (run until should_stop, then CANCELED). The last entry
    repeats.
    """

    def __init__(self, clock, robot, script=('arrive',), plannable=True):
        self.clock = clock
        self.robot = robot
        self.script = list(script)
        self.plannable = plannable
        self.goals = []
        self.clears = 0

    def navigate(self, goal, frame_id, behavior_tree, on_feedback, should_stop):
        step = self.script[min(len(self.goals), len(self.script) - 1)]
        self.goals.append((goal, frame_id, behavior_tree))
        on_feedback(Nav2Feedback(pose=self.robot.pose, distance_remaining=1.0))
        if step == 'hang':
            while not should_stop():
                self.clock.sleep(0.1)
            return Nav2Status.CANCELED
        if step in ('arrive', 'abort_but_arrive'):
            self.robot.pose = goal
            return Nav2Status.SUCCEEDED if step == 'arrive' else Nav2Status.ABORTED
        return step

    def is_plannable(self, start, goal, frame_id):
        return self.plannable

    def clear_costmaps(self):
        self.clears += 1
        return True


class Harness:
    """Everything a task needs, wired with fakes."""

    def __init__(self, tmp_path=None, script=('arrive',), plannable=True,
                 nav_config=NavigationConfig(retry_delay=0.0),
                 motion_config=MotionConfig(), start=Pose2D(0.0, 0.0, 0.0)):
        self.clock = FakeClock()
        self.robot = SimRobot(self.clock, start)
        self.nav2 = FakeNav2(self.clock, self.robot, script, plannable)
        self.token = CancelToken()
        self.feedback = []
        path = (tmp_path / 'saved_pose.yaml') if tmp_path else '/nonexistent/saved_pose.yaml'
        self.locations = LocationStore(path)
        self.nav_config = nav_config
        self.motion_config = motion_config
        self.motion = BaseMotion(self.robot, self.robot, motion_config, self.clock)

    def context(self):
        return TaskContext(
            nav2=self.nav2, poses=self.robot, motion=self.motion, locations=self.locations,
            nav_config=self.nav_config, motion_config=self.motion_config, clock=self.clock,
            token=self.token, feedback=lambda values, force: self.feedback.append(values),
            started_at=self.clock.now())

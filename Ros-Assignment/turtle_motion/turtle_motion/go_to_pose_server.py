"""goToPose action server + speed-limit parameters + set_speed_limits service.

The speed parameters live in this node because it is the node that moves the
turtle. They are re-read on every control cycle, so changes take effect live.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose

from turtle_interfaces.action import GoToPose
from turtle_interfaces.srv import SetSpeedLimits

SPEED_PARAMS = ('max_linear_speed', 'max_angular_speed')
DEFAULT_SPEED = 2.0

POSITION_TOLERANCE = 0.05    # m
YAW_TOLERANCE = 0.02         # rad
CONTROL_PERIOD = 0.05        # s (20 Hz)
K_LINEAR = 1.5
K_ANGULAR = 4.0
ROTATE_IN_PLACE_ANGLE = 0.5  # rad: above this heading error, turn before driving
WORLD_MIN, WORLD_MAX = 0.0, 11.0


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('go_to_pose_server')
        self._cb_group = ReentrantCallbackGroup()

        # ---- parameters -------------------------------------------------
        for name in SPEED_PARAMS:
            self.declare_parameter(
                name, DEFAULT_SPEED,
                ParameterDescriptor(description=f'{name} (must be > 0)'))
        for name in SPEED_PARAMS:  # a bad value from YAML/launch -> fall back
            if not self._speed_param(name) > 0.0:
                self.get_logger().warn(
                    f'{name} must be > 0; using default {DEFAULT_SPEED}')
                self.set_parameters(
                    [Parameter(name, Parameter.Type.DOUBLE, DEFAULT_SPEED)])
        self.add_on_set_parameters_callback(self._validate_parameters)

        self._speed_srv = self.create_service(
            SetSpeedLimits, 'set_speed_limits', self._set_speed_limits_cb,
            callback_group=self._cb_group)

        # ---- turtle I/O -------------------------------------------------
        self._pose = None
        self.create_subscription(
            Pose, '/turtle1/pose', self._pose_cb, 10,
            callback_group=self._cb_group)
        self._cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # ---- action server ----------------------------------------------
        self._busy = False
        self._busy_lock = threading.Lock()
        self._action_server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group)

        self.get_logger().info(
            'goToPose server ready. '
            f'max_linear_speed={self._speed_param("max_linear_speed")}, '
            f'max_angular_speed={self._speed_param("max_angular_speed")}')

    # ------------------------------------------------------------------
    # parameters + service
    # ------------------------------------------------------------------
    def _speed_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _validate_parameters(self, params):
        for p in params:
            if p.name in SPEED_PARAMS:
                if p.type_ != Parameter.Type.DOUBLE or not p.value > 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{p.name} must be a double > 0.0')
        return SetParametersResult(successful=True)

    def _set_speed_limits_cb(self, request, response):
        lin, ang = request.max_linear_speed, request.max_angular_speed
        if not (lin > 0.0 and ang > 0.0):
            response.success = False
            response.message = (
                f'Rejected: speeds must be > 0 (got linear={lin}, angular={ang})')
            self.get_logger().warn(response.message)
            return response

        results = self.set_parameters([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, float(lin)),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, float(ang)),
        ])
        response.success = all(r.successful for r in results)
        response.message = (
            f'Speed limits set: linear={lin:.2f} m/s, angular={ang:.2f} rad/s'
            if response.success else 'Failed to set parameters')
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # turtle helpers
    # ------------------------------------------------------------------
    def _pose_cb(self, msg):
        self._pose = msg

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _wait_for_pose(self, timeout):
        end = time.monotonic() + timeout
        while self._pose is None and rclpy.ok() and time.monotonic() < end:
            time.sleep(0.05)
        return self._pose is not None

    # ------------------------------------------------------------------
    # action callbacks
    # ------------------------------------------------------------------
    def _goal_cb(self, goal_request):
        t = goal_request.target_pose
        if not (WORLD_MIN <= t.x <= WORLD_MAX and WORLD_MIN <= t.y <= WORLD_MAX):
            self.get_logger().warn(
                f'Rejected goal ({t.x:.2f}, {t.y:.2f}): outside the 0..11 window')
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Rejected goal: another goal is running')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        try:
            return self._execute(goal_handle)
        finally:
            self._stop()
            with self._busy_lock:
                self._busy = False

    def _execute(self, goal_handle):
        target = goal_handle.request.target_pose
        target_yaw = normalize_angle(target.yaw)
        self.get_logger().info(
            f'New goal: x={target.x:.2f} y={target.y:.2f} yaw={target.yaw:.2f}')

        result = GoToPose.Result()
        feedback = GoToPose.Feedback()
        result.success = False
        result.final_distance = -1.0

        if not self._wait_for_pose(5.0):
            self.get_logger().error('No /turtle1/pose received - is turtlesim running?')
            goal_handle.abort()
            return result

        def distance():
            return math.hypot(target.x - self._pose.x, target.y - self._pose.y)

        def send_feedback():
            feedback.distance_to_goal = distance()
            feedback.current_x = self._pose.x
            feedback.current_y = self._pose.y
            feedback.current_yaw = self._pose.theta
            goal_handle.publish_feedback(feedback)

        def interrupted():
            """Return True (after closing the goal) if cancelled / shutting down."""
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.final_distance = distance()
                self.get_logger().info('Goal cancelled')
                return True
            if not rclpy.ok():
                goal_handle.abort()
                return True
            return False

        # ---- phase 1: drive to (x, y) ---------------------------------
        while True:
            if interrupted():
                return result
            pose = self._pose
            dx, dy = target.x - pose.x, target.y - pose.y
            dist = math.hypot(dx, dy)
            send_feedback()
            if dist < POSITION_TOLERANCE:
                break

            max_lin = self._speed_param('max_linear_speed')   # read every cycle
            max_ang = self._speed_param('max_angular_speed')

            angle_err = normalize_angle(math.atan2(dy, dx) - pose.theta)
            cmd = Twist()
            cmd.angular.z = clamp(K_ANGULAR * angle_err, -max_ang, max_ang)
            if abs(angle_err) < ROTATE_IN_PLACE_ANGLE:
                cmd.linear.x = clamp(K_LINEAR * dist, 0.0, max_lin)
            self._cmd_pub.publish(cmd)
            time.sleep(CONTROL_PERIOD)

        # ---- phase 2: rotate to the final yaw ---------------------------
        while True:
            if interrupted():
                return result
            yaw_err = normalize_angle(target_yaw - self._pose.theta)
            send_feedback()
            if abs(yaw_err) < YAW_TOLERANCE:
                break
            max_ang = self._speed_param('max_angular_speed')
            cmd = Twist()
            cmd.angular.z = clamp(K_ANGULAR * yaw_err, -max_ang, max_ang)
            self._cmd_pub.publish(cmd)
            time.sleep(CONTROL_PERIOD)

        self._stop()
        goal_handle.succeed()
        result.success = True
        result.final_distance = distance()
        self.get_logger().info(
            f'Goal reached (remaining distance {result.final_distance:.3f})')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

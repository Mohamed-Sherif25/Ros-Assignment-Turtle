"""draw_square action server: reset, then exactly 4 goToPose goals (one per corner)."""
import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from turtle_interfaces.action import DrawSquare, GoToPose
from turtle_interfaces.srv import ResetTurtle

START_X = 5.544445
START_Y = 5.544445
DEFAULT_SIDE = 2.0
MAX_SIDE = 5.3   # keeps the square inside the 0..11 window


class SquareActionServer(Node):
    def __init__(self):
        super().__init__('square_action_server')
        group = ReentrantCallbackGroup()
        self._reset = self.create_client(
            ResetTurtle, 'reset_turtle', callback_group=group)
        self._go_to_pose = ActionClient(
            self, GoToPose, 'go_to_pose', callback_group=group)

        self._busy = False
        self._busy_lock = threading.Lock()
        self._server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info('draw_square server ready.')

    def _goal_cb(self, goal_request):
        side = goal_request.side_length
        if side < 0.0 or side > MAX_SIDE:
            self.get_logger().warn(
                f'Rejected side_length={side}: must be 0 (default) or <= {MAX_SIDE}')
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Rejected goal: a square is already being drawn')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        try:
            return await self._execute(goal_handle)
        finally:
            with self._busy_lock:
                self._busy = False

    def _fail(self, goal_handle, message):
        self.get_logger().error(message)
        goal_handle.abort()
        result = DrawSquare.Result()
        result.success = False
        result.message = message
        return result

    async def _execute(self, goal_handle):
        side = goal_handle.request.side_length or DEFAULT_SIDE
        self.get_logger().info(f'Drawing a square with side {side:.2f}')

        # 1. clean screen + known start pose
        if not self._reset.wait_for_service(timeout_sec=5.0):
            return self._fail(goal_handle, 'reset_turtle service not available')
        reset_res = await self._reset.call_async(ResetTurtle.Request())
        if not reset_res.success:
            return self._fail(goal_handle, f'Reset failed: {reset_res.message}')

        if not self._go_to_pose.wait_for_server(timeout_sec=5.0):
            return self._fail(goal_handle, 'go_to_pose action server not available')

        # 2. the four corners; final yaw = heading of the next side
        x0, y0 = START_X, START_Y
        corners = [
            (x0 + side, y0,        math.pi / 2),
            (x0 + side, y0 + side, math.pi),
            (x0,        y0 + side, -math.pi / 2),
            (x0,        y0,        0.0),
        ]

        feedback = DrawSquare.Feedback()
        result = DrawSquare.Result()

        # 3. one goToPose goal per corner, waiting for each to finish
        for i, (x, y, yaw) in enumerate(corners, start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = f'Cancelled before side {i}'
                return result

            feedback.current_side = i
            goal_handle.publish_feedback(feedback)

            goal = GoToPose.Goal()
            goal.target_pose.x = x
            goal.target_pose.y = y
            goal.target_pose.yaw = yaw

            pose_goal = await self._go_to_pose.send_goal_async(goal)
            if not pose_goal.accepted:
                return self._fail(goal_handle, f'goToPose rejected side {i}')
            res = await pose_goal.get_result_async()
            if res.status != GoalStatus.STATUS_SUCCEEDED or not res.result.success:
                return self._fail(goal_handle, f'goToPose failed on side {i}')

        goal_handle.succeed()
        result.success = True
        result.message = 'Square completed'
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SquareActionServer()
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

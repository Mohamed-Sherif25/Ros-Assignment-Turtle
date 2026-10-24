"""reset_turtle service: clear the screen and teleport the turtle to its start.

Uses only turtlesim's own services: set_pen, teleport_absolute and clear.
The pen is lifted during the teleport so no line is drawn.
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import SetPen, TeleportAbsolute

from turtle_interfaces.srv import ResetTurtle

START_X = 5.544445
START_Y = 5.544445
START_THETA = 0.0
# turtlesim's default pen colour / width
PEN_R, PEN_G, PEN_B, PEN_WIDTH = 179, 184, 255, 3


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        group = ReentrantCallbackGroup()
        self._pen = self.create_client(
            SetPen, '/turtle1/set_pen', callback_group=group)
        self._teleport = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=group)
        self._clear = self.create_client(Empty, '/clear', callback_group=group)
        self.create_service(
            ResetTurtle, 'reset_turtle', self._reset_cb, callback_group=group)
        self.get_logger().info('reset_turtle service ready.')

    def _set_pen(self, off):
        req = SetPen.Request()
        req.r, req.g, req.b, req.width = PEN_R, PEN_G, PEN_B, PEN_WIDTH
        req.off = 1 if off else 0
        return self._pen.call_async(req)

    async def _reset_cb(self, request, response):
        for client in (self._pen, self._teleport, self._clear):
            if not client.wait_for_service(timeout_sec=2.0):
                response.success = False
                response.message = f'Service {client.srv_name} not available'
                self.get_logger().error(response.message)
                return response

        try:
            await self._set_pen(off=True)                # 1. pen up
            tele = TeleportAbsolute.Request()
            tele.x, tele.y, tele.theta = START_X, START_Y, START_THETA
            await self._teleport.call_async(tele)        # 2. back to start
            await self._set_pen(off=False)               # 3. pen down
            await self._clear.call_async(Empty.Request())  # 4. clear drawing
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'Reset failed: {exc}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = 'Screen cleared and turtle back at start pose'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
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

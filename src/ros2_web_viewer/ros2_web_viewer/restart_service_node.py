"""ROS2 Trigger service node that exits on request.

Used as a required launch process so that when this node exits,
the whole launch system shuts down cleanly.
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

_EXIT_DELAY_SEC = 0.2


class RestartServiceNode(Node):
    """Expose /restart_system Trigger and request process exit when called."""

    def __init__(self):
        super().__init__('restart_service_node')
        self._shutdown_timer = None
        self._exit_requested = False
        self.create_service(Trigger, '/restart_system', self._handle_restart_trigger)
        self.get_logger().info('Restart trigger ready on /restart_system')

    def _handle_restart_trigger(self, _request, response):
        if self._exit_requested:
            response.success = False
            response.message = 'Restart already in progress.'
            return response

        self._exit_requested = True
        response.success = True
        response.message = 'Restart requested. Shutting down launch.'
        self.get_logger().warning('Restart requested via /restart_system trigger service')

        # Delay shutdown slightly so the Trigger response can be returned first.
        self._shutdown_timer = self.create_timer(_EXIT_DELAY_SEC, self._shutdown_context)
        return response

    def _shutdown_context(self):
        self._cleanup_shutdown_timer()
        self._shutdown_rclpy()

    def _cleanup_shutdown_timer(self):
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
            self._shutdown_timer = None

    @staticmethod
    def _shutdown_rclpy():
        if rclpy.ok():
            rclpy.shutdown()


def main():
    rclpy.init()
    node = RestartServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cleanup_shutdown_timer()
        node.destroy_node()
        RestartServiceNode._shutdown_rclpy()

    sys.exit(0)


if __name__ == '__main__':
    main()

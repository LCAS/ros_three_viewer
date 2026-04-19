# Copyright 2025 Marc Hanheide
#
# Licensed under the MIT License (the "License");
# you may not use this file except in compliance with the License.
"""Simple std_srvs/Trigger service for ros2_web_viewer button demos."""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class TriggerServiceSimNode(Node):
    """Expose /viewer_demo/trigger and return a counter in the response."""

    def __init__(self):
        super().__init__('trigger_service_sim')
        self._counter = 0
        self._service = self.create_service(Trigger, '/viewer_demo/trigger', self._on_trigger)
        self.get_logger().info('trigger_service_sim started — serving /viewer_demo/trigger')

    def _on_trigger(self, _request: Trigger.Request, response: Trigger.Response):
        self._counter += 1
        response.success = True
        response.message = f'Trigger called {self._counter} time(s)'
        self.get_logger().info(f'/viewer_demo/trigger called — count={self._counter}')
        return response


def main(args=None):
    """Entry point for trigger_service_sim node."""
    rclpy.init(args=args)
    node = TriggerServiceSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

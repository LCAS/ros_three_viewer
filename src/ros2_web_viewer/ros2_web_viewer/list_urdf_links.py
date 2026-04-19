"""List URDF links and whether they include a visual tag.

Usage:
  ros2 run ros2_web_viewer list_urdf_links

The node subscribes to /robot_description with transient-local QoS so it can
receive latched robot descriptions.
"""

import json
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

_LATCHING_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class UrdfLinkLister(Node):
    def __init__(self) -> None:
        super().__init__('list_urdf_links')
        self._done = False

        self.create_subscription(
            String,
            '/robot_description',
            self._on_robot_description,
            _LATCHING_QOS,
        )

        self.get_logger().info('Waiting for /robot_description ...')

    def _on_robot_description(self, msg: String) -> None:
        if self._done:
            return
        self._done = True

        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as exc:
            self.get_logger().error(f'Failed to parse URDF from /robot_description: {exc}')
            self._shutdown()
            return

        links = [link for link in root.findall('link') if link.get('name')]

        if not links:
            self.get_logger().warning('No <link> elements found in URDF.')
            self._shutdown()
            return

        links_with_visual: list[str] = []
        links_without_visual: list[str] = []
        for link in links:
            name = str(link.get('name'))
            if link.find('visual') is not None:
                links_with_visual.append(name)
            else:
                links_without_visual.append(name)

        # Print YAML to stdout so users can copy directly into params.yaml.
        print('urdf_link_whitelist:')
        for name in links_with_visual:
            print(f'  - {json.dumps(name)}')

        print('urdf_link_blacklist:')
        for name in links_without_visual:
            print(f'  - {json.dumps(name)}')

        print(
            f'# summary: {len(links_with_visual)}/{len(links)} link(s) include <visual>',
            flush=True,
        )
        self._shutdown()

    def _shutdown(self) -> None:
        # Trigger shutdown asynchronously to avoid shutting down from within a callback.
        self.create_timer(0.0, lambda: rclpy.shutdown())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UrdfLinkLister()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()

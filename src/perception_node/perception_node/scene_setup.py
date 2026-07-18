#!/usr/bin/env python3
"""One-shot scene tweak: spread the blocks across the house.

They spawn clustered in mobile_arm_sim's launch, and brown-vs-orange is a
pair the camera genuinely confuses up close (same hue; orange's antialiased
rim sheds brown-band pixels). Rather than edit the sim repo, teleport each
block to its own room after it spawns: magenta on the gym mat, red and
orange at opposite ends of the bedroom, brown behind the dining table.
World-frame placement is fine here — scene setup, not the carried-block pin.
"""

import rclpy
from gazebo_msgs.srv import GetEntityState, SetEntityState
from rclpy.node import Node

# One block per area, ~0.7 m from its search pose in named_poses.yaml.
BLOCK_XY = {
    'target_block': (-7.6, -0.1),        # red: bedroom, south end
    'distractor_orange': (-4.5, 2.1),    # bedroom, window end (map-checked clear of the bed)
    'distractor_magenta': (-6.3, -3.7),  # gym mat
    'distractor_brown': (-1.0, 5.0),     # behind the dining table, clear of the balcony set
}


class SceneSetup(Node):

    def __init__(self):
        super().__init__('scene_setup')
        self._get = self.create_client(GetEntityState, '/get_entity_state')
        self._set = self.create_client(SetEntityState, '/set_entity_state')
        self._pending = dict(BLOCK_XY)
        self.create_timer(1.0, self._tick)

    def _tick(self):
        if not self._pending:
            return
        if not (self._get.service_is_ready() and self._set.service_is_ready()):
            return
        for name in list(self._pending):
            self._get.call_async(
                GetEntityState.Request(name=name)
            ).add_done_callback(lambda f, n=name: self._on_state(f, n))

    def _on_state(self, future, name):
        res = future.result()
        if res is None or not res.success or name not in self._pending:
            return   # not spawned yet; next tick retries
        x, y = self._pending[name]
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = 0.025
        req.state.pose.orientation.w = 1.0
        self._set.call_async(req).add_done_callback(
            lambda f, n=name: self._on_moved(f, n))

    def _on_moved(self, future, name):
        res = future.result()
        if res is not None and res.success and name in self._pending:
            xy = self._pending.pop(name)
            self.get_logger().info(f'{name} placed at {xy}')


def main(args=None):
    rclpy.init(args=args)
    node = SceneSetup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

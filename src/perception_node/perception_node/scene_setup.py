#!/usr/bin/env python3
"""One-shot scene fix: replace the blocks, then spread them across the house.

Two things are wrong with the blocks mobile_arm_sim spawns, and neither can be
fixed by moving them.

They are 50 mm cubes. The arm physically cannot reach a 50 mm cube on the floor
— shoulder_lift stops at 1.57, so the upper arm never drops below horizontal
and the finger pads bottom out at 35 mm against a cube centre at 25 mm — and
the gripper cannot open wider than 38 mm, so the fingers could not have
straddled one either. Both numbers are derived in ros_backend/kinematics.py and
its tests. So each cube is deleted and respawned as the upright bar described
by block_spec.py, which the arm can reach with margin and the gripper can
actually close on.

They also spawn clustered, and brown-vs-orange is a pair the camera genuinely
confuses up close (same hue; orange's antialiased rim sheds brown-band pixels),
so each one goes to its own room.

Rather than edit the sim repo for either, do it here: delete the entity, spawn
our own SDF under the same name, at the right place. Same reason the placement
has always been done from this side — mobile_arm_sim is shared, and its own
demo depends on those files.

World-frame placement is fine here — scene setup, not the carried-block pin.
"""

import rclpy
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState, SpawnEntity
from rclpy.node import Node

from perception_node.block_spec import BLOCK_D, BLOCK_H, BLOCK_W

# One block per area, ~0.7 m from its search pose in named_poses.yaml.
BLOCK_XY = {
    'target_block': (-7.6, -0.1),        # red: bedroom, south end
    'distractor_orange': (2.6, 4.2),     # lounge mat: the 2026-07-20 swap proved the old
                                         # failures followed the bedroom_window POSE
                                         # (24 deg AMCL yaw error), not the colour — kept
    'distractor_magenta': (-6.3, -3.7),  # gym mat
    'distractor_brown': (-1.1, -1.9),    # behind the white sofa (SofaC back face at
                                         # x~-0.6); nothing lives at bedroom_window now
}

# Kept identical to the SDFs in mobile_arm_sim so the HSV bands still hit.
BLOCK_RGB = {
    'target_block': '0.9 0.15 0.15',
    'distractor_orange': '1.0 0.40 0.0',
    'distractor_magenta': '0.90 0.10 0.90',
    'distractor_brown': '0.45 0.25 0.10',
}

# Same 400 kg/m^3 as the cubes it replaces (0.05 kg in 1.25e-4 m^3), so the
# contact behaviour stays in the range the friction was tuned for.
DENSITY = 400.0


def block_sdf(name: str) -> str:
    """An upright bar of the current block geometry, coloured like the original.

    Inertia is computed rather than copied: the cubes' 2.08e-5 was right for a
    50 mm cube and is wrong for anything else, and a stale inertia tensor is
    the kind of thing that silently changes how a model tips.
    """
    m = DENSITY * BLOCK_W * BLOCK_D * BLOCK_H
    ixx = m * (BLOCK_D ** 2 + BLOCK_H ** 2) / 12.0
    iyy = m * (BLOCK_W ** 2 + BLOCK_H ** 2) / 12.0
    izz = m * (BLOCK_W ** 2 + BLOCK_D ** 2) / 12.0
    size = f'{BLOCK_W} {BLOCK_D} {BLOCK_H}'
    rgb = BLOCK_RGB[name]
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{m:.6f}</mass>
        <inertia>
          <ixx>{ixx:.8e}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.8e}</iyy><iyz>0</iyz>
          <izz>{izz:.8e}</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>{rgb} 1</ambient>
          <diffuse>{rgb} 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{size}</size></box></geometry>
        <surface>
          <friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""


class SceneSetup(Node):

    def __init__(self):
        super().__init__('scene_setup')
        self._get = self.create_client(GetEntityState, '/get_entity_state')
        self._set = self.create_client(SetEntityState, '/set_entity_state')
        self._delete = self.create_client(DeleteEntity, '/delete_entity')
        self._spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self._pending = dict(BLOCK_XY)
        self._replaced = set()
        self.create_timer(1.0, self._tick)

    def _ready(self):
        return (self._get.service_is_ready() and self._set.service_is_ready()
                and self._delete.service_is_ready() and self._spawn.service_is_ready())

    def _tick(self):
        if not self._pending or not self._ready():
            return
        for name in list(self._pending):
            # Wait for the original to exist before swapping it, otherwise the
            # delete races the sim's own spawn and the block comes back as a cube.
            self._get.call_async(
                GetEntityState.Request(name=name)
            ).add_done_callback(lambda f, n=name: self._on_state(f, n))

    def _on_state(self, future, name):
        res = future.result()
        if res is None or not res.success or name not in self._pending:
            return   # not spawned yet; next tick retries
        if name in self._replaced:
            return   # swap already in flight
        self._replaced.add(name)
        self._delete.call_async(
            DeleteEntity.Request(name=name)
        ).add_done_callback(lambda f, n=name: self._on_deleted(f, n))

    def _on_deleted(self, future, name):
        res = future.result()
        if res is None or not res.success:
            self._replaced.discard(name)      # let the next tick retry
            return
        x, y = self._pending[name]
        req = SpawnEntity.Request()
        req.name = name
        req.xml = block_sdf(name)
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = BLOCK_H / 2
        req.initial_pose.orientation.w = 1.0
        self._spawn.call_async(req).add_done_callback(
            lambda f, n=name: self._on_spawned(f, n))

    def _on_spawned(self, future, name):
        res = future.result()
        if res is None or not res.success:
            self._replaced.discard(name)
            return
        xy = self._pending.pop(name)
        self.get_logger().info(
            f'{name} replaced with a {BLOCK_W}x{BLOCK_D}x{BLOCK_H} m bar at {xy}')


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

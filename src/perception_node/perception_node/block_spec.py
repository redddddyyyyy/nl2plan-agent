"""The block's dimensions, and the three constraints that fix them.

Its own module, with no imports at all, because both the detector and the scene
setup need these numbers and the detector pulls in cv2 and cv_bridge. Placing
the blocks should not be able to fail because an image library did.

The block is an upright 30 x 30 x 150 mm bar. It was a 50 mm cube until
2026-07-29, and each dimension is pinned from a different direction:

  Height, from below. The arm cannot reach a cube on the floor AT ALL.
  shoulder_lift stops at 1.57 so the upper arm never drops below horizontal,
  and the finger pads bottom out at 35 mm anywhere in the workspace against a
  cube centre at 25 mm. At 150 mm tall the grasp point rises to 75 mm, which
  the arm reaches at 0.262 m with real joint margin instead of pinned against
  a stop.

  Height, from above. The lidar is a single plane at 200 mm and the costmaps
  inflate 0.55 m. A block tall enough for the lidar to see becomes an obstacle
  Nav2 plans around, and the robot has to stand 0.25 m away to grasp it — so it
  would refuse to approach the thing it was sent to fetch. 150 leaves 50 mm.

  Width. node.gripper() drives the fingers symmetrically, and the URDF's
  prismatic stops allow only +-5 mm of that, so the faces open from 28 mm to at
  most 38 mm. 30 mm leaves 4 mm either side. The 50 mm cube could never have
  fitted between them at all, at any commanded value.

The cost is a 5:1 aspect ratio: unlike the cube, this can be knocked over.

Derivations are in ros_backend/kinematics.py and docs/physics.md sections 4-5;
the constraints are asserted in tests/test_block_geometry.py.
"""

BLOCK_W = 0.030    # across the gripper's closing axis
BLOCK_D = 0.030
BLOCK_H = 0.150

# Height of the block's centre, which is both where it is spawned and the plane
# the detector back-projects blob centroids onto.
BLOCK_CENTRE_Z = BLOCK_H / 2

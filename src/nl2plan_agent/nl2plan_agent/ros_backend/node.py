"""The one long-lived ROS node behind the backend.

Everything the four tools share lives here: publishers, cached sensor state,
TF, the Gazebo entity-state client, and the 20 Hz magic-grasp pin. The node
spins in a daemon-thread executor; tool calls block the agent thread and
poll these caches. The pin runs on the executor thread, so a held block
stays glued to the gripper straight through a blocking navigate_to.
"""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
import tf2_ros
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray

from .logic import KNOWN_COLORS


# How far below the middle of the finger pads a held block's centre sits.
#
# The GRASP pose puts the pads at z = 0.1532 and the block stands 0.150 tall,
# so the pads close on its top 22 mm and its centre is 0.0782 m lower. Pinning
# it there means grasping does not move the block at all. Checked against the
# block geometry and the GRASP pose in tests/test_block_geometry.py, which is
# what catches this going stale if either changes.
CARRY_HOLD_BELOW_PADS = 0.078


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def _rotate(q, v) -> tuple:
    """Rotate vector `v` by quaternion `q`, via v + 2s(u x v) + 2(u x (u x v))."""
    ux, uy, uz, s = q.x, q.y, q.z, q.w
    cx = uy * v[2] - uz * v[1]
    cy = uz * v[0] - ux * v[2]
    cz = ux * v[1] - uy * v[0]
    ccx = uy * cz - uz * cy
    ccy = uz * cx - ux * cz
    ccz = ux * cy - uy * cx
    return (v[0] + 2 * s * cx + 2 * ccx,
            v[1] + 2 * s * cy + 2 * ccy,
            v[2] + 2 * s * cz + 2 * ccz)


class BackendNode(Node):

    def __init__(self):
        # Detection freshness and dwell times count in sim seconds; on wall
        # clock they would silently drift with sim load.
        super().__init__('nl2plan_backend', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

        self.robot: Optional[tuple] = None   # (x, y, yaw) from AMCL
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.odom: Optional[tuple] = None    # (x, y, yaw) — smooth, for creeps
        self.create_subscription(Odometry, '/odom', self._odom_cb,
                                 qos_profile_sensor_data)

        self.blocks: dict = {}   # color -> latest PoseStamped, any age
        for color in KNOWN_COLORS:
            self.create_subscription(
                PoseStamped, f'/block_pose/{color}',
                lambda msg, c=color: self.blocks.__setitem__(c, msg), 10)

        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._set_state_cli = self.create_client(SetEntityState, '/set_entity_state')

        # Magic grasp: while pinned_entity is set, teleport it to the
        # gripper at 20 Hz, relative to the robot MODEL. Never the map
        # frame: AMCL error would bake into the block's real position.
        self.pinned_entity: Optional[str] = None
        self.create_timer(0.05, self._teleport_pinned)

    def _amcl_cb(self, msg):
        p = msg.pose.pose.position
        self.robot = (p.x, p.y, _yaw(msg.pose.pose.orientation))

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, _yaw(msg.pose.pose.orientation))

    def block_age(self, color: str) -> Optional[float]:
        """Sim-time age of the latest sighting of `color`, in seconds."""
        msg = self.blocks.get(color)
        if msg is None:
            return None
        age = self.get_clock().now() - Time.from_msg(msg.header.stamp)
        return age.nanoseconds * 1e-9

    def set_entity_rel(self, name: str, x: float, y: float, z: float):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'mobile_arm'
        self._set_state_cli.call_async(req)

    def pad_centre(self) -> Optional[tuple]:
        """Middle of the finger pads in base_footprint — where a block is held.

        `gripper_base` is 0.055 m short of it along the gripper's own axis, and
        that axis swings from near-vertical at GRASP to 30 degrees off
        horizontal at DROP, so the difference is not a constant offset in z.
        """
        try:
            t = self.tf_buffer.lookup_transform('base_footprint', 'gripper_base',
                                                rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        p, r = t.transform.translation, t.transform.rotation
        dx, dy, dz = _rotate(r, (0.0, 0.0, 0.055))
        return (p.x + dx, p.y + dy, p.z + dz)

    def _teleport_pinned(self):
        """Hold the block where the pads actually closed on it.

        Pinning the block's centre to `gripper_base`, which is what this did
        until 2026-07-29, teleported it 0.133 m upward the instant the grasp
        fired — visible on video, and it parked the block straddling the
        0.20 m lidar plane a quarter-metre in front of the robot, which paints
        a costmap obstacle with a 0.55 m inflation skirt around the robot's own
        payload. Holding it below the pads instead means the block does not
        move at all when it is grasped, which is what a pin should mean.

        Orientation is left upright rather than following the gripper. The
        gripper is nearly horizontal at DROP and a rigidly-held bar would be
        carried on its side; keeping it level is the cruder model but it is the
        one the rest of the place sequence is built around.
        """
        if self.pinned_entity is None or not self._set_state_cli.service_is_ready():
            return
        pads = self.pad_centre()
        if pads is None:
            return
        self.set_entity_rel(self.pinned_entity,
                            pads[0], pads[1], pads[2] - CARRY_HOLD_BELOW_PADS)

    # ---------- actuators ----------

    def stop_base(self):
        self.cmd_pub.publish(Twist())

    def arm(self, positions):
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        self.arm_pub.publish(msg)

    def gripper(self, half_opening):
        """Drive the fingers apart by `half_opening` each, about the palm centre.

        The two commands have to be opposites. Both finger joints declare
        `axis="1 0 0"` in the URDF and mirror only their *origin*, so sending
        the same number to both — which this did until 2026-07-29 — slides the
        pair sideways in unison and never changes the gap between them at all.
        It sat at 28 mm for every value of GRIPPER_OPEN and GRIPPER_CLOSED
        alike, which is why nothing ever visibly gripped.

        Controller joint order is [left_finger_joint, right_finger_joint], from
        controllers.yaml. Left is the one at negative x, so it takes the
        negative command. Gap between the finger faces is 0.028 + 2*half_opening.
        """
        msg = Float64MultiArray()
        msg.data = [-float(half_opening), float(half_opening)]
        self.gripper_pub.publish(msg)


_node: Optional[BackendNode] = None
_executor: Optional[MultiThreadedExecutor] = None
_spin_thread: Optional[threading.Thread] = None
_lock = threading.Lock()


def _shutdown():
    """Stop the executor and unwind rclpy before the interpreter finalizes.

    Registered via threading._register_atexit, not atexit: plain atexit
    handlers run during finalization and the process aborts in rclpy's
    C++ layer even after the handler completes (measured on Humble).
    """
    global _node, _executor, _spin_thread
    with _lock:
        try:
            if _executor is not None:
                _executor.shutdown(timeout_sec=2.0)
                _executor = None
            if _spin_thread is not None:
                _spin_thread.join(timeout=3.0)
                _spin_thread = None
            if _node is not None:
                _node.destroy_node()
                _node = None
        finally:
            rclpy.try_shutdown()


def get_node() -> BackendNode:
    """Start rclpy + the node + a daemon executor thread once, then reuse."""
    global _node, _executor, _spin_thread
    with _lock:
        if _node is None:
            rclpy.init()
            _node = BackendNode()
            _executor = MultiThreadedExecutor()
            _executor.add_node(_node)
            _spin_thread = threading.Thread(target=_executor.spin, daemon=True,
                                            name='nl2plan-ros-executor')
            _spin_thread.start()
            threading._register_atexit(_shutdown)
        return _node

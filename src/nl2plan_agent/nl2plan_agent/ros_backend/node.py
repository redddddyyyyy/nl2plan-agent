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


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


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

    def _teleport_pinned(self):
        if self.pinned_entity is None or not self._set_state_cli.service_is_ready():
            return
        try:
            t = self.tf_buffer.lookup_transform('base_footprint', 'gripper_base',
                                                rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        self.set_entity_rel(self.pinned_entity,
                            t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z)

    # ---------- actuators ----------

    def stop_base(self):
        self.cmd_pub.publish(Twist())

    def arm(self, positions):
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        self.arm_pub.publish(msg)

    def gripper(self, opening):
        msg = Float64MultiArray()
        msg.data = [float(opening), float(opening)]
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

#!/usr/bin/env python3
"""Detect the four colored blocks and publish a map-frame pose per color.

Fork of mobile_arm_sim's block_detector.py (that repo stays untouched): same
back-projection pipeline, but run once per color band per frame, publishing
/block_pose/<color> instead of the single red /target_block_pose. Detection
range for a 5 cm cube is ~1.1 m; past that it is a dozen pixels and the
size-distance gate rejects it.
"""

import cv2
import numpy as np

# Red wraps around hue 0, so it takes two bands, kept tight because dark
# orange (hue ~13) and brown (~10) sit right against the low band. Orange
# and brown overlap in hue almost entirely; value tells them apart — the
# SDF materials work out to V~255 vs V~115, split at 140 with slack for
# lighting. Magenta is far from everything.
COLOR_BANDS = {
    'red':     [((0, 120, 70), (6, 255, 255)),
                ((174, 120, 70), (180, 255, 255))],
    'orange':  [((8, 150, 140), (22, 255, 255))],
    'brown':   [((5, 100, 40), (22, 255, 139))],
    'magenta': [((135, 80, 70), (172, 255, 255))],
}

MIN_AREA = 400.0   # px^2 — below this it's a reflection or speckle
GROUND_Z = 0.025   # block center height: 5 cm cube on the floor

_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))


def color_mask(hsv, bands):
    """Binary mask of pixels inside any of the color's HSV bands, cleaned."""
    mask = None
    for lo, hi in bands:
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = m if mask is None else (mask | m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)
    return mask


def blob_candidates(mask, min_area=MIN_AREA):
    """Centroids (u, v, area) of every contour >= min_area, biggest first.

    The biggest blob is not always the block: the house itself is full of
    block-colored pixels — the wood floor and furniture out-brown the brown
    block from most viewpoints, and a furniture contour winning largest-blob
    starved the brown block of detections entirely. Offer every candidate;
    the size-distance gate downstream decides which one is a block.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in sorted(contours, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(c)
        if area < min_area:
            break
        m = cv2.moments(c)
        if m['m00'] == 0:
            continue
        out.append((m['m10'] / m['m00'], m['m01'] / m['m00'], area))
    return out


def pixel_to_ground(u, v, K, R_mo, t_mo, ground_z=GROUND_Z):
    """Back-project pixel (u, v) onto the plane z = ground_z, in map frame.

    Returns None when the ray points at or above the horizon — without the
    guard a sky pixel "detects" a block far behind the camera.
    """
    ray_opt = np.linalg.inv(K) @ np.array([u, v, 1.0])
    ray_map = R_mo @ ray_opt
    if ray_map[2] > -1e-3:
        return None
    s = (ground_z - t_mo[2]) / ray_map[2]
    if s <= 0:
        return None
    return t_mo + s * ray_map


def quat_to_rot(x, y, z, w):
    """3x3 rotation matrix from a quaternion; enough numpy to skip scipy."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ROS wrapper below; imports guarded so the math above stays importable
# without a sourced environment.
try:
    import rclpy
    import tf2_ros
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:
    Node = object


class ColorBlockDetector(Node):
    """HSV detector: camera frames in, one map-frame pose topic per color out."""

    def __init__(self):
        super().__init__('color_block_detector')
        self.declare_parameter('min_area', MIN_AREA)

        self._bridge = CvBridge()
        self._K = None
        self._cam_frame = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pubs = {
            color: self.create_publisher(PoseStamped, f'/block_pose/{color}', 10)
            for color in COLOR_BANDS
        }

        # gazebo_ros_camera publishes best-effort; a reliable subscription
        # would never receive a frame.
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw',
                                 self._image_cb, qos_profile_sensor_data)

    def _info_cb(self, msg):
        self._K = np.array(msg.k).reshape(3, 3)
        self._cam_frame = msg.header.frame_id

    def _image_cb(self, msg):
        if self._K is None:
            self.get_logger().info('waiting for /camera/camera_info',
                                   throttle_duration_sec=5.0)
            return
        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        cam = self._camera_pose(msg.header.stamp)
        if cam is None:
            return
        R_mo, t_mo = cam
        min_area = self.get_parameter('min_area').value

        for color, bands in COLOR_BANDS.items():
            for u, v, area in blob_candidates(color_mask(hsv, bands), min_area):
                p_map = pixel_to_ground(u, v, self._K, R_mo, t_mo)
                if p_map is None:
                    continue
                # Size-distance consistency: a 5 cm cube at the projected
                # distance has a predictable pixel area. This is what stops
                # the orange chairs and the wood floor being called blocks —
                # and why candidates get tried in turn rather than only the
                # biggest one.
                dist = float(np.linalg.norm(p_map - t_mo))
                if not self._block_sized(area, dist):
                    continue
                pose = PoseStamped()
                pose.header.stamp = msg.header.stamp
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(p_map[0])
                pose.pose.position.y = float(p_map[1])
                pose.pose.position.z = GROUND_Z
                pose.pose.orientation.w = 1.0
                self._pubs[color].publish(pose)
                self.get_logger().info(
                    f'{color} block at map ({p_map[0]:.2f}, {p_map[1]:.2f}), '
                    f'{area:.0f} px^2 at {dist:.2f} m',
                    throttle_duration_sec=2.0)
                break

    def _camera_pose(self, stamp):
        """(R, t) taking optical-frame points to map, or None.

        Prefer the image stamp; AMCL only refreshes map->odom on filter
        updates, so a parked robot's TF goes stale and exact-stamp lookups
        die on extrapolation — fall back to latest then.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', self._cam_frame, stamp, timeout=Duration(seconds=0.2))
        except tf2_ros.ExtrapolationException:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', self._cam_frame, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                self.get_logger().warning(
                    f'no map->{self._cam_frame} TF: {exc}',
                    throttle_duration_sec=2.0)
                return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as exc:
            self.get_logger().warning(f'no map->{self._cam_frame} TF: {exc}',
                                      throttle_duration_sec=2.0)
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        return quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def _block_sized(self, area, dist):
        expected = (self._K[0, 0] * 0.05 / max(dist, 0.05)) ** 2
        return 0.3 * expected <= area <= 4.0 * expected


def main(args=None):
    rclpy.init(args=args)
    node = ColorBlockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Blocking Nav2 client: bt_navigator active-wait, goal, result, timeout."""

from __future__ import annotations

import math
import time
from typing import Optional

from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose

from .logic import nav_result

ACTIVE_WAIT_S = 30.0
NAV_TIMEOUT_S = 120.0


def wait_nav_active(node, timeout: float = ACTIVE_WAIT_S) -> Optional[str]:
    """Block until /bt_navigator reports lifecycle 'active'; error string on timeout.

    Nav2 bringup occasionally wedges (map_server stuck inactive). That's a
    relaunch, not something to drive from here.
    """
    cli = node.create_client(GetState, '/bt_navigator/get_state')
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if cli.service_is_ready():
                fut = cli.call_async(GetState.Request())
                t_end = time.monotonic() + 2.0
                while not fut.done() and time.monotonic() < t_end:
                    time.sleep(0.05)
                if fut.done() and fut.result() is not None \
                        and fut.result().current_state.id == 3:
                    return None
            time.sleep(0.5)
        return ("Nav2 is not active (bt_navigator never reached 'active'). "
                "The sim needs a relaunch.")
    finally:
        node.destroy_client(cli)


def navigate(node, x: float, y: float, yaw: float,
             timeout: float = NAV_TIMEOUT_S) -> dict:
    """Send one NavigateToPose goal and block to a terminal result."""
    if not node.nav_client.wait_for_server(timeout_sec=5.0):
        return {"success": False,
                "error": "Nav2 action server not available. The sim needs a relaunch."}

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = float(x)
    goal.pose.pose.position.y = float(y)
    goal.pose.pose.orientation.z = math.sin(yaw / 2)
    goal.pose.pose.orientation.w = math.cos(yaw / 2)

    start = time.monotonic()
    send_fut = node.nav_client.send_goal_async(goal)
    while not send_fut.done():
        if time.monotonic() - start > 10.0:
            return {"success": False, "error": "Nav2 did not answer the goal request."}
        time.sleep(0.05)
    handle = send_fut.result()
    if not handle.accepted:
        return {"success": False, "error": "Nav2 rejected the goal."}

    result_fut = handle.get_result_async()
    while not result_fut.done():
        if time.monotonic() - start > timeout:
            handle.cancel_goal_async()
            return {"success": False,
                    "error": f"Navigation timed out after {int(timeout)} s; goal canceled."}
        time.sleep(0.1)

    return nav_result(result_fut.result().status, time.monotonic() - start,
                      {"x": x, "y": y, "theta": yaw})

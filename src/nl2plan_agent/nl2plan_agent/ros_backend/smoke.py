"""Drive one backend tool at a time against the running sim, no LLM.

    python3 -m nl2plan_agent.ros_backend.smoke navigate_to west_room
    python3 -m nl2plan_agent.ros_backend.smoke navigate_to -- -6.9,-0.1,0.0
    python3 -m nl2plan_agent.ros_backend.smoke find_object "red block"
    python3 -m nl2plan_agent.ros_backend.smoke pick red_block
    python3 -m nl2plan_agent.ros_backend.smoke place
"""

from __future__ import annotations

import argparse
import json

from .backend import RosBackend


def main():
    parser = argparse.ArgumentParser(prog="smoke")
    sub = parser.add_subparsers(dest="tool", required=True)
    p_nav = sub.add_parser("navigate_to")
    p_nav.add_argument("target", help="named pose, or x,y,theta")
    p_find = sub.add_parser("find_object")
    p_find.add_argument("description")
    p_pick = sub.add_parser("pick")
    p_pick.add_argument("object_id")
    sub.add_parser("place")
    args = parser.parse_args()

    backend = RosBackend()
    if args.tool == "navigate_to":
        if "," in args.target:
            x, y, theta = (float(v) for v in args.target.split(","))
            result = backend.navigate_to(None, {"x": x, "y": y, "theta": theta})
        else:
            result = backend.navigate_to(args.target, None)
    elif args.tool == "find_object":
        result = backend.find_object(args.description)
    elif args.tool == "pick":
        result = backend.pick(args.object_id)
    else:
        result = backend.place(None)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

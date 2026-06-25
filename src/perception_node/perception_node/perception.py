"""Open-vocabulary object detection + RGB-D-to-world-pose.

Exposes a ROS2 service /nl2plan/find_object (custom srv: string description ->
bool found, string object_id, geometry_msgs/Pose pose, float32 confidence,
string error).

Pipeline:
  1. Grab the latest RGB + depth + camera_info frames (synchronized).
  2. Run GroundingDINO on RGB with the description as the text query.
  3. Pick the highest-confidence detection; bail if confidence < threshold.
  4. Backproject the bbox centroid into 3D using depth + intrinsics.
  5. Transform from camera frame to map frame via tf2.
  6. Maintain an in-memory object table keyed by stable object_id so pick()
     can retrieve the same object later.

Run GroundingDINO with the `groundingdino-py` package or HuggingFace's
IDEA-Research/grounding-dino-tiny for lower latency. On CPU expect ~1-2s per
frame; on GPU ~80-150ms.
"""

# TODO(linux): implement on Linux. The model weights live outside the repo;
# download via groundingdino-py's setup or huggingface_hub.


def main():
    raise NotImplementedError("Implement on Linux. See module docstring.")


if __name__ == "__main__":
    main()

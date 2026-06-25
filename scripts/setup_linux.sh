#!/usr/bin/env bash
# Linux-side bootstrap for the NL2Plan workspace.
# Tested on Ubuntu 22.04 + ROS2 Humble.
#
# Run from the workspace root:
#     bash scripts/setup_linux.sh

set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ROS2 not on PATH. Source /opt/ros/humble/setup.bash and rerun." >&2
  exit 1
fi

echo "==> Installing apt deps"
sudo apt update
sudo apt install -y \
  python3-pip python3-colcon-common-extensions python3-rosdep \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-moveit \
  ros-humble-ackermann-msgs \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-xacro \
  ros-humble-robot-state-publisher

echo "==> Installing Python deps"
pip install --user -r requirements.txt

echo "==> Cloning hybrid-astar-planner as a submodule"
if [ ! -d "src/hybrid_astar_planner" ]; then
  git clone https://github.com/redddddyyyyy/hybrid-astar-planner.git src/_hybrid_astar_repo
  # The repo's ROS2 package lives in ros2_ws/src/hybrid_astar_planner.
  ln -s "$(pwd)/src/_hybrid_astar_repo/ros2_ws/src/hybrid_astar_planner" src/hybrid_astar_planner
fi

echo "==> rosdep update + install"
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "==> Pulling Ollama model (qwen2.5:7b-instruct)"
if command -v ollama >/dev/null 2>&1; then
  ollama pull qwen2.5:7b-instruct
else
  echo "Ollama not installed. Get it from https://ollama.com/download then run: ollama pull qwen2.5:7b-instruct"
fi

echo "==> colcon build"
colcon build --symlink-install

echo
echo "Done. Source the workspace and launch:"
echo "    source install/setup.bash"
echo "    ros2 launch kitchen_world kitchen.launch.py"

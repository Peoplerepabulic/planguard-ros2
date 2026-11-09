#!/usr/bin/env bash
# Runs once when the Codespace is created: build the workspace and source it in new shells.
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
grep -q planguard-ros2 ~/.bashrc || echo "source $(pwd)/install/setup.bash  # planguard-ros2" >> ~/.bashrc
echo "Done. Open a new terminal, then e.g.: ros2 launch planguard_description sim.launch.py"

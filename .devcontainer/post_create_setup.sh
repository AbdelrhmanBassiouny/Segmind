#!/bin/bash
set -euxo pipefail

# workspace directory
WORKSPACE_DIR="/home/devuser/ros_ws/src/Segmind"

# Wait for files to be mounted
while [ ! -f "$WORKSPACE_DIR/requirements.txt" ]; do
  echo "Waiting for repo to mount..."
  sleep 1
done

# Your setup commands
source /home/devuser/.virtualenvs/pycram-segmind/bin/activate
pip install -r "$WORKSPACE_DIR/requirements.txt"
pip install -e "$WORKSPACE_DIR"
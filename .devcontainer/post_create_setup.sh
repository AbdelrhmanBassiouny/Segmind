#!/bin/bash
set -euxo pipefail

# workspace directory
WORKSPACE_DIR="/root/workspace/src/Segmind"

# Wait for files to be mounted
while [ ! -f "$WORKSPACE_DIR/requirements.txt" ]; do
  echo "Waiting for repo to mount..."
  sleep 1
done

# Your setup commands
source /root/.virtualenvs/pycram-segmind/bin/activate
pip install -r "$WORKSPACE_DIR/requirements.txt"
pip install -e "$WORKSPACE_DIR"
#!/bin/bash
set -e

PORT=8080
ADDR="0.0.0.0:$PORT"
DATA_DIR="/.jbdevcontainer/data/code-server"

# Kill any existing code-server running on that port
echo "🔍 Checking for existing code-server on port $PORT..."
if lsof -i tcp:$PORT >/dev/null 2>&1; then
  echo "🛑 Port $PORT is in use. Attempting to kill existing code-server..."
  pkill -f "code-server.*$ADDR" || fuser -k $PORT/tcp || true
fi

# Start code-server
echo "🚀 Starting code-server on $ADDR"
exec code-server --bind-addr $ADDR --user-data-dir $DATA_DIR --auth none "$@"

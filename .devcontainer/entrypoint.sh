#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash
source /home/devuser/ros_ws/install/setup.bash
source /home/devuser/.virtualenvs/pycram-segmind/bin/activate

exec "$@"
#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash
source /root/ros_ws/install/setup.bash
source /root/.virtualenvs/pycram-segmind/bin/activate

exec "$@"
#!/bin/bash


set -e

source /opt/ros/jazzy/setup.bash
source /root/ros_ws/install/setup.bash
source /root/.virtualenvs/pycram-segmind/bin/activate

cd /root/ros_ws/src/Segmind/test

python -m pytest --pdb -s -k "test_pick_up" --tb=short --disable-warnings

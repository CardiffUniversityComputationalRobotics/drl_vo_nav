#!/bin/sh
DIR="${1}"
if [ -z "$DIR" ]; then
  DIR="./runs"
fi
if [ -d "$DIR" ]; then
  echo "${DIR} found"
else
  echo "${DIR} not found, creating ${DIR}"
  mkdir -p "${DIR}"
fi
Xvfb :1 -screen 0 1600x1200x16 &
export DISPLAY=:1.0
ros2 launch drl_vo drl_vo_nav_train.launch.py rviz:=false log_dir:="${DIR}"

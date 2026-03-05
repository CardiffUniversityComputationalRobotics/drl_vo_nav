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
ros2 launch drl_vo drl_vo_nav_train.launch.py log_dir:="${DIR}"

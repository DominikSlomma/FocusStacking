#!/bin/bash
set -e

echo "Updating workspace..."

# Ins temporäre Verzeichnis wechseln
cd /tmp

# Falls schon da, altes Repo löschen
if [ -d FocusStacking ]; then
  rm -rf FocusStacking
fi

# Repository neu klonen
git clone https://github.com/DominikSlomma/FocusStacking

# Alte Quellcodes löschen und durch neue ersetzen
rm -rf /home/ws/src/flask_web_interface/*
rm -rf /home/ws/src/fs_backend/*

cp -r FocusStacking/flask_web_interface/* /home/ws/src/flask_web_interface/
cp -r FocusStacking/fs_backend/* /home/ws/src/fs_backend/

rm -rf FocusStacking

sleep 5
# ROS Workspace neu bauen
source /opt/ros/humble/setup.bash
cd /home/ws
colcon build
source install/setup.bash

echo "Update complete."

ros2 run flask_web_interface web_server

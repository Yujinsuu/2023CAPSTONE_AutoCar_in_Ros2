#!/bin/bash

sudo apt update
sudo apt install python3-pip
sudo apt install -y python3-argcomplete
sudo apt install ros-foxy-ackermann-msgs
sudo apt install ros-foxy-vision-msgs
sudo apt install ros-foxy-robot-localization
sudo apt install ros-foxy-velodyne
sudo apt install ros-foxy-pcl-ros

pip install --upgrade pip
pip install -r requirements.txt

#!/bin/bash -xve

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys 6B05F25D762E3157
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#sudo apt-get update
#sudo apt-get dist-upgrade
#sudo apt-get autoremove
#sudo apt-get clean

sudo apt-get download dpkg
sudo dpkg -i dpkg_1.17.5ubuntu5.8_amd64.deb
sudo apt-get update && sudo apt-get upgrade
sudo apt-get -f install
sudo apt-get update && sudo apt-get upgrade

#required packages
sudo pip install catkin_pkg
sudo pip install empy
sudo pip install pyyaml
sudo pip install rospkg

#ros install
cd ..
git clone https://github.com/ryuichiueda/ros_setup_scripts_Ubuntu14.04_server.git
cd ./ros_setup_scripts_Ubuntu14.04_server
bash ./step0.bash
bash ./step1.bash

#catkin setup
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
source /opt/ros/indigo/setup.bash
catkin_init_workspace
cd ~/catkin_ws
catkin_make

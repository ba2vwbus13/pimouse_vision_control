#!/bin/bash -xve

apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 18DAAE7AECA3745F
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install libopencv-dev python-opencv

#sudo /bin/bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
#sudo ldconfig

#sudo apt-get install libopencv-dev
sudo apt-get install ros-indigo-cv-bridge
sudo apt-get install ros-indigo-cv-camera

#sync and make
rsync -av ./ ~/catkin_ws/src/pimouse_vision_control/

#clone pimouse_ros
cd ~/catkin_ws/src/
git clone https://github.com/ba2vwbus13/pimouse_ros.git

cd ~/catkin_ws
catkin_make


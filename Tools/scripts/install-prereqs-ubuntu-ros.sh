#!/bin/bash

set -e
set -x

if [ `id -u` -eq 0 ]; then
    echo "Do not run this as root; it will sudo as required"
    exit 1
fi

echo "Setting up repo for ROS."

sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) main universe restricted multiverse"

### Install ROS : taken from http://wiki.ros.org/kinetic/Installation/Ubuntu
# Configure your Ubuntu repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Configure Gazebo OSRF repositories
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update

# Installation
echo "Installing ROS."
sudo apt-get -y install ros-kinetic-ros-base ros-kinetic-mavros-extras
sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools
echo "Installing Gazebo8 with ROS binding."
sudo apt-get -y install gazebo8 ros-kinetic-gazebo8-ros

# Initialize rosdep
## TODO : fix that. It should be run only once
sudo rosdep init
rosdep update

# Environment setup
echo "source /opt/ros/kinetic/setup.bash" >> $HOME/.bashrc

# Installing mavros
echo "Installing Mavros."
sudo apt-get -y install ros-kinetic-mavros

# install rqt (visualisation framework)
echo "Installing RQT."
sudo apt-get install -y \
     ros-kinetic-rqt \
     ros-kinetic-rqt-topic \
     ros-kinetic-rqt-service-caller

# Install lubuntu for GUI
## TODO : look for less dependencies solution
### desktop-file-utils dmz-cursor-theme fcitx fcitx-config-gtk2 fcitx-frontend-gtk2 fcitx-ui-classic gnome-system-tools gnome-time-admin gvfs-backends indicator-application-gtk2 libfm-modules light-locker light-locker-settings lightdm-gtk-greeter-settings lubuntu-coreg lubuntu-default-session lxappearance  lxappearance-obconf lxinput lxpanel-indicator-applet-plugin lxrandr lxsession-default-apps lxterminal lxtask network-manager-gnome ntp obconf pm-utils xdg-user-dirs vfs-fuse xdg-user-dirs-gtk x11-utils
sudo apt-get install -y lubuntu-desktop

pushd /tmp
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
popd

#!/bin/bash

set -e
set -x

# first set up the install like any other:
/vagrant/Tools/vagrant/initvagrant.sh

VAGRANT_USER=ubuntu
if [ -e /home/vagrant ]; then
    # prefer vagrant user
    VAGRANT_USER=vagrant
fi
VAGRANT_HOME="/home/$VAGRANT_USER"

sudo --login -u $VAGRANT_USER /vagrant/Tools/scripts/install-prereqs-ubuntu-ros.sh

DOT_PROFILE=/home/$VAGRANT_USER/.profile
echo "source /vagrant/Tools/vagrant/shellinit-ros.sh" |
    sudo -u $VAGRANT_USER dd conv=notrunc oflag=append of=$DOT_PROFILE

sudo -u $VAGRANT_USER -s <<EOF
set -e
set -x

source /opt/ros/kinetic/setup.bash
mkdir -p ardupilot_ws/src
cd ardupilot_ws
catkin init
cd src
mkdir launch
cd launch
roscp mavros apm.launch apm.launch
perl -pe 's%/dev/ttyACM0:57600%udp://:14570\@127.0.0.1:14551\@14555%' -i apm.launch
EOF

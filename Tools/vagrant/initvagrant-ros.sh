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

# then install the ROS prerequisites (may want an install-prereqs-ubuntu-ros?)
# see: https://github.com/ArduPilot/ardupilot_wiki/blob/master/dev/source/docs/using-gazebo-simulator-with-sitl.rst

# Setup your computer to accept software from packages.osrfoundation.org.
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
apt-get update
apt-get install -y build-essential cmake git libboost-all-dev mercurial libcegui-mk2-dev libopenal-dev libswscale-dev libavformat-dev libavcodec-dev  libltdl3-dev libqwt-dev ruby libusb-1.0-0-dev libbullet-dev libhdf5-dev libgraphviz-dev libgdal-dev

# Install dependencies
wget https://bitbucket.org/osrf/release-tools/raw/default/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
ROS_DISTRO=dummy . /tmp/dependencies.sh
GAZEBO_BASE_DEPS=$(perl -pe 's:\\*::g' <<< $GAZEBO_BASE_DEPENDENCIES | xargs)
# GAZEBO_BASE_DEPS=$(echo "$GAZEBO_BASE_DEPS" | perl -pe 's/libignition-transport3-dev/libignition-transport-dev/')
# GAZEBO_BASE_DEPS=$(echo "$GAZEBO_BASE_DEPS" | perl -pe 's/libignition-math3-dev/libignition-math4-dev/' <<<"$GAZEBO_BASE_DEPS")
# GAZEBO_BASE_DEPS=$(echo "$GAZEBO_BASE_DEPS" | perl -pe 's/libignition-msgs0-dev/libignition-msgs-dev/' <<<"$GAZEBO_BASE_DEPS")
GAZEBO_BASE_DEPS=$(echo "$GAZEBO_BASE_DEPS" | perl -pe 's/libsdformat5-dev/libsdformat6-dev/' <<<"$GAZEBO_BASE_DEPS")
apt-get install -y $GAZEBO_BASE_DEPS

BASE_DEPS=$(perl -pe 's:\\*::g' <<< $BASE_DEPENDENCIES | xargs)
apt-get install -y $BASE_DEPS

GAZEBO_HOME="/home/$VAGRANT_USER/gazebo_ws"
# Make a gazebo workspace
sudo -u $VAGRANT_USER -s <<EOF
mkdir "$GAZEBO_HOME"
EOF


# Build and install Ignition Cmakes
sudo -u $VAGRANT_USER -s <<EOF
set -e
set -x

hg clone https://bitbucket.org/ignitionrobotics/ign-cmake $GAZEBO_HOME/ign-cmake
cd $GAZEBO_HOME/ign-cmake
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j4
EOF

cd $GAZEBO_HOME/ign-cmake/build
make install


# Build and install Ignition Maths
sudo -u $VAGRANT_USER -s <<EOF
set -e
set -x

hg clone https://bitbucket.org/ignitionrobotics/ign-math $GAZEBO_HOME/ign-math
cd $GAZEBO_HOME/ign-math
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j4
EOF

cd $GAZEBO_HOME/ign-math/build
make install


# Build and install Ignition Msgs
apt-get install libprotobuf-dev protobuf-compiler libprotoc-dev libignition-math4-dev

sudo -u $VAGRANT_USER -s <<EOF
set -e
set -x

hg clone https://bitbucket.org/ignitionrobotics/ign-msgs $GAZEBO_HOME/ign-msgs
cd $GAZEBO_HOME/ign-msgs
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j4
EOF

cd $GAZEBO_HOME/ign-msgs/build
make install

# Build and install Ignition Tools
sudo -u $VAGRANT_USER -s <<EOF
set -e
set -x

hg clone https://bitbucket.org/ignitionrobotics/ign-tools $GAZEBO_HOME/ign-tools
cd $GAZEBO_HOME/ign-tools
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j4
EOF

cd $GAZEBO_HOME/ign-tools/build
make install


# Build and install SDFormat
sudo -u $VAGRANT_USER -s <<EOF
set -e
set -x

hg clone https://bitbucket.org/osrf/sdformat $GAZEBO_HOME/sdformat
cd $GAZEBO_HOME/sdformat
hg checkout sdf5
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j4
EOF

cd $GAZEBO_HOME/sdformat/build
make install

# Build and install Gazebo

# solve random linking issues:
sudo ln -s /etc/alternatives/libblas.so-x86_64-linux-gnu /usr/lib/libblas.so
sudo ln -s /etc/alternatives/liblapack.so-x86_64-linux-gnu /usr/lib/liblapack.so

sudo -u $VAGRANT_USER -s <<EOF
set -e
set -x

hg clone https://bitbucket.org/osrf/gazebo $GAZEBO_HOME/gazebo
cd $GAZEBO_HOME/gazebo
hg checkout ardupilot
mkdir build
cd build
cmake ../
make -j4 # NOTE: This will take a long time! (~1.5 hours)
EOF

# /usr/local/lib must be available for linking:
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >>$VAGRANT_HOME/.bashrc

cd $GAZEBO_HOME/gazebo/build
make install

apt-get install -u ubuntu-desktop

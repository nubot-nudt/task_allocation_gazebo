#!/bin/bash

# Copyright (C) 2012-2015 Open Source Robotics Foundation
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description:
# This script installs gazebo onto an Ubuntu system.

codename=`lsb_release -sc`

# Make sure we are running a valid Ubuntu distribution
case $codename in
  "trusty" | "utopic")
  ;;
  *)
    echo "This script will only work on Ubuntu trusty or utopic"
    exit 0
esac

# Add the OSRF repository
if [ ! -e /etc/apt/sources.list.d/gazebo-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu ${codename} main\" > /etc/apt/sources.list.d/gazebo-latest.list"
fi

# Download the OSRF keys
has_key=`apt-key list | grep "OSRF deb-builder"`

echo "Downloading keys"
if [ -z "$has_key" ]; then
  wget --quiet http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
fi

 
# Update apt
echo "Retrieving packages"
sudo apt-get update -qq
echo "OK"

# Install gazebo
echo "Installing Gazebo"
sudo apt-get install gazebo5 libgazebo5-dev

echo "Complete."
echo "Type gazebo to start the simulator."

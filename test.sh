#!/bin/bash

if [ "$EUID" -ne 0 ]; then
    echo -e "\033[31mThis script is not running with sudo privileges.\033[0m"
    echo -e "\033[31mPlease re-run with 'sudo'. Exit...\033[0m"
    exit
fi

# Install BoostSHMTemplate
echo -e "\033[32mInstalling BoostSHMTemplate...\033[0m"
cd ./BoostSHMTemplate/

dir_to_remove='build/'

# Check if the directory exists
if [ -d "$dir_to_remove" ]; then
    echo "Directory '$dir_to_remove' exists. Removing..."
    sudo rm -rf "$dir_to_remove"
    echo "Directory removed."
fi

mkdir build && cd build
if ! cmake ..; then
    echo -e "\033[31mBST CMAKE BUILD FAILED\033[0m"
    exit
fi
if ! sudo make install; then
    echo -e "\033[31mBST C++ BUILD FAILED\033[0m"
    exit
fi
cd ../..

# Install the gazebo plugins
echo -e "\033[32mInstalling gazebo plugins...\033[0m"
cd ./GazeboInterface/
if ! sudo cp -r gazebo_interface /usr/local/lib/; then
    echo -e "\033[31mGAZEBO PLUGINS INSTALLATION FAILED\033[0m"
    exit
fi
cd ../..

echo -e "\033[32mUpdating GAZEBO_PLUGIN_PATH...\033[0m"
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/local/lib/gazebo_interface"  >>  ~/.bashrc
source ~/.bashrc

# Install the gazebopy python library
echo -e "\033[32mInstalling gazebopy...\033[0m"
cd gazebopy
pip3 install .
cd ../..

echo -e "\033[32mInstallation complete!\033[0m"

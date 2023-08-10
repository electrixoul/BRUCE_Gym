echo "Installing BRUCE Simulation Interface..."
sleep 2

# Install Eigen3
echo "Installing Eigen3..."
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build
cd build
if ! cmake ..; then
    echo "Eigen3 CMAKE BUILD FAILED"
    exit
fi
if ! sudo make install; then
    echo "Eigen3 C++ BUILD FAILED"
    exit
fi
cd ../..
rm -rf eigen

# Install BoostSHMTemplate
"Installing BoostSHMTemplate..."
cd ./BoostSHMTemplate/build/
if ! sudo cmake --install .; then
    echo "BoostSHMTemplate INSTALLATION FAILED"
    exit
fi
cd ../..

# Install the gazebo plugins
"Installing gazebo plugins..."
cd ./GazeboInterface/build/
if ! sudo cmake --install .; then
    echo "GAZEBO PLUGINS INSTALLATION FAILED"
    exit
fi
cd ../..

echo "Updating GAZEBO_PLUGIN_PATH..."
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/local/lib/gazebo_interface"  >>  ~/.bashrc
source ~/.bashrc

# Install the gazebopy python library
echo "Installing gazebopy..."
cd gazebopy
pip3 install .



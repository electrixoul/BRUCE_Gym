# BRUCE Simulation

BRUCE simulaiton can __ONLY__ run on Linux systems.

This repo is built for and tested in Ubuntu 22.04.1 x86_64. We do not guarentee its performance on other setup.


## Dependencies

Make sure you have the following dependencies correctly installed/configured in your system

### 1. Python >= 3.6

Make sure you have Python 3.6 or above. You can check the version of your python with ```python -V```. 

Python 3.6 is the minimum requirement, and Python 3.8 is preferred.

### 2. Basic Python Packages

Make sure you have these basic packages installed: ```pip3 show numpy numba osqp termcolor scipy```

If not, install them:

```
pip3 install numpy numba osqp termcolor scipy
```

### 3. PySHMXtreme

[PySHMXtreme](https://github.com/Westwood-Robotics/PySHMXtreme) is a shared memory interface using numpy but _unfortunately_ does not work on Windows. 

Install as instructed.

### 4. Gazebo 11

Gazebo 11 is used as the main simulation environment. Please make sure you have the latest version of Gazebo 11. If not, please update/install.

[Install Gazebo using Ubuntu packages](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)

After successfully installing Gazebo 11, create a directory for custom gazebo models: ```mkdir ~/.gazebo/models```

Then copy and paste ```export GAZEBO_PLUGIN_PATH="$GAZEBO_PLUGIN_PATH:/usr/local/lib/gazebo_interface"``` into '~/.bashrc', and then run 'source ~/.bashrc'.

Sudo permissions might be required.


## Interface Installation

Continue with the following installation procedure once you can confirm that your system has satisfied the above requirement.

### 1. Dependencies

Make sure that you have access to the following dependencies:

#### a. [CMake](https://cmake.org/)

You will need to install CMake first in the following steps.

#### b. [Eigen3](https://gitlab.com/libeigen/eigen)

Eigen3 is required to run BRUCE simulation. You do not need to manually install it, as it will be installed automatically in the following steps. Just make sure you have access to it is sufficient.

### 2. Installation

Create a copy of this repo in your local machine, and cd into the main directory of this repo, then run ```./install.sh```. The installation should be automatic.


## Operation

## Misc

Some random hints:
To uninstall packages installed with cmake --install:
While in the build folder, run: ```xargs rm < install_manifest.txt```



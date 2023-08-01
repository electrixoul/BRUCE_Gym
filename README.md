# BRUCE_Gazebointerface

Build for and tested in Ubuntu 22.04.1 x86_64

To install:
cd into Gazebointerface/build/ then

```
sudo cmake --install .
```

To uninstall:
cd into Gazebointerface/build/ then
```
xargs rm < install_manifest.txt
```

To set Gazebo interface plugin path:
cd into the folder where the .so files are located, then do
```
export GAZEBO_PLUGIN_PATH=$PWD
```

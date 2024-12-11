# Proximity plugin

This contains the code that's necessary to create a Gazebo
system plugin for proximity check

## Build

cd proximity_plugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `ProximityPlugin` library under `build`.

## Run

~~~
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~
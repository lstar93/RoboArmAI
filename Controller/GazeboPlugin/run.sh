#!/bin/bash

arg=""

if [ "$1" = "verbose" ]; then
    arg="--verbose"
fi

if [ -f "libroboarm_plugin.so" ]; then
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${PWD%/*/*}/3DModel/
    export GAZEBO_IP=127.0.0.1
    export LD_LIBRARY_PATH=${PWD}
    gazebo $arg roboarm.world
else
    echo "Can't fing plugin library, please run make.sh to build first!"
fi
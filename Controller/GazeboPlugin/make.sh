#!/bin/bash

if [ "$1" = "clean" ]; then
    rm -rf build
    if [ -f "libroboarm_plugin.so" ]; then
        rm -rf libroboarm_plugin.so
    fi 

    if [ -f "pos" ]; then
        rm -rf pos
    fi 
    exit 0
fi

if [ ! -d "build" ]; then
    mkdir build
    cd build

    if [ "$1" = "verbose" ] || [ "$2" = "verbose" ]; then
        verbose="-DPRINT_VERBOSE=ON"
    fi

    if [ "$1" = "Release" ]; then
        cmake -DCMAKE_BUILD_TYPE=Release $verbose ../ 
    else
        cmake -DCMAKE_BUILD_TYPE=Debug $verbose ../
    fi
else
    cd build
fi

make -j4

if [ -f "libroboarm_plugin.so" ]; then
    cp libroboarm_plugin.so ../
fi

if [ -f "pos" ]; then
    cp pos ../
fi
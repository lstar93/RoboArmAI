#!/bin/bash

if [ "$1" = "clean" ]; then
    rm -rf build
    if [ -f "libroboarm_plugin.so" ]; then
        rm -rf libroboarm_plugin.so
    fi 
    exit 0
fi

if [ -d "build" ]; then
    rm -rf build
fi

mkdir build
cd build

if [ "$1" = "Release" ]; then
    cmake -DCMAKE_BUILD_TYPE=Release ../ 
else
    cmake -DCMAKE_BUILD_TYPE=Debug ../
fi

make -j4

if [ -f "libroboarm_plugin.so" ]; then
    cp libroboarm_plugin.so ../
fi
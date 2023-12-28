#!/bin/bash

if [ ! -d "build" ]; then
    mkdir build
fi

cmake -S . -B ./build

make -C ./build

cp ./build/task_1 ./task_1
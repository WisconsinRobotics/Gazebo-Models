#!/bin/bash


source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PWD/build
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$PWD/models
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$PWD

gazebo -u worlds/example_amelio.world --verbose

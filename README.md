# Gazebo-Models
A simulated model of various robots made by Wisconsin Robotics using the popular open source enviorment Gazebo.

## Requirements
The simulation environment Gazebo is required for this repo. To install:

```
sudo apt install gazebo7 libgazebo7-dev
```
At the making of this document Gazebo 7 was the lastest verison released, your exact commands may vary. To see all packages run:
```
sudo apt-cache search gazebo
```

## Dependecnies
Currently our robot models require BadgerCommandLibrary (BCL) for sending and recieving packets and InverseKinematices. So you must the BCL and IK repos in one directory above of Gazebo-Models.

## Building
Gazebo is only supported on Ubuntu Linux. To build:

```
mkdir build
cd build
cmake ..
make
```

## Running
Currently all that is needed to run the simulation is to run `launch` in the top level from the terminal.

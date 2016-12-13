#pragma once


#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <stdio.h>

#include <vector>

#include "Coordinate.h"
#include "Region.h"

#define MAX_MEASUREMENT_NODES 360
#define MAX_RELIABLE_DISTANCE 6000
#define DELIMITER 0xff

class LaserRangeFinder
{
public:
    LaserRangeFinder();
    ~LaserRangeFinder();

    bool Initialize(void);
    bool RefreshData(void);
    void DebugDataDump(void);
	void DataPacker(std::vector<char>&);

private:
    gazebo::sensors::RaySensorPtr device;
    std::vector<Coordinate> frontCoordinates;
    std::vector<Region> frontRegions;
};


#pragma once

#include <vector>
#include <gazebo/sensors/sensors.hh>
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
    sensors::RaySensorPtr device;
    std::vector<Coordinate> frontCoordinates;
    std::vector<Region> frontRegions;
};


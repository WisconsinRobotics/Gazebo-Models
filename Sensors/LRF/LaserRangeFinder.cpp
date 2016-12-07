#define _USE_MATH_DEFINES

#include <iostream>
#include <Windows.h>
#include <algorithm>
#include <cmath>

#include "LaserRangeFinder.h"
#include "Line.h"

namespace LRF = rp::standalone::rplidar;

LaserRangeFinder::LaserRangeFinder()
{
}

LaserRangeFinder::~LaserRangeFinder()
{
}

bool LaserRangeFinder::Initialize(void)
{
    if ((this->device = std::dynamic_pointer_cast<sensors::RaySensor>(
         sensors::SensorManager::Instance()->GetSensor("laser")))== NULL)
    {
			std::cout << "COULD NOT FIND LASER SENSOR" << std::endl;
            return false
	}

    return true;
}

bool LaserRangeFinder::RefreshData(void)
{
    size_t count;

    if (!this->device)
        return false;

    count = MAX_MEASUREMENT_NODES;

    std::vector<double> ranges;
    this->device->Ranges(ranges);

    // clean house for new dataset
    this->frontCoordinates.clear();

    for (unsigned int i = 0; i < count; i++)
    {
        float angle_deg = i;
        float angle_rad = angle_deg * M_PI / 180.0f;
        float distance = ranges[i];

        // filter out garbage - verify
        if (distance == 0 || distance > MAX_RELIABLE_DISTANCE)
            continue;

        // front side
        if ((angle_deg >= 0 && angle_deg <= 90) || (angle_deg >= 270 && angle_deg <= 360))
            this->frontCoordinates.push_back(Coordinate(angle_rad, distance, CoordSystem::Polar));
    }

    // sort vectors
	// makes the coordinates from 270 - 360 degrees first in the list and the coordinates from 0 - 90 degrees last
    std::sort(this->frontCoordinates.begin(), this->frontCoordinates.end(), [](const Coordinate& a, const Coordinate& b) { return a.X < b.X; });

    // update regions
    this->frontRegions.clear();
    Region::GetRegionsFromCoordinateList(this->frontRegions, this->frontCoordinates);
    return true;
}

void LaserRangeFinder::DebugDataDump(void)
{
    std::cout << "==== Begin Dump ====" << std::endl;
    std::cout << "==== Front Dump ====" << std::endl;
    for (Coordinate c : this->frontCoordinates)
        std::cout << "X: " << c.X << " | Y: " << c.Y << std::endl;
    
    std::cout << "==== Region Dump ===" << std::endl;
    for (Region r : this->frontRegions)
    {
        std::cout << "==== Begin Region ====" << std::endl;
        auto coords = r.GetReducedCoordinates();
        for (Coordinate c : coords)
            std::cout << "X: " << c.X << " | Y: " << c.Y << std::endl;
        std::cout << "==== End Region ====" << std::endl;
    }

    std::cout << "===== End Dump =====" << std::endl;
}

void LaserRangeFinder::DataPacker(std::vector<char>& byteVector)
{
	// add a header BABE (after the pig)
	byteVector.push_back(0xBA);
	byteVector.push_back(0xBE);

	// add size of total vector
	short size = 0;
	size += 2; // num bytes for header
	size += 2; // num bytes for size of vector
	size += 1; // num bytes for num of regions in vector

	for (int r = 0; r < this->frontRegions.size(); r++) 
	{
		size += 1; // num bytes for num coords in region
		size += 4 * this->frontRegions[r].GetReducedCoordinates().size(); // 4 bytes per coord
	}

	char sizeH = (char)(size >> sizeof(char));
	char sizeL = (char)size;

	byteVector.push_back(sizeH);
	byteVector.push_back(sizeL);

	// add regions and their coords
	byteVector.push_back(this->frontRegions.size());

	for (int r = 0; r < this->frontRegions.size(); r++)
	{
		Region region = this->frontRegions[r];

		byteVector.push_back(region.GetReducedCoordinates().size());

		for (int c = 0; c < region.GetReducedCoordinates().size(); c++)
		{
			Coordinate coord = region.GetReducedCoordinates()[c];

			byteVector.push_back((char)((int)coord.X >> 8));
			byteVector.push_back((char)((int)coord.X & 0xff));
			byteVector.push_back((char)((int)coord.Y >> 8));
			byteVector.push_back((char)((int)coord.Y & 0xff));
		}
	}
}




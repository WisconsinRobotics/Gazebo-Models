#pragma once

#include <vector>
#include "Coordinate.h"

// XXX: TUNE ME
#define REGION_GAP 10
#define RDP_THRESHOLD 5.0
#define OBJECT_SEPARATION_DISTANCE 600 // 24" - estimated width of turtlebot - change to width of robot as needed

class Region
{
public:
    ~Region();
    static void GetRegionsFromCoordinateList(std::vector<Region>&, const std::vector<Coordinate>&);
    const std::vector<Coordinate>& GetReducedCoordinates(void);

private:
    Region(const std::vector<Coordinate>&);
    Region(const std::vector<Coordinate>&, int, int);
    void RunRamerDouglasPeucker(void);
    void SplitLine(std::vector<Coordinate>&, const std::vector<Coordinate>&, int, int);

    std::vector<Coordinate> reduced;
};


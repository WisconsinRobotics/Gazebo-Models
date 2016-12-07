#define _USE_MATH_DEFINES

#include "Region.h"
#include "Line.h"

#include <cmath>
#include <assert.h>

Region::Region(const std::vector<Coordinate>& all_points)
{
    this->reduced = all_points;
    RunRamerDouglasPeucker();
}

// Note: end index is inclusive
Region::Region(const std::vector<Coordinate>& all_points, int start, int end)
{
    std::vector<Coordinate> subset(all_points.begin() + start, all_points.begin() + end + 1);
    this->reduced = subset;
    RunRamerDouglasPeucker();
}

Region::~Region()
{
}

// assumes coordinate vector is sorted
void Region::GetRegionsFromCoordinateList(std::vector<Region>& regions, const std::vector<Coordinate>& coordinates)
{
    unsigned int region_start, i;
    size_t coord_size;

    // sanity check - more than 2 coordinates
    coord_size = coordinates.size();

    if (coord_size < 2)
    {
        return;
    }
    else if (coord_size == 2)
    {
        regions.push_back(Region(coordinates));
        return;
    }

    region_start = 0;
    for (i = 1; i < coord_size; i++)
    {
        float prevAngle = coordinates[i - 1].Theta;
        float prevDistance = coordinates[i - 1].R;

        float angle = coordinates[i].Theta;
        float distance = coordinates[i].R;

        // distance between point is found by law of cosines
        // though why can't we just use the cartesian converted coordinate?
        float gap_distance = (float) sqrt((prevDistance * prevDistance) + (distance * distance)
            - (2 * prevDistance * distance * cos(angle - prevAngle)));

        if (gap_distance > OBJECT_SEPARATION_DISTANCE)
        {
            regions.push_back(Region(coordinates, region_start, i));
            region_start = i + 1;
        }
    }

	if (regions.size() == 0)
		regions.push_back(Region(coordinates));

}

const std::vector<Coordinate>& Region::GetReducedCoordinates(void)
{
    return this->reduced;
}

void Region::RunRamerDouglasPeucker(void)
{
    std::vector<Coordinate> filtered;
    SplitLine(filtered, this->reduced, 0, this->reduced.size() - 1);
    this->reduced = filtered;
}

// NOTE: end index is INCLUSIVE
void Region::SplitLine(std::vector<Coordinate>& filtered, const std::vector<Coordinate>& raw, int start, int end)
{
    float max_distance = -1;
    int max_distance_index = -1;
    int num_elements;

    // sanity check
    assert(start <= end);

    num_elements = end - start;

    // can't run algorithm on fewer than 2 points
    if (num_elements < 2)
    {
        return;
    }
    else if (num_elements == 2)
    {
        // if there are only 2 elements - we're done, don't do anything
        filtered.push_back(raw[start]);
        filtered.push_back(raw[end]);
        return;
    }

    // get in between point that is furthest away
    for (int i = start; i < end; i++)
    {
        float dist_from_first = Line::DistFromLine(Line(raw[start], raw[end]), raw[i]);
        if (dist_from_first > max_distance)
        {
            max_distance = dist_from_first;
            max_distance_index = i;
        }
    }

    // deviation from threshold OK
    if (max_distance < RDP_THRESHOLD)
    {
        filtered.push_back(raw[start]);
        filtered.push_back(raw[end]);
        return;
    }

    SplitLine(filtered, raw, start, max_distance_index - 1); // left side
    SplitLine(filtered, raw, max_distance_index, end); // right side
}

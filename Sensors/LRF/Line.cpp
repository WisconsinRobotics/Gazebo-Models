#include "Line.h"
#include <cmath>

Line::Line(Coordinate start, Coordinate end)
{
    this->start = Coordinate(start);
    this->end = Coordinate(end);
    this->slope = ((float) end.Y - start.Y) / ((float) end.X - start.X);
    this->intercept = start.Y - (this->slope * start.X);
}

Line::Line(const Line& line)
{
    this->start = Coordinate(line.start);
    this->end = Coordinate(line.end);
    this->slope = ((float)end.Y - start.Y) / ((float)end.X - start.X);
    this->intercept = start.Y - (this->slope * start.X);
}

Line::~Line()
{
}

float Line::GetSlope() const
{
    return slope;
}

float Line::GetIntercept() const
{
    return intercept;
}

bool Line::IsBetween(const Coordinate point) const
{
    return (this->start.X < point.X) && (point.X < this->end.X);
}

float Line::DistFromLine(const Line &line, const Coordinate point)
{
    float dist;
    dist = (float) (((line.slope * point.X) - point.Y + line.intercept) / sqrt(pow(line.slope, 2.0) + 1.0));
    return (dist < 0) ? -dist : dist;
}

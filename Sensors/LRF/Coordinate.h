#pragma once

enum CoordSystem
{
    Cartesian, Polar
};

struct Coordinate
{
    Coordinate(void);
    Coordinate(float, float, CoordSystem);
    Coordinate(const Coordinate&);

    float X, Y, R, Theta;
};


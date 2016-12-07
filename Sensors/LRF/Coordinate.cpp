#define _USE_MATH_DEFINES

#include "Coordinate.h"
#include <cmath>


Coordinate::Coordinate(void)
{
    this->X = 0;
    this->Y = 0;
    this->R = 0;
    this->Y = 0;
}

Coordinate::Coordinate(float a, float b, CoordSystem system)
{
    if (system == Cartesian)
    {
		this->X = a;
		this->Y = b;
        this->R = sqrt(a * a + b * b);
        this->Theta = atan2f(b, a);
    }
    else
    {
		this->X = b * (float)sin(a);
		this->Y = b * (float)cos(a);
		this->Theta = a;
		this->R = b;
    }
}

Coordinate::Coordinate(const Coordinate& coord)
{
    this->X = coord.X;
    this->Y = coord.Y;
    this->R = coord.R;
    this->Theta = coord.Theta;
}

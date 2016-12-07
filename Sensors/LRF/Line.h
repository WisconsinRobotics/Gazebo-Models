#pragma once

#include "Coordinate.h"

class Line
{
public:
    Line(Coordinate, Coordinate);
    Line(const Line&);
    ~Line();
    
    float GetSlope() const;
    float GetIntercept() const;
    bool IsBetween(const Coordinate) const;

    static float DistFromLine(const Line&, const Coordinate);

private:
    Coordinate start, end;
    float slope, intercept;
};




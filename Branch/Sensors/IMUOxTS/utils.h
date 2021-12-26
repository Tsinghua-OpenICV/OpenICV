#ifndef UTILS_H
#define UTILS_H

#include "math.h"
#include "proj_api.h"
#include "geos/geom/Coordinate.h"
#include <cmath>
#include <iostream>

namespace P7_utils
{

    struct Quaternion
    {
        double w, x, y, z;
    };

    struct EulerAngles
    {
        double roll, pitch, yaw;
    };

    geos::geom::Coordinate utm2longlat(double x, double y);

    geos::geom::Coordinate longlat2utm(double x, double y);

    EulerAngles ToEulerAngles(Quaternion q);

}

#endif
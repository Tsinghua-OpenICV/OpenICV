#include "utils.h"

namespace P7_utils
{
    geos::geom::Coordinate utm2longlat(double x, double y)
    {
        projPJ pj_longlat;
        projPJ pj_utm;
        // Initialize LONGLAT projection with epsg:4326
        if (!(pj_longlat = pj_init_plus("+init=epsg:4326")))
        {
            std::cout << "pj_init_plus error: longlat";
        }

        // Initialize UTM projection with epsg:32650
        if (!(pj_utm = pj_init_plus("+init=epsg:32650")))
        {
            std::cout << "pj_init_plus error: utm";
        }

        // Transform UTM projection into LONGLAT projection
        int p = pj_transform(pj_utm, pj_longlat, 1, 1, &x, &y, NULL);

        x = x * 180 / M_PI;
        y = y * 180 / M_PI;

        // Return values as coordinate
        return geos::geom::Coordinate(x, y); // x 为经度， y 为纬度
    }

    geos::geom::Coordinate longlat2utm(double x, double y) // x 为经度， y 为纬度
    {
        x = x /180.0 * M_PI;
        y = y /180.0 * M_PI;
        projPJ pj_longlat;
        projPJ pj_utm;
        // Initialize LONGLAT projection with epsg:4326
        if (!(pj_longlat = pj_init_plus("+init=epsg:4326")))
        {
            std::cout << "pj_init_plus error: longlat";
        }

        // Initialize UTM projection with epsg:32650
        if (!(pj_utm = pj_init_plus("+init=epsg:32650")))
        {
            std::cout << "pj_init_plus error: utm";
        }

        int p = pj_transform(pj_longlat, pj_utm, 1, 1, &x, &y, NULL);

        // Return values as coordinate
        return geos::geom::Coordinate(x, y);
    }

    EulerAngles ToEulerAngles(Quaternion q)
    {
        EulerAngles angles;

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        angles.roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles.pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        angles.yaw = std::atan2(siny_cosp, cosy_cosp);

        return angles;
    }

}
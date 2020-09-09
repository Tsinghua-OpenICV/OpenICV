#ifndef CAR_OBSF_ROS_UTIL_H
#define CAR_OBSF_ROS_UTIL_H

#include <pcl/point_cloud.h>
#include "../objects/obsf_define.h"

namespace tsinghua{
namespace dias{
namespace fusion{

template <typename PointT> 
bool is_xy_point_in_2d_xy_polygon(
    const PointT& point, const pcl::PointCloud<PointT>& polygon) {
    bool in_poly = false;
    double x1, x2, y1, y2;
 
    int nr_poly_points = static_cast<int>(polygon.points.size());
    // start with the last point to make the check last point<->first point the first one
    double xold = polygon.points[nr_poly_points - 1].x;
    double yold = polygon.points[nr_poly_points - 1].y;
    for (int i = 0; i < nr_poly_points; i++) {
        double xnew = polygon.points[i].x;
        double ynew = polygon.points[i].y;
        if (xnew > xold) {
            x1 = xold;
            x2 = xnew;
            y1 = yold;
            y2 = ynew;
        } else {
            x1 = xnew;
            x2 = xold;
            y1 = ynew;
            y2 = yold;
        }
 
        if ((x1 < point.x) == (point.x <= x2) 
                && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1)) {
            in_poly = !in_poly;
        }
        xold = xnew;
        yold = ynew;
    }
 
    return in_poly;
}

template <typename PointT>
bool is_xy_point_in_hdmap(const PointT &p, 
        std::vector<typename pcl::PointCloud<PointT> > polygons) {
    bool in_flag = false;
    for (int j = 0; j < polygons.size(); j++) {
        if (is_xy_point_in_2d_xy_polygon<PointT>(p, polygons[j])) {
            in_flag = true;
            break;
        }
    }
    return in_flag;
}

template <typename PointT> 
void add_offset(PointT &point) {
    point.x += tsinghua::dias::fusion::xoffset;
    point.y += tsinghua::dias::fusion::yoffset;
    point.z += tsinghua::dias::fusion::zoffset;
}

template <typename PointT>
void sub_offset(PointT &point) {
    point.x -= tsinghua::dias::fusion::xoffset;
    point.y -= tsinghua::dias::fusion::yoffset;
    point.z -= tsinghua::dias::fusion::zoffset;
} 

void add_offset(tsinghua::dias::fusion::objects::OBFSVector3d &point) {
    point._x += tsinghua::dias::fusion::xoffset;
    point._y += tsinghua::dias::fusion::yoffset;
    point._z += tsinghua::dias::fusion::zoffset;    
} 

void sub_offset(tsinghua::dias::fusion::objects::OBFSVector3d &point) {
    point._x -= tsinghua::dias::fusion::xoffset;
    point._y -= tsinghua::dias::fusion::yoffset;
    point._z -= tsinghua::dias::fusion::zoffset;
}

void add_offset(Eigen::Vector3d &point) {
    point(0) += tsinghua::dias::fusion::xoffset;
    point(1) += tsinghua::dias::fusion::yoffset;
    point(2) += tsinghua::dias::fusion::zoffset;  
}

void sub_offset(Eigen::Vector3d &point) {
    point(0) -= tsinghua::dias::fusion::xoffset;
    point(1) -= tsinghua::dias::fusion::yoffset;
    point(2) -= tsinghua::dias::fusion::zoffset;
}

} //
} //
} //

#endif

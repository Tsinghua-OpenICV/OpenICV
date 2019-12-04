#ifndef _common
#define _common

#include <cmath>

#include <pcl/point_types.h>

#include <vector>
#include <iostream>


#include <opencv/cv.h>

#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <pcl/io/pcd_io.h>
#include <sstream>
#include <fstream>
#include <Eigen/Core>

#include <iostream>

#include <string>


#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>


#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <boost/thread/thread.hpp>

typedef ::pcl::PointXYZI PointType;

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}


struct scanRegistrationBack{

  ::pcl::PointCloud<PointType>::Ptr laserCloud;
  ::pcl::PointCloud<PointType> cornerPointsSharp;
  ::pcl::PointCloud<PointType> cornerPointsLessSharp;
  ::pcl::PointCloud<PointType> surfPointsFlat;
  ::pcl::PointCloud<PointType> surfPointsLessFlat;
  ::pcl::PointCloud<::pcl::PointXYZ> imuTrans;

};

struct laserOdometryBack{

  float transformSum[6];

  ::pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  ::pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  ::pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
  
};


struct laserMappingBack{

  float transformAftMapped[6];

  ::pcl::PointCloud<PointType>::Ptr laserCloudSurround;
  ::pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
  
};

struct maintenanceBack{

  float transformMapped[6];


  
};





#endif

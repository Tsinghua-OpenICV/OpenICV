/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    Interface for converting a Pandar40 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

    @author Jack O'Quin
 *  @author Yang Sheng
*/

#ifndef _PANDAR_POINTCLOUD_COLORS_H_
#define _PANDAR_POINTCLOUD_COLORS_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pandar_pointcloud/point_types.h>

namespace pandar_pointcloud
{
  // shorter names for point cloud types in this namespace
  typedef pandar_pointcloud::PointXYZIR PPoint;
  typedef pcl::PointCloud<PPoint> PPointCloud;

  class RingColors
  {
  public:

    RingColors(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~RingColors() {}

  private:

    void convertPoints(const PPointCloud::ConstPtr &inMsg);

    ros::Subscriber input_;
    ros::Publisher output_;
  };

} // namespace pandar_pointcloud

#endif // _PANDAR_POINTCLOUD_COLORS_H_

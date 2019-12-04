/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet converts a Pandar40 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "colors.h"

namespace pandar_pointcloud
{
  class RingColorsNodelet: public nodelet::Nodelet
  {
  public:

    RingColorsNodelet() {}
    ~RingColorsNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<RingColors> colors_;
  };

  /** @brief Nodelet initialization. */
  void RingColorsNodelet::onInit()
  {
    colors_.reset(new RingColors(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace pandar_pointcloud


// Register this plugin with pluginlib.  Names must match nodelet_pandar.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(pandar_pointcloud::RingColorsNodelet, nodelet::Nodelet);

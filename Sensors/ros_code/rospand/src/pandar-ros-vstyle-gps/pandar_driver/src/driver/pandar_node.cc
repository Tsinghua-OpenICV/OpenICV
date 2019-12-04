/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Pandar40 3D LIDARs.
 */

#include <ros/ros.h>
#include "driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pandar_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  pandar_driver::PandarDriver dvr(node, private_nh);

  // loop until shut down or end of file
  while(ros::ok() && dvr.poll())
    {
      ros::spinOnce();
    }

  return 0;
}

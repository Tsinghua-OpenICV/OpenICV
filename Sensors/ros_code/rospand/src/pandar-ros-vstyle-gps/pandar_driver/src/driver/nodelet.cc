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
 *  ROS driver nodelet for the Pandar40 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "driver.h"

namespace pandar_driver
{

class DriverNodelet: public nodelet::Nodelet
{
public:

  DriverNodelet():
    running_(false)
  {}

  ~DriverNodelet()
  {
    if (running_)
      {
        NODELET_INFO("shutting down driver thread");
        running_ = false;
        deviceThread_->join();
        NODELET_INFO("driver thread stopped");
      }
  }

private:

  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_;               ///< device thread is running
  boost::shared_ptr<boost::thread> deviceThread_;

  boost::shared_ptr<PandarDriver> dvr_; ///< driver implementation class
};

void DriverNodelet::onInit()
{
  // start the driver
  dvr_.reset(new PandarDriver(getNodeHandle(), getPrivateNodeHandle()));

  // spawn device poll thread
  running_ = true;
  deviceThread_ = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll()
{
  while(ros::ok())
    {
      // poll device until end of file
      running_ = dvr_->poll();
      if (!running_)
        break;
    }
  running_ = false;
}

} // namespace pandar_driver

// Register this plugin with pluginlib.  Names must match nodelet_pandar.xml.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(pandar_driver::DriverNodelet, nodelet::Nodelet);

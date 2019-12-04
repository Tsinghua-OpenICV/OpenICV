/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology Co., Ltd
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the pandar 3D LIDARs
 */

#ifndef _PANDAR_DRIVER_H_
#define _PANDAR_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <pandar_driver/input.h>
#include <pandar_driver/PandarNodeConfig.h>


namespace pandar_driver
{

class PandarDriver
{
public:

  PandarDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~PandarDriver() {}

  bool poll(void);

private:

  ///Callback for dynamic reconfigure
  void callback(pandar_driver::PandarNodeConfig &config,
              uint32_t level);

  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<pandar_driver::
              PandarNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
    double time_offset;              ///< time in seconds added to each pandar time stamp
  } config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  ros::Publisher gpsoutput_;

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};

} // namespace pandar_driver

#endif // _PANDAR_DRIVER_H_

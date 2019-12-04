/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Pandar40 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pandar_msgs/PandarScan.h>
#include <pandar_msgs/PandarGps.h>

#include "driver.h"
#include "convert.h"

namespace pandar_pointcloud
{

PandarDriver::PandarDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh ,  pandar_pointcloud::Convert *cvt)
{
  convert = cvt;
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("pandar"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  config_.model = "Pandar40";
  std::string model_full_name = std::string("Hesai") + config_.model;
  double packet_rate = 3000;                   // packet frequency (Hz)
  std::string deviceName(model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  int udp_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);

  // // Initialize dynamic reconfigure
  // srv_ = boost::make_shared <dynamic_reconfigure::Server<pandar_pointcloud::
  //   CloudNodeConfig> > (private_nh);
  // dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig>::
  //   CallbackType f;
  // f = boost::bind (&PandarDriver::callback, this, _1, _2);
  // srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  // using namespace diagnostic_updater;
  // diag_topic_.reset(new TopicDiagnostic("pandar_packets", diagnostics_,
  //                                       FrequencyStatusParam(&diag_min_freq_,
  //                                                            &diag_max_freq_,
  //                                                            0.1, 10),
  //                                       TimeStampStatusParam()));

  // open Pandar input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      // read data from packet capture file
      input_.reset(new pandar_pointcloud::InputPCAP(private_nh, udp_port,
                                                  packet_rate, dump_file));
    }
  else
    {
      // read data from live socket
      input_.reset(new pandar_pointcloud::InputSocket(private_nh, udp_port));
    }

  // raw packet output topic
  output_ =
    node.advertise<pandar_msgs::PandarScan>("pandar_packets", 10);
  // raw packet output topic
  gpsoutput_ =
    node.advertise<pandar_msgs::PandarGps>("pandar_gps", 1);
}

#define HS_LIDAR_L40_GPS_PACKET_SIZE (512)
#define HS_LIDAR_L40_GPS_PACKET_FLAG_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_YEAR_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_MONTH_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_DAY_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_HOUR_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_MINUTE_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_SECOND_SIZE (2)
#define HS_LIDAR_L40_GPS_ITEM_NUM (7)

//--------------------------------------
typedef struct HS_LIDAR_L40_GPS_PACKET_s{
    unsigned short flag;
    unsigned short year;
    unsigned short month;
    unsigned short day;
    unsigned short second;
    unsigned short minute;
    unsigned short hour;
    unsigned int fineTime;
//    unsigned char unused[496];
}HS_LIDAR_L40_GPS_Packet;

//-------------------------------------------------------------------------------
int HS_L40_GPS_Parse(HS_LIDAR_L40_GPS_Packet *packet , const unsigned char* recvbuf , const int len)
{
    if(len != HS_LIDAR_L40_GPS_PACKET_SIZE)
        return -1;

    int index = 0;
    packet->flag = (recvbuf[index] & 0xff)|((recvbuf[index + 1] & 0xff)<< 8);
    index += HS_LIDAR_L40_GPS_PACKET_FLAG_SIZE;
    packet->year = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += HS_LIDAR_L40_GPS_PACKET_YEAR_SIZE;
    packet->month = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += HS_LIDAR_L40_GPS_PACKET_MONTH_SIZE;
    packet->day = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += HS_LIDAR_L40_GPS_PACKET_DAY_SIZE;
    packet->second = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += HS_LIDAR_L40_GPS_PACKET_SECOND_SIZE;
    packet->minute = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += HS_LIDAR_L40_GPS_PACKET_MINUTE_SIZE;
    packet->hour = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10 + 8;
    index += HS_LIDAR_L40_GPS_PACKET_HOUR_SIZE;
    packet->fineTime = (recvbuf[index]& 0xff)| (recvbuf[index + 1]& 0xff) << 8 |
                       ((recvbuf[index + 2 ]& 0xff) << 16) | ((recvbuf[index + 3]& 0xff) << 24);
    return 0;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool PandarDriver::poll(void)
{
  int readpacket = config_.npackets / 3;
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  pandar_msgs::PandarScanPtr scan(new pandar_msgs::PandarScan);
  scan->packets.resize(readpacket);

  // Since the pandar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  for (int i = 0; i < readpacket; ++i)
    {
      while (true)
        {
          // keep reading until full packet received
          int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
          if (rc == 0) break;       // got a full packet?
          if (rc == 2)
          {
            // gps packet;
            HS_LIDAR_L40_GPS_Packet packet;
            if(HS_L40_GPS_Parse( &packet , &scan->packets[i].data[0] , HS_LIDAR_L40_GPS_PACKET_SIZE) == 0)
            {
              pandar_msgs::PandarGpsPtr gps(new pandar_msgs::PandarGps);
              gps->stamp = ros::Time::now();

              gps->year = packet.year;
              gps->month = packet.month;
              gps->day = packet.day;
              gps->hour = packet.hour;
              gps->minute = packet.minute;
              gps->second = packet.second;

              gps->used = 0;
              if(gps->year > 30 || gps->year < 17)
              {
                ROS_ERROR("Ignore wrong GPS data (year)%d" , gps->year);
                continue;
              }
              convert->processGps(*gps);
              gpsoutput_.publish(gps);
            }
            
          }
          if (rc < 0) return false; // end of file reached?
        }

        convert->pushLiDARData(scan->packets[i]);
    }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Pandar scan.");
  scan->header.stamp = scan->packets[readpacket - 1].stamp;
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);















  // notify diagnostics that a message has been published, updating
  // its status
  // diag_topic_->tick(scan->header.stamp);
  // diagnostics_.update();

  return true;
}

void PandarDriver::callback(pandar_pointcloud::CloudNodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

} // namespace pandar_driver

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2012,2015 The MITRE Corporation

 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  modified 2018 Jiang KUN

 *
 */



#ifndef PCL_IO_PANDAR_GRABBER_H_
#define PCL_IO_PANDAR_GRABBER_H_
#include "pcl/pcl_config.h"
#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <semaphore.h>
#include <pthread.h>
#include <boost/lockfree/queue.hpp>
#include <boost/atomic.hpp>
#include "rawdata.h"
#include "structpandar.hxx"
#define pandar_Grabber_toRadians(x) ((x) * M_PI / 180.0)

namespace pandar_pointcloud
{

  /** @brief pandar input base class */
  class Input
  {
  public:
    Input(Config_pandar& config,uint16_t port);
    virtual ~Input() {};

    /** @brief Read one pandar packet.
     *
     * @param pkt points to pandarPacket message
     *
     * @returns 0 if successful,
     *          -1 if end of file
     *          > 0 if incomplete packet (is this possible?)
     */
    virtual int getPacket(PandarPacket *pkt,
                          const double time_offset) = 0;

  protected:
    uint16_t port_;
    std::string devip_str_;
  };

  /** @brief Live pandar input from socket. */
  class InputSocket: public Input
  {
  public:
    InputSocket( Config_pandar& config, uint16_t port = DATA_PORT_NUMBER);
     virtual ~InputSocket();

    virtual int getPacket(PandarPacket *pkt, const double time_offset);
    void setDeviceIP( const std::string& ip );
  private:

  private:
    int sockfd_;
    in_addr devip_;
  };


  /** @brief pandar input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap, pandar's DSR software,
   * ethereal, wireshark, tcpdump, or the \ref vdump_command.
   * 
   */

  class InputPCAP: public Input
  {
  public:
  
    InputPCAP(Config_pandar& config,uint16_t port = DATA_PORT_NUMBER,
              double packet_rate = 0.0,
              std::string filename="",
              bool read_once=false,
              bool read_fast=false,
              double repeat_delay=0.0);
virtual ~InputPCAP();

    virtual int getPacket(PandarPacket *pkt, const double time_offset);
    void setDeviceIP( const std::string& ip );

  private:
    int packet_rate_;
    std::string filename_;
    pcap_t *pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
  };
 class Convert;
  class PandarDriver
{

  
public:

  PandarDriver(Config_pandar& config, Convert * convert);
  ~PandarDriver() {}

  bool poll(void);

private:

  ///Callback for dynamic reconfigure
  void callback(uint32_t level);

  ///Pointer to dynamic reconfigure service srv_

  // configuration parameters

Config_pandar config_;
  boost::shared_ptr<Input> input_;
  pandar_pointcloud::Convert * convert;
};
 
  

class Convert
{
public:

    Convert(Config_pandar & config);
    ~Convert() {}

    void DriverReadThread();
     void processGps(PandarGps &gpsMsg);
    void pushLiDARData(PandarPacket& packet);
       boost::shared_ptr<pandar_rawdata::PPointCloud>  getpointcloud( );

    int processLiDARData();


private:

    // void processScan(const PandarScanConstPtr &scanMsg);
    void processGps(const  PandarGpsConstPtr &gpsMsg);



    ///Pointer to dynamic reconfigure service srv_
   

    boost::shared_ptr<pandar_rawdata::RawData> data_;

    /// configuration parameters
    // typedef struct {
    //     int npackets;                    ///< number of packets to combine
    // } Config_conv;
    // Config_conv config_;

    time_t gps1;
    pandar_rawdata::gps_struct_t gps2;
    bool hasGps;

    unsigned int lastGPSSecond;
    int lidarRotationStartAngle;
      Config_pandar config_x;
    pandar_pointcloud::PandarDriver drv;
    uint64_t stamp_temp=0;

    pthread_mutex_t piclock;
    boost::mutex out_mutex_;
    boost::shared_ptr<pandar_rawdata::PPointCloud> outMsg;
    boost::shared_ptr<pandar_rawdata::PPointCloud> temMsg;



    sem_t picsem;
    std::list<PandarPacket> LiDARDataSet;
};







} // pandar_driver namespace



#endif /* PCL_IO_pandar_GRABBER_H_ */

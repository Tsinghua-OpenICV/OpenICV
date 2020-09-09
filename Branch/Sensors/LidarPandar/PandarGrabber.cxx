/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2012,2015 The MITRE Corporation
 *  Copyright(c) 2018 JIANG KUN

 *
 */


#include "PandarGrabber.h"
#include "OpenICV/Core/icvFunction.h"
#include <iostream>
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
using namespace std;
namespace pandar_pointcloud
{

  static const size_t packet_size =1240;



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

PointXYZIT tyep; 


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





  ////////////////////////////////////////////////////////////////////////
  // Input base class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number.
   */
  Input::Input(Config_pandar& config,uint16_t port):
    port_(port)
  {
   // private_nh.param("device_ip", devip_str_, std::string(""));
   devip_str_=config.ip_add;
      // if (!devip_str_.empty())
  //    ROS_INFO_STREAM("Only accepting packets from IP address: "
    //                  << devip_str_);
  }

  ////////////////////////////////////////////////////////////////////////
  // InputSocket class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   */
  InputSocket::InputSocket(Config_pandar& config,uint16_t port):
    Input(config, port)
  {
    sockfd_ = -1;
    
    if (!devip_str_.empty()) {
      inet_aton(devip_str_.c_str(),&devip_);
    }    

    // connect to Pandar UDP port
    //ROS_INFO_STREAM("Opening UDP socket: port " << port);
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1)
      {
        perror("socket");               // TODO: ROS_ERROR errno
        return;
      }
  
    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(port);          // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP
  
    if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
      {
        perror("bind");                 // TODO: ROS_ERROR errno
        return;
      }
  
    if (fcntl(sockfd_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
      {
        perror("non-block");
        return;
      }

  //  ROS_DEBUG("Pandar socket fd is %d\n", sockfd_);
  }

  /** @brief destructor */
  InputSocket::~InputSocket(void)
  {
    (void) close(sockfd_);
  }

// return : 0 - lidar
//          2 - gps
//          1 - error
  /** @brief Get one pandar packet. */
  int InputSocket::getPacket(PandarPacket *pkt, const double time_offset)
  {
    double time1 =icv::icvTime::now().time_since_epoch().count();
    int isgps = 0;
    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
      {
        // Unfortunately, the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.

        // poll() until input available
        do
          {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
              {
                if (errno != EINTR)
             //     ROS_ERROR("poll() error: %s", strerror(errno));
                return 1;
              }
            if (retval == 0)            // poll() timeout?
              {
              //  ROS_WARN("Pandar poll() timeout");
                return 1;
              }
            if ((fds[0].revents & POLLERR)
                || (fds[0].revents & POLLHUP)
                || (fds[0].revents & POLLNVAL)) // device error?
              {
                //ROS_ERROR("poll() reports Pandar error");
                return 1;
              }
          } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0],
                                  packet_size,  0,
                                  (sockaddr*) &sender_address,
                                  &sender_address_len);
       // ICV_LOG_INFO<<"try to connect"<<sender_address.sin_addr.s_addr;

        if (nbytes < 0)
          {
            if (errno != EWOULDBLOCK)
              {
                perror("recvfail");
                //ROS_INFO("recvfail");
                return 1;
              }
          }
        else if ((size_t) nbytes == packet_size)
          {

            //ICV_LOG_INFO<<"get packet SIZE CORRECT "<< (int)pkt->data;

            //for (int i=0;i<100;i++) ICV_LOG_INFO<< (int)pkt->data[i];
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if(devip_str_ != ""
               && sender_address.sin_addr.s_addr != devip_.s_addr)
              continue;
            else
              break; //done
          }
          else if ((size_t) nbytes == 512)
          {
            // GPS
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if(devip_str_ != ""
               && sender_address.sin_addr.s_addr != devip_.s_addr)
              continue;
            else
            {
              isgps = 1;
              break; //done
            }
          }


       // ROS_DEBUG_STREAM("incomplete Pandar packet read: "
               //          << nbytes << " bytes");
      }

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred. Add the time offset.
    double time2 = icv::icvTime::now().time_since_epoch().count();
    pkt->stamp = (time2 + time1) / 2.0 + time_offset;

    if(isgps)
    {
      return 2;
    }
    return 0;
  }

  ////////////////////////////////////////////////////////////////////////
  // InputPCAP class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
  InputPCAP::InputPCAP(Config_pandar& config, uint16_t port,
                       double packet_rate, std::string filename,
                       bool read_once, bool read_fast, double repeat_delay):
    Input( config,port),
    packet_rate_(packet_rate),
    filename_(filename)
  {
    pcap_ = NULL;  
    empty_ = true;

    // get parameters using private node handle
    //private_nh.param("read_once", read_once_, false);
    //private_nh.param("read_fast", read_fast_, false);
    //private_nh.param("repeat_delay", repeat_delay_, 0.0);

    //if (read_once_)
      //ROS_INFO("Read input file only once.");
    //if (read_fast_)
      //ROS_INFO("Read input file as quickly as possible.");
    //if (repeat_delay_ > 0.0)
      //ROS_INFO("Delay %.3f seconds before repeating input file.",
       //        repeat_delay_);

    // Open the PCAP dump file
    //ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL)
      {
      //  ROS_FATAL("Error opening Pandar socket dump file.");
        return;
      }

    std::stringstream filter;
    if( devip_str_ != "" )              // using specific IP?
      {
        filter << "src host " << devip_str_ << " && ";
      }
    filter << "udp dst port " << port;
    pcap_compile(pcap_, &pcap_packet_filter_,
                 filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
  }

  /** destructor */
  InputPCAP::~InputPCAP(void)
  {
    pcap_close(pcap_);
  }

// return : 0 - lidar
//          2 - gps
//          1 - error
  /** @brief Get one pandar packet. */
  int InputPCAP::getPacket(PandarPacket *pkt, const double time_offset)
  {
    struct pcap_pkthdr *header;
    const u_char *pkt_data;

    while (true)
      {
        int res;
        if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
          {
            // Skip packets not for the correct port and from the
            // selected IP address.
            if (!devip_str_.empty() &&
                (0 == pcap_offline_filter(&pcap_packet_filter_,
                                          header, pkt_data)))
              continue;

            // Keep the reader from blowing through the file.
          // if (read_fast_ == false)
            //  packet_rate_.sleep();
            
            memcpy(&pkt->data[0], pkt_data+42, packet_size);
            pkt->stamp = icv::icvTime::now().time_since_epoch().count(); // time_offset not considered here, as no synchronization required
            empty_ = false;

            if(header->caplen == ( 512 + 42) )
            {
              // ROS_ERROR("GPS");
              return 2;
            }

            else if (header->caplen == ( 1240 + 42) )
            {
              return 0;                   // success
            }

            // Wrong data , It's not the packet of LiDAR I think.
            continue;
          }

        if (empty_)                 // no data in file?
          {
          //  ROS_WARN("Error %d reading Pandar packet: %s", 
             //        res, pcap_geterr(pcap_));
            return -1;
          }

        if (read_once_)
          {
            //ROS_INFO("end of file reached -- done reading.");
            //return -1;
          }
        
        if (repeat_delay_ > 0.0)
          {
            //ROS_INFO("end of file reached -- delaying %.3f seconds.",
             //        repeat_delay_);
            usleep(rint(repeat_delay_ * 1000000.0));
          }

       // ROS_DEBUG("replaying Pandar dump file");

        // I can't figure out how to rewind the file, because it
        // starts with some kind of header.  So, close the file
        // and reopen it with pcap.
        pcap_close(pcap_);
        pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
        empty_ = true;              // maybe the file disappeared?
      } // loop back and try again
  }

/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////


PandarDriver::PandarDriver( Config_pandar &config,Convert *cvt)
{
  convert = cvt;
  // use private node handle to get parameters
  //private_nh.param("frame_id", config_.frame_id, std::string("pandar"));
  //std::string tf_prefix = tf::getPrefixParam(private_nh);
  //ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  //config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
  config_.frame_id=config.frame_id;
  // get model name, validate string, determine packet rate
  config_.model = config.model;//"Pandar40";
  std::string model_full_name = std::string("Hesai") + config_.model;
  double packet_rate = 3000;                   // packet frequency (Hz)
  std::string deviceName(model_full_name);
  if(config.rpm>0)config_.rpm=config.rpm;
  else config_.rpm=600;
  //private_nh.param("rpm", config_.rpm, 600.0);
  //ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  if(config.npackets<=0)config_.npackets = (int) ceil(packet_rate / frequency);
  else config_.npackets = config.npackets;
  //private_nh.getParam("npackets", config_.npackets);
  
  //ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  dump_file=config.pcapfile;
  //private_nh.param("pcap", dump_file, std::string(""));

  int udp_port;
  if(config.ip_port>0)udp_port=config.ip_port;
  else udp_port=(int)DATA_PORT_NUMBER;

  config_.time_offset=config.time_offset;
  // // Initialize dynamic reconfigure
  // srv_ = boost::make_shared <dynamic_reconfigure::Server<pandar_pointcloud::
  //   CloudNodeConfig> > (private_nh);
  // dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig>::
  //   CallbackType f;
  // f = boost::bind (&PandarDriver::callback, this, _1, _2);
  // srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
 // diagnostics_.setHardwareID(deviceName);
  // const double diag_freq = packet_rate/config_.npackets;
  // diag_max_freq_ = diag_freq;
  // diag_min_freq_ = diag_freq;
  // ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

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
      input_.reset(new pandar_pointcloud::InputPCAP(config,udp_port,
                                                  packet_rate, dump_file));
    }
  else
    {
      // read data from live socket
      input_.reset(new pandar_pointcloud::InputSocket(config,udp_port));
    }

  // raw packet output topic
  // output_ =
  //   node.advertise<pandar_msgs::PandarScan>("pandar_packets", 10);
  // // raw packet output topic
  // gpsoutput_ =
  //   node.advertise<pandar_msgs::PandarGps>("pandar_gps", 1);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool PandarDriver::poll(void)
{
  int readpacket = config_.npackets / 3;
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.

  PandarScanPtr scan(new PandarScan);

  scan->packets.resize(readpacket);

  // Since the pandar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  for (int i = 0; i < readpacket; ++i)
    {

      while (true)
        {
          // keep reading until full packet received
          int rc = input_->getPacket(&scan->packets[i], config_.time_offset);

           // ICV_LOG_INFO<<"get packet "<<i<<" CORRECT "<<(int) scan->packets[i].data[0];

          if (rc == 0) break;       // got a full packet?
          if (rc == 2)
          {
            // gps packet;
            HS_LIDAR_L40_GPS_Packet packet;
            if(HS_L40_GPS_Parse( &packet , &scan->packets[i].data[0] , HS_LIDAR_L40_GPS_PACKET_SIZE) == 0)
            {
              PandarGpsPtr gps(new PandarGps);
              gps->stamp =  icv::icvTime::now().time_since_epoch().count(); 

              gps->year = packet.year;
              gps->month = packet.month;
              gps->day = packet.day;
              gps->hour = packet.hour;
              gps->minute = packet.minute;
              gps->second = packet.second;
              gps->fineTime =packet.fineTime;
              gps->used = 0;
              if(gps->year > 30 || gps->year < 17)
              {
                //ROS_ERROR("Ignore wrong GPS data (year)%d" , gps->year);
                continue;
              }
              convert->processGps(*gps);
             // gpsoutput_.publish(gps);
            }
            
          }
          if (rc < 0) return false; // end of file reached?
        }
                   // ICV_LOG_INFO<<"push packet IN DRIVER";

        convert->pushLiDARData(scan->packets[i]);
                         //   ICV_LOG_INFO<<"FINISH packet IN DRIVER";

    }

  // publish message using time of last packet read
  //ROS_DEBUG("Publishing a full Pandar scan.");

  scan->header.stamp = scan->packets[readpacket - 1].stamp;
  scan->header.frame_id = config_.frame_id;
  //output_.publish(scan);















  // notify diagnostics that a message has been published, updating
  // its status
  // diag_topic_->tick(scan->header.stamp);
  // diagnostics_.update();

  return true;
}




////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////

Convert::Convert(Config_pandar & config):
    data_(new pandar_rawdata::RawData()), drv(config,this)
{
    data_->setup(config);

    // advertise output point cloud (before subscribing to input data)
    //output_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);

   // srv_ = boost::make_shared <dynamic_reconfigure::Server<pandar_pointcloud::
    //       CloudNodeConfig> > (private_nh);
    //dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig>::
    //CallbackType f;
    //f = boost::bind (&Convert::callback, this, _1, _2);
    //srv_->setCallback (f);
    double start_angle=config.start_angle;
    lidarRotationStartAngle = int(start_angle * 100);
    outMsg=boost::shared_ptr<pandar_rawdata::PPointCloud>(new pandar_rawdata::PPointCloud());
    hasGps = 0;
    sem_init(&picsem, 0, 0);
    pthread_mutex_init(&piclock, NULL);
   // pthread_mutex_init(&outlock, NULL);
    boost::thread thrd(boost::bind(&Convert::DriverReadThread, this));
    boost::thread processThr(boost::bind(&Convert::processLiDARData, this));
}


void Convert::DriverReadThread()
{
  int count_=1;

    while(1)
    {
      count_++;
        

        drv.poll();
                

    }
}



// /** @brief Callback for raw scan messages. */
// void Convert::processScan(const PandarScanConstPtr &scanMsg)
// {
//    // if (output_.getNumSubscribers() == 0)         // no one listening?
//     //    return;                                     // avoid much work

//     // allocate a point cloud with same time and frame ID as raw data
//     pandar_rawdata::PPointCloud::Ptr outMsg(new pandar_rawdata::PPointCloud());
//     // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
//     outMsg->header.stamp = scanMsg->header.stamp;
//     // pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
//     // outMsg->is_dense = false;
//     outMsg->header.frame_id = scanMsg->header.frame_id;
//     outMsg->height = 1;

//     // process each packet provided by the driver
//     // for (size_t i = 0; i < scanMsg->packets.size(); ++i)
//     // {
//     //     data_->unpack(scanMsg->packets[i], *outMsg);
//     // }
//     double firstStamp = 0.0f;
//     int ret = data_->unpack(scanMsg, *outMsg , gps1 , gps2 , firstStamp, lidarRotationStartAngle);

//     // publish the accumulated cloud message
// 	//ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
// 		//			 << " Pandar40 points, time: " << outMsg->header.stamp);

//     if(ret == 1)
//     {
//       //  pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);

//         outMsg->header.stamp=firstStamp;
//        // output_.publish(outMsg);
//     }
// }

void Convert::processGps(PandarGps &gpsMsg)
{

    struct tm t;
    t.tm_sec = gpsMsg.second;
    t.tm_min = gpsMsg.minute;
    t.tm_hour = gpsMsg.hour;
    t.tm_mday = gpsMsg.day;
    t.tm_mon = gpsMsg.month - 1;
    t.tm_year = gpsMsg.year + 2000 - 1900;
    t.tm_isdst = 0;
    if(lastGPSSecond != (mktime(&t) + 1))
    {
        lastGPSSecond = (mktime(&t) + 1);
        gps2.gps = mktime(&t) + 1; // the gps always is the last gps, the newest GPS data is after the PPS(Serial port transmition speed...)
        gps2.used = 0;
    }
     // ROS_ERROR("Got a gps data %d " ,gps2.gps);
}

void Convert::pushLiDARData(PandarPacket& packet)
{
    pthread_mutex_lock(&piclock);
    LiDARDataSet.push_back(packet);
    if(LiDARDataSet.size() > 6)
    {
        sem_post(&picsem);
    }
    pthread_mutex_unlock(&piclock);
 //ICV_LOG_INFO<<"push packet in convert "<<(int)packet.data[0];

}
boost::shared_ptr<pandar_rawdata::PPointCloud> Convert::getpointcloud( )
{
  boost::shared_ptr<pandar_rawdata::PPointCloud> copy(new pandar_rawdata::PPointCloud());

  if(out_mutex_.try_lock())
  {
copy->header.frame_id=outMsg->header.frame_id;
copy->header.seq=outMsg->header.seq;
copy->header.stamp=outMsg->header.stamp;
copy->height=outMsg->height;
copy->points.assign(outMsg->points.begin(),outMsg->points.end());
out_mutex_.unlock();
  }

  return copy;
}


int Convert::processLiDARData()
{
       //

    double lastTimestamp = 0.0f;

    outMsg=boost::shared_ptr<pandar_rawdata::PPointCloud>(new pandar_rawdata::PPointCloud());
    temMsg=boost::shared_ptr<pandar_rawdata::PPointCloud>(new pandar_rawdata::PPointCloud());

       // ICV_LOG_INFO<<"start process lidar  5555";

    int frame_id = 0;

    struct timespec ts;
    while(1)
    {

            //  ICV_LOG_INFO<<"start process lidar  6666";

        if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
        {
          //  ROS_ERROR("get time error");
        }

        ts.tv_sec += 1;
        if (sem_timedwait(&picsem, &ts) == -1)
        {
            // ROS_INFO("No Pic");
            continue;
        }

      //  ICV_LOG_INFO<<"start process lidar  111";

        pthread_mutex_lock(&piclock);
          //  ICV_LOG_INFO<<"process packet in convert";

        PandarPacket packet = LiDARDataSet.front();
        LiDARDataSet.pop_front();
        pthread_mutex_unlock(&piclock);
//ICV_LOG_INFO<<"pop packet in convert "<<(int)packet.data[0];
       // if (output_.getNumSubscribers() == 0)         // no one listening?
        //        continue;                                     // avoid much work

        // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
        // pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
        // outMsg->is_dense = false;



 

        double firstStamp = 0.0f;
          //    ICV_LOG_INFO<<"strat decode packet in convert";
        int ret = data_->unpack(packet, *temMsg , gps1 , gps2 , firstStamp, lidarRotationStartAngle);
          // ICV_LOG_INFO<<"finish decode packet in convert";
        if(ret == 1)
        {
       //   ICV_LOG_INFO<<"decode"<<temMsg->points.size();

        boost::mutex::scoped_lock lock(out_mutex_);
       outMsg->header.frame_id = "pandar";
        outMsg->height = 1;
            // ROS_ERROR("timestamp : %f " , firstStamp);
            if(lastTimestamp != 0.0f)
            {
                if(lastTimestamp > firstStamp)
                {
             //       ROS_ERROR("errrrrrrrrr");
                }
            }

            lastTimestamp = firstStamp;
            if(hasGps)
            {
              //pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
              outMsg->header.stamp=firstStamp;
            }
            else
            {
              //pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
              outMsg->header.stamp==icv::icvTime::now().time_since_epoch().count();
            }
  // ICV_LOG_INFO<<"tset 11  ";
          outMsg->points.assign(temMsg->points.begin(),temMsg->points.end());
      //       ICV_LOG_INFO<<"tset 22 ";

          temMsg->clear();

          lock.unlock();


          }

       
           // output_.publish(outMsg);
           // outMsg->clear();


        }
    }


void Convert::processGps(const PandarGpsConstPtr &gpsMsg)
{
    hasGps = 1;
    struct tm t;
    t.tm_sec = gpsMsg->second;
    t.tm_min = gpsMsg->minute;
    t.tm_hour = gpsMsg->hour;
    t.tm_mday = gpsMsg->day;
    t.tm_mon = gpsMsg->month - 1;
    t.tm_year = gpsMsg->year + 2000 - 1900;
    t.tm_isdst = 0;
    if(lastGPSSecond != (mktime(&t) + 1))
    {
        lastGPSSecond = (mktime(&t) + 1);
        gps2.gps = mktime(&t) + 1; // the gps always is the last gps, the newest GPS data is after the PPS(Serial port transmition speed...)
        gps2.used = 0;
    }
    // ROS_ERROR("Got data second : %f " ,(double)gps2.gps);
}

}//namespace



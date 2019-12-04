/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2012,2015 The MITRE Corporation
 *  Copyright(c) 2018 JIANG KUN

 *
 */


#include "RSlidarGrabber.h"
#include "OpenICV/Core/icvFunction.h"
#include <iostream>
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
///#include <pcl/common/impl/io.hpp>
#include <pcl/io/pcd_io.h>
using namespace std;
namespace RSlidar_pointcloud
{

  static const size_t packet_size =1248;


 ////////////////////////////////////////////////////////////////////////
  // Input base class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number.
   */
  Input::Input(Config_RSlidar& config,uint16_t port):
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
  InputSocket::InputSocket(Config_RSlidar& config,uint16_t port):
    Input(config, port)
  {
    sockfd_ = -1;
    
    if (!devip_str_.empty()) {
      inet_aton(devip_str_.c_str(),&devip_);
    }    

    // connect to RSlidar UDP port
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
    ICV_LOG_WARN<<" socket running "<<my_addr.sin_addr.s_addr;

  //  ROS_DEBUG("RSlidar socket fd is %d\n", sockfd_);
  }

  /** @brief destructor */
  InputSocket::~InputSocket(void)
  {
    (void) close(sockfd_);
  }

// return : 0 - lidar
//          2 - gps
//          1 - error
  /** @brief Get one RSlidar packet. */
  int InputSocket::getPacket(RSlidarPacket *pkt, const double time_offset)
  {
    double time1 =icv::SyncClock::now().time_since_epoch().count();
    int isgps = 0;
    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);
           // ICV_LOG_INFO<<"try get socket data";

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
                
              //  ROS_WARN("RSlidar poll() timeout");
        char buffer_data[8] = "re-con";
        memset(&sender_address, 0, sender_address_len);          // initialize to zeros
        sender_address.sin_family = AF_INET;                     // host byte order
        sender_address.sin_port = htons(MSOP_DATA_PORT_NUMBER);  // port in network byte order, set any value
        sender_address.sin_addr.s_addr = devip_.s_addr;          // automatically fill in my IP
        sendto(sockfd_, &buffer_data, strlen(buffer_data), 0, (sockaddr*)&sender_address, sender_address_len);
                return 1;
              }
            if ((fds[0].revents & POLLERR)
                || (fds[0].revents & POLLHUP)
                || (fds[0].revents & POLLNVAL)) // device error?
              {
                //ROS_ERROR("poll() reports RSlidar error");
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

            //ICV_LOG_INFO<<"get packet SIZE CORRECT "<< (int)pkt->data[0];

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
         


       // ROS_DEBUG_STREAM("incomplete RSlidar packet read: "
               //          << nbytes << " bytes");
      }

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred. Add the time offset.
    double time2 = icv::SyncClock::now().time_since_epoch().count();
    pkt->stamp = (time2 + time1) / 2.0 + time_offset;

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
  InputPCAP::InputPCAP(Config_RSlidar& config, uint16_t port,
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
      //  ROS_FATAL("Error opening RSlidar socket dump file.");
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
  /** @brief Get one RSlidar packet. */
  int InputPCAP::getPacket(RSlidarPacket *pkt, const double time_offset)
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
            pkt->stamp = icv::SyncClock::now().time_since_epoch().count(); // time_offset not considered here, as no synchronization required
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
          //  ROS_WARN("Error %d reading RSlidar packet: %s", 
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

       // ROS_DEBUG("replaying RSlidar dump file");

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


RSlidarDriver::RSlidarDriver( Config_RSlidar &config,Convert *cvt)
{
  convert = cvt;
  // use private node handle to get parameters
  //private_nh.param("frame_id", config_.frame_id, std::string("RSlidar"));
  //std::string tf_prefix = tf::getPrefixParam(private_nh);
  //ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  //config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
  config_.frame_id=config.frame_id;
  // get model name, validate string, determine packet rate
  config_.model = config.model;//"RSlidar40";
  std::string model_full_name ;
   double packet_rate;

      // product model
  if (config_.model == "RS16")
  {
    packet_rate = 840;
    model_full_name = "RS-LiDAR-16";
  }
  else if (config_.model == "RS32")
  {
    packet_rate = 1690;
    model_full_name = "RS-LiDAR-32";
  }
  else
  {
    //ROS_ERROR_STREAM("unknown LIDAR model: " << config_.model);
    packet_rate = 2600.0;
  }
  //double packet_rate = 3000;                   // packet frequency (Hz)
  std::string deviceName(model_full_name);
  if(config.rpm>0)config_.rpm=config.rpm;
  else config_.rpm=600;
  //private_nh.param("rpm", config_.rpm, 600.0);
  //ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  //if(config.npackets<=0)
  config_.npackets = (int) ceil(packet_rate / frequency);
  //else config_.npackets = config.npackets;
  //private_nh.getParam("npackets", config_.npackets);
  
  //ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  dump_file=config.pcapfile;
  //private_nh.param("pcap", dump_file, std::string(""));

  int udp_port,difop_port;
  if(config.ip_port>0)udp_port=config.ip_port;
  else udp_port=(int)MSOP_DATA_PORT_NUMBER;
  
if(config.ip_port_difop>0)difop_port=config.ip_port_difop;
  else difop_port=(int)DIFOP_DATA_PORT_NUMBER;
  config_.time_offset=config.time_offset;
    double cut_angle;
    if(config.cut_angle>=0&&config.cut_angle<360)cut_angle=config.cut_angle;
    else config_.cut_angle=-0.1;
  

  // // Initialize dynamic reconfigure
  // srv_ = boost::make_shared <dynamic_reconfigure::Server<RSlidar_pointcloud::
  //   CloudNodeConfig> > (private_nh);
  // dynamic_reconfigure::Server<RSlidar_pointcloud::CloudNodeConfig>::
  //   CallbackType f;
  // f = boost::bind (&RSlidarDriver::callback, this, _1, _2);
  // srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
 // diagnostics_.setHardwareID(deviceName);
  // const double diag_freq = packet_rate/config_.npackets;
  // diag_max_freq_ = diag_freq;
  // diag_min_freq_ = diag_freq;
  // ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  // using namespace diagnostic_updater;
  // diag_topic_.reset(new TopicDiagnostic("RSlidar_packets", diagnostics_,
  //                                       FrequencyStatusParam(&diag_min_freq_,
  //                                                            &diag_max_freq_,
  //                                                            0.1, 10),
  //                                       TimeStampStatusParam()));

  // open RSlidar input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      // read data from packet capture file
      input_1.reset(new RSlidar_pointcloud::InputPCAP(config,udp_port,
                                                  packet_rate, dump_file));
      input_2.reset(new RSlidar_pointcloud::InputPCAP(config,difop_port,
                                                  packet_rate, dump_file));
    }
  else
    {
      // read data from live socket
          ICV_LOG_WARN<<"OK get socket";

      input_1.reset(new RSlidar_pointcloud::InputSocket(config,udp_port));
      input_2.reset(new RSlidar_pointcloud::InputSocket(config,difop_port));
    }
    ICV_LOG_WARN<<"OK driver";

  // raw packet output topic
  // output_ =
  //   node.advertise<RSlidar_msgs::RSlidarScan>("RSlidar_packets", 10);
  // // raw packet output topic
  // gpsoutput_ =
  //   node.advertise<RSlidar_msgs::RSlidarGps>("RSlidar_gps", 1);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool RSlidarDriver::poll(void)
{
  int readpacket = config_.npackets;
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    //ICV_LOG_WARN<<" driver running"<<config_.cut_angle;

  RSlidarScanPtr scan(new RSlidarScan);

  

  // Since the RSlidar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  if(config_.cut_angle>=0)

  {
  scan->packets.reserve(readpacket);
  RSlidarPacket tmp_packet;
  while (true)
    {

      while (true)
        {
          // keep reading until full packet received
          int rc = input_1->getPacket(&tmp_packet, config_.time_offset);

           // ICV_LOG_INFO<<"get packet "<<i<<" CORRECT "<<(int) scan->packets[i].data[0];

          if (rc == 0) break;       // got a full packet?            
          if (rc < 0) return false; // end of file reached?
        }
                   // ICV_LOG_INFO<<"push packet IN DRIVER";

       // 
                         //   ICV_LOG_INFO<<"FINISH packet IN DRIVER";
       scan->packets.push_back(tmp_packet);

      static int ANGLE_HEAD = -36001;  // note: cannot be set to -1, or stack smashing
      static int last_azimuth = ANGLE_HEAD;

      int azimuth = 256 * tmp_packet.data[44] + tmp_packet.data[45];
      // int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

      // Handle overflow 35999->0
      if (azimuth < last_azimuth)
      {
        last_azimuth -= 36000;
      }
      // Check if currently passing cut angle
      if (last_azimuth != ANGLE_HEAD && last_azimuth < config_.cut_angle && azimuth >= config_.cut_angle)
      {
        last_azimuth = azimuth;
        break;  // Cut angle passed, one full revolution collected
      }
      last_azimuth = azimuth;

    }
  }
  else 
  {

   // ICV_LOG_WARN<<" driver polling";

 scan->packets.resize(config_.npackets);
    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = input_1->getPacket(&scan->packets[i], config_.time_offset);
            //ICV_LOG_WARN<<" try get packet ";

        if (rc == 0)
          {
          //  ICV_LOG_WARN<<" get packet "<<i;
            break; } // got a full packet?
        if (rc < 0)
          {
         // ICV_LOG_WARN<<" did not get packet ";
           
            
            return false; 
          
          
          } // end of file reached?
      }
    }

  }

    //ICV_LOG_WARN<<" try push scan ";


  // publish message using time of last packet read
  //ROS_DEBUG("Publishing a full RSlidar scan.");

  scan->header.stamp = scan->packets.back().stamp;
  scan->header.frame_id = config_.frame_id;
  //output_.publish(scan);

  convert->pushLiDARData(*scan);
  return true;
}

void RSlidarDriver::difopPoll(void)
{
  // reading and publishing scans as fast as possible.
  RSlidarPacketPtr difop_packet_ptr(new RSlidarPacket);
  while (true)
  {
    // keep reading
    RSlidarPacket difop_packet_msg;
    int rc = input_2->getPacket(&difop_packet_msg, config_.time_offset);
    if (rc == 0)
    {
      // std::cout << "Publishing a difop data." << std::endl;
     // ROS_DEBUG("Publishing a difop data.");
      *difop_packet_ptr = difop_packet_msg;
      convert->pushLiDARData_difop(*difop_packet_ptr);
      //difop_output_.publish(difop_packet_ptr);
    }
    if (rc < 0)
      return;  // end of file reached?
   // ros::spinOnce();
  }
}


////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////

Convert::Convert(Config_RSlidar & config):
    data_(new RSlidar_rawdata::RawData()), drv(config,this)
{
    data_->loadConfigFile(config);
    model=config.model;

    // advertise output point cloud (before subscribing to input data)
    //output_ = node.advertise<sensor_msgs::PointCloud2>("RSlidar_points", 10);

   // srv_ = boost::make_shared <dynamic_reconfigure::Server<RSlidar_pointcloud::
    //       CloudNodeConfig> > (private_nh);
    //dynamic_reconfigure::Server<RSlidar_pointcloud::CloudNodeConfig>::
    //CallbackType f;
    //f = boost::bind (&Convert::callback, this, _1, _2);
    //srv_->setCallback (f);
    //double start_angle=config.start_angle;
    //lidarRotationStartAngle = int(start_angle * 100);
    outMsg=boost::shared_ptr<::pcl::PointCloud<::pcl::PointXYZI>>(new ::pcl::PointCloud<::pcl::PointXYZI>());
    hasGps = 0;
    sem_init(&picsem, 0, 0);
    pthread_mutex_init(&piclock, NULL);
    pthread_mutex_init(&packlock, NULL);
    ICV_LOG_WARN<<"OK convert";

    boost::thread thrd(boost::bind(&Convert::DriverReadThread, this));
    boost::thread processThr(boost::bind(&Convert::processLiDARData, this));
   // boost::thread processThr(boost::bind(&Convert::processLiDARData, this));
}


void Convert::DriverReadThread()
{
  int count_=1;

    while(1)
    {
      count_++;
 //ICV_LOG_WARN<<"convert count "<<count_;


        drv.poll();
                

    }
}

void Convert::DriverdifopThread()
{
  int count_=1;

    while(1)
    {
      count_++;
        

        drv.difopPoll();
                

    }
}



// /** @brief Callback for raw scan messages. */
void Convert::processScan( RSlidarScan &scanMsg)
{


  

  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outMsg=boost::shared_ptr<::pcl::PointCloud<::pcl::PointXYZI>>(new ::pcl::PointCloud<::pcl::PointXYZI>());
  outPoints->header.stamp = scanMsg.header.stamp;
  outPoints->header.frame_id = scanMsg.header.frame_id;
  outPoints->clear();
  if (model == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scanMsg.packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
   ICV_LOG_INFO<<"pointsize :"<<outPoints->points.size()<<"width"<<outPoints->width;
  }
  else if (model == "RS32")
  {
    outPoints->height = 32;
    outPoints->width = 12 * (int)scanMsg.packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }
		//	boost::this_thread::sleep (boost::posix_time::microseconds (1000));

  // process each packet provided by the driver

  data_->block_num = 0;
 // ICV_LOG_INFO<<"RS 16 2 :"<<scanMsg.packets.size();

  for (size_t i = 0; i < scanMsg.packets.size(); ++i)
  {
    data_->unpack(scanMsg.packets[i], outPoints);
  }
   // ICV_LOG_INFO<<"RS 16 3 :"<<scanMsg.packets.size();

 boost::mutex::scoped_lock lock(out_mutex_);
       outMsg->header.frame_id = "RSlidar";
      outMsg->height = 1;
  outMsg->points.assign(outPoints->points.begin(),outPoints->points.end());
    //::pcl::copyPointCloud(*outPoints,*outMsg);
          lock.unlock();
}



void Convert::pushLiDARData(RSlidarScan& scan)
{
    pthread_mutex_lock(&piclock);
    scan_data_.push_back(scan);

    pthread_mutex_unlock(&piclock);
 //ICV_LOG_INFO<<"push scan in convert "<<scan.packets.size();

}

void Convert::pushLiDARData_difop(RSlidarPacket& packet)
{
    pthread_mutex_lock(&packlock);
    packet_data_.push_back(packet);

    pthread_mutex_unlock(&packlock);
 //ICV_LOG_INFO<<"push packet in convert "<<(int)packet.data[0];

}
boost::shared_ptr<::pcl::PointCloud<::pcl::PointXYZI>> Convert::getpointcloud( )
{
  boost::shared_ptr<::pcl::PointCloud<::pcl::PointXYZI>> copy(new ::pcl::PointCloud<::pcl::PointXYZI>());

  if(out_mutex_.try_lock())
  {
copy->header.frame_id=outMsg->header.frame_id;
copy->header.seq=outMsg->header.seq;
copy->header.stamp=outMsg->header.stamp;
copy->height=outMsg->height;
copy->points.assign(outMsg->points.begin(),outMsg->points.end());
//::pcl::copyPointCloud(*outMsg,*copy);
out_mutex_.unlock();
  }

  return copy;
}


int Convert::processLiDARData()
{
    int det=0;


    while(1)
    {
      RSlidarScan scamtmp;
        pthread_mutex_lock(&piclock);
        //ICV_LOG_INFO<<"process scan in convert"<<scan_data_.size();

        if (scan_data_.size()>0  )
        
        {
         scamtmp  = scan_data_.front();
        scan_data_.pop_front();
        det=1;
        }
        pthread_mutex_unlock(&piclock);     

      if(det==1) {processScan(scamtmp);det=0;}

    }
}




}//namespace



/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Pandar40 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

namespace pandar_pointcloud
{
/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new pandar_rawdata::RawData()), drv(node , private_nh , this)
{
    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    output_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<pandar_pointcloud::
           CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig>::
    CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);
    double start_angle;
    private_nh.param("start_angle", start_angle, 0.0);
    lidarRotationStartAngle = int(start_angle * 100);

    hasGps = 0;
    // subscribe to PandarScan packets
    // pandar_scan_ =
    //     node.subscribe("pandar_packets", 100,
    //                    &Convert::processScan, (Convert *) this,
    //                    ros::TransportHints().tcpNoDelay(true));
    // pandar_gps_ =
    //     node.subscribe("pandar_gps", 1,
    //                    &Convert::processGps, (Convert *) this,
    //                    ros::TransportHints().tcpNoDelay(true));

    sem_init(&picsem, 0, 0);
    pthread_mutex_init(&piclock, NULL);

    boost::thread thrd(boost::bind(&Convert::DriverReadThread, this));
    boost::thread processThr(boost::bind(&Convert::processLiDARData, this));
}

void Convert::DriverReadThread()
{
    while(1)
    {
        drv.poll();
    }
}

void Convert::callback(pandar_pointcloud::CloudNodeConfig &config,
                       uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    data_->setParameters(config.min_range, config.max_range, config.view_direction,
                         config.view_width);
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg)
{
    if (output_.getNumSubscribers() == 0)         // no one listening?
        return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    pandar_rawdata::PPointCloud::Ptr outMsg(new pandar_rawdata::PPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    // pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
    // outMsg->is_dense = false;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;

    // process each packet provided by the driver
    // for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    // {
    //     data_->unpack(scanMsg->packets[i], *outMsg);
    // }
    double firstStamp = 0.0f;
    int ret = data_->unpack(scanMsg, *outMsg , gps1 , gps2 , firstStamp, lidarRotationStartAngle);

    // publish the accumulated cloud message
	ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
					 << " Pandar40 points, time: " << outMsg->header.stamp);

    if(ret == 1)
    {
        pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
        output_.publish(outMsg);
    }
}

void Convert::processGps(pandar_msgs::PandarGps &gpsMsg)
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

void Convert::pushLiDARData(pandar_msgs::PandarPacket packet)
{
    pthread_mutex_lock(&piclock);
    LiDARDataSet.push_back(packet);
    if(LiDARDataSet.size() > 6)
    {
        sem_post(&picsem);
    }
    pthread_mutex_unlock(&piclock);

}

int Convert::processLiDARData()
{
    double lastTimestamp = 0.0f;
    pandar_rawdata::PPointCloud::Ptr outMsg(new pandar_rawdata::PPointCloud());
    int frame_id = 0;
    struct timespec ts;
    while(1)
    {
        if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
        {
            ROS_ERROR("get time error");
        }

        ts.tv_sec += 1;
        if (sem_timedwait(&picsem, &ts) == -1)
        {
            // ROS_INFO("No Pic");
            continue;
        }
        pthread_mutex_lock(&piclock);
        pandar_msgs::PandarPacket packet = LiDARDataSet.front();
        LiDARDataSet.pop_front();
        pthread_mutex_unlock(&piclock);

        if (output_.getNumSubscribers() == 0)         // no one listening?
                continue;                                     // avoid much work

        // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
        // pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
        // outMsg->is_dense = false;
        outMsg->header.frame_id = "pandar";
        outMsg->height = 1;



        double firstStamp = 0.0f;
        int ret = data_->unpack(packet, *outMsg , gps1 , gps2 , firstStamp, lidarRotationStartAngle);



        if(ret == 1)
        {
            // ROS_ERROR("timestamp : %f " , firstStamp);
            if(lastTimestamp != 0.0f)
            {
                if(lastTimestamp > firstStamp)
                {
                    ROS_ERROR("errrrrrrrrr");
                }
            }

            lastTimestamp = firstStamp;
            if(hasGps)
            {
              pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
            }
            else
            {
              pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
            }
            output_.publish(outMsg);
            outMsg->clear();


        }
    }
}

void Convert::processGps(const pandar_msgs::PandarGps::ConstPtr &gpsMsg)
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

} // namespace pandar_pointcloud

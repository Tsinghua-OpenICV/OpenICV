#include "./ThirdParty/GeographicLib/include/LocalCartesian.hpp"
#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>
#include <thread>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "./ins_lib/KFApp.h"
#include "./ins_lib/PSINS.h"
#include "./ins_lib/PSINS.cpp"

#include "tic_toc.h"

using namespace std;
class GlobalOptNode
{

private:
    ros::NodeHandle n;

    ros::Subscriber sub_GPS;
    ros::Subscriber sub_Odom;
    ros::Subscriber sub_Imu;
    /**for debug
    **/
    ros::Publisher pub_lidar_odometry;

    nav_msgs::Path *global_path;
    nav_msgs::Path *gps_path;

    double last_vio_t;
    std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
    std::queue<sensor_msgs::ImuPtr> imuQueue;
    std::mutex m_buf;
    std::mutex m_imu_mutex;
    std::mutex m_gps_mutex;
    CVect3 pwm;
    CVect3 pvm;
    CVect3 gps_pos;
    int gps_count;

    CVect3 att0;
    CVect3 pos0;
    bool inited;
    bool new_imu;
    bool new_gps;
    bool initGPS;

    CKFApp kf;
    //CSINSGPSOD kf2;
    std::thread threadProcess;
    GeographicLib::LocalCartesian geoConverter;

public:
    GlobalOptNode() : n("~")
    {
        NodeSetup();
        threadProcess = std::thread(&GlobalOptNode::process, this);
    }

    ~GlobalOptNode()
    {
        threadProcess.detach();
    }
    /**
    **/
    void NodeSetup()
    {

        new_imu = false;
        new_gps = false;
        inited = false;
        att0 = O31;
        pos0 = O31;
        gps_count = 0;
        initGPS = false;
        //sub_GPS = n.subscribe("/gps/fix", 100, &GlobalOptNode::GPS_callback ,this);
        sub_Imu = n.subscribe("/imu/data", 1024, &GlobalOptNode::imu_callback, this);
        sub_GPS = n.subscribe("/gps/fix_ublox", 100, &GlobalOptNode::GPS_callback, this);

        //CVect3 gpspos = CVect3(34.196255*glv.deg,108.875677*glv.deg, 410.70);
        //kf.Init(CSINS(a2qua(0,0,1.2*DEG), O31, O31));
        //kf2.Init(CSINS(a2qua(0,0,1.2*DEG), O31, gpspos));
    }

    /**
    **/
    void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
    {
        //cout<<"gps_callback"<<endl;
        if (gps_count < 2)
        {
            gps_count++;
        }
        else
        {
            m_gps_mutex.lock();
            gps_pos.i = GPS_msg->latitude * PI / 180;
            gps_pos.j = GPS_msg->longitude * PI / 180;
            gps_pos.k = GPS_msg->altitude;

            m_gps_mutex.unlock();
            new_gps = true;
            gps_count = 0;

            double *xyz;
            if (!initGPS)
            {
                geoConverter.Reset(GPS_msg->latitude, GPS_msg->longitude, GPS_msg->altitude);
                initGPS = true;
            }
            else
            {
                geoConverter.Forward(GPS_msg->latitude, GPS_msg->longitude, GPS_msg->altitude, xyz[0], xyz[1], xyz[2]);
                printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);

                m_imu_mutex.lock();
                geoConverter.Forward(kf.sins.pos.i * 180 / PI, kf.sins.pos.j * 180 / PI, kf.sins.pos.k, xyz[0], xyz[1], xyz[2]);
                m_imu_mutex.unlock();
                printf("gps_ x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);

                cout << "att=" << kf.sins.att.i * 180 / PI
                     << "," << kf.sins.att.j * 180 / PI
                     << "," << kf.sins.att.k * 180 / PI
                     << endl;
            }
        }
    }
    /**
    **/
    void imu_callback(const sensor_msgs::ImuPtr &imu_msg)
    {
        //cout<<"imu_callback"<<endl;

        m_imu_mutex.lock();
        {
            double imu_t = imu_msg->header.stamp.toSec();
            //             CVect3 pwm_temp(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
            //             CVect3 pvm_temp(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);

            CVect3 pwm_temp(imu_msg->angular_velocity.y, imu_msg->angular_velocity.x, imu_msg->angular_velocity.z);
            CVect3 pvm_temp(imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.z);
            pwm = pwm_temp * 0.01;
            pvm = pvm_temp * 0.01;

            Eigen::Quaterniond ImuQ(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
            double roll, pitch, yaw;
            tf::Matrix3x3(tf::Quaternion(ImuQ.x(), ImuQ.y(), ImuQ.z(), ImuQ.w())).getRPY(roll, pitch, yaw);
            att0.i = pitch;
            att0.j = roll;
            att0.k = yaw;
        }
        m_imu_mutex.unlock();
        new_imu = true;
    }

    /**
        TODO: loam_callback
    **/

    void process()
    {
        while (1)
        {
            if (!inited)
            {
                // do someting
                if (new_imu && new_gps)
                {
                    //                     cout<<"att0="<<att0.i*180/PI
                    //                     <<"," <<att0.j*180/PI
                    //                     <<","<<att0.k*180/PI
                    //                     <<endl;

                    CVect3 vel0(10, 0, 0);
                    kf.Init(CSINS(att0, vel0, gps_pos));
                    inited = true;
                }
                continue;
            }

            if (new_imu)
            {
                TicToc timer;
                new_imu = false;
                //kf.SetMeasVG();
                //kf.Update(&pwm, &pvm, 1, 0.01);

                if (new_gps)
                {
                    new_gps = false;
                    m_gps_mutex.lock();
                    CVect3 vgps = kf.sins.vn;
                    ////kf2.SetMeasGPS(gps_pos, vgps);
                    kf.SetMeasGPS(gps_pos, O31);

                    //                      cout<<"gps_pos============>"<<gps_pos.i
                    //                     <<"," <<gps_pos.j
                    //                     <<","<<gps_pos.k
                    //                     <<endl;
                    m_gps_mutex.unlock();
                }

                m_imu_mutex.lock();
                kf.Update(&pwm, &pvm, 1, 0.01);
                m_imu_mutex.unlock();

                //                 cout<<"att="<<kf.sins.att.i*180/PI
                //                     <<"," <<kf.sins.att.j*180/PI
                //                     <<","<<kf.sins.att.k*180/PI
                //                     <<endl;
                //
                //                 cout<<"pos="<<kf.sins.pos.i*180/PI
                //                     <<"," <<kf.sins.pos.j*180/PI
                //                     <<","<<kf.sins.pos.k
                //                     <<endl;
                //
                //
                //                 cout<<"innovation===>"<<kf.innovation
                //                     <<endl;
                //cout<<"time="<<timer.toc()<<endl;
            }

            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "backend_Estimator2");
    ROS_INFO("\033[1;32m---->\033[0m backend_Estimator2 Started.");

    GlobalOptNode go;
    ros::spin();
    return 0;
}

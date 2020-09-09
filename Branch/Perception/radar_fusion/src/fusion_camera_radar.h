// @brief: Enter fusion 

// objective: Collect all sensor data, enter the fusion module
//Author: Chunlei YU. Contact me at: yuchunlei@mail.tsinghua.edu.cn
#ifndef CAR_OBSF_ROS_H
#define CAR_OBSF_ROS_H

#include <iostream>

#include <Eigen/Dense>

#include "ros_fusion/DriveObsArray.h"
#include "ros_fusion/DriveObs.h"
#include "ros_fusion/DriveRadarObsArray.h"
#include "ros_fusion/DriveRadarObs.h"

#include "ros_fusion/ivsensorlrrobj.h"
#include "ros_fusion/radarlrrobject.h"

#include "ros_fusion/ivsensorsrrobj.h"
#include "ros_fusion/radarsrrobject.h"

#include "filters/obsf_uncertainty_fusion_engine.h"
#include "objects/obsf_header.h"
#include "visualization/obsf_pcl_visual.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include "std_msgs/String.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

//#include <eigen_conversions/eigen_msg.h>

namespace tsinghua{
namespace dias{
namespace fusion{

ros::Publisher g_obs_publisher;
ros::Publisher g_obs_publisher_proto;

tsinghua::dias::fusion::ObsfVisual _visualizer;

class Obsfusion_camera_radar{
public:
    Obsfusion_camera_radar(){}
    ~Obsfusion_camera_radar(){}

    static void obs_callback_lidar3d(const ros_fusion::DriveObsArray &msg){
 
    }

    static uint64_t unix2gps(uint64_t unix_time_usec) {
        // const uint64_t leapSeconds = 16;
        const uint64_t leapSeconds =
            unix_time_usec < uint64_t(1435708799) * 1e6 ? 16 : 17;
        int64_t gps = unix_time_usec - (uint64_t(315964800) - leapSeconds) * 1e6;
        return gps > 0 ? gps : 0;
    }

    static void obs_callback_radar_online(const ros_fusion::ivsensorlrrobj &msg) {

    }


    static void obs_callback_local_radar_online(const ros_fusion::ivsensorlrrobj &msg) {
 
        std::cout << "Now we enter the radar callback" << std::endl;
        uint64_t unix_time_usec = static_cast<uint64_t>(msg.header.stamp.toSec() * 1e6);
        uint64_t gps_time_usec = unix2gps(unix_time_usec);
        uint64_t sec =  gps_time_usec / 1000000;
        uint64_t nsec =  (gps_time_usec % 1000000) * 1000;


        Eigen::Matrix4d radar_pose = Eigen::Matrix4d::Identity();

        tsinghua::dias::fusion::OBSFHeader header;
        header._seq = msg.header.seq;
        // header._stamp._sec = msg.header.stamp.sec;
        header._stamp._sec = sec;
        // header._stamp._nsec = msg.header.stamp.nsec;
        header._stamp._nsec = nsec;
        header._frame_id = msg.header.frame_id;           

        tsinghua::dias::fusion::OBSFObs3dFrame obs_3d_frame;
        obs_3d_frame.set_header(header);        

        std::vector<tsinghua::dias::fusion::OBSFObs3d> obs_3d_obstacles;
	    std::set<int> current_ids;
        for (size_t i = 0; i < msg.obs.size(); i++)
        {

                tsinghua::dias::fusion::OBSFObs3d obs_3d;
                obs_3d._header._seq = msg.header.seq;
                obs_3d._header._stamp._sec = sec;
                obs_3d._header._stamp._nsec = nsec;
                obs_3d._header._frame_id = msg.header.frame_id;
                obs_3d._obs_id = msg.obs[i].id;

                Eigen::Matrix<double, 4, 1> location_r;
                Eigen::Matrix<double, 4, 1> location_w;
                double local_pi = tsinghua::dias::fusion::PI;
                location_r << msg.obs[i].x,
                            -msg.obs[i].y,
                            0.0,
                            1.0;
          
                location_w = radar_pose * location_r;
                tsinghua::dias::fusion::OBFSVector3d point;
                point._x = location_w[0];
                point._y = location_w[1];
                point._z = location_w[2];
                // std::cout << "radar object location: "
                //           << point._x << " "
                //           << point._y << " "
                //           << point._z << std::endl;

            
                obs_3d._obs_position = point;
                obs_3d._obs_theta = 0.0;
                Eigen::Matrix<double, 4, 1> velocity_r;

                velocity_r << msg.obs[i].speedx,
                            -msg.obs[i].speedy,
                            0.0,
                            1.0;

                obs_3d._velocity._x = msg.obs[i].speedx;
                obs_3d._velocity._y = msg.obs[i].speedy;

                obs_3d._length = 1.0;
                obs_3d._width = 1.0;
                obs_3d._height = 1.0;
                obs_3d._polygon_points.push_back(point);
                obs_3d._life = 0;
                obs_3d._classification = 0;
		        //obs_3d.rcs = msg.obstacles[i].rcs;

                obs_3d_obstacles.push_back(obs_3d);

        }
/*
	_obstacle_ids = current_ids;
        radar_his_obstacles = obs_3d_obstacles;
*/
	obs_3d_frame.set_obstacles(obs_3d_obstacles);

	_visualizer.visualize_radar_frame(obs_3d_frame);

    }

    // static void callback_cloud_visualization(const sensor_msgs::PointCloud2ConstPtr& msg) 
    // {
    //     std::cout << "we are now in the cloud visualization callback" << std::endl;
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::fromROSMsg(*msg, *query_cloud);


    //     _visualizer.remove_points();  
    //     _visualizer.add_point_cloud(query_cloud, 200, 0, 0, 1, "new_cloud", 0);
    //  }

    // static void callback_velodyne_visualization(const sensor_msgs::PointCloud2ConstPtr& msg) 
    // {
    //     std::cout << "we are now in the velodyne visualization callback" << std::endl;
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr query_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::fromROSMsg(*msg, *query_cloud);


    //     _visualizer.remove_points();  
    //     _visualizer.add_point_cloud(query_cloud, 200, 0, 0, 1, "new_cloud", 0);
    //  }     


    static void publish_obs(const tsinghua::dias::fusion::OBSFHeader &header) {
       
    }

    static void run(int argc, char **argv){
        ros::init(argc, argv, "fusion");

        ros::NodeHandle nh;
        std::string obstacle_cloud_msg_name = "/cloud1";
        std::string obstacle_velodyne_msg_name = "/velodyne_points";
        std::string obstacle_radar_msg_name = "/lrr_topic1";
        std::string obstacle_msg_name = "/DriveObsArray";
        std::cout << "begin the fusion" << std::endl;
/*
        ros::NodeHandle private_node_handle_("~");
        private_node_handle_.getParam("obstacle_3d_msg_name", obstacle_3d_msg_name);
        private_node_handle_.getParam("obstacle_radar_msg_name", obstacle_radar_msg_name);
        private_node_handle_.getParam("obstacle_msg_name", obstacle_msg_name);
*/

        // g_obs_publisher = nh.advertise<ros_fusion::DriveObsArray>(
        //     obstacle_msg_name,
        //     1000);

        // // ros::Subscriber lidar_obs3d_sub = nh.subscribe(
        // //     obstacle_cloud_msg_name,
        // //     1000,
        // //     callback_cloud_visualization);

        // ros::Subscriber velodyne_obs3d_sub = nh.subscribe(
        //     obstacle_velodyne_msg_name,
        //     1000,
        //     callback_velodyne_visualization);            

        // ros::Subscriber radar_obs3d_sub = nh.subscribe(
        //     obstacle_radar_msg_name,
        //     1000,
        //     obs_callback_local_radar_online);
        // _visualizer.initialize_pcl_visualizer();
        ros::spin();
    }
};

}//fusion
}//car
}//tsinghua

#endif

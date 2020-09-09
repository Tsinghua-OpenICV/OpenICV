#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "../../src/utility/tic_toc.h"

//#include <opencv-3.3.1-dev/opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include "../../src/factor/marginalization_factor.h"
using namespace std;


const int WINDOW_SIZE = 10;
const int SIZE_POSE = 7;
class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy_Lat,	double posAccuracy_Lon ,double altitude_Alt);

	void input_Groundtruth(double t, double latitude, double longitude, double altitude);

	void inputIMU(double t ,Eigen::Quaterniond ImuQ);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);


	void get_lidar_odom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);

	void set_sim_gps(const bool sim);
	void set_sim_gpsnoise(const double noise);
	nav_msgs::Path global_path;

	nav_msgs::Path gps_path;

	nav_msgs::Path global_befOpt_path;
	nav_msgs::Path local_path;
	nav_msgs::Path Groundtruth_path;

public:
    /**
     * TODO:for marg
     * **/
    void slideWindow();
    //void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    //void optimization();
    void vector2double();
    void double2vector();
private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void optimize();
	void mapping();
	void updateGlobalPath();
	void updateGPSPath();    ///GPS path
    void update_globalPose_befOpt_MapPath();    // 全局pose，在优化前的轨迹
	void update_localpose_MapPath();
	void update_Groundtruth_MapPath();          // 参考GPS轨迹
	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
	map<double, vector<double>> GPSPositionMap;

	map<double, vector<double>> IMUOrientationMap;


	map<double, vector<double>> globalPose_befOpt_Map;

	map<double, vector<double>> Groundtruth_Map;

	
	bool initGPS;
	bool newGPS;
	bool newIMU;
    bool newOdom;
    bool first_Odom;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;

	std::thread threadmapping;


	/**
	 TODO:just for debug
	**/
	Eigen::Vector3d lidar_odomP;
	Eigen::Quaterniond lidar_odomQ;
	Eigen::Matrix4d GPSBody_T_world;
	Eigen::Matrix4d Lidar_T_gps;

	Eigen::Quaterniond Q_imu_init_2_W;

	Eigen::Matrix4d T_imu_init_2_W;


	Eigen::Matrix4d lidar_T_W;


	Eigen::Matrix4d Wlidar_T_Wimu;
	Eigen::Matrix4d Wlidar_T_Wimu_DOT_T_imu0_2_world;

	Eigen::Matrix4d T_Lidar_2_imu;  //lidar imu 外参
	/**
	add gps noise 
	**/
	bool simulation;
	double w_sigma;// 噪声Sigma值
	cv::RNG rng; // OpenCV随机数产生器
	
	
	/**
     * TODO:for marg
     * **/
    
    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;
    
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    
    Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];
    
    Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];
    
    Eigen::Vector3d odomP_global;
    
    Eigen::Matrix3d odomR_global;
    
    int16_t frame_count;

};

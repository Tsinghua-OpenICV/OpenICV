/**
 * Convert icvbag to rosbag
 */






// icvMsgpackRecorder *_recorder;

// std::string folder_path;
// boost::filesystem::path imagefolderpath_;
// boost::filesystem::path pointcloudpath_;

// std::mutex mtx;

// void lock_record(const icvDataObject *data, int source)
// {
//     double start, end;
//     mtx.lock();
//     _recorder->Record(data, source);
//     mtx.unlock();
// }

// inline void init_golbal(void) 
// {
//     _recorder = new icvMsgpackRecorder("", 0);
//     _recorder->setRecordpath(folder_path);
// }

// inline unsigned long sec2us(double sec) {
//     return (unsigned long)(sec * 1e6);
// }

// void copy_radar_data(
//     /* [in] */  const icv2ros::ivsensorlrrobj &msg,
//     /* [out] */  RadarLongOut &radar_lrr1 )
// {
//     radar_lrr1.header_.seq = msg.header.seq;
//     // std::cout << "radar - " << radar_lrr1.header_.seq  << std::endl;
//     // unsigned long time_radar = sec2us(msg.header.stamp.toSec());
//     radar_lrr1.header_.stamp = sec2us(msg.header.stamp.toSec());
//     // std::cout.precision(64);
//     // std::cout<<"radar ros time: " <<msg.header.stamp.toSec()<<"tous --"<< radar_lrr1.header_.stamp<<"--"<<time_radar<<std::endl;

//     radar_lrr1.targetnum_ = msg.obs.size();
//     for (int i = 0; i< msg.obs.size(); i++) {
//         radar_lrr1.alldata_[i].id = msg.obs[i].id;
//         radar_lrr1.alldata_[i].range = 0;
//         radar_lrr1.alldata_[i].rangerate = 0;
//         radar_lrr1.alldata_[i].angle = 0; // msg.obs[i].angle;
//         radar_lrr1.alldata_[i].x = msg.obs[i].x; 
//         radar_lrr1.alldata_[i].y = msg.obs[i].y;
//         radar_lrr1.alldata_[i].relspeedx = msg.obs[i].speedx;
//         radar_lrr1.alldata_[i].relspeedy = msg.obs[i].speedy;
//         radar_lrr1.alldata_[i].obj_amp = msg.obs[i].obj_amp;
//         radar_lrr1.alldata_[i].objDynProp = msg.obs[i].objDynProp;
//         radar_lrr1.alldata_[i].flag = 0;
//         radar_lrr1.alldata_[i].accx = msg.obs[i].accx;
//         radar_lrr1.alldata_[i].accy = msg.obs[i].accy;
//         radar_lrr1.alldata_[i].rangex_rms = msg.obs[i].rangex_rms;
//         radar_lrr1.alldata_[i].rangey_rms = msg.obs[i].rangey_rms;
//         radar_lrr1.alldata_[i].speedx_rms = msg.obs[i].speedx_rms;
//         radar_lrr1.alldata_[i].speedy_rms = msg.obs[i].speedy_rms;
//         radar_lrr1.alldata_[i].accx_rms = msg.obs[i].accx_rms;
//         radar_lrr1.alldata_[i].accy_rms = msg.obs[i].accy_rms;
//         radar_lrr1.alldata_[i].orient_rms =msg.obs[i].orient_rms;
//         radar_lrr1.alldata_[i].objMeasState = msg.obs[i].objMeasState;
//         radar_lrr1.alldata_[i].objProbExist = msg.obs[i].objProbExist;
//         radar_lrr1.alldata_[i].objClass = msg.obs[i].objClass;
//         radar_lrr1.alldata_[i].ObjectOrientAngel = msg.obs[i].ObjectOrientAngel;
//         radar_lrr1.alldata_[i].ObjectLength = msg.obs[i].ObjectLength;
//         radar_lrr1.alldata_[i].ObjectWidth = msg.obs[i].ObjectWidth;
//     }
// }

// void copy_radar_data(
//     /* [in] */  const icv2ros::ivsensorsrrobj &msg,
//     /* [out] */  RadarSideOut &radar_sr )
// {
//     radar_sr.header_.seq = msg.header.seq;
//     radar_sr.header_.stamp = (unsigned long)(msg.header.stamp.toSec() * 1e6);

//     radar_sr.targetnum_ = msg.obs.size();
//     for (int i=0; i < msg.obs.size(); i++) {
//         radar_sr.alldata_[i].id = msg.obs[i].id;
//         radar_sr.alldata_[i].x = msg.obs[i].x;
//         radar_sr.alldata_[i].y = msg.obs[i].y;
//         radar_sr.alldata_[i].relspeedx = msg.obs[i].speedx;
//         radar_sr.alldata_[i].relspeedy = msg.obs[i].speedy;
//         radar_sr.alldata_[i].obj_amp = msg.obs[i].obj_amp;
//         radar_sr.alldata_[i].trackIndex = msg.obs[i].trackIndex;
//         radar_sr.alldata_[i].trackIndex2 = msg.obs[i].trackIndex2;
//         radar_sr.alldata_[i].trackLifeTime = msg.obs[i].trackLifeTime;
//         radar_sr.alldata_[i].flag = 0;
//     }
// }

// void long_range_radar_callback_lrr1(const icv2ros::ivsensorlrrobj &msg)
// {
//     RadarLongOut radar_lrr1;
    
//     copy_radar_data(msg, radar_lrr1);
//     icvStructureData<RadarLongOut> data;
//     data = radar_lrr1;
//     data.SetSourceTime(radar_lrr1.header_.stamp);
//     // std::cout<<"radar source time 1: "<<radar_lrr1.header_.stamp<<std::endl;
//     lock_record(&data, 5); // TODO
// }

// void long_range_radar_callback_lrr2(const icv2ros::ivsensorlrrobj &msg)
// {
//     RadarLongOut radar_lrr2;

//     copy_radar_data(msg, radar_lrr2);    
//     icvStructureData<RadarLongOut> data;
//     data = radar_lrr2;
//     data.SetSourceTime(radar_lrr2.header_.stamp);
//     // std::cout<<"radar source time 2: "<<radar_lrr2.header_.stamp<<std::endl;
//     lock_record(&data, 6);  //TODO
// }

// void short_range_radar_callback_srr1(const icv2ros::ivsensorsrrobj &msg)
// {
//     RadarSideOut radar_srr1;

//     copy_radar_data(msg, radar_srr1);
//     icvStructureData<RadarSideOut> data;
//     data = radar_srr1;
//     data.SetSourceTime(radar_srr1.header_.stamp);
//     // std::cout<<"radar3 source time: "<<radar_srr1.header_.stamp<<std::endl;
//     lock_record(&data, 7); // todo
// }

// void short_range_radar_callback_srr2(const icv2ros::ivsensorsrrobj &msg)
// {
//     RadarSideOut radar_srr2;

//     copy_radar_data(msg, radar_srr2);
//     icvStructureData<RadarSideOut> data;
//     data = radar_srr2;
//     data.SetSourceTime(radar_srr2.header_.stamp);
//     // std::cout<<"radar4 source time: "<<radar_srr2.header_.stamp<<std::endl;
//     lock_record(&data, 8);  // todo
// }

// void short_range_radar_callback_srr3(const icv2ros::ivsensorsrrobj &msg)
// {
//     RadarSideOut radar_srr3;

//     copy_radar_data(msg, radar_srr3);
//     icvStructureData<RadarSideOut> data;
//     data = radar_srr3;
//     data.SetSourceTime(radar_srr3.header_.stamp);
//     // std::cout<<"radar5 source time: "<<radar_srr3.header_.stamp<<std::endl;
//     lock_record(&data, 9); // todo
// }

// void short_range_radar_callback_srr4(const icv2ros::ivsensorsrrobj &msg)
// {
//     RadarSideOut radar_srr4;
//     copy_radar_data(msg, radar_srr4);
//     icvStructureData<RadarSideOut> data;
//     data = radar_srr4;
//     data.SetSourceTime(radar_srr4.header_.stamp);
//     // std::cout<<"radar6 source time: "<<radar_srr4.header_.stamp<<std::endl;
//     lock_record(&data, 10); // todo
// }

// // 4
// void odom_callback(const nav_msgs::Odometry &msg)
// {
//     Odometry od;

//     od.header.stamp = sec2us(msg.header.stamp.toSec());
//     memcpy(od.header.frame_id, msg.header.frame_id.c_str(), sizeof(od.header.frame_id));
//     memcpy(od.child_frame_id, msg.child_frame_id.c_str(), sizeof(od.child_frame_id));

//     od.twist.twist.linear.x = msg.twist.twist.linear.x;
//     od.twist.twist.linear.y = msg.twist.twist.linear.y;
//     od.twist.twist.linear.z = msg.twist.twist.linear.z;

//     for (int i=0; i <36; i++) {
//         od.twist.covariance[i] = msg.twist.covariance[i];
//     }

//     icvStructureData<Odometry> data;
//     data = od;
//     data.SetSourceTime(od.header.stamp);
//     // std::cout<<"odom source time: "<<od.header.stamp<<std::endl;
//     lock_record(&data, 4);
// }

// void gps_vel_callback(const geometry_msgs::TwistWithCovarianceStamped &msg)
// {
//     TwistWithCovarianceStamped gps_vel = {0};

//     gps_vel.header.stamp = sec2us(msg.header.stamp.toSec());
//     memcpy(
//         gps_vel.header.frame_id, 
//         msg.header.frame_id.c_str(), sizeof(gps_vel.header.frame_id));  

//     gps_vel.twist.twist.linear.x = msg.twist.twist.linear.x;
//     gps_vel.twist.twist.linear.y = msg.twist.twist.linear.y;
//     gps_vel.twist.twist.linear.z = msg.twist.twist.linear.z;
//     for (int i=0; i < 36; i++) {
//         gps_vel.twist.covariance[i] = msg.twist.covariance[i];
//     }

//     icvStructureData<TwistWithCovarianceStamped> data;
//     data = gps_vel;
//     data.SetSourceTime(gps_vel.header.stamp);
//     // std::cout<<"vel source time: "<<gps_vel.header.stamp<<std::endl;
//     lock_record(&data, 3);  
// }

// void imu_callback(const sensor_msgs::Imu& msg) 
// {
//     Imu imu_data;

//     imu_data.header.stamp = sec2us(msg.header.stamp.toSec());
//     memcpy(
//         imu_data.header.frame_id, 
//         msg.header.frame_id.c_str(), sizeof(imu_data.header.frame_id)); 

//     imu_data.linear_acceleration.x = msg.linear_acceleration.x;
//     imu_data.linear_acceleration.y = msg.linear_acceleration.y;
//     imu_data.linear_acceleration.z = msg.linear_acceleration.z;
//     imu_data.angular_velocity.x = msg.angular_velocity.x;
//     imu_data.angular_velocity.y = msg.angular_velocity.y;
//     imu_data.angular_velocity.z = msg.angular_velocity.z;
//     imu_data.orientation.x = msg.orientation.x;
//     imu_data.orientation.y = msg.orientation.y;
//     imu_data.orientation.z = msg.orientation.z;

//     for (int i=0; i <9; i++) {
//         imu_data.orientation_covariance[i] = msg.orientation_covariance[i];
//         imu_data.angular_velocity_covariance[i] = msg.angular_velocity_covariance[i];
//         imu_data.linear_acceleration_covariance[i] = msg.linear_acceleration_covariance[i];
//     }
    
//     icvStructureData<Imu> data;
//     data = imu_data;
//     data.SetSourceTime( imu_data.header.stamp);
//     // std::cout<<"imu source time: "<<imu_data.header.stamp<<std::endl;
//     lock_record(&data, 1); // todo
// } 

// void gps_fix_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
// {
//     //std::cout<<"starting to subscribe gps fix data"<<endl;
//     NavSatFix nav_sat_fix;

//     nav_sat_fix.header.stamp = sec2us(msg->header.stamp.toSec());
//     memcpy(
//         nav_sat_fix.header.frame_id, 
//         msg->header.frame_id.c_str(), sizeof(nav_sat_fix.header.frame_id)); 

//     nav_sat_fix.latitude = msg->latitude;
//     nav_sat_fix.longitude = msg->longitude;
//     nav_sat_fix.altitude = msg->altitude;
//     nav_sat_fix.position_covariance_type = msg->position_covariance_type;
//     nav_sat_fix.status.status = msg->status.status;
//     nav_sat_fix.status.service = msg->status.service;
//     for (int i=0; i < 9; i++) {
//         nav_sat_fix.position_covariance[i] = msg->position_covariance[i];
//     }

//     icvStructureData<NavSatFix> data;
//     data = nav_sat_fix;
//     data.SetSourceTime(nav_sat_fix.header.stamp);
//     // std::cout<<"gps source time: "<<nav_sat_fix.header.stamp<<std::endl;
//     lock_record(&data, 2); //todo
// }


// void image_callback(const sensor_msgs::ImageConstPtr& msg)
// {
//     icvCvMatData img;
//     cv_bridge::CvImagePtr cv_ptr; 
//     try  {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     } catch (cv_bridge::Exception &e)  {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     unsigned long srctime = sec2us(cv_ptr->header.stamp.toSec());
//     std::string timestamp_filename = std::to_string(srctime);
//     img.SetImageName(timestamp_filename);
//     std::string pathfilename = (imagefolderpath_/timestamp_filename).string() + ".jpg";
//     cv::imwrite(pathfilename, cv_ptr->image);
   
//     img = cv_ptr->image;
//     img.SetSourceTime(srctime);
//     // std::cout<<"img source time: "<<srctime<<std::endl;
//     lock_record(&img, 0); 
// }

// cv::Mat imgCallback;
// static void ImageCompressedCallback(const sensor_msgs::CompressedImageConstPtr &msg)
// {
//     try
//     {
//         icvCvMatData img;
//         cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
//         imgCallback = cv_ptr_compressed->image;
//       //cv::imshow("imgCallback",imgCallback);
//       //cv::waitKey(1);
//       //cout<<"cv_ptr_compressed: "<<cv_ptr_compressed->image.cols<<" h: "<<cv_ptr_compressed->image.rows<<endl;
//         unsigned long srctime = sec2us(cv_ptr_compressed->header.stamp.toSec());
//         std::string timestamp_filename = std::to_string(srctime);
//         img.SetImageName(timestamp_filename);
//         std::string pathfilename = (imagefolderpath_/timestamp_filename).string() + ".jpg";
//         cv::imwrite(pathfilename, cv_ptr_compressed->image);
    
//         img = cv_ptr_compressed->image;
//         img.SetSourceTime(srctime);
//     // std::cout<<"img source time: "<<srctime<<std::endl;
//         lock_record(&img, 0); 
//     }
//     catch (cv_bridge::Exception& e)
//     {
//       //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//     }

// }
// pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
// //laserCloudIn = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>); 
// //boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_ = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PCL velodyne Cloud"));
   
// void pointcloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud){
  
// //   pcl::PCLPointCloud2 pcl_pointcloud2;
// //   pcl_conversions::toPCL(*cloud, pcl_pointcloud2);
// //   pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
// //   pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);
//   unsigned long srctime = sec2us(cloud->header.stamp.toSec());
 
//   pcl::fromROSMsg(*cloud, *laserCloudIn);
//     // cloud_viewer_->addCoordinateSystem(3.0);
//     // cloud_viewer_->setBackgroundColor(0, 0, 0);
//     // cloud_viewer_->initCameraParameters();
//     // cloud_viewer_->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
//     // cloud_viewer_->setCameraClipDistances(0.0, 50.0);
    

     
    
    
//     // cloud_viewer_->addPointCloud<pcl::PointXYZI> (laserCloudIn,std::to_string(srctime));
//     // cloud_viewer_->spinOnce ();

//     // cloud_viewer_->removePointCloud(std::to_string(srctime));  
//     std::string timestamp_filename = std::to_string(srctime);
//     std::string pathfilename = (pointcloudpath_/timestamp_filename).string() + ".pcd";
//     ::pcl::io::savePCDFileASCII (pathfilename,*laserCloudIn);
//     icvPointCloudData<pcl::PointXYZI> pc;
//     pc = *laserCloudIn;
//     pc.SetSourceTime(srctime);
//     // std::cout<<"img source time: "<<srctime<<std::endl;
//     lock_record(&pc, 11); 

// }
#include <icv2ros_core.h>

int main(int argc, char **argv)
{
    ROS_INFO("zzz");
    ros::init(argc, argv, "icv2ros");

    ros::NodeHandle nh("~");

    icv2rosCore core(nh);

    return 0;
}



// int main(int argc, char **argv)
// {

    
//     //signal(SIGINT, signal_handler_fun); 
//     ros::init(argc, argv, "icv2ros");
//     ros::NodeHandle nh;
//     std::cout<<"starting to get message from openicv and publish"<<std::endl;
//     // ros::Subscriber radar_front_obs3d_sub = nh.subscribe("/lrr_topic2", 1, long_range_radar_callback_lrr2);
//     // ros::Subscriber radar_rear_obs3d_sub = nh.subscribe("/lrr_topic1", 1, long_range_radar_callback_lrr1);

//     // ros::Subscriber radar_right_front_obs3d_sub = nh.subscribe("/srr_topic3", 1, short_range_radar_callback_srr3);
//     // ros::Subscriber radar_right_rear_obs3d_sub = nh.subscribe("/srr_topic1", 1, short_range_radar_callback_srr1);
//     // ros::Subscriber radar_left_front_obs3d_sub = nh.subscribe("/srr_topic4", 1, short_range_radar_callback_srr4);
//     // ros::Subscriber radar_left_rear_obs3d_sub = nh.subscribe("/srr_topic2", 1, short_range_radar_callback_srr2);

//     // ros::Subscriber gps_vel_sub = nh.subscribe("/gps/vel", 1, gps_vel_callback);
//     // ros::Subscriber imu_data_sub = nh.subscribe("/imu/data", 1, imu_callback);
//     // ros::Subscriber gps_fix_sub = nh.subscribe("/gps/fix", 1, gps_fix_callback);
//     // ros::Subscriber odom_sub = nh.subscribe("/gps/odom", 1,  odom_callback);
//     // ros::Subscriber img_compressed_sub = nh.subscribe("/usb_cam_left/image_raw/compressed", 1, ImageCompressedCallback);    

//     // //ros::Subscriber img_sub = nh.subscribe("/usb_cam_left/image_raw/compressed", 1, image_callback);    
//     // ros::Subscriber lidar_sub = nh.subscribe("/velodyne_points", 1, pointcloud_callback);
//     ros::Publisher random_number_generator = nh.advertise<std_msgs::Float64>("/random_number", 10);
    
//     ros::spin();

//     while(true){
         
//          //random_number_generator.pub();
//     }
    


//     return 0;
// }

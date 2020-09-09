#pragma once

#include <ros/ros.h>

// For disable PCL complile lib, to use PointXYZIL
#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

// using eigen lib
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>

#include <chrono>

#define MIN_CLUSTER_SIZE 20
#define MAX_CLUSTER_SIZE 5000

namespace plane_ground_filter
{
struct PointXYZIL{
  PCL_ADD_POINT4D;
  float intensity;                ///< laser intensity reading
  uint16_t label;                 ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

};

#define PointT pcl::PointXYZI
#define SLRPointXYZIL plane_ground_filter::PointXYZIL
//#define RUN pcl::PointCloud<SLRPointXYZIL>

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(plane_ground_filter::PointXYZIL,(float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, label, label))

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

class PlaneGroundFilter
{
private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_, pub_cluster_points;
  std::string point_topic_;

  int sensor_model_;
  double sensor_height_, clip_height_, min_distance_, max_distance_;
  int num_seg_ = 1;
  int num_iter_, num_lpr_;
  double th_seeds_, th_dist_;
  // Model parameter for ground plane fitting
  // The ground plane model is: ax+by+cz+d=0
  // Here normal:=[a,b,c], d=d
  // th_dist_d_ = threshold_dist - d
  float d_, th_dist_d_;
  MatrixXf normal_;

  std::vector<double> seg_distance_, cluster_distance_, leaf_size_distance_;

  pcl::PointCloud<PointT>::Ptr g_seeds_pc;
  pcl::PointCloud<PointT>::Ptr g_ground_pc;
  pcl::PointCloud<PointT>::Ptr g_not_ground_pc;
  pcl::PointCloud<SLRPointXYZIL>::Ptr g_all_pc;

  void estimate_plane_(void);
  void extract_initial_seeds_(const pcl::PointCloud<PointT> &p_sorted);
  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
  void remove_pt(const pcl::PointCloud<PointT>::Ptr in_pc, const pcl::PointCloud<PointT>::Ptr out_pc);
  void voxel_grid_filter(pcl::PointCloud<PointT>::Ptr in_pc, pcl::PointCloud<PointT>::Ptr out_pc, double leaf_size);
  void cluster_by_distance(const pcl::PointCloud<PointT>::Ptr in_pc, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_pc);
  void cluster_segment(const pcl::PointCloud<PointT>::Ptr in_pc, double in_max_cluster_distance, double leaf_size, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_pc);

public:
  PlaneGroundFilter(ros::NodeHandle &nh);
  ~PlaneGroundFilter();
  void Spin();
};

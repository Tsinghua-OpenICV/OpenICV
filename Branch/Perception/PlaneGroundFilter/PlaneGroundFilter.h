#pragma once

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"

#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvSemaphore.h"

#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"

#include "OpenICV/structure/icvPointCloudData.hxx"
// For disable PCL complile lib, to use PointXYZIL
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <msgpack.hpp>

// using eigen lib
#include <Eigen/Dense>

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
struct box_info
{
  float box_x;
  float box_y;
  float box_z;
  float box_l;
  float box_w;
  float box_h;
  int box_zeta;
  MSGPACK_DEFINE(box_x, box_y,box_z,box_l,box_w,box_h,box_zeta);
};
//#define pcl::PointXYZI pcl::PointXYZI
#define SLRPointXYZIL plane_ground_filter::PointXYZIL
//#define RUN pcl::PointCloud<SLRPointXYZIL>

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(plane_ground_filter::PointXYZIL,(float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, label, label))

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

class PlaneGroundFilter:  public icvFunction 
{
private:
  //ros::Subscriber sub_point_cloud_;
  //ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_, pub_cluster_points;
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

  pcl::PointCloud<pcl::PointXYZI>::Ptr g_seeds_pc;
  pcl::PointCloud<pcl::PointXYZI>::Ptr g_ground_pc;
  pcl::PointCloud<pcl::PointXYZI>::Ptr g_not_ground_pc;
  pcl::PointCloud<SLRPointXYZIL>::Ptr g_all_pc;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> out_pc1;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> out_pc2;
  pcl::PointCloud<pcl::PointXYZRGB> out_pc_all;
  void estimate_plane_(void);
  void extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZI> &p_sorted);
  void point_cb();
  void remove_pt(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc, const pcl::PointCloud<pcl::PointXYZI>::Ptr out_pc);
  void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc, pcl::PointCloud<pcl::PointXYZI>::Ptr out_pc, double leaf_size);
  void cluster_by_distance(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc);
  void cluster_segment(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc, double in_max_cluster_distance, double leaf_size);
  //void box_make(box_info box_i, visualization_msgs::MarkerArray& marker_list);

public:
  PlaneGroundFilter(icv_shared_ptr<const icvMetaData> info);
  virtual ~PlaneGroundFilter(void);
  virtual void Execute() override;
  icv::data::icvPointCloudData<pcl::PointXYZI> *inputdata;
  icv::data::icvPointCloudData<pcl::PointXYZI> *outputdata;
  icv::data::icvPointCloudData<SLRPointXYZIL> *outputdata2;
  icv::data::icvPointCloudData<pcl::PointXYZRGB> *outputdata3;
  icv::data::icvStructureData<box_info> *outputdata4;
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn_org;
  std::string no_ground_topic, ground_topic, all_points_topic, cluster_points_topic;
};
ICV_REGISTER_FUNCTION(PlaneGroundFilter)

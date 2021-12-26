#ifndef CAR_OBSF_PCL_VISUAL_UTIL_H
#define CAR_OBSF_PCL_VISUAL_UTIL_H
#include <string>
#include <Eigen/Core>
#include "../objects/obsf_obs3d.h"
#include "../objects/obsf_obs_frame.h"
#include <pcl/visualization/pcl_visualizer.h>

namespace tsinghua{
namespace dias{
namespace fusion{
typedef pcl::PointXYZI PointT;
//compute the angle from vector v1 to vector v2
double vector_theta_2d_xy(Eigen::Vector3f& v1, Eigen::Vector3f& v2);

//计算从v2旋转到v1的旋转矩阵
Eigen::Matrix3f vector_rot_mat_2d_xy(Eigen::Vector3f& v1, Eigen::Vector3f& v2);

Eigen::Matrix3f theta_to_rot_mat_2d_xy(float theta);

void add_bbox_to_visualization(pcl::visualization::PCLVisualizer* p, 
    const Eigen::Vector3f& rgb, int viewport, 
    tsinghua::dias::fusion::OBSFObs3d &obj, const std::string& id);


}
}
}

#endif

#include "pcl_visual_util.h"

namespace tsinghua{
namespace dias{
namespace fusion{
//compute the angle from vector v1 to vector v2
double vector_theta_2d_xy(Eigen::Vector3f& v1, Eigen::Vector3f& v2) {
    double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum()); 
    double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());
    double cos_theta = (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
    double sin_theta = (v1(0) * v2(1) - v1(1) * v2(0)) / (v1_len * v2_len);
    double theta = acos(cos_theta);
    if (sin_theta < 0) {
        theta = -theta;
    }
    //std::cout << "save theta : " << theta << std::endl;
    return theta;
}

//计算从v2旋转到v1的旋转矩阵
Eigen::Matrix3f vector_rot_mat_2d_xy(Eigen::Vector3f& v1, Eigen::Vector3f& v2) {
    double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum()); 
    double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum()); 
    double cos_theta = (v1.head(2).cwiseProduct(v2.head(2))).sum()
                       / (v1_len * v2_len);
    double sin_theta = (v1(0) * v2(1) - v1(1) * v2(0))
                       / (v1_len * v2_len);

    Eigen::Matrix3f rot_mat;
    rot_mat << cos_theta, sin_theta, 0,
              -sin_theta, cos_theta, 0,
                      0,          0, 1; 
    //std::cout << "drew theta : " << acos(cos_theta) << std::endl;
    return rot_mat;
}

Eigen::Matrix3f theta_to_rot_mat_2d_xy(float theta) {
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    Eigen::Matrix3f rot_mat;
    rot_mat << cos_theta,  sin_theta, 0,
               -sin_theta, cos_theta, 0,
                    0,             0, 1;
    
    return rot_mat;
}

void add_bbox_to_visualization(pcl::visualization::PCLVisualizer* p, 
    const Eigen::Vector3f& rgb, int viewport, 
    tsinghua::dias::fusion::OBSFObs3d &obj, const std::string& id) {
    
    Eigen::Vector3f coord_dir(0.0, 1.0, 0.0);
    float theta = obj._obs_theta;
    Eigen::Matrix3f rot = theta_to_rot_mat_2d_xy(theta);

    Eigen::Vector3f cnt = Eigen::Vector3f(obj._obs_position._x, 
        obj._obs_position._y, obj._obs_position._z);
    Eigen::Vector3f size = Eigen::Vector3f(obj._length, obj._width, obj._height);

    float len = size[0] / 2.0;
    float wid = size[1] / 2.0;
    typename pcl::PointCloud<PointT>::Ptr bbox(new pcl::PointCloud<PointT>);
    bbox->points.resize(4);

    Eigen::Vector3f local_pt;
    local_pt = Eigen::Vector3f(-wid, -len, 0);
    local_pt = rot * local_pt;
    local_pt += cnt;
    bbox->points[0].x = local_pt[0];
    bbox->points[0].y = local_pt[1];
    bbox->points[0].z = local_pt[2];

    local_pt = Eigen::Vector3f(-wid, len, 0);
    local_pt = rot * local_pt;
    local_pt += cnt;
    bbox->points[1].x = local_pt[0];
    bbox->points[1].y = local_pt[1];
    bbox->points[1].z = local_pt[2];

    local_pt = Eigen::Vector3f(wid, len, 0);
    local_pt = rot * local_pt;
    local_pt += cnt;
    bbox->points[2].x = local_pt[0];
    bbox->points[2].y = local_pt[1];
    bbox->points[2].z = local_pt[2];

    local_pt = Eigen::Vector3f(wid, -len, 0);
    local_pt = rot * local_pt;
    local_pt += cnt;
    bbox->points[3].x = local_pt[0];
    bbox->points[3].y = local_pt[1];
    bbox->points[3].z = local_pt[2];
    
    std::ostringstream oss_bbox;
    oss_bbox << id << "_bbox";
    p->addPolygon<PointT>(bbox, rgb[0], rgb[1], rgb[2], oss_bbox.str(), viewport); 
    p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
             2, oss_bbox.str(), viewport); 

    for (int i = 0; i < bbox->points.size(); ++i) {
        PointT vp = bbox->points[i];
        bbox->points[i].z += size[2];
        std::ostringstream oss_vet;
        oss_vet << id << "_bbox_vet_" << i;
        p->addLine<PointT, PointT>(vp, bbox->points[i], 
                rgb[0], rgb[1], rgb[2], oss_vet.str(), viewport);
        p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                                2, oss_vet.str(), viewport);
    } 

    std::ostringstream oss_bbox_up;
    oss_bbox_up << id << "_bbox_up";
    p->addPolygon<PointT>(bbox, rgb[0], rgb[1], rgb[2], oss_bbox_up.str(), viewport); 
    p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
             2, oss_bbox_up.str(), viewport); 
}

}
}
}

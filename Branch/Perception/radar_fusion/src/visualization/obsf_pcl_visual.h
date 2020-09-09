#ifndef OBSF_PCL_VISUAL_H
#define OBSF_PCL_VISUAL_H
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>
#include "../objects/obsf_obs_frame.h"
#include "../objects/obsf_obs3d.h"
#include "../objects/obsf_track.h"
#include <Eigen/Core>

namespace tsinghua{
namespace dias{
namespace fusion{

const float g_velodyne_height = 1.7f;
typedef pcl::PointXYZI PointT;

class ObsfVisual{
public:
    typedef tsinghua::dias::fusion::OBSFObs3d OBSFObs3d;
    typedef tsinghua::dias::fusion::OBSFObs3dFrame OBSFObs3dFrame;
    typedef tsinghua::dias::fusion::OBSFTrack OBSFTrack;
    typedef std::shared_ptr<OBSFObs3d> Obs3dPtr;

public:
    ObsfVisual();
    ~ObsfVisual();

    void add_point_cloud(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
            int r, int g, int b,
            int point_size, const std::string& id, int viewport);
    void add_point_cloud(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
            int r, int g, int b,
            int point_size, const std::string& id, int viewport);
    void add_point_cloud_new(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        int r, int g, int b, const Eigen::Matrix4d& pose_velo2w,
        int point_size, const std::string& id, int viewport); 


    void visualize_fusion_result(OBSFObs3dFrame& lidar_frame, OBSFObs3dFrame& radar_frame,
        std::vector<OBSFTrack> tracks, const Eigen::Matrix4d& pose_velo2w, int frame_id);

    void visualize_lidar_frame(OBSFObs3dFrame& lidar_frame);

    void visualize_radar_frame(OBSFObs3dFrame& radar_frame);

    void visualize_tracks_lidar(std::vector<tsinghua::dias::fusion::OBSFTrack>& tracks);

    void visualize_tracks_radar(std::vector<tsinghua::dias::fusion::OBSFTrack>& tracks);

    void visualize_tracks(std::vector<tsinghua::dias::fusion::OBSFTrack>& tracks);

    void remove_points();

    void initialize_pcl_visualizer();

////
template <typename PointType>
void transform_point_cloud(const pcl::PointCloud<PointType>& cloud_in, 
        pcl::PointCloud<PointType>& cloud_out, 
        const Eigen::Matrix4d& trans_mat) {
    if (cloud_out.points.size() < cloud_in.points.size()) {
        cloud_out.points.resize(cloud_in.points.size());
    }
    for (int i = 0; i < cloud_in.size(); ++i) {
        const PointType& p = cloud_in.at(i);
        Eigen::Vector4d v(p.x, p.y, p.z, 1);
        v = trans_mat * v;
        PointType& pd = cloud_out.points[i];
        pd.x = v.x(); 
        pd.y = v.y(); 
        pd.z = v.z();
    }
}

protected:
public:
    boost::shared_ptr<pcl::visualization::PCLVisualizer>   _pcl_vs;
    int _pcl_vp;
    std::string                                            _output_directory;

    Eigen::Vector3f                                        _camera_center;
    Eigen::Vector3f                                        _view_point;
    Eigen::Vector3f                                        _up;

    Eigen::Vector3f                                        _main_car_points_local[9];

    int index;

        //
    int total;
    int dynamic;
    bool account;
};
} //
} //
} //
#endif

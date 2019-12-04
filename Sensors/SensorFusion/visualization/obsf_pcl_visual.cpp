#include "obsf_pcl_visual.h"
#include "pcl_visual_util.h"

namespace tsinghua{
namespace dias{
namespace fusion{

ObsfVisual::ObsfVisual() {
    //initialize_pcl_visualizer();
    // _pcl_vs = new pcl::visualization::PCLVisualizer("lidar_radar_fusion");
    // _pcl_vs->setSize(2000, 1400);
    // _pcl_vs->addCoordinateSystem(1.0, 0);
    // _pcl_vs->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // _pcl_vp = 0;
	index = 0;
    total = 0;
    dynamic = 0;
    account = true;
}
ObsfVisual::~ObsfVisual() 
{
    if(account)
        std::cout << "error rate is " << dynamic * 1.0 / total << std::endl;

}


void ObsfVisual::add_point_cloud(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        int r, int g, int b,
        int point_size, const std::string& id, int viewport) {

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> handle(cloud, r, g, b);
    _pcl_vs->addPointCloud(cloud, handle, id, viewport);
    _pcl_vs->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            point_size, id, viewport);
    _pcl_vs->spinOnce();


 //   _pcl_vs->setCameraPosition(_camera_center[0], _camera_center[1], _camera_center[2], // + 30,
      //  _view_point[0], _view_point[1], _view_point[2]);

   // index++;
   // std::stringstream file_name;
   // file_name << "/home/chunlei/Documents/data/screen_shot_images/fast/"<< index << ".png";

    //std::cout << "!!!! " << file_name << std::endl;
   // _pcl_vs->saveScreenshot(file_name.str());

}

void ObsfVisual::add_point_cloud(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        int r, int g, int b,
        int point_size, const std::string& id, int viewport) {

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handle(cloud, r, g, b);
    _pcl_vs->addPointCloud(cloud, handle, id, viewport);
    _pcl_vs->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            point_size, id, viewport);
    _pcl_vs->spinOnce();


 //   _pcl_vs->setCameraPosition(_camera_center[0], _camera_center[1], _camera_center[2], // + 30,
      //  _view_point[0], _view_point[1], _view_point[2]);

   // index++;
   // std::stringstream file_name;
   // file_name << "/home/chunlei/Documents/data/screen_shot_images/fast/"<< index << ".png";

    //std::cout << "!!!! " << file_name << std::endl;
   // _pcl_vs->saveScreenshot(file_name.str());

}

void ObsfVisual::add_point_cloud_new(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        int r, int g, int b, const Eigen::Matrix4d& pose_velo2w,
        int point_size, const std::string& id, int viewport) 
{

    Eigen::Vector3f vect(_camera_center[0], _camera_center[1], _camera_center[2] - 10);
    // if (use_global_translation) {
    //     trans_from_eigen_vector(vect, pose_velo2w);
    // } else
    {
        Eigen::Vector4d vect4(vect[0], vect[1], vect[2], 1);
        vect4 = pose_velo2w * vect4;
        vect[0] = vect4[0];
        vect[1] = vect4[1];
        vect[2] = vect4[2];
    }

    PointT camera_center_w;
    camera_center_w.x = vect[0];
    camera_center_w.y = vect[1];
    camera_center_w.z = vect[2];

    vect = _view_point;
    // if (use_global_translation) {
    //     trans_from_eigen_vector(vect, pose_velo2w);
    // } else {
    Eigen::Vector4d vect4(vect[0], vect[1], vect[2], 1);
    vect4 = pose_velo2w * vect4;
    vect[0] = vect4[0];
    vect[1] = vect4[1];
    vect[2] = vect4[2];
    //}

    PointT view_point_w;
    view_point_w.x = vect[0];
    view_point_w.y = vect[1];
    view_point_w.z = vect[2];

    typename pcl::PointCloud<PointT>::Ptr w_car_points(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr v_car_points(new pcl::PointCloud<PointT>);
    v_car_points->resize(9);
    for (int j = 0; j < 9; j++){
        v_car_points->points[j].x = _main_car_points_local[j][0];
        v_car_points->points[j].y = _main_car_points_local[j][1];
        v_car_points->points[j].z = _main_car_points_local[j][2];
    }


	transform_point_cloud<PointT>(*v_car_points, *w_car_points, pose_velo2w);

    typename pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);    
	transform_point_cloud<pcl::PointXYZI>(*cloud, *transformed_cloud, pose_velo2w);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> handle(cloud, r, g, b);
    _pcl_vs->addPointCloud(transformed_cloud, handle, id, viewport);
    
    _pcl_vs->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            point_size, id, viewport);
    _pcl_vs->setCameraPosition(camera_center_w.x, camera_center_w.y, camera_center_w.z, // + 30,
        view_point_w.x, view_point_w.y, view_point_w.z, //1, 0, 0);
        w_car_points->points[3].x - w_car_points->points[4].x,
        w_car_points->points[3].y - w_car_points->points[4].y,
        w_car_points->points[3].z - w_car_points->points[4].z);

	_pcl_vs->spinOnce();
}


void ObsfVisual::visualize_fusion_result(OBSFObs3dFrame& lidar_frame, OBSFObs3dFrame& radar_frame,
    std::vector<OBSFTrack> tracks, const Eigen::Matrix4d& pose_velo2w, int frame_id) {

    std::ostringstream oss_prefix;
    oss_prefix << std::setfill('0') << std::setw(6) << frame_id;
    Eigen::Vector3f vect(_camera_center[0], _camera_center[1], _camera_center[2] - 10);
    // if (use_global_translation) {
    //     trans_from_eigen_vector(vect, pose_velo2w);
    // } else
    {
        Eigen::Vector4d vect4(vect[0], vect[1], vect[2], 1);
        vect4 = pose_velo2w * vect4;
        vect[0] = vect4[0];
        vect[1] = vect4[1];
        vect[2] = vect4[2];
    }

    PointT camera_center_w;
    camera_center_w.x = vect[0];
    camera_center_w.y = vect[1];
    camera_center_w.z = vect[2];

    vect = _view_point;
    // if (use_global_translation) {
    //     trans_from_eigen_vector(vect, pose_velo2w);
    // } else {
    Eigen::Vector4d vect4(vect[0], vect[1], vect[2], 1);
    vect4 = pose_velo2w * vect4;
    vect[0] = vect4[0];
    vect[1] = vect4[1];
    vect[2] = vect4[2];
    //}

    PointT view_point_w;
    view_point_w.x = vect[0];
    view_point_w.y = vect[1];
    view_point_w.z = vect[2];

    typename pcl::PointCloud<PointT>::Ptr w_car_points(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr v_car_points(new pcl::PointCloud<PointT>);
    v_car_points->resize(9);
    for (int j = 0; j < 9; j++){
        v_car_points->points[j].x = _main_car_points_local[j][0];
        v_car_points->points[j].y = _main_car_points_local[j][1];
        v_car_points->points[j].z = _main_car_points_local[j][2];
    }

    // if (use_global_translation){
    //     transform_perception_cloud(v_car_points, pose_velo2w, w_car_points);
    // } else {
    transform_point_cloud<PointT>(*v_car_points, *w_car_points, pose_velo2w);
    //}
    _pcl_vs->setCameraPosition(camera_center_w.x, camera_center_w.y, camera_center_w.z, // + 30,
        view_point_w.x, view_point_w.y, view_point_w.z, //1, 0, 0);
        w_car_points->points[3].x - w_car_points->points[4].x,
        w_car_points->points[3].y - w_car_points->points[4].y,
        w_car_points->points[3].z - w_car_points->points[4].z);

    std::ostringstream vd_oss;
    vd_oss << "main_car";
    std::string disp_text = vd_oss.str();
    _pcl_vs->addText3D(disp_text, view_point_w, 0.8, 1, 0, 0, "main_car", 0);
    // visualize_lidar_frame(lidar_frame);
    // visualize_radar_frame(radar_frame);
    visualize_tracks_lidar(tracks);
    visualize_tracks_radar(tracks);
    visualize_tracks(tracks);
    _pcl_vs->spinOnce();
    static int file_idx = 0;
    // std::stringstream ss;
    // ss << file_idx;
    char buffer[32];
    snprintf(buffer, 32, "%08d", file_idx);
    std::string ss(buffer);
    // std::string file_name = "/home/huiyujiang/fusion_result/" + ss + ".png";
    // _pcl_vs->saveScreenshot(file_name);
    file_idx++;
    _pcl_vs->removeAllPointClouds();
    _pcl_vs->removeAllShapes();
}

void ObsfVisual::visualize_lidar_frame(OBSFObs3dFrame& lidar_frame) 
{
    std::vector<OBSFObs3d>& obs = lidar_frame.get_obstacles();
    Eigen::Vector3f rgb(1, 0, 0);
    std::string bbox_id;
    std::string pc_id;
    for (int i = 0; i < (int)obs.size(); i++) {
        std::ostringstream oss;
        oss << "lidar_obj_" << i;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->resize(obs[i]._cloud_points.size());
        for (int j = 0; j < (int)obs[i]._cloud_points.size(); j++) {
            cloud->points[j].x = obs[i]._cloud_points[j]._x;
            cloud->points[j].y = obs[i]._cloud_points[j]._y;
            cloud->points[j].z = obs[i]._cloud_points[j]._z;
        }
        add_point_cloud(cloud, 180, 180, 180, 1, oss.str(), 0);

        add_bbox_to_visualization(_pcl_vs.get(), rgb, 0, obs[i], oss.str());
    }
}

void ObsfVisual::visualize_radar_frame(OBSFObs3dFrame& radar_frame) {
	
    _pcl_vs->removeAllShapes();
    std::vector<OBSFObs3d>& obs = radar_frame.get_obstacles();
    Eigen::Vector3f rgb1(0, 1, 0);	// green
    Eigen::Vector3f rgb2(1, 1, 0);	// yellow
    std::string bbox_id;
    for (int i = 0; i < (int)obs.size(); i++)
    {
        std::ostringstream oss;
        oss << "radar_obj_" << i;

    // calculate the position change in this time interval    
    Eigen::Vector2d currentPos(obs[i]._obs_position._x, obs[i]._obs_position._y);//
    //Eigen::Vector2d previousPos(obs[i]._obs_previous_position._x, obs[i]._obs_previous_position._y);
    //Eigen::Vector2d diff = currentPos - previousPos;
    //double dis = diff.norm();

    // change the threshold according to the distance of the obstacle
    double thres = 0;
    if(obs[i]._range <= 10) 
        thres = 0.5;
    else if(obs[i]._range <= 30)
        thres = 0.7;
    // else if(obs[i]._range <= 30)
    //     thres = 0.9;
    else
        thres = 1.0;

    double speed = obs[i]._velocity._x * obs[i]._velocity._x + obs[i]._velocity._y * obs[i]._velocity._y; 
    speed = sqrt(speed); 

    //bool static_obs = (speed <= 0.2 || (speed <= 0.5 && dis <= thres));
    if(account)
        total++;

    //if(!static_obs)
    //{
        add_bbox_to_visualization(_pcl_vs.get(), rgb1, 0, obs[i], oss.str());
        //dynamic++;
    //}
    //else
        //add_bbox_to_visualization(_pcl_vs.get(), rgb2, 0, obs[i], oss.str());

        std::ostringstream vd_oss;

        vd_oss << "(" << obs[i]._obs_id << ","
               << std::setprecision(5) << obs[i]._velocity._x << "  ," 
               << std::setprecision(5) << obs[i]._velocity._y << "  ,"
	       //<< std::setprecision(5) << obs[i].rcs << " "	// show the rcs
               //<< dis
               <<  ")";

        PointT loc;
        loc.x = obs[i]._obs_position._x ;
        loc.y = obs[i]._obs_position._y + 2;
        loc.z = obs[i]._obs_position._z;
        std::string id = oss.str();
        std::string disp_text = vd_oss.str();
	//if(!static_obs)
            _pcl_vs->addText3D(disp_text, loc, 0.5, rgb1[0], rgb1[1], rgb1[2], id + "_radar_text", 0);
	//else
	    //_pcl_vs->addText3D(disp_text, loc, 0.5, rgb2[0], rgb2[1], rgb2[2], id + "_radar_text", 0);
    }

    _pcl_vs->spinOnce();

/*
    index++;
    std::stringstream file_name;
    file_name << "/home/chunlei/Documents/duData/screen_shot/rcs1/"<< index << ".png";

    //std::cout << "!!!! " << file_name << std::endl;
    _pcl_vs->saveScreenshot(file_name.str());
*/
}

void ObsfVisual::visualize_tracks_lidar(std::vector<tsinghua::dias::fusion::OBSFTrack>& tracks) {
    Eigen::Vector3f rgb(0, 1, 1);
    for (int i = 0; i < (int)tracks.size(); i++) {
        Obs3dPtr obs_ptr = tracks[i].get_obs_lidar();
        if (obs_ptr == NULL) {
            continue;
        }
        std::ostringstream oss;
        oss << "track_lidar_" << i;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->resize(obs_ptr->_cloud_points.size());
        for (int j = 0; j < (int)obs_ptr->_cloud_points.size(); j++) {
            cloud->points[j].x = obs_ptr->_cloud_points[j]._x;
            cloud->points[j].y = obs_ptr->_cloud_points[j]._y;
            cloud->points[j].z = obs_ptr->_cloud_points[j]._z;
        }
        add_point_cloud(cloud, 255, 255, 0, 1, oss.str(), 0);
        //add_bbox_to_visualization(_pcl_vs.get(), rgb, 0, *obs_ptr, oss.str());

        double speed = obs_ptr->_velocity._x * obs_ptr->_velocity._x +
                             obs_ptr->_velocity._y * obs_ptr->_velocity._y;
        speed = sqrt(speed);

        Obs3dPtr obs_ptr2 = tracks[i].get_obs();
        std::ostringstream vd_oss;
        vd_oss << "(" << obs_ptr2->_obs_id << ","
               << std::setprecision(2) << speed <<  ")";
        PointT loc;
        loc.x = obs_ptr2->_obs_position._x + 2;
        loc.y = obs_ptr2->_obs_position._y;
        loc.z = obs_ptr2->_obs_position._z;
        std::string id = oss.str();
        std::string disp_text = vd_oss.str();
        _pcl_vs->addText3D(disp_text, loc, 0.5, rgb[0], rgb[1], rgb[2], id + "_lidar_text", 0);
    }
}

void ObsfVisual::visualize_tracks_radar(std::vector<tsinghua::dias::fusion::OBSFTrack>& tracks) {
    Eigen::Vector3f rgb(0, 1, 0);
    for (int i = 0; i < (int)tracks.size(); i++) {
        Obs3dPtr obs_ptr = tracks[i].get_obs_radar();
        if (obs_ptr == NULL) {
            continue;
        }
        std::ostringstream oss;
        oss << "track_radar_" << i;
        //add_bbox_to_visualization(_pcl_vs.get(), rgb, 0, *obs_ptr, oss.str());

        double speed = obs_ptr->_velocity._x * obs_ptr->_velocity._x +
                             obs_ptr->_velocity._y * obs_ptr->_velocity._y;
        speed = sqrt(speed);

        Obs3dPtr obs_ptr2 = tracks[i].get_obs();
        std::ostringstream vd_oss;
        vd_oss << "(" << obs_ptr2->_obs_id << ","
               << std::setprecision(2) << speed <<  ")";
        PointT loc;
        loc.x = obs_ptr2->_obs_position._x - 2;
        loc.y = obs_ptr2->_obs_position._y;
        loc.z = obs_ptr2->_obs_position._z;
        std::string id = oss.str();
        std::string disp_text = vd_oss.str();
        _pcl_vs->addText3D(disp_text, loc, 0.5, rgb[0], rgb[1], rgb[2], id + "_radar_text", 0);
    }
}

void ObsfVisual::visualize_tracks(std::vector<tsinghua::dias::fusion::OBSFTrack>& tracks) {
    Eigen::Vector3f rgb(1, 0, 0);
    for (int i = 0; i < (int)tracks.size(); i++) {
        Obs3dPtr obs_ptr = tracks[i].get_obs();
        std::ostringstream oss;
        oss << "track_" << i;
        //add_bbox_to_visualization(_pcl_vs.get(), rgb, 0, *obs_ptr, oss.str());
        double speed = obs_ptr->_velocity._x * obs_ptr->_velocity._x +
                             obs_ptr->_velocity._y * obs_ptr->_velocity._y;
        speed = sqrt(speed);

        std::ostringstream vd_oss;
        vd_oss << "(" << obs_ptr->_obs_id << ","
               << std::setprecision(2) << speed <<  ")";
        PointT loc;
        loc.x = obs_ptr->_obs_position._x;
        loc.y = obs_ptr->_obs_position._y;
        loc.z = obs_ptr->_obs_position._z;
        std::string id = oss.str();
        std::string disp_text = vd_oss.str();
        _pcl_vs->addText3D(disp_text, loc, 0.5, rgb[0], rgb[1], rgb[2], id + "_text", 0);
    }
}

void ObsfVisual::remove_points() {
    _pcl_vs->removeAllPointClouds();
}

void ObsfVisual::initialize_pcl_visualizer() {
    _pcl_vs = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
        new pcl::visualization::PCLVisualizer("lidar_radar_fusion"));
    _pcl_vs->setSize(2500, 1400);
    _pcl_vs->addCoordinateSystem(1.0, 0);
    _pcl_vs->setBackgroundColor(0.05, 0.05, 0.05, 0.0f);
    _pcl_vp = 0;

    _main_car_points_local[0][0] = 0;
    _main_car_points_local[0][1] = 0;
    _main_car_points_local[0][2] = 0;

    _main_car_points_local[1][0] = 0;
    _main_car_points_local[1][1] = 0;
    _main_car_points_local[1][2] = -g_velodyne_height;
    _main_car_points_local[2][0] = 3;
    _main_car_points_local[2][1] = 0;
    _main_car_points_local[2][2] = -g_velodyne_height;

    _main_car_points_local[3][0] = 2.5;
    _main_car_points_local[3][1] = 1.0;
    _main_car_points_local[3][2] = -g_velodyne_height;
    _main_car_points_local[4][0] = 2.5;
    _main_car_points_local[4][1] = -1.0;
    _main_car_points_local[4][2] = -g_velodyne_height;
    _main_car_points_local[5][0] = -2.5;
    _main_car_points_local[5][1] = -1.0;
    _main_car_points_local[5][2] = -g_velodyne_height;
    _main_car_points_local[6][0] = -2.5;
    _main_car_points_local[6][1] = 1.0;
    _main_car_points_local[6][2] = -g_velodyne_height;

    _main_car_points_local[7][0] = 0;
    _main_car_points_local[7][1] = 0;
    _main_car_points_local[7][2] = 160;
    _main_car_points_local[8][0] = -40;
    _main_car_points_local[8][1] = 0;
    _main_car_points_local[8][2] = 50;

    _camera_center = _main_car_points_local[7];
    _view_point = Eigen::Vector3f(0, 0, 0);
    _up = Eigen::Vector3f(0, 1, 0);
}
} 
}
} 

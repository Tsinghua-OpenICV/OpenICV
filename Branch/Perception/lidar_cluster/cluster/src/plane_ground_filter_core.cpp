#include "plane_ground_filter_core.h"

bool point_cmp(PointT a, PointT b)
{
    return a.z < b.z;
}

int *rand_rgb()
{
  int *rgb = new int[3];
  rgb[0] = rand() % 255;
  rgb[1] = rand() % 255;
  rgb[2] = rand() % 255;
  return rgb;
}

PlaneGroundFilter::PlaneGroundFilter(ros::NodeHandle &nh)
{
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    sub_point_cloud_ = nh.subscribe("/velodyne_points", 10, &PlaneGroundFilter::point_cb, this);

    std::string no_ground_topic, ground_topic, all_points_topic, cluster_points_topic;

    nh.getParam("no_ground_point_topic", no_ground_topic);
    nh.getParam("ground_point_topic", ground_topic);
    nh.getParam("all_points_topic", all_points_topic);
    nh.getParam("cluster_points_topic", cluster_points_topic);

    nh.getParam("clip_height", clip_height_);
    ROS_INFO("clip_height: %f", clip_height_);
    nh.getParam("sensor_height", sensor_height_);
    ROS_INFO("sensor_height: %f", sensor_height_);
    nh.getParam("min_distance", min_distance_);
    ROS_INFO("min_distance: %f", min_distance_);
    nh.getParam("max_distance", max_distance_);
    ROS_INFO("max_distance: %f", max_distance_);

    nh.getParam("sensor_model", sensor_model_);
    ROS_INFO("sensor_model: %d", sensor_model_);
    nh.getParam("num_iter", num_iter_);
    ROS_INFO("num_iter: %d", num_iter_);
    nh.getParam("num_lpr", num_lpr_);
    ROS_INFO("num_lpr: %d", num_lpr_);
    nh.getParam("th_seeds", th_seeds_);
    ROS_INFO("th_seeds: %f", th_seeds_);
    nh.getParam("th_dist", th_dist_);
    ROS_INFO("th_dist: %f", th_dist_);

    seg_distance_ = {20, 35, 50, 70};
    cluster_distance_ = {0.6, 1.2, 1.5, 2.0, 2.5};
    leaf_size_distance_ = {0.15, 0.12, 0.10, 0.08, 0.06};

    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
    pub_all_points_ = nh.advertise<sensor_msgs::PointCloud2>(all_points_topic, 10);
    pub_cluster_points = nh.advertise<sensor_msgs::PointCloud2>(cluster_points_topic, 10);

    g_seeds_pc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    g_ground_pc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    g_not_ground_pc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    g_all_pc = pcl::PointCloud<SLRPointXYZIL>::Ptr(new pcl::PointCloud<SLRPointXYZIL>);

    ros::spin();
}

PlaneGroundFilter::~PlaneGroundFilter() {}

void PlaneGroundFilter::Spin(){}

void PlaneGroundFilter::remove_pt(const pcl::PointCloud<PointT>::Ptr in_pc, const pcl::PointCloud<PointT>::Ptr out_pc)
{
    pcl::ExtractIndices<PointT> rem;

    rem.setInputCloud(in_pc);
    pcl::PointIndices indices;

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
      double distance = sqrt(in_pc->points[i].x * in_pc->points[i].x + in_pc->points[i].y * in_pc->points[i].y);

      if (in_pc->points[i].z > clip_height_ || distance < min_distance_ || distance > max_distance_)
        {
            indices.indices.push_back(i);
        }
    }
    rem.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    rem.setNegative(true); //ture to remove the indices
    rem.filter(*out_pc);
}

void PlaneGroundFilter::estimate_plane_(void)
{
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);   //计算均值和协方差矩阵
    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal   使用最小奇异向量作为法向量
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;      //th_dist = 0.3

    // return the equation parameters
}


void PlaneGroundFilter::extract_initial_seeds_(const pcl::PointCloud<PointT> &p_sorted)
{
    double sum = 0;
    int cnt = 0;
    for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++)     //num_lpr = 15 20
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;
    g_seeds_pc->clear();

    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seeds_)    //th_seeds = 1.2
        {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }

}

void PlaneGroundFilter::cluster_by_distance(pcl::PointCloud<PointT>::Ptr in_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_pc)
{
    std::vector<pcl::PointCloud<PointT>::Ptr> segment_pc_array(5);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        PointT current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;
        current_point.intensity = in_pc->points[i].intensity;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        // 如果点的距离大于100m, 忽略该点
        if (origin_distance >= 100)
        {
            continue;
        }

        if (origin_distance < seg_distance_[0])
        {
            segment_pc_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1])
        {
            segment_pc_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2])
        {
            segment_pc_array[2]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[3])
        {
            segment_pc_array[3]->points.push_back(current_point);
        }
        else
        {
            segment_pc_array[4]->points.push_back(current_point);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr every_cluster_points(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
      cluster_segment(segment_pc_array[i], cluster_distance_[i], leaf_size_distance_[i], every_cluster_points);
      *out_pc += *every_cluster_points;
    }
}


void PlaneGroundFilter::cluster_segment(pcl::PointCloud<PointT>::Ptr in_pc, double in_max_cluster_distance, double leaf_size, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_pc)
{
    pcl::PointCloud<PointT>::Ptr temp_points(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(in_pc);
    vox.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox.filter(*temp_points);

    std::vector<pcl::PointIndices> local_indices;
    pcl::EuclideanClusterExtraction<PointT> euc;
    euc.setInputCloud(temp_points);
    euc.setClusterTolerance(in_max_cluster_distance);
    euc.setMinClusterSize(MIN_CLUSTER_SIZE);
    euc.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euc.extract(local_indices);

    if (local_indices.empty())
       return;

    pcl::PointCloud<pcl::PointXYZRGB> cloud_cluster_point;

    for(std::vector<pcl::PointIndices>::const_iterator it = local_indices.begin(); it != local_indices.end(); ++it)
    {
      pcl::PointXYZRGB point_temp;
      int *rgb = rand_rgb();

      for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
        point_temp.x = temp_points->points[*pit].x;
        point_temp.y = temp_points->points[*pit].y;
        point_temp.z = temp_points->points[*pit].z;
        point_temp.b = rgb[0];
        point_temp.g = rgb[1];
        point_temp.r = rgb[2];

        cloud_cluster_point.push_back(point_temp);
      }
    }

    *out_pc = cloud_cluster_point;
}


void PlaneGroundFilter::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    auto t1 = std::chrono::steady_clock::now();

    pcl::PointCloud<PointT> laserCloudIn;
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn);

    pcl::PointCloud<PointT> laserCloudIn_org;
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn_org);

    SLRPointXYZIL point;

    for (size_t i = 0; i < laserCloudIn.points.size(); i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.label = 0u; // 0 means uncluster
        g_all_pc->points.push_back(point);
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp);

    pcl::PointCloud<PointT>::iterator it = laserCloudIn.points.begin();
    for (int i = 0; i < laserCloudIn.points.size(); i++)
    {
        if (laserCloudIn.points[i].z < -1.5 * sensor_height_)
        {
            it++;
        }
        else
        {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);

    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;

    for (int i = 0; i < num_iter_; i++)
    {
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        MatrixXf points(laserCloudIn_org.points.size(), 3);
        int j = 0;
        for (auto p : laserCloudIn_org.points)
        {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (result[r] < th_dist_d_)
            {
                g_all_pc->points[r].label = 1u; // means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
            else
            {
                g_all_pc->points[r].label = 0u; // means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }

    pcl::PointCloud<PointT>::Ptr final_no_ground(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr final_vox(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZRGB>);

    remove_pt(g_not_ground_pc, final_no_ground);
    auto t2 = std::chrono::steady_clock::now();

  //  voxel_grid_filter(final_no_ground, final_vox, 0.13);

    cluster_by_distance(final_no_ground, cluster_points);

    auto t3 = std::chrono::steady_clock::now();

    // publish ground points
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud_ptr->header.stamp;
    ground_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_ground_.publish(ground_msg);

    // publish not ground points
    sensor_msgs::PointCloud2 groundless_msg;
    pcl::toROSMsg(*final_no_ground, groundless_msg);
    groundless_msg.header.stamp = in_cloud_ptr->header.stamp;
    groundless_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_no_ground_.publish(groundless_msg);

    // publish all points
    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud_ptr->header.stamp;
    all_points_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_all_points_.publish(all_points_msg);
    g_all_pc->clear();

    // publish cluster points
    sensor_msgs::PointCloud2 cluster_msg;
    pcl::toROSMsg(*cluster_points, cluster_msg);
    cluster_msg.header.stamp = in_cloud_ptr->header.stamp;
    cluster_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_cluster_points.publish(cluster_msg);

    auto delta_t1 = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    auto delta_t2 = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);

    std::cout << "----------------------" << std::endl;
    std::cout << "The cost time of lidar filter ground is: " << delta_t1.count() * 1000 << "ms" << std::endl;
    if (delta_t1.count() * 1000.0 > 100.0)
      ROS_WARN("The lidar filter cost time is more 100ms.");

    std::cout << "----------------------" << std::endl;
    std::cout << "The cost time of lidar cluster is: " << delta_t2.count() * 1000 << "ms" << std::endl;
    if (delta_t2.count() * 1000.0 > 100.0)
      ROS_WARN("The lidar filter cost time is more 100ms.");
}

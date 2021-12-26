#include "PlaneGroundFilter.h"

bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b)
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

PlaneGroundFilter::PlaneGroundFilter(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
{
    ICV_LOG_INFO<<"Plane Ground Filter Started";
    Register_Sub("robosense_points");
    Register_Pub("Bounding_Box3D");

    
    no_ground_topic="points_no_ground";Register_Pub(no_ground_topic);
    ground_topic="points_ground";Register_Pub(ground_topic);
    all_points_topic="all_points";Register_Pub(all_points_topic);
    cluster_points_topic="cluster_points";Register_Pub(cluster_points_topic);
    clip_height_=4.0;
    sensor_height_=1.77;
    min_distance_=2.0;
    max_distance_=100.0;
    sensor_model_=64;
    num_iter_=3;
    num_lpr_=15;
    th_seeds_=1.2;
    th_dist_=0.30;
    seg_distance_ = {20, 35, 50, 70};
    cluster_distance_ = {0.6, 1.2, 1.5, 2.0, 2.5};
    leaf_size_distance_ = {0.15, 0.12, 0.10, 0.08, 0.06};
    inputdata=new icv::data::icvPointCloudData<pcl::PointXYZI>();
    outputdata=new icv::data::icvPointCloudData<pcl::PointXYZI>();
    outputdata2=new icv::data::icvPointCloudData<SLRPointXYZIL>();
    outputdata3=new icv::data::icvPointCloudData<pcl::PointXYZRGB>();
    outputdata4=new icv::data::icvStructureData<box_info>();

    g_seeds_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    g_ground_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    g_not_ground_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    g_all_pc = pcl::PointCloud<SLRPointXYZIL>::Ptr(new pcl::PointCloud<SLRPointXYZIL>);

}

PlaneGroundFilter::~PlaneGroundFilter() {
    ICV_LOG_INFO<<"Plane Ground Filter Finished";
}
void PlaneGroundFilter::Execute(){
    icvSubscribe("robosense_points",inputdata);
    if (inputdata->is_not_empty()){
       //std::cout<<"ckpt0"<<endl;
       
       laserCloudIn=inputdata->getvalue();

      //  string filename="pandar_cloud/"+std::to_string(inputdata->GetSourceTime())+".pcd";
      //  pcl::io::savePCDFileASCII (filename,laserCloudIn);
       point_cb();
    }
}
// void PlaneGroundFilter::box_make(box_info box_i, visualization_msgs::MarkerArray& marker_list)
// {
//   int marker_id=marker_list.markers.size();
//   visualization_msgs::Marker marker_line[12];
//   for (int i=0;i<12;i++)
//   {
//     marker_line[i].header.frame_id = "/rslidar";
//     marker_line[i].header.stamp = ros::Time::now();
//   	marker_line[i].ns = "";
//     marker_line[i].id = marker_id;
//     marker_line[i].type = visualization_msgs::Marker::LINE_LIST;
//     marker_line[i].action = visualization_msgs::Marker::ADD;
    
//     marker_line[i].pose.orientation.w = 1.0;
//     marker_line[i].scale.x = 0.05;//设置线宽
        
//     marker_line[i].color.r = 1.0f;
//     marker_line[i].color.g = 1.0f;
//     marker_line[i].color.b = 0;
//     marker_line[i].color.a = 1.0;
    
//     marker_line[i].lifetime = ros::Duration(0.5);
//     marker_line[i].frame_locked = true;
//     marker_id++;
//   }
//   float c_zeta=cos(box_i.box_zeta*M_PI/180);
//   float s_zeta=sin(box_i.box_zeta*M_PI/180);     
//   geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;
  
// 	p1.x=(box_i.box_l)*c_zeta-(box_i.box_w)*s_zeta+box_i.box_x;
// 	p1.y=(box_i.box_l)*s_zeta+(box_i.box_w)*c_zeta+box_i.box_y;
// 	p1.z=(box_i.box_h)+box_i.box_z;
  
//   p2.x=(box_i.box_l)*c_zeta-(-box_i.box_w)*s_zeta+box_i.box_x;
// 	p2.y=(box_i.box_l)*s_zeta+(-box_i.box_w)*c_zeta+box_i.box_y;
// 	p2.z=(box_i.box_h)+box_i.box_z;
	
// 	p3.x=(-box_i.box_l)*c_zeta-(box_i.box_w)*s_zeta+box_i.box_x;
// 	p3.y=(-box_i.box_l)*s_zeta+(box_i.box_w)*c_zeta+box_i.box_y;
// 	p3.z=(box_i.box_h)+box_i.box_z;
	
// 	p4.x=(-box_i.box_l)*c_zeta-(-box_i.box_w)*s_zeta+box_i.box_x;
// 	p4.y=(-box_i.box_l)*s_zeta+(-box_i.box_w)*c_zeta+box_i.box_y;
// 	p4.z=(box_i.box_h)+box_i.box_z;
	
// 	p5.x=(box_i.box_l)*c_zeta-(box_i.box_w)*s_zeta+box_i.box_x;
// 	p5.y=(box_i.box_l)*s_zeta+(box_i.box_w)*c_zeta+box_i.box_y;
// 	p5.z=(-box_i.box_h)+box_i.box_z;
	
// 	p6.x=(box_i.box_l)*c_zeta-(-box_i.box_w)*s_zeta+box_i.box_x;
// 	p6.y=(box_i.box_l)*s_zeta+(-box_i.box_w)*c_zeta+box_i.box_y;
// 	p6.z=(-box_i.box_h)+box_i.box_z;
	
// 	p7.x=(-box_i.box_l)*c_zeta-(box_i.box_w)*s_zeta+box_i.box_x;
// 	p7.y=(-box_i.box_l)*s_zeta+(box_i.box_w)*c_zeta+box_i.box_y;
// 	p7.z=(-box_i.box_h)+box_i.box_z;
	
// 	p8.x=(-box_i.box_l)*c_zeta-(-box_i.box_w)*s_zeta+box_i.box_x;
// 	p8.y=(-box_i.box_l)*s_zeta+(-box_i.box_w)*c_zeta+box_i.box_y;
// 	p8.z=(-box_i.box_h)+box_i.box_z;
	
// 	marker_line[0].points.push_back(p1);marker_line[0].points.push_back(p2);      
//   marker_line[1].points.push_back(p2);marker_line[1].points.push_back(p4);
//   marker_line[2].points.push_back(p4);marker_line[2].points.push_back(p3);
//   marker_line[3].points.push_back(p1);marker_line[3].points.push_back(p3);
  
//   marker_line[4].points.push_back(p2);marker_line[4].points.push_back(p6);
//   marker_line[5].points.push_back(p4);marker_line[5].points.push_back(p8);
//   marker_line[6].points.push_back(p3);marker_line[6].points.push_back(p7);
//   marker_line[7].points.push_back(p1);marker_line[7].points.push_back(p5);
  
//   marker_line[8].points.push_back(p5);marker_line[8].points.push_back(p6);
//   marker_line[9].points.push_back(p6);marker_line[9].points.push_back(p8);
//   marker_line[10].points.push_back(p8);marker_line[10].points.push_back(p7);
//   marker_line[11].points.push_back(p5);marker_line[11].points.push_back(p7);
        
//   for (int i=0;i<12;i++)
//     marker_list.markers.push_back(marker_line[i]);
// }
void PlaneGroundFilter::remove_pt(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc, const pcl::PointCloud<pcl::PointXYZI>::Ptr out_pc)//删除范围外的点
{
    pcl::ExtractIndices<pcl::PointXYZI> rem;

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


void PlaneGroundFilter::extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZI> &p_sorted)
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

void PlaneGroundFilter::cluster_by_distance(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc) //in_pc is the groundless cloudpoint
{
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_pc_array(5);
    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
        segment_pc_array[i] = tmp;
    }
    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
      double distance = sqrt(in_pc->points[i].x * in_pc->points[i].x + in_pc->points[i].y * in_pc->points[i].y);

      if (in_pc->points[i].z > clip_height_ || distance <=2 || distance >= 100)
        {
            continue;
        }
      
          if (distance < seg_distance_[0]) //20, 35, 50, 70
          {
            segment_pc_array[0]->points.push_back(in_pc->points[i]); //four circles around origin, segmenting the plane into 5 regions
          }
          else if (distance < seg_distance_[1])
          {
            segment_pc_array[1]->points.push_back(in_pc->points[i]);
          }
          else if (distance < seg_distance_[2])
          {
            segment_pc_array[2]->points.push_back(in_pc->points[i]);
          }
          else if (distance < seg_distance_[3])
          {
            segment_pc_array[3]->points.push_back(in_pc->points[i]);
          }
          else
          {
            segment_pc_array[4]->points.push_back(in_pc->points[i]);
          }
    }
   
    out_pc1.clear();
    out_pc_all.clear();
    //marker_list.markers.clear();
    
    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
      cluster_segment(segment_pc_array[i], cluster_distance_[i], leaf_size_distance_[i]); //5 thresholds in the 5 rings
    //   *out_pc += *every_cluster_points;
      for(int i=0;i<out_pc2.size();i++)
      {
        out_pc1.push_back(out_pc2[i]);
        out_pc_all += out_pc2[i];
      }
      
    }
    

}


void PlaneGroundFilter::cluster_segment(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc, double in_max_cluster_distance, double leaf_size)
{
    out_pc2.clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vox; //pcl voxel grid, very useful
    vox.setInputCloud(in_pc);
    vox.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox.filter(*temp_points);

    std::vector<pcl::PointIndices> local_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euc; //pcl cluster
    euc.setInputCloud(temp_points);
    euc.setClusterTolerance(in_max_cluster_distance);
    euc.setMinClusterSize(MIN_CLUSTER_SIZE);
    euc.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euc.extract(local_indices); //jxy: here we can achieve the boundary extraction
    
    
    if (local_indices.empty())
       return;
       
    for(std::vector<pcl::PointIndices>::const_iterator it = local_indices.begin(); it != local_indices.end(); ++it)//对每个聚类后的点云如下操作
    {
      pcl::PointCloud<pcl::PointXYZRGB> cloud_cluster_point;
      pcl::PointXYZRGB point_temp;
      box_info box_i;
      int flag=0;

      float max_x,max_y,max_z,min_x,min_y,min_z;
      for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
      	/*flag++;
      	box_i.box_x=box_i.box_x+temp_points->points[*pit].x;
      	box_i.box_y=box_i.box_y+temp_points->points[*pit].y;
      	box_i.box_z=box_i.box_z+temp_points->points[*pit].z;*/
      	if (flag == 0)
      	{
      	  flag=1;
      	  max_x = temp_points->points[*pit].x;
          max_y = temp_points->points[*pit].y;
          max_z = temp_points->points[*pit].z;
          min_x=max_x;
          min_y=max_y;
          min_z=max_z;
      	}
      	else
      	{
      	  if (temp_points->points[*pit].x>max_x)
      	    max_x=temp_points->points[*pit].x;
     	    if (temp_points->points[*pit].y>max_y)
      	    max_y=temp_points->points[*pit].y;
          if (temp_points->points[*pit].z>max_z)
      	    max_z=temp_points->points[*pit].z;
      	  if (temp_points->points[*pit].x<min_x)
      	    min_x=temp_points->points[*pit].x;
      	  if (temp_points->points[*pit].y<min_y)
      	    min_y=temp_points->points[*pit].y;
      	  if (temp_points->points[*pit].z<min_z)
      	    min_z=temp_points->points[*pit].z;
      	}
      	
        point_temp.x = temp_points->points[*pit].x;
        point_temp.y = temp_points->points[*pit].y;
        point_temp.z = temp_points->points[*pit].z;

        cloud_cluster_point.push_back(point_temp);
      }

      box_i.box_x=(max_x+min_x)/2;
      box_i.box_y=(max_y+min_y)/2;
      box_i.box_z=(max_z+min_z)/2; //获取了中心点坐标
    
      float box_v=0;
      for(int zeta=0;zeta<180;zeta++)
      {
      	
      	float box_l_temp=0;
      	float box_w_temp=0;
      	float box_h_temp=0;
      	float box_v_temp=0;
      	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      	{
          //jxy: calculate distance of all the points and only costing 50ms? amazing.
          float temp_x = (temp_points->points[*pit].x-box_i.box_x) * cos(zeta*M_PI/180)+(temp_points->points[*pit].y-box_i.box_y) * sin(zeta*M_PI/180);
      	  float temp_y = -(temp_points->points[*pit].x-box_i.box_x) * sin(zeta*M_PI/180)+(temp_points->points[*pit].y-box_i.box_y) * cos(zeta*M_PI/180);
      	  float temp_z = temp_points->points[*pit].z-box_i.box_z;
      	  if (fabs(temp_x)>box_l_temp)
      	    box_l_temp=fabs(temp_x);
      	  if (fabs(temp_y)>box_w_temp)
      	    box_w_temp=fabs(temp_y);
      	  if (fabs(temp_z)>box_h_temp)
      	    box_h_temp=fabs(temp_z);
        }
      	box_v_temp=box_l_temp*box_w_temp*box_h_temp;
      	if (box_v != 0)
      	{
      	  if (box_v_temp<box_v) //rotating the box, and get the smallest box on some direction
          {
      	    box_i.box_l=box_l_temp;
      	    box_i.box_w=box_w_temp;
      	    box_i.box_h=box_h_temp;
            box_v=box_v_temp;
      	    box_i.box_zeta=zeta;
      	  }
      	}
      	else
      	{
	        box_i.box_l=box_l_temp;
      	  box_i.box_w=box_w_temp;
      	  box_i.box_h=box_h_temp;
      	  box_v=box_v_temp;
      	  box_i.box_zeta=zeta; //获取了包围盒尺寸和方向
      	}
      }
      out_pc2.push_back(cloud_cluster_point);
      if (box_v!=0)
      {
          cout<<"box_h: "<<box_i.box_h<<"box_l: "<<box_i.box_l<<"box_w: "<<box_i.box_w<<"box_x: "<<box_i.box_x<<"box_y: "<<box_i.box_y<<"box_z: "<<box_i.box_z<<"box_zeta: "<<box_i.box_zeta<<endl;
          outputdata4->setvalue(box_i);
          icvPublish("Bounding_Box3D",outputdata4);
        
      }
    } 
}


void PlaneGroundFilter::point_cb()
{
    laserCloudIn_org=laserCloudIn;
    cout<<"laserCloudIn points size: "<<laserCloudIn.points.size()<<endl;
    auto t1 = std::chrono::steady_clock::now();
     std::vector<int> indices;
    if(laserCloudIn.size() > 0){
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    
    SLRPointXYZIL point;
    int i_cnt=0;
    double z_min_list[num_lpr_]; //currently 15
    for (size_t i = 0; i < laserCloudIn.points.size(); i++)//将输入转换为带标签格式
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        if (point.z < -1.5 * sensor_height_||abs(point.x)>100||abs(point.y)>100) //delete those too far
        {
        }
        else
        {
            point.label = 0u; // 0 means uncluster
            g_all_pc->points.push_back(point); //all the points are put in g_all_pc
            if (i_cnt<num_lpr_)
            {
                z_min_list[i_cnt]=point.z;
                i_cnt++;
            }
            else
            {
                for (int j=0;j<num_lpr_;j++)
                {
                    if (point.z<z_min_list[j])
                    {
                        z_min_list[j]=point.z; //the lowest 15 points
                        break;
                    }
                }
            }
        }
    }
   cout<<"g_all_pc points size: "<<g_all_pc->points.size()<<endl;

    
    //extract_initial_seeds
    

    double sum=0;
    for (int i=0;i<num_lpr_;i++)
      sum+=z_min_list[i];
    double lpr_height = sum/num_lpr_; //the lowest height
    g_seeds_pc->clear();

    for (int i = 0; i < laserCloudIn.points.size(); i++)
    {
        if (laserCloudIn.points[i].z < lpr_height + th_seeds_)    //th_seeds = 1.2
        {
            g_seeds_pc->points.push_back(laserCloudIn.points[i]); //those not higher than the level 1.2m above the ground
        }
    }
    //extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc; //keeping the original ground extraction method
    cout<<"g_ground_pc points size: "<<g_ground_pc->points.size()<<endl;

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
        if (i = num_iter_ - 1)
        {
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
    }
    cout<<"g_not_ground_pc points size: "<<g_not_ground_pc->points.size()<<endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr final_no_ground(new pcl::PointCloud<pcl::PointXYZI>); //unused
    //pcl::PointCloud<pcl::PointXYZI>::Ptr final_vox(new pcl::PointCloud<pcl::PointXYZI>);

    //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_points;

    //remove_pt(g_not_ground_pc, final_no_ground);
    auto t2 = std::chrono::steady_clock::now();

  //  voxel_grid_filter(final_no_ground, final_vox, 0.13);

    cluster_by_distance(g_not_ground_pc); //core clustering algorithm, aiming at those not belonging to the ground
    cout<<"out_pc_all points size: "<<out_pc_all.points.size()<<endl;

    auto t3 = std::chrono::steady_clock::now();

    // publish ground points
    outputdata->setvalue(*g_ground_pc);
    icvPublish(ground_topic,outputdata);

    // publish not ground points
    outputdata->setvalue(*final_no_ground);
    icvPublish(no_ground_topic,outputdata);

    // publish all points

    outputdata2->setvalue(*g_all_pc);
    icvPublish(all_points_topic,outputdata2);
    g_all_pc->clear();

    // publish cluster points
  
    outputdata3->setvalue(out_pc_all);
    icvPublish(cluster_points_topic,outputdata3);

    //publish markers

    //pub_box_marker_.publish(marker_list);
    auto delta_t1 = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    auto delta_t2 = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);

    std::cout << "----------------------" << std::endl;
    std::cout << "The cost time of lidar filter ground is: " << delta_t1.count() * 1000 << "ms" << std::endl;
    if (delta_t1.count() * 1000.0 > 100.0)
      ICV_LOG_WARN<<"The lidar filter cost time is more 100ms.";

    std::cout << "----------------------" << std::endl;
    std::cout << "The cost time of lidar cluster is: " << delta_t2.count() * 1000 << "ms" << std::endl;
    if (delta_t2.count() * 1000.0 > 100.0)
      ICV_LOG_WARN<<"The lidar filter cost time is more 100ms.";
    }
}

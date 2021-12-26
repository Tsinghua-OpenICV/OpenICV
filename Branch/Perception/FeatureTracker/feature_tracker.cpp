#include "feature_tracker.h"

#define HEIGHT  480
#define WIDTH  768

using namespace icv;
using namespace cv;
using namespace std;

FeatureTracker::FeatureTracker(
	icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
{
    ICV_LOG_TRACE << " Feature Tracker started";    
    
	try 
	{
		module = torch::jit::load("model_0713.pt");
	}
	catch (const c10::Error &e) 
	{
		ICV_LOG_ERROR << "error loading the model";
		
	}
	try 
	{
		module.to(at::kCUDA);
	}
	catch (const c10::Error &e) 
	{
		ICV_LOG_ERROR << "error sending model to gpu";
		
	}
    //lane_output_ = new icvlane();
	inputdata = new  icv::data::icvCvMatData();
    outputdata = new  icv::data::icvCvMatData();
  
	Register_Sub("first_cam");
	Register_Pub("lanedetection_result");
	
}
FeatureTracker::~FeatureTracker(){
    ICV_LOG_TRACE << " Feature Tracker End";
}
void FeatureTracker::Execute(){
    icvSubscribe("first_cam",inputdata);
    
    if (inputdata->is_not_empty()){
     
        cv::Mat Orig_Image=inputdata->getvalue();
        imshow("Raw Image", Orig_Image);
        waitKey(1);
       out_img=Classfier(Orig_Image,module);
       outputdata->setvalue(out_img);
       icvPublish("lanedetection_result",outputdata);
       imshow("Lane Detection Result", out_img);
       waitKey(1);

    }
  
	
    
}
cv::Mat FeatureTracker::Classfier(cv::Mat &image,torch::jit::script::Module module)
{
	cv::Mat image_cv_mat = image.clone();
    cv::resize(image_cv_mat,image_cv_mat, cv::Size(WIDTH,HEIGHT));
    if(image_cv_mat.rows != HEIGHT || image_cv_mat.cols != WIDTH)   //判断图片大小
    {
        std::cout<<"something wrong with input image"<<std::endl;
        std::cout << "image.rows=" << image_cv_mat.rows << "    image.cols=" << image_cv_mat.cols << std::endl;
        cv::Mat zerosimg;
        return zerosimg;
    }
    torch::Tensor image_tensor = torch::from_blob(image_cv_mat.data, {1, image_cv_mat.rows, image_cv_mat.cols, 3}, torch::kByte);
    //std::cout<<"image_tensor.dim()"<<image_tensor.dim()<<endl;
    image_tensor = image_tensor.permute({0, 3, 1, 2});
    image_tensor = image_tensor.toType(torch::kFloat);
    image_tensor[0][0] = image_tensor[0][0].sub(torch::Scalar(103.939));
    image_tensor[0][1] = image_tensor[0][1].sub(torch::Scalar(116.779));
    image_tensor[0][2] = image_tensor[0][2].sub(torch::Scalar(123.68));
	image_tensor = image_tensor.cuda();
	
    auto output = module.forward(std::vector<torch::jit::IValue>{image_tensor}).toTuple()->elements();
	auto pole_torch_tensor = output[0].toTensor().cpu();
    auto lane_torch_tensor = output[1].toTensor().cpu();
	//std::cout<<"pole_torch_tensor.dim()"<<pole_torch_tensor.dim()<<endl;
	//std::cout<<"lane_torch_tensor.dim()"<<lane_torch_tensor.dim()<<endl;
    auto pole_out=pole_torch_tensor.accessor<long,2>();
	auto lane_out=lane_torch_tensor.accessor<long,2>();
    cv::Mat zerosimg = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
	//std::cout<<"zerosimg.rows="<<zerosimg.rows<<"       zerosimg.cols="<<zerosimg.cols<<std::endl;
    for (int i = 0; i < WIDTH * HEIGHT; ++i) 
    {
        int y = i / WIDTH;
        int x = i % WIDTH;
        if (pole_out[y][x] > 0)
		{
			//image_cv_mat.at<unsigned char>(3 * i + 1) = 125;
			zerosimg.at<unsigned char>(y,x) = 125;
		}
	
        if (lane_out[y][x] > 0)
		{
			//image_cv_mat.at<unsigned char>(3 * i + 2) = 255;
			zerosimg.at<unsigned char>(y,x) = 225;
		}
    }
    return zerosimg;
}


// bool FeatureTracker::inBorder(const cv::Point2f &pt)
// {
//     const int BORDER_SIZE = 1;
//     int img_x = cvRound(pt.x);
//     int img_y = cvRound(pt.y);
//     return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
// }

// double distance(cv::Point2f pt1, cv::Point2f pt2)
// {
//     //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
//     double dx = pt1.x - pt2.x;
//     double dy = pt1.y - pt2.y;
//     return sqrt(dx * dx + dy * dy);
// }

// void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
// {
//     int j = 0;
//     for (int i = 0; i < int(v.size()); i++)
//         if (status[i])
//             v[j++] = v[i];
//     v.resize(j);
// }

// void reduceVector(vector<int> &v, vector<uchar> status)
// {
//     int j = 0;
//     for (int i = 0; i < int(v.size()); i++)
//         if (status[i])
//             v[j++] = v[i];
//     v.resize(j);
// }

// FeatureTracker::FeatureTracker()
// {
//     stereo_cam = 0;
//     n_id = 0;
//     hasPrediction = false;
// }

// void FeatureTracker::setMask()
// {
//     mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));     //将图像设置成单一灰度和颜色white

//     // prefer to keep features that are tracked for long time
//     vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;   //构造(cnt，pts，id)序列让他们在排序的过程中仍然能一一对应

//     for (unsigned int i = 0; i < cur_pts.size(); i++)
//         cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

//     sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
//          {
//              //对光流跟踪到的特征点cnt_pts，按照被跟踪到的次数cnt从大到小排序
//             return a.first > b.first;
//          });

//     cur_pts.clear();
//     ids.clear();
//     track_cnt.clear();

//     for (auto &it : cnt_pts_id)
//     {
//         if (mask.at<uchar>(it.second.first) == 255)
//         {
//             //当前特征点位置对应的mask值为255，则保留当前特征点，将对应的特征点位置pts，id，被追踪次数cnt分别存入
//             cur_pts.push_back(it.second.first);
//             ids.push_back(it.second.second);
//             track_cnt.push_back(it.first);
//             cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
//         }   //在mask中将当前特征点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的点（使跟踪点不集中在一个区域上）
//     }
// }

// double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
// {
//     //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
//     double dx = pt1.x - pt2.x;
//     double dy = pt1.y - pt2.y;
//     return sqrt(dx * dx + dy * dy);
// }

// map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
// {
//     TicToc t_r;
//     cur_time = _cur_time;
//     cur_img = _img;
//     row = cur_img.rows;
//     col = cur_img.cols;
//     cv::Mat rightImg = _img1;
//     /*
//     {
//         cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));    //自适应直方图均衡、、、、、、、
//         clahe->apply(cur_img, cur_img);
//         if(!rightImg.empty())
//             clahe->apply(rightImg, rightImg);
//     }
//     */
//     cur_pts.clear();

//     if (prev_pts.size() > 0)
//     {
//         TicToc t_o;
//         vector<uchar> status;
//         vector<float> err;
//         if(hasPrediction)
//         {
//             cur_pts = predict_pts;
//             /***
//             -prevImg: 深度为8位的前一帧图像或金字塔图像。
//             -nextImg：和prevImg有相同的大小和类型，后一帧图像或金字塔。
//             -prevPts：计算光流所需要的输入2D点矢量，点坐标必须是单精度浮点数。
//             -nextPts：输出2D点矢量(也是单精度浮点数坐标)，点矢量中包含的是在后一帧图像上计算得到的输入特征新位置。
//             -status：输出状态矢量(元素是无符号char类型，uchar)，如果相应特征的流发现则矢量元素置为1，否则，为0。
//             -err：输出误差矢量。
//             -winSize：每个金字塔层搜索窗大小。
//             -maxLevel：金字塔层的最大数目；如果置0，金字塔不使用(单层)；如果置1，金字塔2层，等等以此类推。
//             -criteria：指定搜索算法收敛迭代的类型
//             -minEigTheshold：算法计算的光流等式的2x2常规矩阵的最小特征值。
//             ***/
//             cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
//             cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
//             int succ_num = 0;
//             for (size_t i = 0; i < status.size(); i++)
//             {
//                 if (status[i])
//                     succ_num++;
//             }
//             if (succ_num < 10)
//                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
//         }
//         else
//             cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
//         // reverse check
//         if(FLOW_BACK)
//         {
//             vector<uchar> reverse_status;
//             vector<cv::Point2f> reverse_pts = prev_pts;
//             cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
//             cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
//             //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
//             for(size_t i = 0; i < status.size(); i++)
//             {
//                 if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
//                 {
//                     status[i] = 1;
//                 }
//                 else
//                     status[i] = 0;
//             }
//         }
        
//         for (int i = 0; i < int(cur_pts.size()); i++)   //cur_pts=forw_pts将位于图像边界外的点标记为0
//             if (status[i] && !inBorder(cur_pts[i]))
//                 status[i] = 0;
//         reduceVector(prev_pts, status);
//         reduceVector(cur_pts, status);
//         reduceVector(ids, status);
//         reduceVector(track_cnt, status);
//         ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
//         //printf("track cnt %d\n", (int)ids.size());
//     }

//     for (auto &n : track_cnt)   //光流追踪成功,特征点被成功跟踪的次数就加1
//         n++;                    //数值代表被追踪的次数，数值越大，说明被追踪的就越久


//     if (1)
//     {
//         //rejectWithF();        //通过基本矩阵剔除outliers
//         //2D-2D对极几何相关知识点，两帧上的一系列对应的特征点能够复原出两帧之间的相对位姿变化，也就是基础矩阵E。
//         //但是这些特征点中肯定会有一些outlier，所以通过这个opencv的函数，能够巧妙地剔除这些outlier。
//         //如果函数传入的是归一化坐标，那么得到的是本质矩阵E，如果传入的是像素坐标，那么得到的是基础矩阵。
//         ROS_DEBUG("set mask begins");
//         TicToc t_m;
//         setMask();       //对跟踪点进行排序并去除密集点,这个函数的操作对象是cur_pts=forw_pts,
//                          //它是通过光流法在当前帧中仍然能追踪到的那些特征点在当前帧中的像素坐标！所以forw_pts里面放着的都是老特征点。
//                          //首先对他们根据追踪到的次数进行排序，从而确定了优先级
//                          //然后再把特征点周围的密集的特征点剔除掉
//         ROS_DEBUG("set mask costs %fms", t_m.toc());

//         ROS_DEBUG("detect feature begins");
//         TicToc t_t;
//         int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
//         if (n_max_cnt > 0)
//         {
//             if(mask.empty())
//                 cout << "mask is empty " << endl;
//             if (mask.type() != CV_8UC1)
//                 cout << "mask type wrong " << endl;
//             cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);        //提取角点
//         }
//         else
//             n_pts.clear();
//         ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

//         for (auto &p : n_pts)
//         {
//             cur_pts.push_back(p);
//             ids.push_back(n_id++);
//             track_cnt.push_back(1);
//         }
//         //printf("feature cnt after add %d\n", (int)ids.size());
//     }

//     cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
//     pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

//     if(!_img1.empty() && stereo_cam)
//     {
//         ids_right.clear();
//         cur_right_pts.clear();
//         cur_un_right_pts.clear();
//         right_pts_velocity.clear();
//         cur_un_right_pts_map.clear();
//         if(!cur_pts.empty())
//         {
//             //printf("stereo image; track feature on right image\n");
//             vector<cv::Point2f> reverseLeftPts;
//             vector<uchar> status, statusRightLeft;
//             vector<float> err;
//             // cur left ---- cur right
//             cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
//             // reverse check cur right ---- cur left
//             if(FLOW_BACK)
//             {
//                 cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
//                 for(size_t i = 0; i < status.size(); i++)
//                 {
//                     if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
//                         status[i] = 1;
//                     else
//                         status[i] = 0;
//                 }
//             }

//             ids_right = ids;
//             reduceVector(cur_right_pts, status);
//             reduceVector(ids_right, status);
//             // only keep left-right pts
//             /*
//             reduceVector(cur_pts, status);
//             reduceVector(ids, status);
//             reduceVector(track_cnt, status);
//             reduceVector(cur_un_pts, status);
//             reduceVector(pts_velocity, status);
//             */
//             cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
//             right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
//         }
//         prev_un_right_pts_map = cur_un_right_pts_map;
//     }
//     if(SHOW_TRACK)
//         drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

//     prev_img = cur_img;
//     prev_pts = cur_pts;
//     prev_un_pts = cur_un_pts;
//     prev_un_pts_map = cur_un_pts_map;
//     prev_time = cur_time;
//     hasPrediction = false;

//     prevLeftPtsMap.clear();
//     for(size_t i = 0; i < cur_pts.size(); i++)
//         prevLeftPtsMap[ids[i]] = cur_pts[i];

//     map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
//     for (size_t i = 0; i < ids.size(); i++)
//     {
//         int feature_id = ids[i];
//         double x, y ,z;
//         x = cur_un_pts[i].x;
//         y = cur_un_pts[i].y;
//         z = 1;
//         double p_u, p_v;
//         p_u = cur_pts[i].x;
//         p_v = cur_pts[i].y;
//         int camera_id = 0;
//         double velocity_x, velocity_y;
//         velocity_x = pts_velocity[i].x;
//         velocity_y = pts_velocity[i].y;

//         Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//         xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
//         featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
//     }

//     if (!_img1.empty() && stereo_cam)
//     {
//         for (size_t i = 0; i < ids_right.size(); i++)
//         {
//             int feature_id = ids_right[i];
//             double x, y ,z;
//             x = cur_un_right_pts[i].x;
//             y = cur_un_right_pts[i].y;
//             z = 1;
//             double p_u, p_v;
//             p_u = cur_right_pts[i].x;
//             p_v = cur_right_pts[i].y;
//             int camera_id = 1;
//             double velocity_x, velocity_y;
//             velocity_x = right_pts_velocity[i].x;
//             velocity_y = right_pts_velocity[i].y;

//             Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//             xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
//             featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
//         }
//     }

//     //printf("feature track whole time %f\n", t_r.toc());
//     return featureFrame;        //前端追踪成功


// }
// //先把特征点坐标(像素坐标)转为归一化坐标，再转回到像素坐标，然后再用findFundamentalMat()找outlier。
// //它之所以这么做，是因为需要一个去畸变的过程，在m_camera->liftProjective()里实现，
// //m_camera是定义在camera.h的一个父类的对象，下面有多个相机模型的子类，
// //例如在PinholeCamera.cc里，这里面就重写liftProjective()了方法，实现了去畸变的功能。

// void FeatureTracker::rejectWithF()      //一个去畸变的过程//通过基本矩阵剔除outliers
// {
//     if (cur_pts.size() >= 8)
//     {
//         ROS_DEBUG("FM ransac begins");
//         TicToc t_f;
//         vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
//         for (unsigned int i = 0; i < cur_pts.size(); i++)
//         {   //根据不同的相机模型将二维坐标转换到三维坐标;对于PINHOLE（针孔相机）可将像素坐标转换到归一化平面并去畸变
//             Eigen::Vector3d tmp_p;      //对于CATA（卡特鱼眼相机）将像素坐标投影到单位圆内
//             //TODO 这里是利用像素坐标，基于m_camera的相机模型(pinhole)，获得对应的归一化坐标，
//             //自己写的时候看看能不能把这个引用的库砍掉
//             m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
//             tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
//             tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
//             un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

//             m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
//             tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
//             tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
//             un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
//         }

//         vector<uchar> status;
//         cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
//         int size_a = cur_pts.size();
//         reduceVector(prev_pts, status);
//         reduceVector(cur_pts, status);
//         reduceVector(cur_un_pts, status);
//         reduceVector(ids, status);
//         reduceVector(track_cnt, status);
//         ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
//         ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
//     }
// }

// void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
// {
//     for (size_t i = 0; i < calib_file.size(); i++)
//     {
//         ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
//         // camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
//         cv::FileStorage fs( calib_file[i], cv::FileStorage::READ );
//         camodocal::PinholeCameraPtr camera( new camodocal::PinholeCamera );
//         camodocal::PinholeCamera::Parameters params = camera->getParameters( );
//         params.readFromYamlFile( calib_file[i] );
//         camera->setParameters( params );
//         m_camera.push_back(camera);
//     }
//     if (calib_file.size() == 2)
//         stereo_cam = 1;
// }

// void FeatureTracker::showUndistortion(const string &name)
// {
//     cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
//     vector<Eigen::Vector2d> distortedp, undistortedp;
//     for (int i = 0; i < col; i++)
//         for (int j = 0; j < row; j++)
//         {
//             Eigen::Vector2d a(i, j);
//             Eigen::Vector3d b;
//             m_camera[0]->liftProjective(a, b);
//             distortedp.push_back(a);
//             undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
//             //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
//         }
//     for (int i = 0; i < int(undistortedp.size()); i++)
//     {
//         cv::Mat pp(3, 1, CV_32FC1);
//         pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
//         pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
//         pp.at<float>(2, 0) = 1.0;
//         //cout << trackerData[0].K << endl;
//         //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
//         //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
//         if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
//         {
//             undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
//         }
//         else
//         {
//             //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
//         }
//     }
//     // turn the following code on if you need
//     // cv::imshow(name, undistortedImg);
//     // cv::waitKey(0);
// }

// vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
// {
//     vector<cv::Point2f> un_pts;
//     for (unsigned int i = 0; i < pts.size(); i++)
//     {
//         Eigen::Vector2d a(pts[i].x, pts[i].y);
//         Eigen::Vector3d b;
//         cam->liftProjective(a, b);
//         un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
//     }
//     return un_pts;
// }

// vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
//                                             map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
// {
//     vector<cv::Point2f> pts_velocity;
//     cur_id_pts.clear();
//     for (unsigned int i = 0; i < ids.size(); i++)
//     {
//         cur_id_pts.insert(make_pair(ids[i], pts[i]));
//     }

//     // caculate points velocity
//     if (!prev_id_pts.empty())
//     {
//         double dt = cur_time - prev_time;
        
//         for (unsigned int i = 0; i < pts.size(); i++)
//         {
//             std::map<int, cv::Point2f>::iterator it;
//             it = prev_id_pts.find(ids[i]);
//             if (it != prev_id_pts.end())
//             {
//                 double v_x = (pts[i].x - it->second.x) / dt;
//                 double v_y = (pts[i].y - it->second.y) / dt;
//                 pts_velocity.push_back(cv::Point2f(v_x, v_y));
//             }
//             else
//                 pts_velocity.push_back(cv::Point2f(0, 0));

//         }
//     }
//     else
//     {
//         for (unsigned int i = 0; i < cur_pts.size(); i++)
//         {
//             pts_velocity.push_back(cv::Point2f(0, 0));
//         }
//     }
//     return pts_velocity;
// }

// void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
//                                vector<int> &curLeftIds,
//                                vector<cv::Point2f> &curLeftPts, 
//                                vector<cv::Point2f> &curRightPts,
//                                map<int, cv::Point2f> &prevLeftPtsMap)
// {
//     //int rows = imLeft.rows;
//     int cols = imLeft.cols;
//     if (!imRight.empty() && stereo_cam)
//         cv::hconcat(imLeft, imRight, imTrack);
//     else
//         imTrack = imLeft.clone();
//     cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

//     for (size_t j = 0; j < curLeftPts.size(); j++)
//     {
//         double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
//         cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
//     }
//     if (!imRight.empty() && stereo_cam)
//     {
//         for (size_t i = 0; i < curRightPts.size(); i++)
//         {
//             cv::Point2f rightPt = curRightPts[i];
//             rightPt.x += cols;
//             cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
//             //cv::Point2f leftPt = curLeftPtsTrackRight[i];
//             //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
//         }
//     }
    
//     map<int, cv::Point2f>::iterator mapIt;
//     for (size_t i = 0; i < curLeftIds.size(); i++)
//     {
//         int id = curLeftIds[i];
//         mapIt = prevLeftPtsMap.find(id);
//         if(mapIt != prevLeftPtsMap.end())
//         {
//             cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
//         }
//     }

//     //draw prediction
//     /*
//     for(size_t i = 0; i < predict_pts_debug.size(); i++)
//     {
//         cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
//     }
//     */
//     //printf("predict pts size %d \n", (int)predict_pts_debug.size());

//     //cv::Mat imCur2Compress;
//     //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
// }


// void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
// {
//     hasPrediction = true;
//     predict_pts.clear();
//     predict_pts_debug.clear();
//     map<int, Eigen::Vector3d>::iterator itPredict;
//     for (size_t i = 0; i < ids.size(); i++)
//     {
//         //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
//         int id = ids[i];
//         itPredict = predictPts.find(id);
//         if (itPredict != predictPts.end())
//         {
//             Eigen::Vector2d tmp_uv;
//             m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
//             predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
//             predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
//         }
//         else
//             predict_pts.push_back(prev_pts[i]);
//     }
// }


// void FeatureTracker::removeOutliers(set<int> &removePtsIds)
// {
//     std::set<int>::iterator itSet;
//     vector<uchar> status;
//     for (size_t i = 0; i < ids.size(); i++)
//     {
//         itSet = removePtsIds.find(ids[i]);
//         if(itSet != removePtsIds.end())
//             status.push_back(0);
//         else
//             status.push_back(1);
//     }

//     reduceVector(prev_pts, status);
//     reduceVector(ids, status);
//     reduceVector(track_cnt, status);
// }


// cv::Mat FeatureTracker::getTrackImage()
// {
//     return imTrack;
// }

// HDMapObservation* FeatureTracker::getDetection(const map<HDMapType, std::string> & detFilePath,torch::jit::script::Module module) {
//     HDMapObservation * hdmapObs = new HDMapObservation();

//     std::fstream infile;

//     for (auto & filePath : detFilePath) {
//         infile.open(filePath.second);
//         if (filePath.first == HDMapType::POLE) {
//             std::string line;

//             int lamp_num = 0;
//             // lamps.clear();
//             // std::cout << lamp_file << std::endl;
//             while (getline(infile, line))
//             {
//                 std::stringstream ss(line);
//                 double start_pt_x, start_pt_y, end_pt_x, end_pt_y;
//                 vector<Eigen::Vector2d> pt_obs;
//                 // ss >> start_pt_x >> start_pt_y >> end_pt_x >> end_pt_y;
//                 ss >> start_pt_x >> start_pt_y >> end_pt_x >> end_pt_y;
//                 Eigen::Vector2d pt = Eigen::Vector2d(start_pt_x, start_pt_y);
//                 Eigen::Vector3d pt3d;
//                 m_camera[0]->liftProjective(pt, pt3d);
//                 pt_obs.push_back(Eigen::Vector2d(pt3d.x(), pt3d.y()));
                
//                 pt = Eigen::Vector2d(end_pt_x, end_pt_y);
//                 m_camera[0]->liftProjective(pt, pt3d);
//                 pt_obs.push_back(Eigen::Vector2d(pt3d.x(), pt3d.y()));
                
//                 hdmapObs->hdmap_observation.push_back(pt_obs);
//                 hdmapObs->hdmap_type.push_back(HDMapType::POLE);
//                 hdmapObs->hdmap_correspondence.push_back(-1);
//             }
//         }
//         if (filePath.first == HDMapType::LANE) {
//             std::string line;

//             int lamp_num = 0;
//             // lamps.clear();
//             // std::cout << lamp_file << std::endl;
//             while (getline(infile, line))
//             {
//                 std::stringstream ss(line);
//                 double start_pt_x, start_pt_y, end_pt_x, end_pt_y;
//                 vector<Eigen::Vector2d> pt_obs;
//                 // ss >> start_pt_x >> start_pt_y >> end_pt_x >> end_pt_y;
//                 ss >> start_pt_x >> start_pt_y >> end_pt_x >> end_pt_y;
//                 Eigen::Vector2d pt = Eigen::Vector2d(start_pt_x, start_pt_y);
//                 Eigen::Vector3d pt3d;
//                 m_camera[0]->liftProjective(pt, pt3d);
//                 pt_obs.push_back(Eigen::Vector2d(pt3d.x(), pt3d.y()));

//                 pt = Eigen::Vector2d(end_pt_x, end_pt_y);
//                 m_camera[0]->liftProjective(pt, pt3d);
//                 pt_obs.push_back(Eigen::Vector2d(pt3d.x(), pt3d.y()));
                
//                 hdmapObs->hdmap_observation.push_back(pt_obs);
//                 hdmapObs->hdmap_type.push_back(HDMapType::LANE);
//                 hdmapObs->hdmap_correspondence.push_back(-1);
//             }
//         }
//         infile.close();
//         if (filePath.first == HDMapType::SEG) {
//             //cv::Mat detection = cv::imread(filePath.second, 0);            
//             //cv::resize(detection, detection, cv::Size(768, 432), 0, 0, cv::INTER_NEAREST); // TODO: should fix the input image size
            
//             cv::Mat templeft = cv::imread(filePath.second);
//             templeft = templeft.rowRange(0, ROW);

//             cv::Mat detection =  Classfier(templeft,module);
//             hdmapObs->segmentation = detection.clone();

//             cv::Mat lane_detection = detection.clone();
//             cv::Mat pole_detection = detection.clone();
//             for (int i = 0; i < detection.rows; ++i) {
//                 for (int j = 0; j < detection.cols; ++j) {
//                     if (detection.at<uchar>(i,j) == 125) {
//                         lane_detection.at<uchar>(i,j) = 0;
//                     }
//                     else if (detection.at<uchar>(i,j) == 255) {
//                         pole_detection.at<uchar>(i,j) = 0;
//                     }
//                 }
//             }

//             // cv::Mat erodeStruct = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6,3));
//             // cv::Mat erodeStruct = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
//             cv::Mat erodeStruct = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(8,8));
//             cv::erode(lane_detection, lane_detection, erodeStruct);

//             erodeStruct = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6,1));
//             cv::erode(pole_detection, pole_detection, erodeStruct);

//             for (int i = 0; i < detection.rows; ++i) {
//                 for (int j = 0; j < detection.cols; ++j) {
//                     if (lane_detection.at<uchar>(i,j) == 0 && pole_detection.at<uchar>(i,j) == 0) {
//                         detection.at<uchar>(i,j) = 0;
//                     }
//                 }
//             }
//             // cv::erode(detection, detection, erodeStruct);
//             // cv::erode(detection, detection, erodeStruct);
//             // TODO: check whether there is lane observation, should be improved
//             hdmapObs->has_lane = false;
//             for (int i = 0; i < detection.rows; ++i) {
//                 for (int j = 0; j < detection.cols; ++j) {
//                     if (detection.at<uchar>(i,j) == 255) {
//                         hdmapObs->has_lane = true;
//                         break;
//                     }
//                 }
//                 if (hdmapObs->has_lane) 
//                     break;
//             }
            
//             cv::Mat map1, map2;
//             m_camera[0]->initUndistortMap(map1, map2); // TODO: should be moved to initialization part.
//             cv::remap(detection, detection, map1, map2, cv::INTER_LINEAR);

//             /* option 1: direct output the detection map
//             hdmapObs->segmentation = detection;
//             */

//             // option 2: output the distance transformation
//             cv::threshold(detection, detection, 0, 255, cv::THRESH_BINARY);
//             detection = ~detection;
//             // cv::imshow("undistortion", detection);
//             cv::Mat mask = cv::Mat(detection.size(), CV_32FC1);
//             cv::distanceTransform(detection, mask, CV_DIST_L2,3);
//             // std::cout << mask << std::endl;
//             for (int i = 0; i < mask.rows; ++i) {
//                 for (int j = 0; j < mask.cols; ++j)
//                     if (mask.at<float>(i,j) > 10)
//                         mask.at<float>(i,j) = 10;
//             }
//             // std::cout << mask << std::endl;
//             // cv::normalize(mask, mask, 0, 1, cv::NORM_MINMAX);

//             hdmapObs->distTransfrom = mask;
//             cv::imshow("undistortion", hdmapObs->distTransfrom);
//             // get distance transformation
//             // cv::waitKey(0);

//         }
//     } 
//     return hdmapObs;
// }

// void FeatureTracker::showDetectionResult(HDMapObservation * hdmapObs) {
//     // draw lamp
//     cv::Mat show_img = cur_img;
//     cv::cvtColor(show_img, show_img, cv::COLOR_GRAY2BGR);
//     for (int i = 0; i < hdmapObs->hdmap_observation.size(); ++i) {
//         if (hdmapObs->hdmap_type[i] == HDMapType::POLE)
//             cv::line(show_img, cv::Point2d(hdmapObs->hdmap_observation[i][0].x(), hdmapObs->hdmap_observation[i][0].y()),
//                                cv::Point2d(hdmapObs->hdmap_observation[i][1].x(), hdmapObs->hdmap_observation[i][1].y()), cv::Scalar(255,0,0), 5);
//         else if (hdmapObs->hdmap_type[i] == HDMapType::LANE) 
//             cv::line(show_img, cv::Point2d(hdmapObs->hdmap_observation[i][0].x(), hdmapObs->hdmap_observation[i][0].y()),
//                                cv::Point2d(hdmapObs->hdmap_observation[i][1].x(), hdmapObs->hdmap_observation[i][1].y()), cv::Scalar(255,0,0), 5);
//     }
//     // cv::imshow("detect", show_img);
//     // cv::imwrite("/media/doris/000182C1000F703C/thu/image_sampled/" + std::to_string(cur_time) + ".jpg", show_img);
//     // cv::waitKey(0);    
// }

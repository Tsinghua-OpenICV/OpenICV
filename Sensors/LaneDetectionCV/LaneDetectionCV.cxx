#include "LaneDetectionCV.h"
#include <unistd.h>

void LaneDetectionCV::Execute(
	std::vector<icvNodeInput *>& inputPorts, 
	std::vector<icvNodeOutput *>& outputPorts) 
{
    cv::Mat srcImage, img_resize;
    cv::Rect rect(128, 140, 512, 256);
    time_t time_new;
   
    srcImage = read_Input<icv::data::icvCvMatData>(0);
    if (!is_not_empty(0)) {
	return;
    }
    time_new = read_Input<icv::data::icvCvMatData>(0).GetSourceTime();

    InputTaskPair input ;
    input.first = input_index_ ;
    // ignore timestamp
    
    img_resize = srcImage(rect);
    input.second.image = img_resize;
    ICV_LOG_TRACE << "leave execute - " << input_index_ << " image size - " << img_resize.size();
    input_queue_.Enqueue(input);
    input_index_++;
}

double valueAtIPM(std::vector<float>& f, float x) 
{
	float ans = 0.f;
	for (int i = (int)f.size() - 1; i >= 0; --i)
		ans = ans * x + f[i];
	return ans;
}

void vector_Update(std::vector<float> &vector_a, float value)
{
	vector_a.push_back(value);
	vector_a.erase(vector_a.begin());
}

ld_Frame framePub(LaneDetection lane)
{
    CJudgeCenter JudgeCenter ;
    ld_Frame ld_obj;
    std::vector<double> candidate_dis; 
    std::vector<int> candidate_id; 
    float lane_angle = 0.0;
    std::vector<float> Lane_angle(2);
    ld_Point tmp_Point;
    ld_Coeff tmp_Coeff;
    ld_LaneParam tmp_LaneParam_Pixel;
    ld_LaneParam tmp_LaneParam_World;
    
    std::vector<float> hist_v_LaneWidth(10, 3.5f);

    ld_obj.lane_Pixel.clear();
    ld_obj.lane_World.clear();
    ld_obj.lane_Coeff.clear();
    candidate_id.clear();
    candidate_dis.clear();
    Lane_angle.clear();

    for (int i = 0; i < lane.v_PolyfitParam.v_ld_coeff.size(); i++)
    {
        tmp_Coeff.a=lane.v_PolyfitParam.v_ld_coeff[i].a;
        tmp_Coeff.b=lane.v_PolyfitParam.v_ld_coeff[i].b;
        tmp_Coeff.c=lane.v_PolyfitParam.v_ld_coeff[i].c;
        tmp_Coeff.d=lane.v_PolyfitParam.v_ld_coeff[i].d;
        tmp_Coeff.id=lane.v_PolyfitParam.v_ld_coeff[i].global_id;
        ld_obj.lane_Coeff.push_back(tmp_Coeff);

        cv::Point2f dot_p;
        std::vector<float> coeff(4);
        coeff[3] = tmp_Coeff.a;
        coeff[2] = tmp_Coeff.b;
        coeff[1] = tmp_Coeff.c;
        coeff[0] = tmp_Coeff.d;

        dot_p.x = 0;
        dot_p.y = valueAtIPM(coeff, 0);
        if (-3.5<dot_p.y && dot_p.y<3.5) {							
            lane_angle = atan(3 * coeff[3] * dot_p.x * dot_p.x + 2 * coeff[2] * dot_p.x + coeff[1]);					
            candidate_dis.push_back(dot_p.y);
            candidate_id.push_back(i);
            Lane_angle.push_back(lane_angle);				
        }
    }

    double left_0 = 0.0;
    double right_0 = 0.0;
    int flag_left = 0;
    int flag_right = 0;
    bool b_flag1 = true;
    bool b_flag2 = true;

    int i_flag_l = -1;
    int i_flag_r = -1;

    if (candidate_dis.size()>1) {
        for(int ii=0; ii<candidate_dis.size(); ii++) {
            if (candidate_dis[ii]<0) {
                if (b_flag1) {
                    left_0 = candidate_dis[ii];
                    i_flag_l = ii;
                }
                b_flag1 = false;

                if (candidate_dis[ii]>left_0) {
                    left_0 = candidate_dis[ii];
                    i_flag_l = ii;
                }
                flag_left = 1;
            }
            if (candidate_dis[ii]>0) {
                if (b_flag2) {
                    right_0 = candidate_dis[ii];
                    i_flag_r = ii;
                }
                b_flag2 = false;

                if (candidate_dis[ii] < right_0){
                    right_0 = candidate_dis[ii];
                    i_flag_r = ii;
                }	
                flag_right = 1;
            }
        }
    }
    else
    {
        // ROS_INFO("lane detection is not reliable OR no lane ");
    }

    double angle_final =0.0;
    double curve_radius = 0.0;
    float curve_position_x= 0.0;
    if (i_flag_l==-1 && i_flag_r!=-1) {
        angle_final = Lane_angle[i_flag_r];
        curve_radius = fabs(pow(1.0+pow((ld_obj.lane_Coeff[candidate_id[i_flag_r]].c),2),(3/2))/(2*ld_obj.lane_Coeff[candidate_id[i_flag_r]].b));

    } else if (i_flag_l!=-1 && i_flag_r==-1) {
        angle_final = Lane_angle[i_flag_l];
        curve_radius = fabs(pow(1.0+pow((ld_obj.lane_Coeff[candidate_id[i_flag_l]].c),2),(3/2))/(2*ld_obj.lane_Coeff[candidate_id[i_flag_l]].b));

    } else if (i_flag_l!=-1 && i_flag_r!=-1) {
        angle_final = (Lane_angle[i_flag_l]+Lane_angle[i_flag_r])/2;
        curve_radius = (fabs(pow(1.0+pow((ld_obj.lane_Coeff[candidate_id[i_flag_l]].c),2),(3/2))/(2*ld_obj.lane_Coeff[candidate_id[i_flag_l]].b)) \
        +fabs(pow(1.0+pow((ld_obj.lane_Coeff[candidate_id[i_flag_r]].c),2),(3/2))/(2*ld_obj.lane_Coeff[candidate_id[i_flag_r]].b)))/2.0;
    }
    if (curve_radius>3000) {
        curve_radius=3000;
    }


    double mid = 0;
    double bias_dis = 0;
    double lane_width = 3.5;
    char tmp_str[20]="";

    if(flag_left && flag_right && (fabs(left_0)+fabs(right_0))* cos(angle_final)> 3.0)
    {
        mid =(left_0+right_0)/2;
        bias_dis = mid * cos(angle_final);
        lane_width = (fabs(left_0)+fabs(right_0))* cos(angle_final);
        //如果道路宽度有突变，则参考历史容器中的大小，做为纠正值
        if (fabs(lane_width - hist_v_LaneWidth[hist_v_LaneWidth.size()-1])>0.3)
        {
            for (int ii=0;ii<5;ii++) {
                lane_width += hist_v_LaneWidth[hist_v_LaneWidth.size()-1-ii];
            }
            lane_width = lane_width/6.0;
        }
        //将道路宽度加入历史容器中
        vector_Update(hist_v_LaneWidth, lane_width);    
    } else {
        for (int ii=0; ii<10; ii++) {
            lane_width += hist_v_LaneWidth[hist_v_LaneWidth.size() - 1 - ii];
        }
        lane_width = lane_width/11.0;
        vector_Update(hist_v_LaneWidth, lane_width);
    }

    // JudgeCenter.SetParam(lane_width,lane_width*0.15,angle_final);
    // JudgeCenter.Run(lane.v_PolyfitParam.v_ld_coeff);
    
    ld_obj.lane_Coeff.clear();
    for (int i = 0; i < lane.v_PolyfitParam.v_ld_coeff.size(); i++)
    {	
        tmp_Coeff.a = lane.v_PolyfitParam.v_ld_coeff[i].a;
        tmp_Coeff.b = lane.v_PolyfitParam.v_ld_coeff[i].b;
        tmp_Coeff.c = lane.v_PolyfitParam.v_ld_coeff[i].c;
        tmp_Coeff.d = lane.v_PolyfitParam.v_ld_coeff[i].d;
        tmp_Coeff.id = lane.v_PolyfitParam.v_ld_coeff[i].global_id;
        ld_obj.lane_Coeff.push_back(tmp_Coeff);
    }

    ld_obj.header.stamp = time(0);
    ld_obj.lane_width = lane_width ;
    ld_obj.bias_dis = bias_dis;
    ld_obj.cl_flag = flag_left && flag_right ;
    ld_obj.bias_theta = angle_final;
    ld_obj.curve_radius = curve_radius;

    return ld_obj ;
}

int LaneDetectionCV::process()
{
    LaneDetection lane_;

    InputTaskPair task;
    FramePair pub_ld ;

    std::thread::id this_id = std::this_thread::get_id();
    
    while (true)
    {
        ICV_LOG_TRACE << "### id - " << this_id << ", fetch  task";
        input_queue_.WaitDequeue(&task);
        ICV_LOG_TRACE << "### id - " << this_id << ", start work";

        pub_ld.first = task.first ;
        if (!lane_.initialize_variable(task.second.image)) {
            return -1;
        }

        // std::cout<<"process --1"<<std::endl;
        if (!lane_.initialize_Img(task.second.image)) {
            return 0;
        }
        // std::cout<<"process --2"<<std::endl;
        // detecting lane markings
        lane_.lane_marking_detection();
        // supermarking generation and low-level association
        lane_.seed_generation();

        // CRF graph configuration & optimization using hungarian method
        lane_.graph_generation();

        lane_.validating_final_seeds(); 
        // std::cout<<"process --3"<<std::endl;

        task.second.image = lane_.kalman().clone();
        pub_ld.second = framePub(lane_);

        show_queue_.Enqueue(task);        
	pub_queue_.Enqueue(pub_ld);
        ICV_LOG_TRACE << "### id - " << this_id << ", work done";
    }

    return 0 ;
}

void LaneDetectionCV::display()
{
    InputTaskPair show;
    std::pair<int, ld_Frame> show_pub_ld ;
    std::thread::id this_id = std::this_thread::get_id();
	
    while (true)
    {
        ICV_LOG_TRACE << "$$$ id - " << this_id << " show - " << show_index_;
        show_queue_.WaitDequeue2(&show, show_index_);
        // std::cout<<"enter display--1"<<std::endl;
        pub_queue_.WaitDequeue2(&show_pub_ld, show_index_);
        // std::cout<<"enter display--2"<<std::endl;
	
	lane_output_->setoutvalue(show_pub_ld.second);
	Send_Out(lane_output_, 0);

        show_index_++;
        frame_count_++;

        t2 = clock() ;
        std::cout<<"mean process time: "<<(t2-t1)/1000/frame_count_ <<" ms"<<std::endl ;
        
	cv::imshow("LD ICV test", show.second.image);
	waitKey(1);

        // todo
        // msg = cv_bridge::CvImage(show.second.header, "bgr8", show.second.image).toImageMsg();
        // image_pub_.publish(msg);
        // pub_frame_.publish(show_pub_ld.second);
    }
}

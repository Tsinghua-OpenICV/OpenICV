#pragma warning(disable: 4819)
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <thread>
#include "LaneDetection.h"


void LaneDetection::setcount(int maincount){
	image_num = maincount;
}

void  LaneDetection::init_id_arr() {
	for (int i = 1; i < 25; i++) {
		id_arr.push_back(i);
	}
}
int  LaneDetection::get_id() {
	if (id_arr.size() > 0) {
		int id = id_arr[0];
		id_arr.erase(id_arr.begin());
		return id;
	}else{ 
		return -1;
	}
}


void  LaneDetection::check_id(int id) {
	id_arr.push_back(id);
}

float LaneDetection::dist_pt(cv::Point2f pt_1, cv::Point2f pt_2) {
	
	return sqrtf(pow(pt_1.x - pt_2.x,2)+pow(pt_1.y - pt_2.y,2));
}
// Initializing variables depending on the resolution of the input image.
double valueAt(std::vector<float>& f, float x) {
	float ans = 0.f;
	for (int i = (int)f.size() - 1; i >= 0; --i)
		ans = ans * x + f[i];
	return ans;
}


LaneDetection::LaneDetection()
{
	vp_pt.y = VP_Y;
	vp_pt.x = VP_X;
	f_image_scale = 2.5;
	f_world_scale =1;
	f_visual_offset = 384;
	init_id_arr();
	
	std::vector<cv::Point2f> origPoints;


	origPoints.push_back(cv::Point2f(754 / f_image_scale, 777 / f_image_scale));
	origPoints.push_back(cv::Point2f(1055 / f_image_scale, 787 / f_image_scale));
	
	origPoints.push_back(cv::Point2f(1245 / f_image_scale, 1074 / f_image_scale));
	origPoints.push_back(cv::Point2f(533 / f_image_scale, 1055 / f_image_scale));

	std::vector<cv::Point2f> dstPoints;
	std::vector<cv::Point2f> dstPoints_visual;

	dstPoints_visual.push_back(cv::Point2f(121 / f_world_scale + f_visual_offset, (1.812+3.485)*100 / f_world_scale));
	dstPoints_visual.push_back(cv::Point2f(-119 / f_world_scale + f_visual_offset, (3.48+1.824)*100 / f_world_scale));
	dstPoints_visual.push_back(cv::Point2f(-221/ f_world_scale + f_visual_offset, 31.5 / f_world_scale));
	dstPoints_visual.push_back(cv::Point2f(216 / f_world_scale + f_visual_offset, 63 / f_world_scale));

	dstPoints.push_back(cv::Point2f(1.253 / f_world_scale, 6.72 / f_world_scale));
	dstPoints.push_back(cv::Point2f(-1.22 / f_world_scale, 6.72 / f_world_scale));

	dstPoints.push_back(cv::Point2f(-1.22 / f_world_scale, 1.515 / f_world_scale));   // cal2
	dstPoints.push_back(cv::Point2f(1.253 / f_world_scale, 1.515 / f_world_scale)); // cal2

	ipm.init(cv::Size(768, 480), cv::Size(768, 480), origPoints, dstPoints);
	ipm_visual.init(cv::Size(768, 480), cv::Size(768, 480), origPoints, dstPoints_visual);

	vp_candidate_x.create(VP_Y_DELTA_NUM * 2, VP_X_DELTA_NUM * 2, CV_32FC1); 
	vp_candidate_y.create(VP_Y_DELTA_NUM * 2, VP_X_DELTA_NUM * 2, CV_32FC1); 
	vp_countlines.create(VP_Y_DELTA_NUM * 2, VP_X_DELTA_NUM * 2, CV_32FC1); 
}

bool LaneDetection::initialize_variable(cv::Mat image) {

	// Image variable setting
	cv::Mat img_src = image.clone();
	if (img_src.empty()) {
		std::cout << "Err: Cannot find an input image for initialization: " << image << std::endl;
		return false;
	}

	img_size = img_src.size();    
	img_height=250;
	img_width = img_src.cols;       
	img_roi_height=100;
	img_depth = img_src.depth();            

	max_lw.resize(img_height);
	min_lw.resize(img_height);
	max_lw_d.resize(img_width);

	for (int hh = img_roi_height; hh < img_height; ++hh) {
		max_lw[hh] = (int)((MAX_LW_N - MAX_LW_F)*(hh - img_roi_height) / (img_size.height - img_roi_height) + MAX_LW_F);
		
		min_lw[hh] = (int)((MIN_LW_N - MIN_LW_F)*(hh - img_roi_height) / (img_size.height - img_roi_height) + MIN_LW_F);		
	}

	int w = img_width - 1;     
	while (img_width - 1 - w < w) {
		max_lw_d[w] = (int)(MAX_LW_D*(fabs(w - (img_width - 1) / 2.0)) / ((img_width - 1) / 2.0));
		max_lw_d[img_width - 1 - w] = (int)(MAX_LW_D*(fabs(w - (img_width - 1) / 2.0)) / ((img_width - 1) / 2.0));
		w--;
	} 


	for (int hh = 0; hh < 3; ++hh) {
		bbb.push_back(0);
	}
	aaa.now.push_back(0);
	aaa.minus1.push_back(0);
	aaa.minus2.push_back(0);

	for (int m = 0; m < VP_Y_DELTA_NUM * 2; m++)
	{
		for (int n = 0; n < VP_X_DELTA_NUM * 2; n++)
		{
			vp_candidate_x.at<float>(m, n) = VP_X - VP_X_DELTA + n * VP_WINDOW * 2 + VP_WINDOW;
			vp_candidate_y.at<float>(m, n) = VP_Y - VP_Y_DELTA + m * VP_WINDOW * 2 + VP_WINDOW;
			vp_countlines.at<float>(m, n) = 0;
		}
	}

	return true;
}


bool LaneDetection::initialize_Img(cv::Mat image) {

	cv::Mat img_src = image.clone();
	img_clr = image.clone(); 

	if (img_src.empty()) {
		std::cout << "Err: Cannot find the input image: " << image << std::endl;
		return false;
	}

	img_gray = cv::Mat(img_size, img_depth);

	if (img_src.channels() == 1) 
	{
		img_src.copyTo(img_gray);                     
	}
	else {
		cv::cvtColor(img_src, img_gray, CV_BGR2GRAY);
	}

	lm.clear();
	marking_seed.resize(0);
	nodes.resize(0);
	edges.resize(0);
	return true;
}



void LaneDetection::lane_marking_detection() {

	cv::Mat img_hsv;
	
	cv::cvtColor(img_clr, img_hsv, CV_BGR2HSV);

	for (int h = img_roi_height; h < img_height;) {

		int hf_size = 2 + 13 * (h - img_roi_height + 1) / (img_height - img_roi_height);

		std::vector<int> scan_line(img_width);
		
		cv::Mat kernd = cv::Mat::zeros(1,2*hf_size+1,CV_8SC1);
		int8* kerndptr = kernd.ptr<int8>(0) ;
		for (int i=0;i<hf_size;i++)
		{
			kerndptr[i] = 1 ;
		}
		
		for (int i=hf_size+1; i<2*hf_size+1 ;i++)
		{
			kerndptr[i]= -1;
		}
		
		
		cv::Mat kerns = cv::Mat::zeros(1,2*hf_size+1,CV_8SC1);
		int8* kernsptr = kerns.ptr<int8>(0) ;
		for (int i=0;i<hf_size;i++)
		{
			kernsptr[i] = 1 ;
		}
		
		for (int i=hf_size+1; i<2*hf_size+1 ;i++)
		{
			kernsptr[i]= 1;
		}
		
		cv::Mat src = img_gray.rowRange(h,h+1).clone();
		
		cv::Mat diff_row, sum_row ;
		cv::filter2D(src, diff_row, CV_16S,kernd);
		cv::filter2D(src, sum_row, CV_16S,kerns);
		int16* diff_ptr = diff_row.ptr<int16>(0) ;
		int16* sum_ptr = sum_row.ptr<int16>(0) ;	

		double lane_time1 = clock();
		int e_flag = 0; // edge flag
		cv::Point2i l_pt, r_pt;
		int m_flag = 0;
		int countt = 0 ;
		for (int w = hf_size +1 ; w < img_width - hf_size -1 ; w++ )
		{

			int l_val = (diff_ptr[w] + sum_ptr[w]) >> 1 ;
			int r_val = (sum_ptr[w] - diff_ptr[w]) >> 1 ;
			
			if (10*r_val > 11*l_val + 5*hf_size)
				scan_line[w] = 1; // right edge = 1, gray level: white is small;
			if (10*l_val > 11*r_val + 5*hf_size) 
				scan_line[w] = -1; // left edge = -1;
	
			if (scan_line[w] == 1) {
				if (e_flag >= 0) {
					e_flag++; //this w is a right climber.
				}
				else { //this w is a right cliff
					scan_line[w + (e_flag >> 1) + 1] = -10; //the left pixels are climbing, but stopped now. so the center of the climbers should be recorded as left edge.
					if (m_flag == 1)
					{
						m_flag = 2;
						r_pt.x = w + (e_flag >> 1)+1;
						r_pt.y = h;
					}
					e_flag = 0;
				}
			}
			else if (scan_line[w] == -1) {
				if (e_flag <= 0) {
					e_flag--;
				}
				else {
					scan_line[w - (e_flag >> 1) + 1] = 10; //e_flag is positive, so this memory execute is smaller than w, thus is safe.
					m_flag = 1;
					l_pt.x = w - (e_flag >> 1) + 1;
					l_pt.y = h;
					e_flag = 0;
				}
			}
			else {
				if (e_flag > 0) {
					scan_line[w - (e_flag >> 1)+1] = 10;
					m_flag = 1;
					l_pt.x = w - (e_flag >> 1) + 1;
					l_pt.y = h;
					e_flag = 0;
				}
				else if (e_flag < 0) {
					scan_line[w + (e_flag >> 1) + 1] = -10;
					if (m_flag == 1)
					{
						m_flag = 2;
						r_pt.x = w + (e_flag >> 1)+1;
						r_pt.y = h;
					}
					e_flag = 0;
				}
			}
			
			if (m_flag == 2) {
				if (((r_pt.x - l_pt.x) >= min_lw[h]) && ((r_pt.x - l_pt.x) <= (max_lw[h] + max_lw_d[w]))) {

					// lane update
					double tt1 = clock();
					LANE_MARKING lm_new;
					countt++;
					
					int Hsum=0,Ssum=0,Vsum=0;

					
					for(int x=l_pt.x;  x<r_pt.x+1;x++){
						int Htemp = img_hsv.at<cv::Vec3b>(h, x)[0];
						int Stemp = img_hsv.at<cv::Vec3b>(h, x)[1];
						int Vtemp = img_hsv.at<cv::Vec3b>(h, x)[2];
						Hsum += Htemp;
						Ssum += Stemp;
						Vsum += Vtemp;
					}
				
					lm_new.H = Hsum / (r_pt.x - l_pt.x+1);
					lm_new.S = Ssum / (r_pt.x - l_pt.x+1);
					lm_new.V = Vsum / (r_pt.x - l_pt.x+1);

					lm_new.str_p = l_pt;                                      //start piont
					lm_new.end_p = r_pt;                                      //end point
					lm_new.cnt_p.x = ((l_pt.x + r_pt.x) >> 1);          //center point x
					lm_new.cnt_p.y = r_pt.y;                                  //center point y
					if (lm_new.cnt_p.x > (img_size.width >> 1)) {
						lm_new.inn_p = l_pt;
					}
					else {
						lm_new.inn_p = r_pt;
					}
					lm_new.size = r_pt.x - l_pt.x;

					lm.push_back(lm_new); //lm are detected lane markings				
					w = r_pt.x + 5;
					m_flag = 0;
					if (lm.size() >= MAX_LANE_MARKING - 1)
					{
						std::cout << "lm break!!!" << std::endl;
						break;
					}
				}
				m_flag = 0;
			}
		}
		double lane_time4 = clock();
		if (lm.size() >= MAX_LANE_MARKING - 1) {
			break;
		}
		
		h += SCAN_INTERVAL1;
	}
}

void LaneDetection::seed_generation() {

	// STEP 1-1. Generating Seeds: Making a bunch of seeds consisting of lane markings near each others.
	int flag_group = 0;
	int flag_dist = 0;
	int marking_seed_index; //record the seed where the grouped lm goes
	for (int ii = lm.size() - 1; ii >= 0; ii--) {          
		flag_group = 0;
		for (int jj = marking_seed.size() - 1; jj >= 0; jj--) {

			bool marking_long=0;
			if(marking_seed[jj].index.size()>10){
				marking_long = 1;
			}
			
			flag_dist = dist_ftn1(ii, marking_seed[jj].index[marking_seed[jj].index.size() - 1], marking_seed[jj].cnt_dir, marking_seed[jj].coeff, marking_long);

			if (flag_dist == 1) {
				flag_group = 1;
				marking_seed[jj].index.push_back(ii);
				if (marking_seed[jj].cnt_dir < -99) {
					marking_seed[jj].cnt_dir = slope_ftn(lm[ii].cnt_p, marking_seed[jj].cnt_p);
				}
				else {
					marking_seed[jj].cnt_dir = 0.8*marking_seed[jj].cnt_dir + 0.2*slope_ftn(lm[ii].cnt_p, marking_seed[jj].cnt_p);
				}
				marking_seed[jj].cnt_p = lm[ii].cnt_p;
				
				std::vector<cv::Point2f> pts;
				std::vector<float> coeff(3);
				
				for (int pp = 0; pp < marking_seed[jj].index.size(); pp++) {
					int idx_lm = marking_seed[jj].index[pp];
					pts.push_back(lm[idx_lm].cnt_p);
				}
				float length = marking_seed[jj].index.size();
				bool isolated_short = (length < 200);
				if (isolated_short) {
					coeff.resize(2);
					poly2(pts, pts.size(), coeff);
				}
				else {
					poly3(pts, pts.size(), coeff);
				}
				marking_seed[jj].coeff = coeff;

				break;
			}
		}
		if (flag_group == 0) {
			MARKING_SEED seed_new;
			seed_new.flag = 0;
			seed_new.index.resize(0);
			seed_new.index.push_back(ii);
			seed_new.cnt_dir = -100;
			seed_new.cnt_p = lm[ii].cnt_p;
			marking_seed.push_back(seed_new);
		}

	}

    for (int i = 0; i < marking_seed.size(); i++) {
        reverse(marking_seed[i].index.begin(), marking_seed[i].index.end());
    }

	
	int count_i, count_j;
	float var;
	int seedSsum;
	int seedHsum;
	int seedVsum;
	float seedV;
	float seedS;
	float seedH;
	
	for (int ii = 0; ii < marking_seed.size(); ii++) {
		count_i = marking_seed[ii].index.size();

        int lm_idx = marking_seed[ii].index[0];

        // by @wentuopu keep markings near the vanishing point//jxy: distance to the center of the top of roi, e.g. detected vp.
        //can verify use vp
		int dist_center_x = (marking_seed[ii].cnt_p.x - img_width / 2);
        int dist_center_y = (marking_seed[ii].cnt_p.y - img_roi_height);
        float dist = sqrt(dist_center_x * dist_center_x + dist_center_y * dist_center_y);
		//jxy: add seedS validation
		seedSsum = 0;
		seedHsum = 0;
		seedVsum = 0;
		for (int jj = 0; jj < marking_seed[ii].index.size(); ++jj)
		{
			int idx = marking_seed[ii].index[jj];
			seedSsum += lm[idx].S;
			seedHsum += lm[idx].H;
			seedVsum += lm[idx].V;
		}
		seedS = (float)seedSsum / marking_seed[ii].index.size();
		seedH = (float)seedHsum / marking_seed[ii].index.size();
		seedV = (float)seedVsum / marking_seed[ii].index.size();

		
		if(!((seedS<80) || (10<seedH && seedH<44))){
			marking_seed[ii].flag = -2;
		}
		else if (count_i < VALID_SEED_MARKING_NUMBER_THRES && dist > 70) {//TODO: a more robust solution, now number threshold is 6, dist threshold is 20*2^0.5
			marking_seed[ii].flag = -1;	
		}
		else if (count_i < VALID_SEED_MARKING_NUMBER_THRES) {
			float mean = 0.f;
			for (int jj = 0; jj < count_i; jj++) {
				int idx_i = marking_seed[ii].index[jj];
				mean = mean + lm[idx_i].size;
			}
			mean = (float)mean / (float)count_i;
			float var = 0.f;
			for (int jj = 0; jj < count_i; jj++) {
				int idx_i = marking_seed[ii].index[jj];
				var = var + (lm[idx_i].size - mean)*(lm[idx_i].size - mean);
			}
			var = var / (float)count_i;

			if (var > 6.0) {
				marking_seed[ii].flag = -1;
			}
		}

	}

	// STEP 1-3. Seed specification: Getting information of each seeds, position & direction
	std::vector<int> val_seed;

	
	for (int ii = 0; ii < marking_seed.size(); ii++) {
		if (marking_seed[ii].flag < 0) {
			continue;
		}
		seed_specification(marking_seed[ii], 1);
		val_seed.push_back(ii);
	}

	
	// STEP 2. Seed Growing - Dist_mat Generation
	int n_of_valid_seeds = val_seed.size();
	cv::Mat dist_mat = cv::Mat(n_of_valid_seeds, n_of_valid_seeds, CV_32FC1);

	for (int ii = 0; ii < n_of_valid_seeds; ++ii) {
		dist_mat.at<float>(ii, ii) = -1.f;
		for (int jj = ii + 1; jj < n_of_valid_seeds; ++jj) {
			dist_mat.at<float>(ii, jj) = dist_ftn2(val_seed[jj], val_seed[ii]);
			dist_mat.at<float>(jj, ii) = dist_mat.at<float>(ii, jj);
		}
	}
    
	
	// STEP 2-1. Low Level Association Process #1 - Head -> Tail
    // by @wentuopu
    // change the algorithm, every time the marking will only search the best marking in front whose dist is larger than threshold. The global optimum is not guaranteed. 

	for (int ii = 0; ii < n_of_valid_seeds; ++ii) {
		int cnct_count = 0;
		int cnct_idx = -1;
        float temp_dist_mat = -2;
		int valid_flag = 0;
		for (int jj = 0; jj < ii; ++jj) {
			if (dist_mat.at<float>(ii, jj) > LOW_LEVEL_ASS_THRES) {
				cnct_count++;
                //cnct_idx = jj;
		        float temp_max = LOW_LEVEL_ASS_THRES;
		        int max_id = -1;
                for (int kk = jj; kk < n_of_valid_seeds; kk++) {
                    if (dist_mat.at<float>(jj, kk) > temp_max) {
                        temp_max = dist_mat.at<float>(jj, kk);
                        max_id = kk;
                    }
                }
                if (max_id == ii) {
                   valid_flag = 1;
                   if (dist_mat.at<float>(ii, jj) > temp_dist_mat) {
                       cnct_idx = jj;
                       temp_dist_mat = dist_mat.at<float>(ii, jj);
                   }
                }
			}
		}
       
		if (valid_flag == 1) {
			//	The only seed which comes down to 'cnct_idx' is 'ii'. Thus, 'cnct_idx' has to be connected to 'ii'.
			MARKING_SEED* seed_dst = &marking_seed[val_seed[cnct_idx]];
			MARKING_SEED* seed_connect = &marking_seed[val_seed[ii]];
			count_j = seed_connect->index.size();
			for (int kk = 0; kk < count_j; kk++) {
				seed_dst->index.push_back(seed_connect->index[kk]);
			}
			seed_connect->index.resize(0);
			seed_dst->flag = 1;
			seed_connect->flag = -1;	// seed # which become included in i
			seed_specification(*seed_dst, 0);
			seed_dst->str_dir = seed_connect->str_dir;
			seed_dst->str_p = seed_connect->str_p;
			seed_dst->length = seed_dst->length + seed_connect->length;
			
            for (int ll = 0; ll < n_of_valid_seeds; ++ll)
                dist_mat.at<float>(ll, ii) = -1;
		}
	}
}

void LaneDetection::seed_specification(MARKING_SEED& marking_seed_curr, int mode) {

	float temp_x = 0;
	float temp_y = 0;

	std::vector<float> coeff2;
	std::vector<cv::Point2f> points;
	coeff2.resize(2);
	int n_of_lm = marking_seed_curr.index.size();

	for (int ii = 0; ii < n_of_lm; ii++) {
		int idx_lm = marking_seed_curr.index[ii];
		temp_x += (float)lm[idx_lm].cnt_p.x;
		temp_y += (float)lm[idx_lm].cnt_p.y;
		points.push_back(lm[idx_lm].cnt_p);
	}
	poly2(points, points.size(), coeff2);
	marking_seed_curr.cnt_dir = CV_PI / 2 - atan(coeff2[1]);
	marking_seed_curr.cnt_p.x = (int)(temp_x / n_of_lm);
	marking_seed_curr.cnt_p.y = (int)(temp_y / n_of_lm);

	if (mode == 1) {	// initial seed
		marking_seed_curr.str_p = lm[marking_seed_curr.index[0]].cnt_p;
		marking_seed_curr.end_p = lm[marking_seed_curr.index[n_of_lm - 1]].cnt_p;
		marking_seed_curr.length = length_ftn(marking_seed_curr.str_p, marking_seed_curr.end_p);
		if (n_of_lm < VALID_SEED_MARKING_NUMBER_THRES) {
			marking_seed_curr.end_dir = marking_seed_curr.cnt_dir;
			marking_seed_curr.str_dir = marking_seed_curr.cnt_dir;
		}
		else {
			int n_samples = std::max(5, (int)(0.3f*n_of_lm));
			poly2(points, n_samples, coeff2);
			marking_seed_curr.str_dir = (float)(CV_PI / 2 - atan(coeff2[1]));
			points.resize(0);
			for (int ii = n_of_lm - 1; ii >= n_of_lm - n_samples; ii--) {
				int idx_i = marking_seed_curr.index[ii];
				points.push_back(lm[idx_i].cnt_p);
			}
			poly2(points, n_samples, coeff2);
			marking_seed_curr.end_dir = (float)(CV_PI / 2 - atan(coeff2[1]));
		}
	}

}

void LaneDetection::graph_generation() {

	srand((unsigned)time(NULL));

	// STEP 1. Graph Formulation
	std::vector<int> grp_seed;

	for (int ii = 0; ii < marking_seed.size(); ii++) {
		if (marking_seed[ii].flag < 0) {
			continue;
		}
		if (marking_seed[ii].index.size() < VALID_SEED_MARKING_NUMBER_THRES) {
			continue;
		}
		grp_seed.push_back(ii);
	}


	// STEP 2-1. Node Generation - Generating valid node 
	int n_of_grp_seeds = grp_seed.size();
	cv::Mat vert_mat = cv::Mat(n_of_grp_seeds, n_of_grp_seeds, CV_32SC1);
	std::vector<int> row_sum(n_of_grp_seeds);
	std::vector<int> col_sum(n_of_grp_seeds);
	std::vector<int> ele_sum(n_of_grp_seeds);

	for (int ii = 0; ii < n_of_grp_seeds; ii++) {
		for (int jj = 0; jj < n_of_grp_seeds; jj++) {
			vert_mat.at<int>(ii, jj) = dist_ftn3(grp_seed[ii], grp_seed[jj], ii, jj);
		}
		vert_mat.at<int>(ii, ii) = -1;
	}


	// STEP 2-2. Separating nodes to each groups
	int n_of_node_grps = 0;
	for (int ii = 0; ii < n_of_grp_seeds; ii++) {
		for (int jj = 0; jj < n_of_grp_seeds; jj++) {
			if (vert_mat.at<int>(ii, jj) == 1) {
				vert_mat.at<int>(ii, jj) = n_of_node_grps + 100;
				node_grouping(vert_mat, n_of_grp_seeds, 0, ii, n_of_node_grps + 100);
				node_grouping(vert_mat, n_of_grp_seeds, 0, jj, n_of_node_grps + 100);
				node_grouping(vert_mat, n_of_grp_seeds, 1, jj, n_of_node_grps + 100);
				node_grouping(vert_mat, n_of_grp_seeds, 1, ii, n_of_node_grps + 100);
				n_of_node_grps++;
			}
		}
	}


	// STEP 2-3. Node indexing & initialization
	nodes.resize(0);
	for (int ii = 0; ii < n_of_grp_seeds; ii++) {
		for (int jj = 0; jj < n_of_grp_seeds; jj++) {
			if (vert_mat.at<int>(ii, jj) >= 100) {
				NODE_CRF node_new;
				node_new.vert_idx1 = ii;
				node_new.vert_idx2 = jj;
				node_new.idx = vert_mat.at<int>(ii, jj) - 100;
				node_new.unary = 0 ;
				nodes.push_back(node_new);
			}
		}
	}
	
	// STEP 2-4. Node Grouping
	std::vector<NODE_GRP> node_grp(n_of_node_grps);
	for (int ii = 0; ii < nodes.size(); ii++) {
		int node_grp_idx = nodes[ii].idx;
		node_grp[node_grp_idx].idx.push_back(ii);
	}
	

	// Hungarian Method
		// 1) Sorting! in the order of Unary term - Unary term:Logistic function, Sorting - bubble sort
		// 2) Labling using the Constraint - with clear rules! with 4)
		// 3) Calculating the pairwise term with finding Edges - Nodes which are in the same group have the same edges
		// 4) iteration back to the 1) - clear rules, with 2)


	// STEP 3. Hungarian Methos, Edge Indexing, Initialization	

	for (int nn = 0; nn < n_of_node_grps; nn++) {
		// STEP 3-1. Sorting! in the order of Unary term - Unary term:Logistic function, Sorting - bubble sort
		for (int ii = node_grp[nn].idx.size() - 1; ii > 0; ii--) {
			for (int jj = 0; jj < ii; jj++) {
				if (nodes[node_grp[nn].idx[jj]].unary < nodes[node_grp[nn].idx[jj + 1]].unary) {
					int temp_val = node_grp[nn].idx[jj + 1];
					node_grp[nn].idx[jj + 1] = node_grp[nn].idx[jj];
					node_grp[nn].idx[jj] = temp_val;
				}
			}
		}

		if (node_grp[nn].idx.size() == 1) {	// trivial case which doesn't need the inference 
			continue;
		}
		for (int n_iter = 0; n_iter < node_grp[nn].idx.size(); n_iter++) {
			// STEP 3-2. For each iteration in Hungarian Methods, Find the possible edges
			for (int ii = 0; ii < n_of_grp_seeds; ii++) {
				row_sum[ii] = 0;
				col_sum[ii] = 0;
				ele_sum[ii] = 0;
			}
			nodes[node_grp[nn].idx[n_iter]].label = 1;
			int n_of_labels = 1;
			row_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx1]++;
			col_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx2]++;
			ele_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx1]++;
			ele_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx2]++;

			for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
				if (ii == n_iter) {
					continue;
				}
				if (row_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]>0) {
					nodes[node_grp[nn].idx[ii]].label = 0;
					continue;
				}
				if (col_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]>0) {
					nodes[node_grp[nn].idx[ii]].label = 0;
					continue;
				}
				nodes[node_grp[nn].idx[ii]].label = 1;
				n_of_labels++;
				row_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]++;
				col_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]++;
				ele_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]++;
				ele_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]++;
			}
			// Discarding those which cannot construct an edge
			if (n_of_labels <= 1) {
				continue;
			}
			// Indexing nodes consisting of the edge
			EDGE_CRF edge_new;
			for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
				if (nodes[node_grp[nn].idx[ii]].label == 1) {
					edge_new.node_idx.push_back(node_grp[nn].idx[ii]);
				}
			}
			int iden_flag = 0;	// 0 if different, 1 if identical
			for (int ii = 0; ii < edges.size(); ii++) {
				if (edges[ii].node_idx.size() != edge_new.node_idx.size()) {
					iden_flag = 0;
					continue;
				}
				for (int jj = 0; jj < edge_new.node_idx.size(); jj++) {
					if (edges[ii].node_idx[jj] != edge_new.node_idx[jj]) {
						iden_flag = 0;
						break;
					}
					iden_flag = 1;
				}
				if (iden_flag == 1) {
					break;
				}
			}

			if ((edges.size() != 0) && (iden_flag == 1)) {
				continue;	// this edges is already included
			}

			// STEP 3-3. Pairwise cost calculation
			int n_of_pts = 0;
			std::vector<cv::Point2f> pts;
			for (int ii = 0; ii < n_of_grp_seeds; ii++) {
				if (ele_sum[ii] > 0) {
					int count_i = marking_seed[grp_seed[ii]].index.size();
					for (int jj = 0; jj < count_i; jj++) {
						if (count_i > 15) {
							if (jj % (count_i / 14) != 0) {
								continue;
							}
						}
						cv::Point2f pts_new;
						pts_new.x = (float)lm[marking_seed[grp_seed[ii]].index[jj]].cnt_p.x;
						pts_new.y = (float)lm[marking_seed[grp_seed[ii]].index[jj]].cnt_p.y;
						pts.push_back(pts_new);
					}
				}
			}

			edge_new.pairwise = pairwise_ftn(pts);
			edge_new.grp_idx = nn;
			edges.push_back(edge_new);

		}
	}

	// CRF Formulation, Hungarian Method	
	std::vector<int> final_label;
	final_label.resize(nodes.size(), -1);

	double energy = 0;
	double min_energy = 0;
	int expt_flag = 0;
	for (int nn = 0; nn < n_of_node_grps; nn++) {

		min_energy = 0;
		for (int n_iter = 0; n_iter<node_grp[nn].idx.size(); n_iter++) {
			// Exception # 1
			if (node_grp[nn].idx.size() == 1) {
				if (nodes[node_grp[nn].idx[0]].unary > 0.5) {
					final_label[node_grp[nn].idx[0]] = 1;
				}
				else {
					final_label[node_grp[nn].idx[0]] = 0;
				}
				continue;
			}
			// Exception # 2
			expt_flag = 0;
			for (int ii = 0; ii<node_grp[nn].idx.size(); ii++) {
				if (nodes[node_grp[nn].idx[ii]].unary > 0.5) {
					break;
				}
				if (ii == node_grp[nn].idx.size() - 1) {
					expt_flag = 1;
				}
			}
			if (expt_flag == 1) {
				continue;
			}

			// 2) Labling using the Constraint - with clear rules! with 4)
			for (int ii = 0; ii < n_of_grp_seeds; ii++) {
				row_sum[ii] = 0;
				col_sum[ii] = 0;
				ele_sum[ii] = 0;
			}
			nodes[node_grp[nn].idx[n_iter]].label = 1;
			int n_of_labels = 1;
			row_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx1]++;
			col_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx2]++;
			ele_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx1]++;
			ele_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx2]++;
			for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
				if (ii == n_iter) {
					continue;
				}
				if (row_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]>0) {
					nodes[node_grp[nn].idx[ii]].label = 0;
					continue;
				}
				if (col_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]>0) {
					nodes[node_grp[nn].idx[ii]].label = 0;
					continue;
				}
				nodes[node_grp[nn].idx[ii]].label = 1;
				n_of_labels++;
				row_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]++;
				col_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]++;
				ele_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]++;
				ele_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]++;
			}

			
			for (int ii = 0; ii < edges.size(); ii++) {
				if (edges[ii].grp_idx != nn) {
					continue;
				}
				if (edges[ii].node_idx.size() != n_of_labels) {
					edges[ii].label = 0;
					continue;
				}
				for (int jj = 0; jj < edges[ii].node_idx.size(); jj++) {
					if (nodes[edges[ii].node_idx[jj]].label != 1) {
						edges[ii].label = 0;
						break;
					}
					edges[ii].label = 1;
				}
			}
			//  Calculating Energy & Updating labels
			energy = 0;
			for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
				if (nodes[node_grp[nn].idx[ii]].label == 1) {
					energy = energy - nodes[node_grp[nn].idx[ii]].unary;
				}
				else {
					energy = energy - (1 - nodes[node_grp[nn].idx[ii]].unary);
				}
			}
			expt_flag = 0;
			for (int ii = 0; ii < edges.size(); ++ii) {
				if (edges[ii].grp_idx != nn) {
					continue;
				}
				if (edges[ii].label == 1) {
					energy = energy - edges[ii].pairwise;
					if (edges[ii].pairwise < 0.5) {
						expt_flag = 1;
					}
				}
				else {
					energy = energy - (1 - edges[ii].pairwise);
				}
			}

			if (energy < min_energy) {
				min_energy = energy;
				for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
					final_label[node_grp[nn].idx[ii]] = nodes[node_grp[nn].idx[ii]].label;
					if (expt_flag == 1) {
						final_label[node_grp[nn].idx[ii]] = 0;
					}
				}
			}
		}         
	}


	// Seeds association according to the nodes those having been labeled by 1 : s_i -> s_j ( s_i is absorbed into s_j )
	int s_i, s_j;
	int count_j;
	for (int ii = 0; ii < nodes.size(); ++ii) {
		if (nodes[ii].label != 1) {
			continue;
		}
		s_i = grp_seed[nodes[ii].vert_idx1];
		s_j = grp_seed[nodes[ii].vert_idx2];
		count_j = marking_seed[s_j].index.size();
		for (int jj = 0; jj < marking_seed[s_i].index.size(); ++jj) {
			marking_seed[s_j].index.push_back(marking_seed[s_i].index[jj]);
		}

		marking_seed[s_j].flag = 1;
		marking_seed[s_i].flag = -1;
		seed_specification(marking_seed[s_j], 0);
		marking_seed[s_j].str_dir = marking_seed[s_i].str_dir;
		marking_seed[s_j].str_p = marking_seed[s_i].str_p;
		marking_seed[s_j].length = marking_seed[s_i].length + marking_seed[s_j].length;
	}

}

void LaneDetection::validating_final_seeds() {
	//Clear PolyfitParam
	v_PolyfitParam.v_ld_coeff.clear();
	v_PolyfitParam.v_WholeLane_World.clear();
	v_PolyfitParam.v_WholeLane_Pixel.clear();
	
	int count_coeff = 0;
	for (int jj = 1; jj >= 0; --jj) { 
		bbb[jj + 1] = bbb[jj]; 
	}

	aaa.minus2 = aaa.minus1;
	aaa.minus1 = aaa.now;
	aaa.now.resize(0);
	aaamarking.resize(0);

	std::vector<std::vector<float> > coeff_pic; 
	std::vector<int> marking_id_pic; 
	float vote_vp_x;
	float vote_vp_y;
	cv::Point2f dot_p;

	clock_t begin2, end2;
	begin2 = clock(); 

	for (int ii = 0; ii < marking_seed.size(); ii++) 
	{
		float length = length_ftn(marking_seed[ii].end_p, marking_seed[ii].str_p);

		if ((marking_seed[ii].flag == 0) && (marking_seed[ii].index.size() > VALID_SEED_MARKING_NUMBER_THRES)) { 
			marking_seed[ii].flag = 1;
		}
		if (marking_seed[ii].flag < 1) {
			continue;
		}
		
		if (length < 20) {
			
			continue;
		}
		if (marking_seed[ii].length < 50) {
			marking_seed[ii].flag = 0;
		}
		if (marking_seed[ii].length / length < 0.20) {
			marking_seed[ii].flag = 0;
			
		}
		if ((length == marking_seed[ii].length) && (length < 62)) {
			marking_seed[ii].flag = 0;
			
		}
		std::vector<float> coeff(3);
		std::vector<cv::Point2f> pts;
		
		for (int pp = 0; pp < marking_seed[ii].index.size(); pp++) {
			int idx_lm = marking_seed[ii].index[pp];
			pts.push_back(lm[idx_lm].cnt_p);
		}
		bool isolated_short = (length < 200);
		
		if (isolated_short) {
			coeff.resize(2);
			poly2(pts, pts.size(), coeff);
		}
		else {
			poly3(pts, pts.size(), coeff);
		}
		
		coeff_pic.push_back(coeff);
		marking_id_pic.push_back(ii);
		
		for (int m = 0; m < VP_Y_DELTA_NUM * 2; m++)
		{
			for (int n = 0; n < VP_X_DELTA_NUM * 2; n++)
			{
				if (pass_rectangle(coeff, vp_candidate_x.at<float>(m, n), vp_candidate_y.at<float>(m, n), VP_WINDOW, WINDOW_EXPANSION)){
					vp_countlines.at<float>(m, n) = vp_countlines.at<float>(m, n) + 1;
				}
			}
		}
		
		double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
		cv::minMaxLoc(vp_countlines, &minVal, &maxVal, &minLoc, &maxLoc);
		std::vector<int> max_x_list;
		std::vector<int> max_y_list;
		float min_dist_to_last = 1000; //big enough

		if (image_num == image_num_initial)
		{
			last_vp.x = VP_X;
			last_vp.y = VP_Y;
		}

		for (int m = 0; m < VP_Y_DELTA_NUM * 2; m++)
		{
			for (int n = 0; n < VP_X_DELTA_NUM * 2; n++)
			{
				if (vp_countlines.at<float>(m, n) == maxVal){
					max_x_list.push_back(m);
					max_y_list.push_back(n);
					float temp_distance = sqrt(pow((vp_candidate_x.at<float>(m,n)-last_vp.y),2)+pow(vp_candidate_x.at<float>(m,n)-last_vp.x,2));
					if(temp_distance<min_dist_to_last){
						vote_vp_x = vp_candidate_x.at<float>(m, n);
						vote_vp_y = vp_candidate_y.at<float>(m, n);
					}
				}
			}
		}

		vp_pt.y = vote_vp_y;
		vp_pt.x = vote_vp_x;

		last_vp.y = vote_vp_y;
		last_vp.x = vote_vp_x; 
	}
	end2 = clock();

	for (int ii = 0; ii < coeff_pic.size(); ii++)
	{
		std::vector<float> coeff=coeff_pic[ii];
		int marking_id=marking_id_pic[ii];		
		vp_pt.y = vote_vp_y;
		vp_pt.x = vote_vp_x;
	

		float vanish_point_x = vp_pt.x;
		float vanish_point_y = vp_pt.y;

		std::vector<float> dist_to_vp(img_height-1);
		int mindist = 1;
		float xx;

		for (int yy = 1; yy < img_height; ++yy) { 
			xx = valueAt(coeff, yy);
			dist_to_vp[yy - 1] = 0;
		}
		for (int yy = 1; yy < img_height; ++yy) {
			xx = valueAt(coeff, yy);
			dist_to_vp[yy - 1] = sqrt((float)(xx - vanish_point_x)*(xx - vanish_point_x) + (float)(yy - vanish_point_y)*(yy - vanish_point_y));
		}
		for (int yy = 1; yy < img_height; ++yy) {
			if (dist_to_vp[yy - 1] < dist_to_vp[mindist - 1]){
				mindist = yy - 1;
			}
		}

		if (dist_to_vp[mindist] > VP_WINDOW*WINDOW_EXPANSION*6) {
			continue; 
		}
		else if(dist_to_vp[mindist] > VP_WINDOW*WINDOW_EXPANSION*4 && marking_seed[marking_id].index.size()<50){
			continue; 
		}
		count_coeff = count_coeff + 1;
		bbb[0] = count_coeff;

		if (coeff.size() == 3) {
			aaa.now.push_back(coeff[2]);
			aaa.now.push_back(coeff[1]);
			aaa.now.push_back(coeff[0]); 
		}
		else {
			aaa.now.push_back(0);
			aaa.now.push_back(coeff[1]);
			aaa.now.push_back(coeff[0]); 
		}
		aaamarking.push_back(marking_id); 
		for (int jj = 0; jj < marking_seed[marking_id].index.size(); ++jj) {
			int temp_i = marking_seed[marking_id].index[jj];
		}
	}

	if(coeff_pic.size()==0){
		bbb[0] = 0;
	}

}

//-------@--kalman--------------
cv::Mat LaneDetection::kalman() { 
	cv::Mat img_kalman = img_clr.clone();
	cv::Mat img_test_kalman_pixel = img_clr.clone();

	//When no lines are detected, each numempty should add by one.
	if (bbb[0] == 0)
	{
		std::cout << "No lines!" << std::endl;
		image_num = image_num + 1;
		for (int jj = 0; jj < num_Xm; ++jj)
		{
			if(numempty.at<float>(jj,0)<0){
				numempty.at<float>(jj, 0)=0;
			}
			numempty.at<float>(jj, 0) = numempty.at<float>(jj, 0) + 1;
			if (numempty.at<float>(jj, 0) > 5)
			{
				Xm = del_num(Xm, jj * 3 + 2);
				Xm = del_num(Xm, jj * 3 + 1);
				Xm = del_num(Xm, jj * 3);
				numempty = del_num(numempty, jj);
				Pm = del_row(Pm, jj * 3 + 2);
				Pm = del_row(Pm, jj * 3 + 1);
				Pm = del_row(Pm, jj * 3);
				Pm = Pm.t(); 
				Pm = del_row(Pm, jj * 3 + 2);
				Pm = del_row(Pm, jj * 3 + 1);
				Pm = del_row(Pm, jj * 3);
				Pm = Pm.t(); 
				num_Xm = num_Xm - 1;

			}
		}
		return img_kalman;
	}

	//Clear PolyfitParam
	v_PolyfitParam.v_ld_coeff.clear();
	v_PolyfitParam.v_WholeLane_World.clear();
	v_PolyfitParam.v_WholeLane_Pixel.clear();
	cv::Mat Yaaa(bbb[0] * 3, 1, CV_32FC1); 
	cv::Mat Yaaa_1(bbb[1] * 3, 1, CV_32FC1); 
	cv::Mat Yaaa_2(bbb[2] * 3, 1, CV_32FC1); 
	cv::Mat marking_ids(bbb[0], 1, CV_32FC1);

	for (int jj = 0; jj<bbb[0]; ++jj) {  
		Yaaa.at<float>(jj * 3, 0) = aaa.now[jj * 3];
		Yaaa.at<float>(jj * 3 + 1, 0) = aaa.now[jj * 3 + 1];
		Yaaa.at<float>(jj * 3 + 2, 0) = aaa.now[jj * 3 + 2];
		marking_ids.at<float>(jj,0) = aaamarking[jj];
	}
	for (int jj = 0; jj<bbb[1]; ++jj) { 
		Yaaa_1.at<float>(jj * 3, 0) = aaa.minus1[jj * 3];
		Yaaa_1.at<float>(jj * 3 + 1, 0) = aaa.minus1[jj * 3 + 1];
		Yaaa_1.at<float>(jj * 3 + 2, 0) = aaa.minus1[jj * 3 + 2];
	}
	for (int jj = 0; jj<bbb[2]; ++jj) {
		Yaaa_2.at<float>(jj * 3, 0) = aaa.minus2[jj * 3];
		Yaaa_2.at<float>(jj * 3 + 1, 0) = aaa.minus2[jj * 3 + 1];
		Yaaa_2.at<float>(jj * 3 + 2, 0) = aaa.minus2[jj * 3 + 2];
	}

	 //ONLY ONCE
	if (image_num == image_num_initial || flag_initial == 1) { 
		Xm = Yaaa;
		num_Xm = bbb[0];
		cv::Mat sigma(bbb[0] * 3, 1, CV_32FC1);
		for (int jj = 0; jj<bbb[0]; ++jj) {
			sigma.at<float>(jj * 3, 0) = 0.0006; 
			sigma.at<float>(jj * 3 + 1, 0) = 2.4;
			sigma.at<float>(jj * 3 + 2, 0) = 500;
		}
		cv::Mat V = cv::Mat::eye(bbb[0] * 3, bbb[0] * 3, CV_32FC1);
		for (int jj = 0; jj<bbb[0] * 3; ++jj) {
			V.at<float>(jj, jj) = sigma.at<float>(0, jj);
		}
		R = V*V; 
		cv::Mat W = 3 * V;
		Q = W*W;
		Phi = cv::Mat::eye(bbb[0] * 3, bbb[0] * 3, CV_32FC1);
		H = cv::Mat::eye(bbb[0] * 3, bbb[0] * 3, CV_32FC1);
		Pm = cv::Mat::eye(bbb[0] * 3, bbb[0] * 3, CV_32FC1);
		numempty = cv::Mat::zeros(bbb[0], 1, CV_32FC1);
		lane_g_id = cv::Mat::zeros(bbb[0],1, CV_8U);
		for (int jj = 0; jj<bbb[0]; ++jj) {
			lane_g_id.at<uchar>(jj, 0) = (uchar)get_id(); 
		}

		flag_initial = 0;
	}

	//合并距离过近的舱 将过近的参数存放到rubbish中，然后剔除XM
	int count_cabin = 0;
	cv::Mat distmat_Xm = cv::Mat::zeros(num_Xm, num_Xm, CV_32FC1);
	for (int jj = 0; jj<num_Xm; ++jj) {
		for (int kk = 0; kk<num_Xm; ++kk) {
			distmat_Xm.at<float>(jj, kk) = dist_line(Xm.at<float>(jj * 3), Xm.at<float>(jj * 3 + 1), Xm.at<float>(jj * 3 + 2), Xm.at<float>(kk * 3), Xm.at<float>(kk * 3 + 1), Xm.at<float>(kk * 3 + 2));
		}
	}
	std::vector<float> rubbish;
	for (int jj = 0; jj<num_Xm; ++jj) {
		for (int kk = jj + 1; kk<num_Xm; ++kk) {
			if ((distmat_Xm.at<float>(jj, kk)<KALMAN_THRES) && !(isinside(rubbish, kk))) {
				rubbish.push_back(kk); 
				count_cabin = count_cabin + 1;
			}
		}
	}

	std::sort(rubbish.begin(), rubbish.end());
	num_Xm = num_Xm - count_cabin;
	for (int jj = count_cabin - 1; jj >= 0; --jj) {  
		Xm = del_num(Xm, rubbish[jj] * 3 + 2);
		Xm = del_num(Xm, rubbish[jj] * 3 + 1);
		Xm = del_num(Xm, rubbish[jj] * 3);
		numempty = del_num(numempty, rubbish[jj]);
		Pm = del_row(Pm, rubbish[jj] * 3 + 2);
		Pm = del_row(Pm, rubbish[jj] * 3 + 1);
		Pm = del_row(Pm, rubbish[jj] * 3);
		Pm = Pm.t(); 
		Pm = del_row(Pm, rubbish[jj] * 3 + 2);
		Pm = del_row(Pm, rubbish[jj] * 3 + 1);
		Pm = del_row(Pm, rubbish[jj] * 3);
		Pm = Pm.t(); 
		check_id((int)lane_g_id.at<uchar>((int)rubbish[jj],0));
		lane_g_id = del_row_8u(lane_g_id, rubbish[jj]); 
	}
	

	int num_new = bbb[0];

	//求解新息和原始线的对应性
	cv::Mat distmat = cv::Mat::zeros(num_Xm, num_new, CV_32FC1);
	cv::Mat identity = cv::Mat::zeros(num_Xm, 1, CV_32FC1);
	for (int jj = 0; jj<num_Xm; ++jj) {
		for (int kk = 0; kk<num_new; ++kk) {
			distmat.at<float>(jj, kk) = dist_line(Xm.at<float>(jj * 3), Xm.at<float>(jj * 3 + 1), Xm.at<float>(jj * 3 + 2), Yaaa.at<float>(kk * 3), Yaaa.at<float>(kk * 3 + 1), Yaaa.at<float>(kk * 3 + 2));
		}
	}
	for (int jj = 0; jj<num_Xm; ++jj) {
		cv::Mat temp = distmat.row(jj).clone(); 
		double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
		cv::minMaxLoc(temp, &minVal, &maxVal, &minLoc, &maxLoc);
		
		if (minVal<KALMAN_THRES) {
			identity.at<float>(jj, 0) = minLoc.x;
			if(numempty.at<float>(jj,0)>0){
				numempty.at<float>(jj, 0)=0;
			}
			numempty.at<float>(jj, 0) = numempty.at<float>(jj, 0)-1; 
		}
		else {
			identity.at<float>(jj, 0) = -1; 
			if(numempty.at<float>(jj,0)<0){
				numempty.at<float>(jj, 0)=0;
			}
			numempty.at<float>(jj, 0) = numempty.at<float>(jj, 0) + 1;
		}
	}

	//去除漏检5次的线及numempty，改变协方差矩阵，更改identity阵
	for (int jj = num_Xm - 1; jj >= 0; --jj) {
		if (numempty.at<float>(jj, 0) >= 5) {
			Xm = del_num(Xm, jj * 3 + 2);
			Xm = del_num(Xm, jj * 3 + 1);
			Xm = del_num(Xm, jj * 3);
			numempty = del_num(numempty, jj);
			Pm = del_row(Pm, jj * 3 + 2);
			Pm = del_row(Pm, jj * 3 + 1);
			Pm = del_row(Pm, jj * 3);
			Pm = Pm.t(); 
			Pm = del_row(Pm, jj * 3 + 2);
			Pm = del_row(Pm, jj * 3 + 1);
			Pm = del_row(Pm, jj * 3);
			Pm = Pm.t(); 
			num_Xm = num_Xm - 1;
			identity = del_num(identity, jj);
			check_id((int)lane_g_id.at<uchar>(jj, 0));
			lane_g_id = del_row_8u(lane_g_id, jj); 
		}
	}

	//增舱
	int temp_trace;
	if (image_num - image_num_initial>3) {
		float num_old = (bbb[0] + bbb[1] + bbb[2]) / 3.0;
		
		cv::Mat flag = cv::Mat::zeros(num_new, 1, CV_32FC1);
		if (num_old>num_Xm) {
			for (int kk = 0; kk<num_new; ++kk) {
				if (ismember(identity, kk) == 0) {
					for (int mm = 0; mm<bbb[1]; ++mm) {
				
						if (dist_line(Yaaa.at<float>(kk * 3), Yaaa.at<float>(kk * 3 + 1), Yaaa.at<float>(kk * 3 + 2), Yaaa_1.at<float>(mm * 3), Yaaa_1.at<float>(mm * 3 + 1), Yaaa_1.at<float>(mm * 3 + 2))<KALMAN_THRES) {
							flag.at<float>(kk, 0) = 0.5;
							temp_trace = mm;
							
							break;
						}
						flag.at<float>(kk, 0) = -1;
					}
					if (flag.at<float>(kk, 0) == 0.5) {
						for (int mm = 0; mm<bbb[2]; ++mm) {
							if (dist_line(Yaaa_1.at<float>(temp_trace * 3), Yaaa_1.at<float>(temp_trace * 3 + 1), Yaaa_1.at<float>(temp_trace * 3 + 2), Yaaa_2.at<float>(mm * 3), Yaaa_2.at<float>(mm * 3 + 1), Yaaa_2.at<float>(mm * 3 + 2))<KALMAN_THRES) {
								flag.at<float>(kk, 0) = 1;
								break;
							}
							flag.at<float>(kk, 0) = -1;
						}
					}
					if(flag.at<float>(kk,0) == 1){
						for (int mm = 0; mm < num_Xm; ++mm)
						{
							float tempdist = dist_line(Xm.at<float>(mm * 3), Xm.at<float>(mm * 3 + 1), Xm.at<float>(mm * 3 + 2), Yaaa.at<float>(kk * 3), Yaaa.at<float>(kk * 3 + 1), Yaaa.at<float>(kk * 3 + 2));
							if (tempdist<KALMAN_THRES){
								flag.at<float>(kk,0) = -1;
								break; 
							}
						}
					}
				}
			}
			for (int kk = 0; kk<num_new; ++kk) {
				if (flag.at<float>(kk, 0) == 1) {
					Xm.push_back(Yaaa.at<float>(kk * 3));
					Xm.push_back(Yaaa.at<float>(kk * 3 + 1));
					Xm.push_back(Yaaa.at<float>(kk * 3 + 2));
					cv::Mat Pm_temp = cv::Mat::zeros(num_Xm * 3 + 3, num_Xm * 3 + 3, CV_32FC1);
					for (int mm = 0; mm<num_Xm * 3; ++mm) {
						Pm_temp.at<float>(mm, mm) = Pm.at<float>(mm, mm);
					}
					for (int mm = 0; mm<3; ++mm) {
						Pm_temp.at<float>(mm + num_Xm * 3, mm + num_Xm * 3) = 1;
					}
					Pm = Pm_temp;
					numempty.push_back((float)0);
					identity.push_back((float)kk);
					num_Xm = num_Xm + 1;
					int tmp_id = get_id();
					lane_g_id.push_back((uchar)tmp_id);
				}
			}
		}
	}
	if (num_Xm == 0) {
		std::cout << "Fail!!" << std::endl;
	}

	if (num_Xm != 0) {

	//改变舱数后，detect出对应新息
	cv::Mat Y(num_Xm * 3, 1, CV_32FC1);
	float near280sum=0;
	float near400sum=0;
	float near280;
	float near400;
	int count280=0;
	int count400=0;

	for (int jj = 0; jj<num_Xm; ++jj) {
		if (identity.at<float>(jj, 0) != -1) {
			Y.at<float>(jj * 3, 0) = Yaaa.at<float>(identity.at<float>(jj, 0) * 3, 0);
			Y.at<float>(jj * 3 + 1, 0) = Yaaa.at<float>(identity.at<float>(jj, 0) * 3 + 1, 0);
			Y.at<float>(jj * 3 + 2, 0) = Yaaa.at<float>(identity.at<float>(jj, 0) * 3 + 2, 0);
			
			int marking_id = marking_ids.at<float>(identity.at<float>(jj, 0), 0);
			for (int kk = 0; kk < marking_seed[marking_id].index.size(); kk++) {
				int temp_i = marking_seed[marking_id].index[kk];
				
				if(lm[temp_i].str_p.y > 275 && lm[temp_i].str_p.y < 285 && numempty.at<float>(jj,0)<-20){ //TODO:find a more robust method
					count280++;
					near280sum+=lm[temp_i].size;
				}
			}
			for (int kk = 0; kk < marking_seed[marking_id].index.size(); kk++) {
				int temp_i = marking_seed[marking_id].index[kk];
				if(lm[temp_i].str_p.y > 395 && lm[temp_i].str_p.y < 405 && numempty.at<float>(jj,0)<-20){
					count400++;
					near400sum += lm[temp_i].size;
				}
			}
		}
		
		else {
			Y.at<float>(jj * 3, 0) = Xm.at<float>(jj * 3, 0);
			Y.at<float>(jj * 3 + 1, 0) = Xm.at<float>(jj * 3 + 1, 0);
			Y.at<float>(jj * 3 + 2, 0) = Xm.at<float>(jj * 3 + 2, 0); 
		}
	}

	if (count280 != 0 && count400 != 0 && edge_set_flag == 1){
		near280 = near280sum / count280;
		near400 = near400sum / count400;
		
		tempLW_F = (int)(near280 - (280 - 240) / (float)(400 - 280) * (near400 - near280));
		tempLW_N = (int)(near400 + (img_height - 400) / (float)(400 - 280) * (near400 - near280));

		edge_set_flag = 0;
	}

	//改变舱数后相应调整各矩阵
	cv::Mat sigma(num_Xm * 3, 1, CV_32FC1);
	for (int jj = 0; jj<num_Xm; ++jj) {
		sigma.at<float>(jj * 3, 0) = 0.0006; 
		sigma.at<float>(jj * 3 + 1, 0) = 2.4;
		sigma.at<float>(jj * 3 + 2, 0) = 500;
	}
	cv::Mat V = cv::Mat::eye(num_Xm * 3, num_Xm * 3, CV_32FC1);
	for (int jj = 0; jj<num_Xm * 3; ++jj) {
		V.at<float>(jj, jj) = sigma.at<float>(jj, 0);
	}
	R = V*V; 
	cv::Mat W = 3 * V;
	Q = W*W;
	Phi = cv::Mat::eye(num_Xm * 3, num_Xm * 3, CV_32FC1);
	H = cv::Mat::eye(num_Xm * 3, num_Xm * 3, CV_32FC1);

	//卡尔曼滤波
	cv::Mat Xw = Phi*Xm;
	cv::Mat Pw = Phi*Pm*Phi.t() + Q;
	cv::Mat K = Pw*H.t() / (H*Pw*H.t() + R);
	Pm = (cv::Mat::eye(num_Xm * 3, num_Xm * 3, CV_32FC1) - K*H)*Pw;
	Xm = Xw + K*(Y - Xw);

	//画图
	cv::Point2f dot_p;
	std::vector<float> coeff_temp;

	//合并路沿：将较近且靠外的参数存放到rubbish中，然后显示为其他颜色，但是不删除Xm
	int count_cabin2 = 0;
	cv::Mat distmat_Xm2 = cv::Mat::zeros(num_Xm, num_Xm, CV_32FC1);
	for (int jj = 0; jj < num_Xm; ++jj)
	{
		for (int kk = 0; kk < num_Xm; ++kk)
		{
			distmat_Xm2.at<float>(jj, kk) = dist_line(Xm.at<float>(jj * 3), Xm.at<float>(jj * 3 + 1), Xm.at<float>(jj * 3 + 2), Xm.at<float>(kk * 3), Xm.at<float>(kk * 3 + 1), Xm.at<float>(kk * 3 + 2));
		}
	}
	std::vector<float> rubbish2;
	for (int jj = 0; jj < num_Xm; ++jj)
	{
		for (int kk = jj + 1; kk < num_Xm; ++kk)
		{
			if ((distmat_Xm2.at<float>(jj, kk) < 3*KALMAN_THRES)) //3倍距离阈值，因为路沿与真实线较远
			{
				coeff_temp.resize(0);
				coeff_temp.push_back(Xm.at<float>(0, jj * 3 + 2));
				coeff_temp.push_back(Xm.at<float>(0, jj * 3 + 1));
				coeff_temp.push_back(Xm.at<float>(0, jj * 3)); //current lane, coeff index is equal to power, which is in reverse order of Xm

				double jj_bottom = valueAt(coeff_temp, 470);

				coeff_temp.resize(0);
				coeff_temp.push_back(Xm.at<float>(0, kk * 3 + 2));
				coeff_temp.push_back(Xm.at<float>(0, kk * 3 + 1));
				coeff_temp.push_back(Xm.at<float>(0, kk * 3)); //current lane, coeff index is equal to power, which is in reverse order of Xm

				double kk_bottom = valueAt(coeff_temp, 470);

				int to_del; 
				if(coeff_temp[1]<0){
					if(jj_bottom<kk_bottom){
						to_del=jj;
					}
					else{
						to_del=kk;
					}
				}
				else{
					if(jj_bottom<kk_bottom){
						to_del=kk;
					}
					else{
						to_del=jj;
					}
				}

				if(!(isinside(rubbish2, to_del))){
					rubbish2.push_back(to_del); 
					count_cabin2 = count_cabin2 + 1;
				}
			}
		}
	}
	//0505联合两种去伪方法
	std::sort(rubbish2.begin(), rubbish2.end());
	num_Xm = num_Xm - count_cabin2;
	for (int jj = count_cabin2 - 1; jj >= 0; --jj)
	{
		Xm = del_num(Xm, rubbish2[jj] * 3 + 2);
		Xm = del_num(Xm, rubbish2[jj] * 3 + 1);
		Xm = del_num(Xm, rubbish2[jj] * 3);
		numempty = del_num(numempty, rubbish2[jj]);
		Pm = del_row(Pm, rubbish2[jj] * 3 + 2);
		Pm = del_row(Pm, rubbish2[jj] * 3 + 1);
		Pm = del_row(Pm, rubbish2[jj] * 3);
		Pm = Pm.t(); 
		Pm = del_row(Pm, rubbish2[jj] * 3 + 2);
		Pm = del_row(Pm, rubbish2[jj] * 3 + 1);
		Pm = del_row(Pm, rubbish2[jj] * 3);
		Pm = Pm.t(); 
		check_id((int)lane_g_id.at<uchar>((int)rubbish2[jj], 0));
		lane_g_id = del_row_8u(lane_g_id, rubbish2[jj]); 
														
	}


	for (int jj = 0; jj<num_Xm; ++jj) { //each jj is a line
		coeff_temp.resize(0);
		coeff_temp.push_back(Xm.at<float>(0, jj * 3 + 2));
		coeff_temp.push_back(Xm.at<float>(0, jj * 3 + 1));
		coeff_temp.push_back(Xm.at<float>(0, jj * 3)); //current lane, coeff index is equal to power, which is in reverse order of Xm

		// std::cout << "coeff_temp=" << coeff_temp[0] << " " << coeff_temp[1] << " " << coeff_temp[2] << std::endl;
		double jj_bottom = valueAt(coeff_temp, 470);

		//----lane_data_pub---@chenqi&xiebo--08.20-----------

		v_SingleLane_Pixel.clear();
		v_SingleLane_World.clear();
		LANE_Point p_tmp;

		std::vector<cv::Point2f> v_pts;
		cv::Point2f tmp_pts;
		
		//可以选择是否按筛选画图
		if(isinside(rubbish2,jj)){
		//if(0){
			// std::cout << jj << " is not ok." << std::endl;
			//std::cout << "delete: " << jj_bottom << std::endl;
			for (int yy = img_roi_height; yy < img_height; ++yy) {
				dot_p.y = yy;
				dot_p.x = valueAt(coeff_temp, dot_p.y);

				p_tmp.x=dot_p.x;
				p_tmp.y=dot_p.y;
				// v_SingleLane_Pixel.push_back(p_tmp);

				p_tmp.x=ipm.applyHomography(dot_p).x;
				p_tmp.y=ipm.applyHomography(dot_p).y;
				// v_SingleLane_World.push_back(p_tmp);
				tmp_pts.x=p_tmp.x;
				tmp_pts.y=p_tmp.y;
				v_pts.push_back(tmp_pts);
			}
			
		}
		else if (numempty.at<float>(jj, 0) <= 0) {
			// std::cout << jj << " is ok." << std::endl;
			//std::cout << jj_bottom << std::endl;
			for (int yy = img_roi_height; yy < img_height; ++yy) {
				dot_p.y = yy;
				dot_p.x = valueAt(coeff_temp, dot_p.y);
				char tmp_str[20] = "";
				sprintf(tmp_str, " %d", lane_g_id.at<uchar>(jj, 0));

				p_tmp.x=dot_p.x;
				p_tmp.y=dot_p.y;
				// v_SingleLane_Pixel.push_back(p_tmp);

				p_tmp.x=ipm.applyHomography(dot_p).x;
				p_tmp.y=ipm.applyHomography(dot_p).y;
				tmp_pts.x=p_tmp.x;
				tmp_pts.y=p_tmp.y;
				v_pts.push_back(tmp_pts);
				cv::circle(img_kalman, dot_p, 2, cv::Scalar(0, 255, 0), 1, 8, 0);
			
			}
			//pixel画图
			if (identity.at<float>(jj, 0) != -1)
			{
				int marking_id = marking_ids.at<float>(identity.at<float>(jj, 0), 0);
				for (int kk = 0; kk < marking_seed[marking_id].index.size(); kk++)
				{
					int temp_i = marking_seed[marking_id].index[kk];
					cv::line(img_test_kalman_pixel, lm[temp_i].str_p, lm[temp_i].end_p, cv::Scalar(255, 255, 0), 2, 8, 0);
				}
			}
		}
		else {
			//预测的线根据历史宽度进行pixel画图
			for (int yy = img_roi_height; yy < img_height; ++yy) {
				dot_p.y = yy;
				dot_p.x = valueAt(coeff_temp, dot_p.y);
				p_tmp.x=dot_p.x;
				p_tmp.y=dot_p.y;
				p_tmp.x=ipm.applyHomography(dot_p).x;
				p_tmp.y=ipm.applyHomography(dot_p).y;
				tmp_pts.x=p_tmp.x;
				tmp_pts.y=p_tmp.y;
				v_pts.push_back(tmp_pts);

				double temp_width = (int)((tempLW_N - tempLW_F) * (yy - img_roi_height) / (img_size.height - img_roi_height) + tempLW_F);
				cv::Point2f start_p;
				cv::Point2f end_p;
				start_p.y = yy;
				start_p.x = valueAt(coeff_temp, dot_p.y) - temp_width / 2;

				end_p.y = yy;
				end_p.x = valueAt(coeff_temp, dot_p.y) + temp_width / 2;
				// cv::line(img_kalman, start_p, end_p, cv::Scalar(0, 0, 255), 2, 8, 0);
				cv::circle(img_kalman, dot_p, 2, cv::Scalar(0, 255, 0), 1, 8, 0);
			}
		}

		std::vector<float> coeff(3);

		poly3(v_pts, v_pts.size(), coeff);
		Polyfit_Param.a = 0;
		Polyfit_Param.b = coeff[2];
		Polyfit_Param.c = coeff[1];
		Polyfit_Param.d = coeff[0];
		Polyfit_Param.global_id =(int)lane_g_id.at<uchar>(jj, 0);
		v_PolyfitParam.v_ld_coeff.push_back(Polyfit_Param);

		v_PolyfitParam.v_WholeLane_Pixel.push_back(v_SingleLane_Pixel);
		v_PolyfitParam.v_WholeLane_World.push_back(v_SingleLane_World);
	}
	}
	else {
		flag_initial = 1;
		std::cout << "reinitialize..." << std::endl;
	}
	
	image_num = image_num + 1;
	return img_test_kalman_pixel;
	// return img_kalman ;

}

bool LaneDetection::pass_rectangle(std::vector<float> coeff, float center_x, float center_y, float window, float window_expansion){
	//TODO: find a more robust method
	window=window*window_expansion;
	std::vector<float> corner_x;
	std::vector<float> corner_y;
	std::vector<bool> bigger_x;
	corner_x.resize(4);
	corner_y.resize(4);
	bigger_x.resize(4);
	corner_x[0] = center_x - window; //left up
	corner_y[0] = center_y - window;
	corner_x[1] = center_x + window; //right up
	corner_y[1] = center_y - window;
	corner_x[2] = center_x - window; //left down
	corner_y[2] = center_y + window;
	corner_x[3] = center_x + window; //right down
	corner_y[3] = center_y + window;
	float corner_x_temp;
	for(int ii=0;ii<4;ii++){
		corner_x_temp = valueAt(coeff,corner_y[ii]);
		if(corner_x_temp>corner_x[ii]){
			bigger_x[ii] = true;
		}
		else{
			bigger_x[ii] = false;
		}
	}
	
	if(bigger_x[0]==bigger_x[1] && bigger_x[1]==bigger_x[2] && bigger_x[2]==bigger_x[3]){
		return false;
	}

	return true;
}

float LaneDetection::isinside(std::vector<float>rubbish, int kk) {
	int n = rubbish.size();
	for (int jj = 0; jj<n; ++jj) {
		if (rubbish[jj] == kk) {
			//std::cout << kk << " is inside." << std::endl;
			return 1;
		}
	}
	return 0;
}

float LaneDetection::ismember(cv::Mat identity, int kk) {
	int n = identity.rows;
	for (int jj = 0; jj<n; ++jj) {
		if (identity.at<float>(jj, 0) == kk) {
			//std::cout << kk << " is inside." << std::endl;
			return 1;
		}
	}
	return 0;
}

float LaneDetection::dist_line(float A1, float B1, float C1, float A2, float B2, float C2) {
	float dist = 0;
	int yy = 280;
	int count=0;
	for (int yy = img_roi_height - 10; yy < img_height; yy=yy+10) {
		dist = dist + fabs((yy*yy*A1 + yy*B1 + C1) - (yy*yy*A2 + yy*B2 + C2));
		count=count+1;
	}
	dist = dist / (float)count;
	return dist;
}

cv::Mat LaneDetection::del_num(cv::Mat ori, int jj) {
	int n = ori.rows; //这是对列向量进行操作，因此按行操作
	cv::Mat temp(n - 1, 1, CV_32FC1);
	int count_temp = 0;
	for (int kk = 0; kk<n; ++kk) {
		if (kk != jj) {
			temp.at<float>(count_temp, 0) = ori.at<float>(kk, 0);
			count_temp = count_temp + 1;
		}
	}
	return temp;
}

cv::Mat LaneDetection::del_row(cv::Mat ori, int jj) {
	int n = ori.rows;
	int m = ori.cols;
	cv::Mat temp;
	int count_temp = 0;
	for (int kk = 0; kk<n; ++kk) {
		if (kk != jj) {
			cv::Mat dst = ori.row(kk);
			temp.push_back(dst);
			count_temp = count_temp + 1;
		}
	}
	return temp;
}

cv::Mat LaneDetection::del_row_8u(cv::Mat ori, int jj) {
	int n = ori.rows; //这是对列向量进行操作，因此按行操作
	cv::Mat temp(n - 1, 1, CV_8U);
	int count_temp = 0;
	for (int kk = 0; kk<n; ++kk) {
		if (kk != jj) {
			temp.at<int>(count_temp, 0) = ori.at<int>(kk, 0);
			count_temp = count_temp + 1;
		}
	}
	return temp;
}
//-------@--kalman---------------------

float LaneDetection::marking_thres(float input) {

	float thres = 0;
	return input / 5 + 1;
}
int LaneDetection::dist_ftn1(int s_i, int s_j, double slope, std::vector<float> coeff, bool marking_long) {

	// For Seed Generation

	double value = 0;
	double slope_new = slope_ftn(lm[s_i].cnt_p, lm[s_j].cnt_p);
	CvPoint i, j;
	i = lm[s_i].cnt_p;
	j = lm[s_j].cnt_p;
	value = sqrt((i.x - j.x)*(i.x - j.x) + (i.y - j.y)*(i.y - j.y));
	int tolerance = max_lw[i.y] + max_lw_d[i.x];

	if (value < SEED_MARKING_DIST_THRES*(marking_long+1)) {
		if(marking_long){
			double xx = valueAt(coeff, i.y);
			if(fabs(i.x-xx)>tolerance/2.0){ //error more than half lane width, reject
				return 0;
			}
			else{
				return 1;
			}
		}
		else{ //for short lanes, polyfit may not be accurate enough
			if (slope <= -99) {
				return 1;
			}
			if ((value>50) && (fabs(slope - slope_new) > 1.1)) {
				return 0;
			}
			if (fabs(slope - slope_new) < 0.8) {
				return 1;
			}
			if ((lm[s_i].cnt_p.x <= lm[s_j].end_p.x) && (lm[s_i].cnt_p.x >= lm[s_j].str_p.x)) {
				return 1;
			}
		}
	}
	return 0;
}
float LaneDetection::dist_ftn2(int i, int j) {

	// For Low level Association
	//std::cout << marking_seed[i].str_p << marking_seed[i].end_p << std::endl;
	if (marking_seed[i].end_p.y > marking_seed[j].str_p.y) {
		return 0;
	}

	// Rough Verification
	std::vector<float> slp;
	slp.resize(7);
	slp[0] = marking_seed[i].cnt_dir;
	slp[1] = marking_seed[j].cnt_dir;
	if ((abs(slp[0] - slp[1])>0.5) && (abs(abs(slp[0] - slp[1]) - 3.141592) < 2.641592)) {
		return 0;
	}
	slp[2] = slope_ftn(marking_seed[i].cnt_p, marking_seed[j].cnt_p);
	slp[3] = slope_ftn(marking_seed[i].str_p, marking_seed[j].str_p);
	slp[4] = slope_ftn(marking_seed[i].str_p, marking_seed[j].end_p);
	slp[5] = slope_ftn(marking_seed[i].end_p, marking_seed[j].str_p);
	slp[6] = slope_ftn(marking_seed[i].end_p, marking_seed[j].end_p);

	// slope variance check
	float slp_mean = (slp[0] + slp[1] + slp[2] + slp[3] + slp[4] + slp[5] + slp[6]) / 7;
	float temp = 0;
	for (int i = 0; i < 7; i++) {
		temp += (slp[i] - slp_mean)*(slp[i] - slp_mean);
	}
	float slp_var = temp / 7;
	if (slp_var > 0.1) {
		return 0;
	}

	// distance ftn between two seeds	
	float sig = 0.25;
	float diff1, diff2;
	diff1 = slp[0] - slp[5];
	diff2 = slp[1] - slp[5];

	if (((abs(diff1) + abs(diff2)) > 0.2) && (diff1*diff2 > 0)) {
		return 0;
	}
    

	if (abs(diff1) > 1.570796) {
		diff1 = abs(diff1 - 3.141592);
	}
	if (abs(diff2) > 1.570796) {
		diff2 = abs(diff2 - 3.141592);
	}

	return (float)(exp(-(diff1)*(diff1) / sig*sig) + exp(-(diff2)*(diff2) / sig*sig));
}


int LaneDetection::dist_ftn3(int i, int j, int s_i, int s_j) {

	// Graph Validity of (i to j)

	// Location 1
	if (marking_seed[i].end_p.y >= marking_seed[j].str_p.y) {
		return 0;
	}

	// Location 2
	double diff1 = marking_seed[j].str_p.x - (tan(CV_PI / 2 - marking_seed[i].end_dir)*(marking_seed[j].str_p.y - marking_seed[i].end_p.y) + marking_seed[i].end_p.x);
	double diff2 = marking_seed[i].end_p.x - (tan(CV_PI / 2 - marking_seed[j].str_dir)*(marking_seed[i].end_p.y - marking_seed[j].str_p.y) + marking_seed[j].str_p.x);
	if (abs(diff1) + abs(diff2) > 65) {
		return 0;
	}

	// Slope
	double inter_dir = slope_ftn(marking_seed[i].end_p, marking_seed[j].str_p);
	double diff3 = (marking_seed[i].end_dir - inter_dir) / CV_PI * 180;
	double diff4 = (marking_seed[j].str_dir - inter_dir) / CV_PI * 180;


    // by @wentuopu reduce the threshold for a more strict rule.
	if (abs(diff3) + abs(diff4) > 30) {
		return 0;
	}
	
	return 1;
}

float LaneDetection::slope_ftn(cv::Point2f pos1, cv::Point2f pos2) {

	cv::Point2f temp_pos;
	if (pos1.y > pos2.y) {
		temp_pos = pos1;
		pos1 = pos2;
		pos2 = temp_pos;
	}
	return (float)(acos((double)((pos2.x - pos1.x) / sqrt((float)((pos1.x - pos2.x)*(pos1.x - pos2.x) + (pos1.y - pos2.y)*(pos1.y - pos2.y))))));
}
float LaneDetection::length_ftn(cv::Point2f str_p, cv::Point2f end_p) {

	return sqrt((float)(str_p.x - end_p.x)*(str_p.x - end_p.x) + (float)(str_p.y - end_p.y)*(str_p.y - end_p.y));

}


float LaneDetection::pairwise_ftn(std::vector<cv::Point2f>& pts) {

	cv::Point2f dots; 
	float error = 0;
    std::vector<float> coeff(2);
    poly2(pts, pts.size(), coeff);
	for(int ii=0;ii<pts.size();++ii){
		dots.y = (int)pts[ii].y;
		dots.x = (int)(coeff[0]+coeff[1]*dots.y);
		error = error + (float)((pts[ii].x-dots.x)*(pts[ii].x-dots.x));
	}
	double sig = 50;
    double ferr = error / pts.size();
	double pairwise = exp(-(ferr*ferr)/sig/sig);
	
	return (float)pairwise;

}
void LaneDetection::node_grouping(cv::Mat& mat_in, int size, int type, int n, int label) {

	if (type == 0) {
		for (int ii = 0; ii < size; ii++) {
			if (mat_in.at<int>(n, ii) == 1) {
				mat_in.at<int>(n, ii) = label;
				node_grouping(mat_in, size, 0, ii, label);
				node_grouping(mat_in, size, 1, ii, label);
			}
		}
	}

	if (type == 1) {
		for (int ii = 0; ii < size; ii++) {
			if (mat_in.at<int>(ii, n) == 1) {
				mat_in.at<int>(ii, n) = label;
				node_grouping(mat_in, size, 0, ii, label);
				node_grouping(mat_in, size, 1, ii, label);
			}
		}
	}
}


void LaneDetection::poly2(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff) {

	float norm_f = 1.f;
	float temp;
	// float err=0;
	cv::Mat a = cv::Mat(2, 2, CV_32FC1);
	cv::Mat b = cv::Mat(2, 1, CV_32FC1);
	cv::Mat c = cv::Mat(2, 2, CV_32FC1);
	cv::Mat d = cv::Mat(2, 1, CV_32FC1);
	cv::Mat e = cv::Mat(2, 1, CV_32FC1);

	for (int ii = 0; ii < n; ii++) {
		points[ii].x = points[ii].x / norm_f;
		points[ii].y = points[ii].y / norm_f;
	}

	// configuring matrix 'a'
	a.at<float>(0, 0) = (float)n;
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].y;
	}
	a.at<float>(0, 1) = (float)temp;
	a.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].y * points[ii].y;
	}
	a.at<float>(1, 1) = (float)temp;

	// configuring matrix 'b'
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].x;
	}
	b.at<float>(0, 0) = (float)temp;
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].x * points[ii].y;
	}
	b.at<float>(1, 0) = (float)temp;

	// matrix operation
	c = a.inv();
	d = c*b;
	coeff[0] = d.at<float>(0, 0)*norm_f;
	coeff[1] = d.at<float>(1, 0)*norm_f;
}
// by @wentuopu modify fitting function by add a scale and use SVD trick for a more stable solution.
void LaneDetection::poly3(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff) {
    
	float norm_f = 255.f;
	float temp;
	// float err = 0;
	cv::Mat a = cv::Mat(3, 3, CV_32FC1);
	cv::Mat b = cv::Mat(3, 1, CV_32FC1);
	cv::Mat c = cv::Mat(3, 3, CV_32FC1);
	cv::Mat d = cv::Mat(3, 1, CV_32FC1);
	cv::Mat e = cv::Mat(3, 1, CV_32FC1);

	for (int ii = 0; ii < n; ii++) {
		points[ii].x = points[ii].x / norm_f;
		points[ii].y = points[ii].y / norm_f;
	}
	// configuring matrix 'a'
	a.at<float>(0, 0) = (float)n;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y;
	}
	a.at<float>(0, 1) = (float)temp;
	a.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y;
	}
	a.at<float>(0, 2) = (float)temp;
	a.at<float>(1, 1) = (float)temp;
	a.at<float>(2, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(1, 2) = (float)temp;
	a.at<float>(2, 1) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(2, 2) = (float)temp;

	// configuring matrix 'b'
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].x;
	}
	b.at<float>(0, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].x * points[i].y;
	}
	b.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].x;
	}
	b.at<float>(2, 0) = (float)temp;
    // SVD
    cv::Mat w, u, vt;
    cv::SVD::compute(a, w, u, vt);
    cv::transpose(u, u);
    cv::transpose(vt, vt);
    cv::Mat W = cv::Mat::zeros(3,3,CV_32FC1);
    for (int i = 0; i < 3; i++) {
        W.at<float>(i,i) = 1.0 / (w.at<float>(i));
    }
   
    d = u * b;
    d = W * d;
    d = vt * d;
	
	coeff[0] = d.at<float>(0, 0)*norm_f;
	coeff[1] = d.at<float>(1, 0);
	coeff[2] = d.at<float>(2, 0) / norm_f;
}

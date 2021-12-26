#ifndef __LANEDETECTION_
#define __LANEDETECTION_


#include <iostream> 
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <fstream>
#include <math.h>
#include <ctime>
#include "IPM.h"


#define MAX_LANE_MARKING   2000

// Max lane width, delta, lane marking can be wider if it is too left or right
#define MAX_LW_D   0		 //20/2		
#define SCAN_INTERVAL1   1 //lower  
#define SCAN_INTERVAL2   1 //upper
#define KALMAN_THRES     20

#define VP_X   256
#define VP_Y   85
#define VP_X_DELTA_NUM   8  //originally VP_X_DELTA, 6*24, cabin number by x. TODO: find a more robust method


#define VP_Y_DELTA_NUM  3  //originally VP_Y_DELTA, 3*24, cabin number by y.
#define VP_WINDOW   3		 //half of the vp vote cabin size
#define WINDOW_EXPANSION 2 //expand small voting rectangle by this

// Lane Marking Grouping
#define MAX_LANE_SEED 200
#define SEED_MARKING_DIST_THRES 10
#define VALID_SEED_MARKING_NUMBER_THRES 6 // by jxy as a test, default is 6
#define LOW_LEVEL_ASS_THRES (1.95)		   // by @wentuopu change threshold for low association

#define MAX_LW_N  32	// Max lane width, nearest
#define MAX_LW_F  5		// Max lane width, farest
#define MIN_LW_N  15	// Min lane width, nearest
#define MIN_LW_F  1		// Min lane width, farest

typedef char int8 ;
typedef unsigned char uint8 ;

typedef short int16 ;
typedef unsigned short uint16 ;

using namespace std;

typedef struct LANE_Point{
	float x;
	float y;
} LANE_Point;

typedef struct LD_COEFF{ 
	float a;  
	float b;  
	float c;  
	float d;   
	int global_id;  //lane ID
	int local_id;
	int lane_style;  // 0:Solid line   1:Dash line 
	int lane_color;  // 0:White    1:Yellow
} LD_COEFF;

typedef struct LANE_POLYFIT_PARAM{
	std::vector<LD_COEFF> v_ld_coeff;
	std::vector<std::vector<LANE_Point> > v_WholeLane_Pixel;
	std::vector<std::vector<LANE_Point> > v_WholeLane_World;
} LANE_POLYFIT_PARAM;


typedef struct LANE_MARKING {
	cv::Point2f str_p;
	cv::Point2f cnt_p;
	cv::Point2f end_p;
	cv::Point2f inn_p;
	int size;
	int R;
	int G;
	int B;
	int H;
	int S;
	int V;
}LANE_MARKING;
typedef struct MARKING_SEED {
	std::vector<int> index;
	int flag;	// 1: valid results, 0: candidnates, -1: invalid supermarking
	float cnt_dir;		
	float str_dir;
	float end_dir;
	float length;
	cv::Point2f cnt_p;	  // 
	cv::Point2f str_p;    // 
	cv::Point2f end_p;	  // 
	std::vector<float> coeff;
	float dash_score;   //zero distance between marks 
}MARKING_SEED;
typedef struct NODE_CRF {
	int vert_idx1;
	int vert_idx2;
	int label;	// 
	int idx;	// group #
	double unary;
	//int edge;//??
}NODE_CRF;
typedef struct EDGE_CRF {
	std::vector<int> node_idx;	// nodes index
	int grp_idx;		// group # that it belongs to
	int label;
	float pairwise;
}EDGE_CRF;
typedef struct NODE_GRP {
	std::vector<int> idx;	// node #
}NODE_GRP;
//--------------2018.04.27-----------------

//--------------------

typedef struct LANE_COEFF {
	std::vector<float> now;
	std::vector<float> minus1;
	std::vector<float> minus2; //meaning see Matlab. coeff of lanes in current and past images.
} LANE_COEFF;


typedef struct LaneReturn {
	cv::Mat image;
	cv::Mat Xm;
}LaneReturn;

class LaneDetection {
private:
	int image_num = 1;
	bool flag_initial = 0; //by jxy kalman initial flag
	// cv::VideoWriter kalman_video;
	int edge_set_flag = 1;
	int tempLW_F;
	int tempLW_N;
	//------------------@xiebo--2018.04.27---------------------

	//int image_num = 131;
	int image_num_initial = 1;
	cv::Mat Xm;//输出车道线结果（卡尔曼估计值）
	cv::Mat R;
	cv::Mat Q;
	cv::Mat Phi;
	cv::Mat H;
	cv::Mat Pm;//各矩阵意义均见标准卡尔曼滤波
	int num_Xm; //当前车道线数量
	std::vector<int> bbb;
	cv::Mat numempty;
	cv::Mat lane_g_id;

	//cv::Mat lane_g_style;

    LANE_COEFF aaa;
	std::vector<int> aaamarking;

	cv::Mat vp_candidate_x;
	cv::Mat vp_candidate_y;
	cv::Mat vp_countlines;
	
	float VP_X_DELTA = VP_X_DELTA_NUM * VP_WINDOW * 2; //big rectangle
	float VP_Y_DELTA = VP_Y_DELTA_NUM * VP_WINDOW * 2;
	cv::Point2f last_vp;

//ipm by chenqi
 public:
	 float f_world_scale;
	 float f_image_scale;
	 float f_visual_offset;
	 IPM ipm;
	 IPM ipm_visual;
 public:

	LaneDetection();
	~LaneDetection() {
	}
	// float add_neon(float *arr, int len);

	void setcount(int count);
//	bool initialize_variable(std::string& img_name);
	bool initialize_variable(cv::Mat image);
//	bool initialize_Img(std::string& img_name);
	bool initialize_Img(cv::Mat image);
	void lane_marking_detection();
	float marking_thres(float input);
	int dist_ftn1(int i, int sj, double slope, std::vector<float> coeff, bool marking_long);
	
	void seed_generation();
	void seed_specification(MARKING_SEED& marking_seed_curr, int mode);
	float dist_ftn2(int idx1, int idx2);
	float slope_ftn(cv::Point2f i, cv::Point2f j);
	float length_ftn(cv::Point2f str_p, cv::Point2f end_p);
//--------------2018.04.27----------------	
	float dist_line(float A1, float B1, float C1, float A2, float B2, float C2);
	cv::Mat del_num(cv::Mat ori, int jj); //CV::32FC1
	cv::Mat del_row(cv::Mat ori, int jj);
	float isinside(std::vector<float>rubbish, int kk);
//---------2018.08.15-----------
	cv::Mat del_row_8u(cv::Mat ori, int jj);

//---------2018.06.13-----------
	bool pass_rectangle(std::vector<float> coeff, float center_x, float center_y, float window, float window_expansion);
//-----------------------------
	float ismember(cv::Mat identity, int kk);
	LaneReturn kalman();
//------------------------------------
	void graph_generation();
	int dist_ftn3(int i, int j, int s_i, int s_j);
	float unary_ftn(int i, int j);
	void node_grouping(cv::Mat& mat_in, int size, int type, int n, int label);
	float pairwise_ftn(std::vector<cv::Point2f>& pts);
	
	void validating_final_seeds();
	//bool judgepixel(float u, float v, cv::Mat procImg);

	void poly3(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff);
	void poly2(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff);

	//-------------------@xiebo--dynamic vanish point--------------------
	std::vector<cv::Vec4i> lines_std;
	std::vector< std::vector<int> > points;
	cv::Mat image, img, gray;

	float epsilon;
	float m, c;
	int minlength;
	std::vector<int> temp;
	double temperr;
	//--------------------------------------
	
	cv::Point vp_pt;

	//LD_COEFF
	LD_COEFF Polyfit_Param;
	LANE_POLYFIT_PARAM v_PolyfitParam;
	
//	std::vector<LD_COEFF> v_PolyfitParam;
	//Lane_Pixel 
	std::vector<LANE_Point> v_SingleLane_Pixel;
	//Lane_World
	std::vector<LANE_Point> v_SingleLane_World;

//  id generation by ki
	std::vector<int> id_arr;
	void init_id_arr();
	void show_id_arr();
	int get_id();
	void check_id(int id);
//  distance pt by ki
	float dist_pt(cv::Point2f pt_1, cv::Point2f pt_2);
//  recognize lane style
	std::vector<int> g_lane_style;
//-----------------------------------------------------------------
	
	std::vector<float> g_lane_style_data;

private:
	// Image
   	cv::Mat img_clr;
	cv::Size img_size;
	cv::Mat img_gray;
	int img_height;
	int img_width;
	int img_roi_height;
	int img_depth;

	// Lane marking variable
	std::vector<int> max_lw;
	std::vector<int> min_lw;
	std::vector<int> max_lw_d;
	std::vector<LANE_MARKING> lm;
	std::vector<MARKING_SEED> marking_seed;

	// Graphical Model
	std::vector<NODE_CRF> nodes;
	std::vector<EDGE_CRF> edges;
};


#endif
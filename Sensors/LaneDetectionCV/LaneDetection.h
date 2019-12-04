#include <iostream>
#include <armadillo>
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


using namespace arma;
// ax^3+bx^2+cx+d=0

struct LANE_Point{
	float x;
	float y;
};
//ax^3+bx^2+cx+d=0
struct LD_COEFF{ 
	float a;  
	float b;  
	float c;  
	float d;   
	int id;  //lane ID
};

struct LANE_POLYFIT_PARAM{
	std::vector<LD_COEFF> v_ld_coeff;
	std::vector<std::vector<LANE_Point> > v_WholeLane_Pixel;
	std::vector<std::vector<LANE_Point> > v_WholeLane_World;
};


struct LANE_MARKING {
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
};
struct MARKING_SEED {
	std::vector<int> index;
	int flag;	// 1: valid results, 0: candidnates, -1: invalid supermarking
	float cnt_dir;
	float str_dir;
	float end_dir;
	float length;
	cv::Point2f cnt_p;
	cv::Point2f str_p;
	cv::Point2f end_p;
	std::vector<float> coeff;
};
struct NODE_CRF {
	int vert_idx1;
	int vert_idx2;
	int label;	// 
	int idx;	// group #
	double unary;
	//int edge;//??
};
struct EDGE_CRF {
	std::vector<int> node_idx;	// nodes index
	int grp_idx;		// group # that it belongs to
	int label;
	float pairwise;
};
struct NODE_GRP {
	std::vector<int> idx;	// node #
};
//--------------2018.04.27-----------------

//--------------------

class LaneDetection {

//ipm by chenqi
 public:
	 float f_world_scale;
	 float f_image_scale;
	 float f_visual_offset;
	 IPM ipm;
 public:

	LaneDetection();
	~LaneDetection() {
	}

	void setcount(int count);

//	bool initialize_variable(std::string& img_name);
	bool initialize_variable(cv::Mat image);
//	bool initialize_Img(std::string& img_name);
	bool initialize_Img(cv::Mat image);
	void lane_marking_detection(bool verbose = false);
	float marking_thres(float input);
	int dist_ftn1(int i, int sj, double slope, std::vector<float> coeff, bool marking_long);
	
	void seed_generation(bool verbose = false);
	void seed_specification(MARKING_SEED& marking_seed_curr, int mode);
	float dist_ftn2(int idx1, int idx2);
	float slope_ftn(cv::Point2f i, cv::Point2f j);
	float length_ftn(cv::Point2f str_p, cv::Point2f end_p);
//--------------2018.04.27----------------	
	float dist_line(float A1, float B1, float C1, float A2, float B2, float C2);
	cv::Mat del_num(cv::Mat ori, int jj);
	cv::Mat del_row(cv::Mat ori, int jj);
	float isinside(std::vector<float>rubbish, int kk);
//---------2018.06.13-----------
	bool pass_rectangle(std::vector<float> coeff, float center_x, float center_y, cv::Mat img_test_val, float window, float window_expansion);
//-----------------------------
	float ismember(cv::Mat identity, int kk);
	cv::Mat kalman(bool verbose);
//------------------------------------
	void graph_generation(bool verbose);
	int dist_ftn3(int i, int j, int s_i, int s_j);
	float unary_ftn(int i, int j);
	void node_grouping(cv::Mat& mat_in, int size, int type, int n, int label);
	float pairwise_ftn(std::vector<cv::Point2f>& pts);
	
	cv::Mat validating_final_seeds(bool verbose);
	bool judgepixel(float u, float v, cv::Mat procImg);
	cv::Mat Xmoutput();

	float poly4(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff);
	float poly3(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff);
	float poly2(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff);
//-------------------@xiebo--dynamic vanish point--------------------
	std::vector<cv::Vec4i> lines_std;
	std::vector< std::vector<int> > points;
	cv::Mat image, img, gray;
	mat A, b, prevRes;
	mat Atemp, btemp, res, aug, error, soln;
	float epsilon;
	float m, c;
	int minlength;
	std::vector<int> temp;
	double temperr;
	//--------------------------------------
	void init(cv::Mat image, mat prevRes);
	void makeLines(int flag);
	void eval();
	cv::Point vp_pt;

	//LD_COEFF
	LD_COEFF Polyfit_Param;
	LANE_POLYFIT_PARAM v_PolyfitParam;
	
//	std::vector<LD_COEFF> v_PolyfitParam;
	//Lane_Pixel 
	std::vector<LANE_Point> v_SingleLane_Pixel;
	//Lane_World
	std::vector<LANE_Point> v_SingleLane_World;

	
	
//-----------------------------------------------------------------
	//void display_test1(IplImage*);
	//void display_test2(IplImage*);
	//void memory_release();

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

	// Marking detection
	//int* ELW;
	//int num_of_ELW;

	// Graphical Model
	std::vector<NODE_CRF> nodes;
	std::vector<EDGE_CRF> edges;
};

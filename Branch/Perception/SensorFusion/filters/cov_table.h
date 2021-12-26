// @brief: An estimation of radar uncertainty

// objective: representing the radar uncertainty
//Author: Chunlei YU. Contact me at: yuchunlei@mail.tsinghua.edu.cn


#ifndef OBSF_COV_TABLE_H
#define OBSF_COV_TABLE_H

#include <vector>
#include <opencv2/video/tracking.hpp>
#include <Eigen/Core>

#define M_PI 3.1415926

const int para_num = 60;    // 4 * 15
// bottom - up order. From the smallest angle/range to the largest. 
const float covs[] = 
{
                      0.01946, -0.00389, -0.00389, 0.00449, 
                      0.00259, 0.00023, 0.00023, 0.00978,
                      0.00483, 0.00035, 0.00035, 0.02849,
                      0.05252, 0.00685, 0.00685, 0.27816,
                      0.00167, 0.00103, 0.00103, 0.02131,
                      0.00008, 0.00005, 0.00005, 0.00159,
                      0.0000004, -0.000033, -0.000033, 0.00269,
                      0.03494, 0.00463, 0.00463, 0.04262,
                      1.48508, -0.47015, -0.47015, 0.48932,
                      0.00297, -0.000096, -0.000096, 0.00387,
                      0.01946, -0.00389, -0.00389, 0.00449, 
                      0.00259, 0.00023, 0.00023, 0.00978,
                      0.00483, 0.00035, 0.00035, 0.02849,
                      0.05252, 0.00685, 0.00685, 0.27816,
                      0.00167, 0.00103, 0.00103, 0.02131
};

namespace tsinghua{
namespace car{
namespace fusion{

class radar_cov 
{
public:
    radar_cov()
    {
        cov_matrix.clear();
        for(int id = 0; id < para_num; id += 4)
        {
            cv::Mat_<float> cov = *(cv::Mat_<float>(2, 2) << 
                                    covs[id], covs[id+1],
                                    covs[id+2], covs[id+3]);
            cov_matrix.push_back(cov);
        }
        
    }
    ~radar_cov(){cov_matrix.clear();}
    // return the covariance matrix according to the position of the obs_radar
    // theta in radians, range in meters
    // 
    cv::Mat & get_cov_matrix(double theta, double range)
    {
        int i = 0;
        if(theta < -10*M_PI/180.0) i = 0;
        else if(theta < 10*M_PI/180.0) i = 1;
        else i = 2;

        int j = 0;
        if(range < 20) j = 0;
        else if(range < 40) j = 1;
        else if(range < 60) j = 2;
        else if(range < 80) j = 3;
        else j = 4;

        return cov_matrix[i*5 + j];
    }
public:
    // store the covariance matrix. 
    std::vector<cv::Mat> cov_matrix;
};

}
}    
}

#endif

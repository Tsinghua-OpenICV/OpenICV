// @brief: kalman filter 
// states: x, y, vx, vy 
// measurements: x, y, vx, vy

// objective: uncertainty fusion

//Author: Chunlei YU. Contact me at: yuchunlei@mail.tsinghua.edu.cn

#ifndef OBSF_FUSION_KALMAN_FILTER_H
#define OBSF_FUSION_KALMAN_FILTER_H
#include "base_filter_wrap.h"
#include <opencv2/video/tracking.hpp>
//#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
//#include <eigen3/Eigen/Core/Matrix.h>

const int Te = 0.05;

namespace tsinghua {
namespace dias {
namespace fusion{


class FusionKalmanFilter: public BaseFilterWrap {
public:
    FusionKalmanFilter();

    ~FusionKalmanFilter();
    
    void initialize(const tsinghua::dias::fusion::OBSFObs3d& state);
    
    Eigen::VectorXf correct(const Eigen::VectorXf& measurement);
    
    Eigen::VectorXf predict(const double time_diff); 
    
    Eigen::VectorXf predict(const Eigen::VectorXf& control, double time_diff);
    
    Eigen::VectorXf update_with_object(tsinghua::dias::fusion::OBSFObs3d& new_object);

    bool update_with_object(
    tsinghua::dias::fusion::OBSFObs3d& new_object, Eigen::Vector4f & s, bool id_tracked
    );

    Eigen::Matrix4f getCovarianceMatrix(){return P;}

    /*history track kalman*/
    void initialize_hist(const tsinghua::dias::fusion::hist_track_obs& state);
    Eigen::VectorXf predict_hist(const double time_diff); 
    Eigen::VectorXf update_with_object_hist(tsinghua::dias::fusion::hist_track_obs& new_object);


private:
    Eigen::Vector4f states;
    Eigen::Vector4f states_buf;

    Eigen::Matrix4f P;
    Eigen::Matrix4f A;
    Eigen::Matrix4f C;

    Eigen::Matrix<float, 4, 2> ta;
    Eigen::Matrix4f Q;
    Eigen::Matrix4f R;

    Eigen::Matrix<float, 4, 4> K;

    //boost::shared_ptr<radar_cov> _radar_cov; 
};

}
}    
}

#endif

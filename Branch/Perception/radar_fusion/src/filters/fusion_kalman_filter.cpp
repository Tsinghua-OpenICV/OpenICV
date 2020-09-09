#include "fusion_kalman_filter.h"
#include <iostream>

namespace tsinghua {
namespace dias {
namespace fusion{

FusionKalmanFilter::FusionKalmanFilter() 
{
    _name = "ConstantVelocityKalmanFilterWrap";
    //_kalman_filter = boost::shared_ptr<cv::KalmanFilter>(new cv::KalmanFilter());  
    //_radar_cov = boost::shared_ptr<radar_cov>(new radar_cov());  
}

FusionKalmanFilter::~FusionKalmanFilter() {
    
}

void FusionKalmanFilter::initialize(const tsinghua::dias::fusion::OBSFObs3d& state)
{

    // float _obs_theta_rate = ;
    states(0) = state._obs_position._x;
    states(1) = state._obs_position._y;
    states(2) = state._obs_theta;
    states(3) = state._velocity._x;
    states(4) = state._velocity._y;
    states(5) = 1;
    // std::cout << "kk333333:\n" << state.P << std::endl;
    // std::cout << "kk111111:" << state._obs_theta<< std::endl; 
    // P.setZero();
    // P = state.P;
    P.setIdentity();
    // std::cout << "kk111111:\n" << P << std::endl; 
    states_buf  = A * states;
    P	    = ((A*P)*A.transpose())+Q;
    // std::cout << "kk23222:\n" << P << std::endl; 
    C.setIdentity();

}

Eigen::VectorXf FusionKalmanFilter::correct(const Eigen::VectorXf& measurement) 
{
    Eigen::VectorXf s;
    return s;
}

Eigen::VectorXf FusionKalmanFilter::predict(const double time_diff)
{

	float dt = time_diff;
	float dt2 = time_diff*time_diff;
    Eigen::VectorXf s;
    s.resize(4);
    A.setIdentity();
    A(0, 3) = time_diff;
    A(1, 4) = time_diff;
    A(2, 5) = 0;
    ta.setZero();
	ta(0, 0) = dt2/2+2.645;
	ta(1, 1) = dt2/2+2.645;
    ta(2, 2) = dt2/2+2.645;
	ta(3, 0) = dt+2.645;
	ta(4, 1) = dt+2.645;
    ta(5, 2) = dt+2.645;
	Q = ta*ta.transpose();
    return s;
}

Eigen::VectorXf FusionKalmanFilter::predict(
    const Eigen::VectorXf& control, 
    const double time_diff
    ) 
{
    
    Eigen::VectorXf pre;
    return pre;
}


/* */
bool FusionKalmanFilter::update_with_object(
    tsinghua::dias::fusion::OBSFObs3d& new_object, Eigen::Vector4f & s, bool id_tracked
    )
{
    // /**/
    // //Eigen::VectorXf s;
    // Eigen::Vector4f measurement;
    // measurement(0) = new_object._obs_position._x;
    // measurement(1) = new_object._obs_position._y;
    // measurement(2) = new_object._velocity._x;
    // measurement(3) = new_object._velocity._y;

    // R = new_object.P;



    // K = P * C.transpose() * (C * P * C.transpose() + R).inverse();

    // Eigen::Vector4f predict_measurement(states_buf(0), states_buf(1), states_buf(2), states_buf(3));

    // double mahalanobis_distance = (measurement - predict_measurement).transpose() * 
    //                                     (R + C * P * C.transpose()).inverse() * (measurement - predict_measurement);

    // mahalanobis_dis = sqrt(mahalanobis_distance);

    // if(!id_tracked){
    //     if(mahalanobis_dis > 3.0)
    //         return false;
    // }

 
    // //states = states + K * (measurement - predict_measurement);
    // states_buf = states_buf + K * (measurement - predict_measurement);
    // P = (Eigen::Matrix4f::Identity() - K * C) * P * (Eigen::Matrix4f::Identity() - K * C).transpose() + K * R * K.transpose();
    // //P	  = P - (K*(( H * P * C.T())+R)* K.T());
    // //Eigen::Vector4f s;
    // s = states;
    // return true;
    // //return s;
    
}

Eigen::VectorXf FusionKalmanFilter::update_with_object(
    tsinghua::dias::fusion::OBSFObs3d& new_object
    )
{
    Eigen::Matrix<float, 6, 1>  measurement;
    measurement(0) = new_object._obs_position._x;
    measurement(1) = new_object._obs_position._y;
    measurement(2) = new_object._obs_theta;
    measurement(3) = new_object._velocity._x;
    measurement(4) = new_object._velocity._y;
    measurement(5) = 1;
    R.setZero();
    R(0,0) = new_object.rangex_rms;
    R(1,1) = new_object.rangey_rms;
    R(2,2) = new_object.ori_rms;
    R(3,3) = new_object.speedx_rms;
    R(4,4) = new_object.speedy_rms;
    R(5,5) = new_object.oriRate_rms;
    // std::cout << "P:\n" << P << std::endl;
    // std::cout << "C:\n" << C << std::endl;
    // std::cout << "R:\n" << R << std::endl;
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    // std::cout << "KKKKKK:\n" << K << std::endl;
    Eigen::Matrix<float, 6, 1> predict_measurement;
    predict_measurement << states_buf(0), states_buf(1),states_buf(2),states_buf(3), states_buf(4),states_buf(5);
    // std::cout << "predict_measurement:\n" << predict_measurement << std::endl;
    double mahalanobis_distance = (measurement - predict_measurement).transpose() * (R + C * P * C.transpose()).inverse() * (measurement - predict_measurement);
    mahalanobis_dis = sqrt(mahalanobis_distance);
    states_buf = states_buf + K * (measurement - predict_measurement);
    P = (Eigen::Matrix<float, 6, 6>::Identity() - K * C) * P * (Eigen::Matrix<float, 6, 6>::Identity() - K * C).transpose() + K * R * K.transpose();
    states = states_buf;
    // std::cout << "kk4444444:\n" << P << std::endl; 
    // std::cout << "==================:\n" << states << std::endl; 
    return states;
    
}

// /*history track kalman*/
// void FusionKalmanFilter::initialize_hist(const tsinghua::dias::fusion::hist_track_obs& state)
// {
//     // float _obs_theta_rate = ;
//     states(0) = state._obs_position._x;
//     states(1) = state._obs_position._y;
//     states(2) = state._obs_theta;
//     states(3) = state._velocity._x;
//     states(4) = state._velocity._y;
//     states(5) = 0.0f;

//     P.setZero();
//     P = state.P;
//     states_buf  = A * states;
//     P	    = ((A*P)*A.transpose())+Q;
//     C.setIdentity();
// }
   
// Eigen::VectorXf FusionKalmanFilter::predict_hist(const double time_diff)
// {
// 	float dt = time_diff;
// 	float dt2 = time_diff*time_diff;
//     Eigen::VectorXf s;
//     s.resize(4);
//     A.setIdentity();
//     A(0, 3) = time_diff;
//     A(1, 4) = time_diff;
//     A(2, 5) = time_diff;
//     ta.setZero();
// 	ta(0, 0) = dt2/2+2.645;
// 	ta(1, 1) = dt2/2+2.645;
//     ta(2,2) = dt2/2+2.645;
// 	ta(3, 0) = dt+2.645;
// 	ta(4, 1) = dt+2.645;
//     ta(5, 2) = dt+2.645;
// 	Q = ta*ta.transpose();
//     return s;  
// }

// Eigen::VectorXf FusionKalmanFilter::update_with_object_hist(tsinghua::dias::fusion::hist_track_obs& new_object)
// {
//     Eigen::Matrix<float, 6, 1>  measurement;
//     measurement(0) = new_object._obs_position._x;
//     measurement(1) = new_object._obs_position._y;
//     measurement(2) = new_object._obs_theta;
//     measurement(3) = new_object._velocity._x;
//     measurement(4) = new_object._velocity._y;
//     measurement(5) = 0;
//     R.setZero();
//     R(0,0) = new_object.rangex_rms;
//     R(1,1) = new_object.rangey_rms;
//     R(2,2) = new_object.ori_rms;
//     R(3,3) = new_object.speedx_rms;
//     R(4,4) = new_object.speedy_rms;
//     R(5,5) = 0;
//     K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
//     Eigen::Matrix<float, 6, 1> predict_measurement;
//     predict_measurement << states_buf(0), states_buf(1),states_buf(2),states_buf(3), states_buf(4),states_buf(5);
//     double mahalanobis_distance = (measurement - predict_measurement).transpose() * (R + C * P * C.transpose()).inverse() * (measurement - predict_measurement);
//     mahalanobis_dis = sqrt(mahalanobis_distance);
//     states = states_buf + K * (measurement - predict_measurement);
//     P = (Eigen::Matrix<float, 6, 6>::Identity() - K * C) * P * (Eigen::Matrix<float, 6, 6>::Identity() - K * C).transpose() + K * R * K.transpose();
//     // states = states_buf;
//     return states;
// }

}//t
}//d
}//f

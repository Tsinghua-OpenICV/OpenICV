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

    // system evoluation matrix
    // A.setIdentity();
    // A(0, 2) = Te;
    // A(1, 3) = Te;
    // initialize states to the states of the detected obstacle
    states(0) = state._obs_position._x;
    states(1) = state._obs_position._y;
    states(2) = state._velocity._x;
    states(3) = state._velocity._y;

    P.setZero();
    // P(0,0) = state.rangex_rms;
    // P(1,1) = state.speedx_rms;
    // P(2,2) = state.rangey_rms;
    // P(3,3) = state.speedy_rms;
    P = state.P;

    states_buf  = A * states;
    P	    = ((A*P)*A.transpose())+Q;

    // initialize the Q, R error m_matrix
    // Q.setIdentity();
    // Q *= 0.02;

    // R from each observation
    // R = state.P;

    // #if 0
    //     P.setIdentity();
    //     P *= 100;
    // #endif
    //     P = state.P;    //maybe, this is more reasonable.

    // C is an identity matrix.
    C.setIdentity();
    // K.setOnes();
    // H.setOnes();



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
    A(0, 2) = time_diff;
    A(1, 3) = time_diff;
    
    //states  = A * states;
    ta.setZero();
	ta(0, 0) = dt2/2+2.645;
	ta(1, 1) = dt2/2+2.645;
	ta(2, 0) = dt+2.645;
	ta(3, 1) = dt+2.645;
	Q = ta*ta.transpose();

    // R from each observation
    // R = setZero();
    // Rmat(0, 0) = 0.09f;
	// Rmat(1, 1) = 0.04f;


    // states_buf  = A * states;
    // P	    = ((A*P)*A.transpose())+Q;

    // s = states;

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
    /**/
    //Eigen::VectorXf s;
    Eigen::Vector4f measurement;
    measurement(0) = new_object._obs_position._x;
    measurement(1) = new_object._obs_position._y;
    measurement(2) = new_object._velocity._x;
    measurement(3) = new_object._velocity._y;

    R = new_object.P;



    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();

    Eigen::Vector4f predict_measurement(states_buf(0), states_buf(1), states_buf(2), states_buf(3));

    double mahalanobis_distance = (measurement - predict_measurement).transpose() * 
                                        (R + C * P * C.transpose()).inverse() * (measurement - predict_measurement);

    mahalanobis_dis = sqrt(mahalanobis_distance);

    if(!id_tracked){
        if(mahalanobis_dis > 3.0)
            return false;
    }

 
    //states = states + K * (measurement - predict_measurement);
    states_buf = states_buf + K * (measurement - predict_measurement);
    P = (Eigen::Matrix4f::Identity() - K * C) * P * (Eigen::Matrix4f::Identity() - K * C).transpose() + K * R * K.transpose();
    //P	  = P - (K*(( H * P * C.T())+R)* K.T());
    //Eigen::Vector4f s;
    s = states;
    return true;
    //return s;
    
}

Eigen::VectorXf FusionKalmanFilter::update_with_object(
    tsinghua::dias::fusion::OBSFObs3d& new_object
    )
{
    /**/
    //Eigen::VectorXf s;
    Eigen::Vector4f measurement;
    measurement(0) = new_object._obs_position._x;
    measurement(1) = new_object._obs_position._y;
    measurement(2) = new_object._velocity._x;
    measurement(3) = new_object._velocity._y;

    // R = new_object.P;
    R.setZero();
    R(0,0) = new_object.rangex_rms;
    R(2,2) = new_object.speedx_rms;
    R(1,1) = new_object.rangey_rms;
    R(3,3) = new_object.speedy_rms;

    //Eigen::Matrix4f temp = C * P * C.transpose() + R;
    //K = P * C.transpose() * temp.inverse();
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    //std::cout << " K: " << K << std::endl;

    Eigen::Vector4f predict_measurement(states_buf(0), states_buf(1), states_buf(2), states_buf(3));

    double mahalanobis_distance = (measurement - predict_measurement).transpose() * (R + C * P * C.transpose()).inverse() * (measurement - predict_measurement);
    mahalanobis_dis = sqrt(mahalanobis_distance);

    // if(!id_tracked){
    //     if(mahalanobis_dis > 3.0)
    //         return states;
    // }    

    // std::cout << "states_buf:" << states_buf << std::endl << std::endl;
    // std::cout << "K: " << K << std::endl;
    // std::cout << "P: " << P << std::endl;
    // std::cout << "R: " << R << std::endl;

    states_buf = states_buf + K * (measurement - predict_measurement);
    P = (Eigen::Matrix4f::Identity() - K * C) * P * (Eigen::Matrix4f::Identity() - K * C).transpose() + K * R * K.transpose();
    states = states_buf;
    // std::cout << "states:" << states << std::endl << std::endl;
    return states;
    
}

/*history track kalman*/
void FusionKalmanFilter::initialize_hist(const tsinghua::dias::fusion::hist_track_obs& state)
{
    states(0) = state._obs_position._x;
    states(1) = state._obs_position._y;
    states(2) = state._velocity._x;
    states(3) = state._velocity._y;
    P.setZero();
    P = state.P;
    states_buf  = A * states;
    P	    = ((A*P)*A.transpose())+Q;
    C.setIdentity();
}
   
Eigen::VectorXf FusionKalmanFilter::predict_hist(const double time_diff)
{
	float dt = time_diff;
	float dt2 = time_diff*time_diff;
    Eigen::VectorXf s;
    s.resize(4);
    A.setIdentity();
    A(0, 2) = time_diff;
    A(1, 3) = time_diff;
    ta.setZero();
	ta(0, 0) = dt2/2+2.645;
	ta(1, 1) = dt2/2+2.645;
	ta(2, 0) = dt+2.645;
	ta(3, 1) = dt+2.645;
	Q = ta*ta.transpose();
    return s;   
}

Eigen::VectorXf FusionKalmanFilter::update_with_object_hist(tsinghua::dias::fusion::hist_track_obs& new_object)
{
    Eigen::Vector4f measurement;
    measurement(0) = new_object._obs_position._x;
    measurement(1) = new_object._obs_position._y;
    measurement(2) = new_object._velocity._x;
    measurement(3) = new_object._velocity._y;
    R.setZero();
    R(0,0) = new_object.rangex_rms;
    R(1,1) = new_object.speedx_rms;
    R(2,2) = new_object.rangey_rms;
    R(3,3) = new_object.speedy_rms;
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    Eigen::Vector4f predict_measurement(states_buf(0), states_buf(1), states_buf(2), states_buf(3));
    double mahalanobis_distance = (measurement - predict_measurement).transpose() * (R + C * P * C.transpose()).inverse() * (measurement - predict_measurement);
    mahalanobis_dis = sqrt(mahalanobis_distance);
    states_buf = states_buf + K * (measurement - predict_measurement);
    P = (Eigen::Matrix4f::Identity() - K * C) * P * (Eigen::Matrix4f::Identity() - K * C).transpose() + K * R * K.transpose();
    states = states_buf;
    return states;
}


}//
}//
}//

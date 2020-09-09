#ifndef OBSF_BASE_FILTER_WRAP_H
#define OBSF_BASE_FILTER_WRAP_H

#include "../objects/obsf_define.h"
#include "../objects/obsf_obs2d.h"
#include "../objects/obsf_obs3d.h"
//#include "../objects/obsf_obs3d_lidar.h"
//#include <eigen3/Eigen/Core>

#include <eigen3/Eigen/Core>

namespace tsinghua {
namespace dias {
namespace fusion{


class BaseFilterWrap{
public:
    BaseFilterWrap(){
        _name = "BaseFilterWrap";
    };
    virtual ~BaseFilterWrap(){};
    
    virtual void initialize(const tsinghua::dias::fusion::OBSFObs3d& state) = 0;
    virtual Eigen::VectorXf correct(const Eigen::VectorXf& measurement) = 0;
    virtual Eigen::VectorXf predict(double time_diff) = 0;
    virtual Eigen::VectorXf predict(const Eigen::VectorXf& control, double time_diff) = 0;
    std::string name(){
        return _name;
    }
    // virtual template <typename OBSType>;
    // virtual Eigen::VectorXf update_with_object(tsinghua::dias::fusion::OBSType& new_object) = 0;
    virtual Eigen::VectorXf update_with_object(tsinghua::dias::fusion::OBSFObs3d& new_object) = 0;
    virtual bool update_with_object(
    tsinghua::dias::fusion::OBSFObs3d& new_object, Eigen::Vector4f & s, bool id_tracked
    ) = 0;

    virtual Eigen::MatrixXf getCovarianceMatrix() = 0;
    // /*history track kalman*/
    // virtual void initialize_hist(const tsinghua::dias::fusion::hist_track_obs& state) = 0;
    // virtual Eigen::VectorXf predict_hist(const double time_diff) = 0; 
    // virtual Eigen::VectorXf update_with_object_hist(tsinghua::dias::fusion::hist_track_obs& new_object) = 0;
/*
    virtual void update_without_object(OBSFObs3d& state, 
        boost::shared_ptr< OBSFObs3d >& old_object, 
        const Eigen::VectorXf& track_predict, double time_diff) = 0;
*/
    double mahalanobis_dis;
protected:
    std::string _name;
};


}
}    
}

#endif

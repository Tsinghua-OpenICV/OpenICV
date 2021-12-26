#ifndef icvExtensionOpenCV_conversion_h
#define icvExtensionOpenCV_conversion_h

#include "OpenICV/Basis/icvPrimitiveTensorData.hxx"
#include "OpenICV/structure/icvCvMatData.hxx"


icv::data::icvPrimitiveTensorData<cv::Scalar>& operator=(icv::data::icvPrimitiveTensorData<cv::Scalar>& lhs, const cv::Mat& rhs);
cv::Mat& operator=(cv::Mat& lhs, const icv::data::icvPrimitiveTensorData<cv::Scalar>& rhs);

#endif // #define icvExtensionOpenCV_conversion_h

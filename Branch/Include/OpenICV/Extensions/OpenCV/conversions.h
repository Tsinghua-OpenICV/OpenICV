#ifndef icvExtensionOpenCV_conversion_h
#define icvExtensionOpenCV_conversion_h

#include "OpenICV/Basis/icvPrimitiveTensorData.hxx"
#include "OpenICV/Extensions/OpenCV/icvCvMatData.h"

icv::icvPrimitveTensorData<cv::Scalar>& operator=(icv::icvPrimitveTensorData<cv::Scalar>& lhs, const cv::Mat& rhs);
cv::Mat& operator=(cv::Mat& lhs, const icv::icvPrimitveTensorData<cv::Scalar>& rhs);

#endif // #define icvExtensionOpenCV_conversion_h

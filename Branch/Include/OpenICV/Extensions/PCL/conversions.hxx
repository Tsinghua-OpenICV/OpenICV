#ifndef icvExtensionPCL_conversion_h
#define icvExtensionPCL_conversion_h

#include "OpenICV/Basis/icvPrimitiveArrayData.hxx"
#include "OpenICV/Basis/icvPrimitiveTensorData.hxx"
#include "OpenICV/Extensions/PCL/icvPointCloudData.hxx"

template <typename PointT>
icv::icvPrimitiveArrayData<PointT>& operator=(icv::icvPrimitiveArrayData<PointT>& lhs, const pcl::PointCloud<PointT>& rhs);
template <typename PointT>
pcl::PointCloud<PointT>& operator=(pcl::PointCloud<PointT>& lhs, const icv::icvPrimitiveArrayData<cv::Scalar>& rhs);

#endif // #define icvExtensionPCL_conversion_h

#include <math.h>

#include <common.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <fstream>

class laserMapping {

public:
	laserMapping() :
		laserCloudCornerLast(new pcl::PointCloud<PointType>()),
		laserCloudSurfLast(new pcl::PointCloud<PointType>()),
		laserCloudCornerStack(new pcl::PointCloud<PointType>()),
		laserCloudSurfStack(new pcl::PointCloud<PointType>()),
		laserCloudCornerStack2(new pcl::PointCloud<PointType>()),
		laserCloudSurfStack2(new pcl::PointCloud<PointType>()),
		laserCloudOri(new pcl::PointCloud<PointType>()),
		coeffSel(new pcl::PointCloud<PointType>()),
		laserCloudSurround(new pcl::PointCloud<PointType>()),
		laserCloudSurround2(new pcl::PointCloud<PointType>()),
		laserCloudCornerFromMap(new pcl::PointCloud<PointType>()),
		laserCloudSurfFromMap(new pcl::PointCloud<PointType>()),
		laserCloudFullRes(new pcl::PointCloud<PointType>()),
		kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>()),
		kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>())

	{
		outfile2.open("/home/xiesc/cornerLog.txt"); outfile.open("/home/xiesc/SurfLog.txt");
		num_id = 0; num_id2 = 0; timeLaserCloudCornerLast = 0; timeLaserCloudSurfLast = 0; timeLaserCloudSurfLast = 0; timeLaserCloudFullRes = 0;
		timeLaserOdometry = 0;  newLaserCloudCornerLast = false;
		newLaserCloudSurfLast = false;
		newLaserCloudFullRes = false;
		newLaserOdometry = false;
		laserCloudCenWidth = 10;//cube中点对应的索引，可能会随着位置的移动变化
		laserCloudCenHeight = 5;
		laserCloudCenDepth = 10;
		memset(transformSum, 0, sizeof(transformSum)); memset(transformIncre, 0, sizeof(transformIncre)); memset(transformTobeMapped, 0, sizeof(transformTobeMapped)); memset(transformBefMapped, 0, sizeof(transformBefMapped)); memset(transformAftMapped, 0, sizeof(transformAftMapped));
		systemInit = true;
		for (int i = 0; i < laserCloudNum; i++) {
			laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
			laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
			laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>());
			laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
		}

		//用于直接输入groundtruth的初始化
		// groundtruth = readGroundTruth("/home/xiesc/01.txt");

		//
	}


	laserMappingBack laserMappingHandler(const laserOdometryBack& odometryValueBack)
	{


		laserMappingBack mappingBackValue;

		// if (systemInit){
		//     memset(mappingBackValue.transformAftMapped,0,sizeof(mappingBackValue.transformAftMapped));

		//         systemInit = false;
		//         return mappingBackValue;


		// };

		laserCloudCornerLastHandler(odometryValueBack.laserCloudCornerLast);
		laserCloudSurfLastHandler(odometryValueBack.laserCloudSurfLast);
		laserCloudFullResHandler(odometryValueBack.laserCloudFullRes);
		laserOdometryHandler(odometryValueBack.transformSum);


		std::vector<int> pointSearchInd;
		std::vector<float> pointSearchSqDis;

		PointType pointOri, pointSel, pointProj, coeff;

		cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
		cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

		cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matD1(3, 1, CV_32F, cv::Scalar::all(0));
		//cv::Mat matD2(1, 3, CV_32F, cv::Scalar::all(0));

		cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

		bool isDegenerate = false;
		cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

		pcl::VoxelGrid<PointType> downSizeFilterCorner;
		downSizeFilterCorner.setLeafSize(0.1, 0.1, 0.1);//

		pcl::VoxelGrid<PointType> downSizeFilterSurf;
		downSizeFilterSurf.setLeafSize(0.2, 0.2, 0.2);//

		pcl::VoxelGrid<PointType> downSizeFilterMap;
		downSizeFilterMap.setLeafSize(0.3, 0.3, 0.3);//



		int frameCount = stackFrameNum - 1;
		int mapFrameCount = mapFrameNum - 1;



		if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry) {
			newLaserCloudCornerLast = false;
			newLaserCloudSurfLast = false;
			newLaserCloudFullRes = false;
			newLaserOdometry = false;

			frameCount++;
			if (frameCount >= stackFrameNum) {
				transformAssociateToMap();

				int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
				for (int i = 0; i < laserCloudCornerLastNum; i++) {
					pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
					laserCloudCornerStack2->push_back(pointSel);
				}

				int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
				for (int i = 0; i < laserCloudSurfLastNum; i++) {
					pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
					laserCloudSurfStack2->push_back(pointSel);
				}
			}

			if (frameCount >= stackFrameNum) {
				frameCount = 0;



				PointType pointOnYAxis;
				pointOnYAxis.x = 0.0;
				pointOnYAxis.y = 10.0;
				pointOnYAxis.z = 0.0;
				pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

				int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
				int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
				int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;

				if (transformTobeMapped[3] + 25.0 < 0) centerCubeI--;
				if (transformTobeMapped[4] + 25.0 < 0) centerCubeJ--;
				if (transformTobeMapped[5] + 25.0 < 0) centerCubeK--;

				while (centerCubeI < 3) {
					for (int j = 0; j < laserCloudHeight; j++) {
						for (int k = 0; k < laserCloudDepth; k++) {
							int i = laserCloudWidth - 1;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; i >= 1; i--) {
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
							//新的位置等于要舍弃位置的点云，然后clear
						}
					}

					centerCubeI++;
					laserCloudCenWidth++;
				}

				while (centerCubeI >= laserCloudWidth - 3) {
					for (int j = 0; j < laserCloudHeight; j++) {
						for (int k = 0; k < laserCloudDepth; k++) {
							int i = 0;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; i < laserCloudWidth - 1; i++) {
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeI--;
					laserCloudCenWidth--;
				}

				while (centerCubeJ < 3) {
					for (int i = 0; i < laserCloudWidth; i++) {
						for (int k = 0; k < laserCloudDepth; k++) {
							int j = laserCloudHeight - 1;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; j >= 1; j--) {
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeJ++;
					laserCloudCenHeight++;
				}

				while (centerCubeJ >= laserCloudHeight - 3) {
					for (int i = 0; i < laserCloudWidth; i++) {
						for (int k = 0; k < laserCloudDepth; k++) {
							int j = 0;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; j < laserCloudHeight - 1; j++) {
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeJ--;
					laserCloudCenHeight--;
				}

				while (centerCubeK < 3) {
					for (int i = 0; i < laserCloudWidth; i++) {
						for (int j = 0; j < laserCloudHeight; j++) {
							int k = laserCloudDepth - 1;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; k >= 1; k--) {
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeK++;
					laserCloudCenDepth++;
				}

				while (centerCubeK >= laserCloudDepth - 3) {
					for (int i = 0; i < laserCloudWidth; i++) {
						for (int j = 0; j < laserCloudHeight; j++) {
							int k = 0;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; k < laserCloudDepth - 1; k++) {
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeK--;
					laserCloudCenDepth--;
				}


				//以上部分其实就在剪切地图，每次距离变远以后，就整个cube平移。最开始激光雷达位于坐标原点，
				// 实际上对应cube索引是第10 5 10。 当激光雷达位置距离边缘小于3个cube以后，整个大cube向
				//该方向平移一个cube（或几个，直到距离边缘大于3个为止），同理中间位置的索引也跟着变，这样
				// 下一次计算的时候不会出现问题。也就是平移过一次以后，如果位于原点的值在进行计算对应的就不是
				//10 5 10了，有可能是9 5 10。

				//centerCube 对应的是预测激光雷达在的位置对应的索引

				int laserCloudValidNum = 0;
				int laserCloudSurroundNum = 0;
				for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
					for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
						for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
							if (i >= 0 && i < laserCloudWidth &&
								j >= 0 && j < laserCloudHeight &&
								k >= 0 && k < laserCloudDepth) {

								float centerX = 50.0 * (i - laserCloudCenWidth);
								float centerY = 50.0 * (j - laserCloudCenHeight);
								float centerZ = 50.0 * (k - laserCloudCenDepth);

								bool isInLaserFOV = false;
								for (int ii = -1; ii <= 1; ii += 2) {
									for (int jj = -1; jj <= 1; jj += 2) {
										for (int kk = -1; kk <= 1; kk += 2) {
											float cornerX = centerX + 25.0 * ii;
											float cornerY = centerY + 25.0 * jj;
											float cornerZ = centerZ + 25.0 * kk;

											//这个cube对应的8个角点的坐标

											float squaredSide1 = (transformTobeMapped[3] - cornerX)
												* (transformTobeMapped[3] - cornerX)
												+ (transformTobeMapped[4] - cornerY)
												* (transformTobeMapped[4] - cornerY)
												+ (transformTobeMapped[5] - cornerZ)
												* (transformTobeMapped[5] - cornerZ);

											float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
												+ (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
												+ (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

											float check1 = 100.0 + squaredSide1 - squaredSide2
												- 10.0 * sqrt(3.0) * sqrt(squaredSide1);

											float check2 = 100.0 + squaredSide1 - squaredSide2
												+ 10.0 * sqrt(3.0) * sqrt(squaredSide1);

											if (check1 < 0 && check2 > 0) {
												isInLaserFOV = true;
											}
										}
									}
								}

								if (isInLaserFOV) {
									laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j
										+ laserCloudWidth * laserCloudHeight * k;
									laserCloudValidNum++;
								}
								laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j
									+ laserCloudWidth * laserCloudHeight * k;
								laserCloudSurroundNum++;
							}
						}
					}
				}

				laserCloudCornerFromMap->clear();
				laserCloudSurfFromMap->clear();
				for (int i = 0; i < laserCloudValidNum; i++) {
					*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
					*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
				}
				int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
				int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

				int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();
				for (int i = 0; i < laserCloudCornerStackNum2; i++) {
					pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
				}

				int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();
				for (int i = 0; i < laserCloudSurfStackNum2; i++) {
					pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
				}

				laserCloudCornerStack->clear();
				// downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
				// downSizeFilterCorner.filter(*laserCloudCornerStack);
				*laserCloudCornerStack = *laserCloudCornerStack2;

				int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

				laserCloudSurfStack->clear();
				// downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
				// downSizeFilterSurf.filter(*laserCloudSurfStack);

				*laserCloudSurfStack = *laserCloudSurfStack2;
				int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

				laserCloudCornerStack2->clear();
				laserCloudSurfStack2->clear();



				if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100) {

					planeParTobe = planeRANSAC(laserCloudSurfStack);

					kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
					kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);


					// getGroundTruth();        


					for (int iterCount = 0; iterCount < 15; iterCount++) {
						laserCloudOri->clear();
						coeffSel->clear();

						num_corner = 0;
						num_surf = 0;

						for (int i = 0; i < laserCloudCornerStackNum; i++) {

							// cerr<<"3"<<","<<i<<laserCloudCornerStackNum<<endl;




							pointOri = laserCloudCornerStack->points[i];
							pointAssociateToMap(&pointOri, &pointSel);
							kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

							if (pointSearchSqDis[4] < 1.0) {
								float cx = 0;
								float cy = 0;
								float cz = 0;
								for (int j = 0; j < 5; j++) {
									cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
									cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
									cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
								}
								cx /= 5;
								cy /= 5;
								cz /= 5;

								float a11 = 0;
								float a12 = 0;
								float a13 = 0;
								float a22 = 0;
								float a23 = 0;
								float a33 = 0;
								for (int j = 0; j < 5; j++) {
									float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
									float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
									float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

									a11 += ax * ax;
									a12 += ax * ay;
									a13 += ax * az;
									a22 += ay * ay;
									a23 += ay * az;
									a33 += az * az;
								}
								a11 /= 5;
								a12 /= 5;
								a13 /= 5;
								a22 /= 5;
								a23 /= 5;
								a33 /= 5;

								matA1.at<float>(0, 0) = a11;
								matA1.at<float>(0, 1) = a12;
								matA1.at<float>(0, 2) = a13;
								matA1.at<float>(1, 0) = a12;
								matA1.at<float>(1, 1) = a22;
								matA1.at<float>(1, 2) = a23;
								matA1.at<float>(2, 0) = a13;
								matA1.at<float>(2, 1) = a23;
								matA1.at<float>(2, 2) = a33;
								//matD1.copyTo(matD2);
								cv::eigen(matA1, matD1, matV1);
								//ICV_LOG_TRACE<<matD2.at<float>(0, 0);
								//ICV_LOG_TRACE <<matD2.at<float>(0, 1);

								if (matD1.at<float>(0, 0) >( 3 * matD1.at<float>(1, 0))) {

									float x0 = pointSel.x;
									float y0 = pointSel.y;
									float z0 = pointSel.z;
									float x1 = cx + 0.1 * matV1.at<float>(0, 0);
									float y1 = cy + 0.1 * matV1.at<float>(0, 1);
									float z1 = cz + 0.1 * matV1.at<float>(0, 2);
									float x2 = cx - 0.1 * matV1.at<float>(0, 0);
									float y2 = cy - 0.1 * matV1.at<float>(0, 1);
									float z2 = cz - 0.1 * matV1.at<float>(0, 2);

									float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
										* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
										+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
										* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

									float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

									float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

									float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

									float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
										+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

									float ld2 = a012 / l12;

									pointProj = pointSel;
									pointProj.x -= la * ld2;
									pointProj.y -= lb * ld2;
									pointProj.z -= lc * ld2;

									// float s = 1 - 0.9 * fabs(ld2)/sqrt(sqrt(pointSel.x * pointSel.x
									// + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

									float s = 1 - 0.9 * fabs(ld2);


									coeff.x = s * la;
									coeff.y = s * lb;
									coeff.z = s * lc;
									coeff.intensity = s * ld2;


									//用于测试，使得intensity等于实际的loss
									// coeff.x =  la;
									// coeff.y =  lb;
									// coeff.z =  lc;
									// coeff.intensity =  ld2;

									if (s > 0.1) {
										laserCloudOri->push_back(pointOri);
										coeffSel->push_back(coeff);
										num_corner++;
										//   outfile2<< iterCount<<","<<num_corner++<<","<<s<<endl;

									}
								}
							}
						}

						for (int i = 0; i < laserCloudSurfStackNum; i++) {

							// cerr<<"3"<<","<<i<<laserCloudCornerStackNum<<endl;




							pointOri = laserCloudSurfStack->points[i];
							pointAssociateToMap(&pointOri, &pointSel);
							kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

							if (pointSearchSqDis[4] < 0.2) {
								for (int j = 0; j < 5; j++) {
									matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
									matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
									matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
								}
								cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

								cv::Mat AB;
								AB = matA0 * matX0;
								// cerr<<AB<<endl;
								// bool flag_planer = true;
								// for (int ii =0 ;ii<5;ii++){
								//     if (fabs(AB.at<float>(ii,0)+1)>0.05){
								//         flag_planer = false;
								//         break;
								//     }
								// }
								//  cerr<<flag_planer<<endl;
								// if (!flag_planer)
								//     break;

								float pa = matX0.at<float>(0, 0);
								float pb = matX0.at<float>(1, 0);
								float pc = matX0.at<float>(2, 0);
								float pd = 1;

								float ps = sqrt(pa * pa + pb * pb + pc * pc);
								pa /= ps;
								pb /= ps;
								pc /= ps;
								pd /= ps;

								bool planeValid = true;
								for (int j = 0; j < 5; j++) {
									if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
										pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
										pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
										planeValid = false;
										break;
									}
								}

								if (planeValid) {
									float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

									pointProj = pointSel;
									pointProj.x -= pa * pd2;
									pointProj.y -= pb * pd2;
									pointProj.z -= pc * pd2;

									float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
										+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));

									// float s = 1 - 0.9 * fabs(pd2);

									coeff.x = s * pa;
									coeff.y = s * pb;
									coeff.z = s * pc;
									coeff.intensity = s * pd2;

									//用于测试，使得intensity等于实际的loss
									// coeff.x =  pa;
									// coeff.y =  pb;
									// coeff.z =  pc;
									// coeff.intensity = pd2;


									if (s > 0.1) {
										laserCloudOri->push_back(pointOri);
										coeffSel->push_back(coeff);

										num_surf++;
										//  outfile<< iterCount<<","<<num_surf++<<","<<s<<endl;

									}
								}
							}
						}

						// if (laserCloudOri->points.size()>100)
						// {


						// //add normal constrain
						//     float lmd = 0.0;


						//    //通过3个点来表示一个平面
						//    //1
						//     pointOri.x = 0;
						//     pointOri.z = 0;
						//     pointOri.y = (-planeParTobe(3)-planeParTobe(0)*pointOri.x-planeParTobe(2)*pointOri.z)/planeParTobe(1);
						//     pointAssociateToMap(&pointOri, &pointSel); 

						//     float pa = planeParBef(0);
						//     float pb = planeParBef(1);
						//     float pc = planeParBef(2);
						//     float pd = planeParBef(3);

						//     float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;



						//     coeff.x =  lmd*pa;
						//     coeff.y =  lmd*pb;
						//     coeff.z =  lmd*pc;
						//     coeff.intensity = lmd*pd2;


						//     laserCloudOri->push_back(pointOri);
						//     coeffSel->push_back(coeff);

						//     num_surf++;
						//     //2
						//     pointOri.x = 1;
						//     pointOri.z = 1;
						//     pointOri.y = (-planeParTobe(3)-planeParTobe(0)*pointOri.x-planeParTobe(2)*pointOri.z)/planeParTobe(1);
						//     pointAssociateToMap(&pointOri, &pointSel); 

						//      pa = planeParBef(0);
						//      pb = planeParBef(1);
						//      pc = planeParBef(2);
						//      pd = planeParBef(3);

						//      pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;



						//     coeff.x =  lmd*pa;
						//     coeff.y =  lmd*pb;
						//     coeff.z =  lmd*pc;
						//     coeff.intensity = lmd*pd2;


						//     laserCloudOri->push_back(pointOri);
						//     coeffSel->push_back(coeff);

						//     num_surf++;
						//     //3
						//     pointOri.x = -1;
						//     pointOri.z = 1;
						//     pointOri.y = (-planeParTobe(3)-planeParTobe(0)*pointOri.x-planeParTobe(2)*pointOri.z)/planeParTobe(1);
						//     pointAssociateToMap(&pointOri, &pointSel); 

						//      pa = planeParBef(0);
						//      pb = planeParBef(1);
						//      pc = planeParBef(2);
						//      pd = planeParBef(3);

						//      pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;



						//     coeff.x =  lmd*pa;
						//     coeff.y =  lmd*pb;
						//     coeff.z =  lmd*pc;
						//     coeff.intensity = lmd*pd2;


						//     laserCloudOri->push_back(pointOri);
						//     coeffSel->push_back(coeff);                    
						//     num_surf++;  
						//     }








						float srx = sin(transformTobeMapped[0]);
						float crx = cos(transformTobeMapped[0]);
						float sry = sin(transformTobeMapped[1]);
						float cry = cos(transformTobeMapped[1]);
						float srz = sin(transformTobeMapped[2]);
						float crz = cos(transformTobeMapped[2]);

						int laserCloudSelNum = laserCloudOri->points.size();
						if (laserCloudSelNum < 50) {
							continue;
						}

						cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
						cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
						cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
						cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
						cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
						cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
						for (int i = 0; i < laserCloudSelNum; i++) {
							pointOri = laserCloudOri->points[i];
							coeff = coeffSel->points[i];

							float arx = (crx*sry*srz*pointOri.x + crx * crz*sry*pointOri.y - srx * sry*pointOri.z) * coeff.x
								+ (-srx * srz*pointOri.x - crz * srx*pointOri.y - crx * pointOri.z) * coeff.y
								+ (crx*cry*srz*pointOri.x + crx * cry*crz*pointOri.y - cry * srx*pointOri.z) * coeff.z;

							float ary = ((cry*srx*srz - crz * sry)*pointOri.x
								+ (sry*srz + cry * crz*srx)*pointOri.y + crx * cry*pointOri.z) * coeff.x
								+ ((-cry * crz - srx * sry*srz)*pointOri.x
									+ (cry*srz - crz * srx*sry)*pointOri.y - crx * sry*pointOri.z) * coeff.z;

							float arz = ((crz*srx*sry - cry * srz)*pointOri.x + (-cry * crz - srx * sry*srz)*pointOri.y)*coeff.x
								+ (crx*crz*pointOri.x - crx * srz*pointOri.y) * coeff.y
								+ ((sry*srz + cry * crz*srx)*pointOri.x + (crz*sry - cry * srx*srz)*pointOri.y)*coeff.z;

							matA.at<float>(i, 0) = arx;
							matA.at<float>(i, 1) = ary;
							matA.at<float>(i, 2) = arz;
							matA.at<float>(i, 3) = coeff.x;
							matA.at<float>(i, 4) = coeff.y;
							matA.at<float>(i, 5) = coeff.z;
							matB.at<float>(i, 0) = -coeff.intensity;
						}
						cv::transpose(matA, matAt);
						matAtA = matAt * matA;
						matAtB = matAt * matB;
						cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

						if (iterCount == 0) {
							cv::Mat matE(6, 1, CV_32F, cv::Scalar::all(0));
							//cv::Mat matE1(1, 6, CV_32F, cv::Scalar::all(0));

							cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
							cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
							//matE.copyTo(matE1);
							cv::eigen(matAtA, matE, matV);
							matV.copyTo(matV2);

							isDegenerate = false;
							float eignThre[6] = { 100, 100, 100, 100, 100, 100 };
							for (int i = 5; i >= 0; i--) {
								if (matE.at<float>(i, 0) < eignThre[i]) {
									for (int j = 0; j < 6; j++) {
										matV2.at<float>(i, j) = 0;
									}
									isDegenerate = true;
								}
								else {
									break;
								}
							}
							matP = matV.inv() * matV2;
						}

						// if (isDegenerate) {
						//     cerr<<"error"<<endl;
						// cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
						// matX.copyTo(matX2);
						// matX = matP * matX2;
						// }

						transformTobeMapped[0] += matX.at<float>(0, 0);
						transformTobeMapped[1] += matX.at<float>(1, 0);
						transformTobeMapped[2] += matX.at<float>(2, 0);
						transformTobeMapped[3] += matX.at<float>(3, 0);
						transformTobeMapped[4] += matX.at<float>(4, 0);
						transformTobeMapped[5] += matX.at<float>(5, 0);

						float deltaR = sqrt(
							pow(rad2deg(matX.at<float>(0, 0)), 2) +
							pow(rad2deg(matX.at<float>(1, 0)), 2) +
							pow(rad2deg(matX.at<float>(2, 0)), 2));
						float deltaT = sqrt(
							pow(matX.at<float>(3, 0) * 100, 2) +
							pow(matX.at<float>(4, 0) * 100, 2) +
							pow(matX.at<float>(5, 0) * 100, 2));

						if (deltaR < 0.005 && deltaT < 0.05 && matX.at<float>(0, 0) < 0.002) {
							break;
						}
						if (iterCount>9) {
							cerr << "inter_count_max_map" << iterCount << endl;
						}
					}


					// getGroundTruth();
					transformUpdate();
				}


				// if (num_id >0){
				//     pcl::PointCloud<PointType>::Ptr errorColoredPointCloudCorner  = pointsErrorMonitor(laserCloudOri,coeffSel,true);
				//     pcl::PointCloud<PointType>::Ptr errorColoredPointCloudSurf  = pointsErrorMonitor(laserCloudOri,coeffSel,false);
				//     std::stringstream filename;
				//     std::stringstream filename2;
				//     filename << "/home/xiesc/monitor_corner/"<<num_id<<".pcd";
				//     filename2 << "/home/xiesc/monitor_surf/"<<num_id<<".pcd";    
				//     pcl::io::savePCDFileASCII (filename.str(), *errorColoredPointCloudCorner);
				//     pcl::io::savePCDFileASCII (filename2.str(), *errorColoredPointCloudSurf);

				//     pcl::PointCloud<PointType>::Ptr largeErrorCorner (new pcl::PointCloud<PointType>());
				//     pcl::PointCloud<PointType>::Ptr largeErrorSurf (new pcl::PointCloud<PointType>());
				//     pcl::PointCloud<PointType>::Ptr largeErrorCornerTarget (new pcl::PointCloud<PointType>());
				//     pcl::PointCloud<PointType>::Ptr largeErrorSurfTarget (new pcl::PointCloud<PointType>());

				//     largeErrorEvaluation(laserCloudOri,coeffSel,largeErrorCorner,largeErrorSurf,largeErrorCornerTarget,largeErrorSurfTarget);
				//     std::stringstream filename3;
				//     std::stringstream filename4;
				//     std::stringstream filename5;
				//     std::stringstream filename6;
				//     filename3 << "/home/xiesc/monitor_corner_largeerror/"<<num_id<<"ori.pcd";
				//     filename4 << "/home/xiesc/monitor_corner_largeerror/"<<num_id<<"tar.pcd";
				//     filename5 << "/home/xiesc/monitor_surf_largeerror/"<<num_id<<"ori.pcd";
				//     filename6 << "/home/xiesc/monitor_surf_largeerror/"<<num_id<<"tar.pcd";
				//     if(largeErrorCorner->points.size()>0)
				//     pcl::io::savePCDFileASCII (filename3.str(), *largeErrorCorner);
				//     if(largeErrorCornerTarget->points.size()>0)
				//     pcl::io::savePCDFileASCII (filename4.str(), *largeErrorCornerTarget);
				//     if(largeErrorSurf->points.size()>0)
				//     pcl::io::savePCDFileASCII (filename5.str(), *largeErrorSurf);
				//     if(largeErrorSurfTarget->points.size()>0)
				//     pcl::io::savePCDFileASCII (filename6.str(), *largeErrorSurfTarget);


				// }

				// if (num_id !=0){
				// pcl::PointCloud<PointType> FeaturesPointsCloudSurf;
				// pcl::PointCloud<PointType> FeaturesPointsCloudCorner;
				// pcl::PointCloud<PointType> FeaturesPointsCloudRegistrationSurf;
				// pcl::PointCloud<PointType> FeaturesPointsCloudRegistrationCorner;

				// int laserCloudSelNum = laserCloudOri->points.size();
				// cerr<<laserCloudSelNum<<endl;
				// // cerr<<"1"<<laserCloudSelNum<<endl;
				// for (int i = 0;i<laserCloudSelNum;i++)
				// {
				//         pointAssociateToMap(&laserCloudOri->points[i], &pointSel);

				//         if (i <num_corner){
				//             FeaturesPointsCloudCorner.push_back(pointSel);
				//             kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
				//             for(int j = 0 ;j<5;j++){
				//             FeaturesPointsCloudRegistrationCorner.push_back(laserCloudCornerFromMap->points[pointSearchInd[j]]);
				//             }
				//         }
				//         else{
				//             FeaturesPointsCloudSurf.push_back(pointSel);
				//             kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
				//             for(int j = 0 ;j<5;j++){   
				//             FeaturesPointsCloudRegistrationSurf.push_back(laserCloudSurfFromMap->points[pointSearchInd[j]]);
				//             }   


				//         }

				// }
				// std::stringstream filename;
				// filename << "/home/xiesc/testpcd_FeaturesPointsCloudCorner/"<<num_id<<".pcd";

				// pcl::io::savePCDFileASCII (filename.str(), FeaturesPointsCloudCorner);

				// if (FeaturesPointsCloudSurf.size()>0){
				// std::stringstream filename4;
				// filename4 << "/home/xiesc/testpcd_FeaturesPointsCloudSurf/"<<num_id<<".pcd";

				// pcl::io::savePCDFileASCII (filename4.str(), FeaturesPointsCloudSurf);}

				// std::stringstream filename2;
				// filename2 << "/home/xiesc/testpcd_FeaturesPointsCloudRegistrationCorner/"<<num_id<<".pcd";

				// pcl::io::savePCDFileASCII (filename2.str(), FeaturesPointsCloudRegistrationCorner);
				// if (FeaturesPointsCloudSurf.size()>0){
				// std::stringstream filename3;
				// filename3 << "/home/xiesc/testpcd_FeaturesPointsCloudRegistrationSurf/"<<num_id<<".pcd";

				// pcl::io::savePCDFileASCII (filename3.str(), FeaturesPointsCloudRegistrationSurf);}


				// std::stringstream filename5;
				// filename5 << "/home/xiesc/testpcd_Error/"<<num_id<<".pcd";

				// pcl::io::savePCDFileASCII (filename5.str(), *coeffSel);



				// }

				// getGroundTruth();
				// transformUpdate();


				for (int i = 0; i < laserCloudCornerStackNum; i++) {
					pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

					int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
					int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
					int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

					if (pointSel.x + 25.0 < 0) cubeI--;
					if (pointSel.y + 25.0 < 0) cubeJ--;
					if (pointSel.z + 25.0 < 0) cubeK--;

					if (cubeI >= 0 && cubeI < laserCloudWidth &&
						cubeJ >= 0 && cubeJ < laserCloudHeight &&
						cubeK >= 0 && cubeK < laserCloudDepth) {
						int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
						laserCloudCornerArray[cubeInd]->push_back(pointSel);
					}
				}
				pcl::PointCloud<PointType>::Ptr surfCloudForRANSAC(new pcl::PointCloud<PointType>());
				for (int i = 0; i < laserCloudSurfStackNum; i++) {
					pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);
					surfCloudForRANSAC->push_back(pointSel);
					int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
					int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
					int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

					if (pointSel.x + 25.0 < 0) cubeI--;
					if (pointSel.y + 25.0 < 0) cubeJ--;
					if (pointSel.z + 25.0 < 0) cubeK--;

					if (cubeI >= 0 && cubeI < laserCloudWidth &&
						cubeJ >= 0 && cubeJ < laserCloudHeight &&
						cubeK >= 0 && cubeK < laserCloudDepth) {
						int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
						laserCloudSurfArray[cubeInd]->push_back(pointSel);
					}
				}
				planeParBef = planeRANSAC(surfCloudForRANSAC);

				for (int i = 0; i < laserCloudValidNum; i++) {
					int ind = laserCloudValidInd[i];

					laserCloudCornerArray2[ind]->clear();
					downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
					downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

					laserCloudSurfArray2[ind]->clear();
					downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
					downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

					pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
					laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
					laserCloudCornerArray2[ind] = laserCloudTemp;

					laserCloudTemp = laserCloudSurfArray[ind];
					laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
					laserCloudSurfArray2[ind] = laserCloudTemp;
				}


				mapFrameCount++;
				if (mapFrameCount >= mapFrameNum) {
					mapFrameCount = 0;

					laserCloudSurround2->clear();
					for (int i = 0; i < laserCloudSurroundNum; i++) {
						int ind = laserCloudSurroundInd[i];
						*laserCloudSurround2 += *laserCloudCornerArray[ind];
						*laserCloudSurround2 += *laserCloudSurfArray[ind];
					}

					laserCloudSurround->clear();
					downSizeFilterMap.setInputCloud(laserCloudSurround2);
					downSizeFilterMap.filter(*laserCloudSurround);



					mappingBackValue.laserCloudSurround = laserCloudSurround;


					// outfile<<num_id2<<",cloud"<<std::endl;
					num_id2++;

				}

				int laserCloudFullResNum = laserCloudFullRes->points.size();
				for (int i = 0; i < laserCloudFullResNum; i++) {
					pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
				}

				mappingBackValue.laserCloudFullRes = laserCloudFullRes;

				mappingBackValue.transformAftMapped[0] = transformAftMapped[0];
				mappingBackValue.transformAftMapped[1] = transformAftMapped[1];
				mappingBackValue.transformAftMapped[2] = transformAftMapped[2];
				mappingBackValue.transformAftMapped[3] = transformAftMapped[3];
				mappingBackValue.transformAftMapped[4] = transformAftMapped[4];
				mappingBackValue.transformAftMapped[5] = transformAftMapped[5];






				// outfile<<num_id<<",odometry"<<std::endl;
				num_id++;



			}//framecount
		}//newdatain
		float errorAll = 0.0;
		for (int i = 0; i < coeffSel->points.size(); i++) {
			errorAll = errorAll + fabs(coeffSel->points[i].intensity);
		}

		outfile << errorAll / (coeffSel->size()) << std::endl;

		return mappingBackValue;
	}





private:

	//用于输入groundtruth的变量
	double **groundtruth;

	//
	//用于存储当前RANSAC的平面法向量和前一帧的平面法向量
	Eigen::VectorXf planeParBef;
	Eigen::VectorXf planeParTobe;

	//

	int num_corner;
	int num_surf;

	std::ofstream outfile;
	std::ofstream outfile2;
	int num_id;
	int num_id2;
	bool systemInit;
	static const float scanPeriod ;

	static const int stackFrameNum = 0;
	static const int mapFrameNum = 0;

	double timeLaserCloudCornerLast;
	double timeLaserCloudSurfLast;
	double timeLaserCloudFullRes;
	double timeLaserOdometry;

	bool newLaserCloudCornerLast;
	bool newLaserCloudSurfLast;
	bool newLaserCloudFullRes;
	bool newLaserOdometry;

	int laserCloudCenWidth;//cube中点对应的索引，可能会随着位置的移动变化
	int laserCloudCenHeight;
	int laserCloudCenDepth;
	static const int laserCloudWidth = 21;//cube的数量，不变的
	static const int laserCloudHeight = 11;
	static const int laserCloudDepth = 21;
	static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

	int laserCloudValidInd[125];
	int laserCloudSurroundInd[125];

	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack2;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack2;
	pcl::PointCloud<PointType>::Ptr laserCloudOri;
	pcl::PointCloud<PointType>::Ptr coeffSel;
	pcl::PointCloud<PointType>::Ptr laserCloudSurround;
	pcl::PointCloud<PointType>::Ptr laserCloudSurround2;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];

	pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

	float transformSum[6];
	float transformIncre[6];
	float transformTobeMapped[6];
	float transformBefMapped[6];
	float transformAftMapped[6];

	int imuPointerFront;
	int imuPointerLast;

	void transformAssociateToMap()
	{
		//*befmap里面记录的实际上就是里程计节点累加的结果 tobemap里面存的是正在迭代求解的对于全局的坐标变换
		// aftmap是在tobemap迭代结束以后，拿到tobemap的值（如果有Imu的话还会有个加权）
		// 所以一个新的tansformsum进来，他跟之前存下来的befmap做对比，其实就是得到了新的一帧相对于之前处理过
		// 那一帧过程中的相对运动，再把这个相对运动加到afrmap上，其实就得到的初始的相对与全局的坐标变换，作为tobemap进入迭代
		float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
			- sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
		float y1 = transformBefMapped[4] - transformSum[4];
		float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
			+ cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

		float x2 = x1;
		float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
		float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

		transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
		transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
		transformIncre[5] = z2;

		float sbcx = sin(transformSum[0]);
		float cbcx = cos(transformSum[0]);
		float sbcy = sin(transformSum[1]);
		float cbcy = cos(transformSum[1]);
		float sbcz = sin(transformSum[2]);
		float cbcz = cos(transformSum[2]);

		float sblx = sin(transformBefMapped[0]);
		float cblx = cos(transformBefMapped[0]);
		float sbly = sin(transformBefMapped[1]);
		float cbly = cos(transformBefMapped[1]);
		float sblz = sin(transformBefMapped[2]);
		float cblz = cos(transformBefMapped[2]);

		float salx = sin(transformAftMapped[0]);
		float calx = cos(transformAftMapped[0]);
		float saly = sin(transformAftMapped[1]);
		float caly = cos(transformAftMapped[1]);
		float salz = sin(transformAftMapped[2]);
		float calz = cos(transformAftMapped[2]);

		float srx = -sbcx * (salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz)
			- cbcx * sbcy*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
				- calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
			- cbcx * cbcy*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
				- calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx);
		transformTobeMapped[0] = -asin(srx);

		float srycrx = sbcx * (cblx*cblz*(caly*salz - calz * salx*saly)
			- cblx * sblz*(caly*calz + salx * saly*salz) + calx * saly*sblx)
			- cbcx * cbcy*((caly*calz + salx * saly*salz)*(cblz*sbly - cbly * sblx*sblz)
				+ (caly*salz - calz * salx*saly)*(sbly*sblz + cbly * cblz*sblx) - calx * cblx*cbly*saly)
			+ cbcx * sbcy*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
				+ (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly) + calx * cblx*saly*sbly);
		float crycrx = sbcx * (cblx*sblz*(calz*saly - caly * salx*salz)
			- cblx * cblz*(saly*salz + caly * calz*salx) + calx * caly*sblx)
			+ cbcx * cbcy*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
				+ (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz) + calx * caly*cblx*cbly)
			- cbcx * sbcy*((saly*salz + caly * calz*salx)*(cbly*sblz - cblz * sblx*sbly)
				+ (calz*saly - caly * salx*salz)*(cbly*cblz + sblx * sbly*sblz) - calx * caly*cblx*sbly);
		transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
			crycrx / cos(transformTobeMapped[0]));

		float srzcrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
			- calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
			- (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
				- calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
			+ cbcx * sbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
		float crzcrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
			- calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
			- (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
				- calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
			+ cbcx * cbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
		transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
			crzcrx / cos(transformTobeMapped[0]));

		x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
		y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
		z1 = transformIncre[5];

		x2 = x1;
		y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
		z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

		transformTobeMapped[3] = transformAftMapped[3]
			- (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
		transformTobeMapped[4] = transformAftMapped[4] - y2;
		transformTobeMapped[5] = transformAftMapped[5]
			- (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
	}

	void transformUpdate()
	{


		for (int i = 0; i < 6; i++) {
			transformBefMapped[i] = transformSum[i];
			transformAftMapped[i] = transformTobeMapped[i];
		}
	}

	void pointAssociateToMap(PointType const * const pi, PointType * const po)
	{
		float x1 = cos(transformTobeMapped[2]) * pi->x
			- sin(transformTobeMapped[2]) * pi->y;
		float y1 = sin(transformTobeMapped[2]) * pi->x
			+ cos(transformTobeMapped[2]) * pi->y;
		float z1 = pi->z;

		float x2 = x1;
		float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
		float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

		po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
			+ transformTobeMapped[3];
		po->y = y2 + transformTobeMapped[4];
		po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
			+ transformTobeMapped[5];
		po->intensity = pi->intensity;
	}

	void pointAssociateTobeMapped(PointType const * const pi, PointType * const po)
	{
		//是上面一个函数的逆过程
		float x1 = cos(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
			- sin(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
		float y1 = pi->y - transformTobeMapped[4];
		float z1 = sin(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
			+ cos(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);

		float x2 = x1;
		float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
		float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

		po->x = cos(transformTobeMapped[2]) * x2
			+ sin(transformTobeMapped[2]) * y2;
		po->y = -sin(transformTobeMapped[2]) * x2
			+ cos(transformTobeMapped[2]) * y2;
		po->z = z2;
		po->intensity = pi->intensity;
	}
	void laserCloudCornerLastHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudCornerLast2)
	{


		laserCloudCornerLast->clear();
		*laserCloudCornerLast = *laserCloudCornerLast2;

		newLaserCloudCornerLast = true;
	}

	void laserCloudSurfLastHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudSurfLast2)
	{


		laserCloudSurfLast->clear();
		*laserCloudSurfLast = *laserCloudSurfLast2;

		newLaserCloudSurfLast = true;
	}

	void laserCloudFullResHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudFullRes2)
	{


		laserCloudFullRes->clear();
		*laserCloudFullRes = *laserCloudFullRes2;
		newLaserCloudFullRes = true;
	}

	void laserOdometryHandler(const float* transformSum2)
	{

		transformSum[0] = transformSum2[0];
		transformSum[1] = transformSum2[1];
		transformSum[2] = transformSum2[2];

		transformSum[3] = transformSum2[3];
		transformSum[4] = transformSum2[4];
		transformSum[5] = transformSum2[5];

		newLaserOdometry = true;
	}



	//return the point cloud with the registration error
	//the first ${num_corner} points are the corner feature points
	//the rests are the surf feature points
	//feature type determines the returned points, if feature type is true, return corner points. if feature points is false .return surf points

	pcl::PointCloud<PointType>::Ptr pointsErrorMonitor(pcl::PointCloud<PointType>::Ptr selectedCloud, pcl::PointCloud<PointType>::Ptr errorCloud, bool featureType)
	{
		pcl::PointCloud<PointType>::Ptr errorColoredPointCloud(new pcl::PointCloud<PointType>());
		PointType tempPoint;
		// float maxError  = errorCloud->points[0].intensity;
		// for (int i = 1; i <errorCloud->points.size();i++)
		// {
		//     if (errorCloud->points[i].intensity>maxError)
		//         maxError = errorCloud->points[i].intensity;

		// }

		if (featureType)
		{
			for (int i = 0; i <num_corner; i++)
			{
				tempPoint.x = selectedCloud->points[i].x;
				tempPoint.y = selectedCloud->points[i].y;
				tempPoint.z = selectedCloud->points[i].z;
				// tempPoint.intensity = float(errorCloud->points[i].intensity)/maxError;
				tempPoint.intensity = float(errorCloud->points[i].intensity);
				errorColoredPointCloud->push_back(tempPoint);
			}
		}
		else
		{
			for (int i = num_corner; i <errorCloud->points.size(); i++)
			{
				tempPoint.x = selectedCloud->points[i].x;
				tempPoint.y = selectedCloud->points[i].y;
				tempPoint.z = selectedCloud->points[i].z;
				// tempPoint.intensity = float(errorCloud->points[i].intensity)/maxError;
				tempPoint.intensity = float(errorCloud->points[i].intensity);
				errorColoredPointCloud->push_back(tempPoint);
			}
		}

		return errorColoredPointCloud;

	}



	//return the point cloud with the large error

	void largeErrorEvaluation(pcl::PointCloud<PointType>::Ptr &selectedCloud, pcl::PointCloud<PointType>::Ptr &errorCloud,
		pcl::PointCloud<PointType>::Ptr &largeErrorCorner, pcl::PointCloud<PointType>::Ptr &largeErrorSurf,
		pcl::PointCloud<PointType>::Ptr &largeErrorCornerTarget, pcl::PointCloud<PointType>::Ptr &largeErrorSurfTarget)
	{
		for (int i = 0; i < errorCloud->points.size(); i++)
		{


			PointType  pointOri, pointSel;
			std::vector<int> pointSearchInd;
			std::vector<float> pointSearchSqDis;
			if (i<num_corner & errorCloud->points[i].intensity > 0.3)
			{
				pointOri = selectedCloud->points[i];
				pointAssociateToMap(&pointOri, &pointSel);
				kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
				pointOri.intensity = errorCloud->points[i].intensity;
				largeErrorCorner->push_back(pointOri);
				for (int j = 0; j<5; j++)
				{
					largeErrorCornerTarget->push_back(laserCloudCornerFromMap->points[pointSearchInd[j]]);

				}



			}
			else if (i >= num_corner & errorCloud->points[i].intensity > 0.3)
			{
				pointOri = selectedCloud->points[i];
				pointAssociateToMap(&pointOri, &pointSel);
				kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
				pointOri.intensity = errorCloud->points[i].intensity;
				largeErrorSurf->push_back(pointOri);
				for (int j = 0; j<5; j++)
				{
					largeErrorSurfTarget->push_back(laserCloudSurfFromMap->points[pointSearchInd[j]]);

				}

			}
		}


	}








	int CountLines(string filename)//获取文件的行数
	{
		std::ifstream ReadFile;
		int n = 0;
		std::string temp;
		ReadFile.open(filename.c_str(), ios::in);//ios::in 表示以只读的方式读取文件
		if (ReadFile.fail())//文件打开失败:返回0
		{
			ReadFile.close();
			return 0;
		}
		else//文件存在,返回文件行数
		{
			while (getline(ReadFile, temp))
			{
				n++;
			}
			ReadFile.close();
			return n;
		}

	}



	//this function is use to read the KITTI ground truth 
	//the each row of return value is cooresponding to the rigid transformation matrix from camera coordinates now to the initial camera coordinates
	double **readGroundTruth(string inputFile)
	{

		std::ifstream file;
		int lines;
		int i = 0;
		int j = 0;

		file.open(inputFile.c_str(), ios::in);

		lines = CountLines(inputFile);
		// double matrix[lines][12];
		double** matrix = (double **)malloc(lines * sizeof(double *));
		for (int i = 0; i<lines; i++)
			matrix[i] = (double *)malloc(12 * sizeof(double));
		double *matrix_ = &matrix[0][0];
		while (!file.eof()) //读取数据到数组
		{

			if (i == 12)
			{
				i = 0;
				j++;
				matrix_ = &matrix[j][0];
			}

			file >> *matrix_;

			matrix_++;
			i++;


		}
		file.close(); //关闭文件


		return matrix;

	}


	//Rotation Matrix To angle RT = Ry * Rx* Rz ,so the rotation order is z first,x second and y is the last.
	//the return value are ry,rx,rz
	double *rotationMatrixToAngle(Eigen::Matrix<double, 3, 3> Rmatrix)
	{
		double *euAngle = (double *)malloc(3 * sizeof(double));
		// std::cout<<Rmatrix(1,2)<<std::endl;
		if (Rmatrix(1, 2) == 1.0 | Rmatrix(1, 2) == -1.0)
		{
			euAngle[2] = 0.0;//set arbitrarily


			if (Rmatrix(1, 2) == 1.0)
			{
				euAngle[1] = 3.1415926 / 2;
				euAngle[0] = euAngle[2] + atan2(Rmatrix(0, 1), Rmatrix(0, 0));
			}
			else
			{
				euAngle[1] = -3.1415926 / 2;
				euAngle[0] = -euAngle[2] + atan2(-Rmatrix(0, 1), -Rmatrix(0, 0));
			}
		}
		else
		{
			euAngle[1] = -asin(Rmatrix(1, 2));
			euAngle[2] = atan2(Rmatrix(1, 0) / cos(euAngle[1]), Rmatrix(1, 1) / cos(euAngle[1]));
			euAngle[0] = atan2(Rmatrix(0, 2) / cos(euAngle[1]), Rmatrix(2, 2) / cos(euAngle[1]));
		}

		return euAngle;

	}





	//This function will make the transformsum equal to the groundthruth
	//the groundtruth is the global variable containing the groundthruth of Camera 1 motion
	// the RLC and TLC represent the Rotation and the Translation matrix from Lidar to Camera 1
	void getGroundTruth()
	{
		//用于直接输入groundtruth来观测结果

		Eigen::Matrix<double, 4, 4> RTCam;
		Eigen::Matrix<double, 3, 3> RLC;
		Eigen::Matrix<double, 3, 1> TLC;
		Eigen::Matrix<double, 4, 4> RTLC;

		RTCam << groundtruth[num_id][0], groundtruth[num_id][1], groundtruth[num_id][2], groundtruth[num_id][3],
			groundtruth[num_id][4], groundtruth[num_id][5], groundtruth[num_id][6], groundtruth[num_id][7],
			groundtruth[num_id][8], groundtruth[num_id][9], groundtruth[num_id][10], groundtruth[num_id][11],
			0, 0, 0, 1;

		RLC << 7.967514e-03, -9.999679e-01, -8.462264e-04,
			-2.771053e-03, 8.241710e-04, -9.999958e-01,
			9.999644e-01, 7.969825e-03, -2.764397e-03;

		TLC << -1.377769e-02, -5.542117e-02, -2.918589e-01;

		RTLC.topLeftCorner(3, 3) = RLC;
		RTLC.topRightCorner(3, 1) = TLC;
		RTLC.bottomLeftCorner(1, 4) << 0, 0, 0, 1;
		Eigen::Matrix<double, 4, 4> RTLL;
		RTLL << 0.0, 0.0, 1.0, 0.0,
			1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0;



		Eigen::Matrix<double, 4, 4> RTL = (RTLL.inverse())*(RTLC.inverse())*RTCam*RTLC*RTLL;
		// std::cout<<RTLC.inverse()*RTLC<<std::endl;
		// std::cout<<RTLL.inverse()*RTLL<<std::endl;

		// std::cout<<RTCam<<std::endl;
		// std::cout<<RTLC<<std::endl;
		// std::cout<<RTLL<<std::endl;                
		// std::cout<<RTCam*RTLC*RTLL<<std::endl;


		double *Euangle = rotationMatrixToAngle(RTL.topLeftCorner(3, 3));
		transformTobeMapped[0] = Euangle[1];
		transformTobeMapped[1] = Euangle[0];
		transformTobeMapped[2] = Euangle[2];
		transformTobeMapped[3] = RTL(0, 3);
		transformTobeMapped[4] = RTL(1, 3);
		transformTobeMapped[5] = RTL(2, 3);

		//groundtruth输入部分结束

		// std::cout<<RTLC<<std::endl;

	}

	Eigen::VectorXf planeRANSAC(pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*_cloud, *cloud);

		// pcl::io::savePCDFileASCII<pcl::PointXYZ> ("/home/xiesc/1.pcd", *cloud);
		std::vector<int> inliers;

		// created RandomSampleConsensus object and compute the appropriated model

		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
			model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

		Eigen::VectorXf planePara;

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold(.04);
		ransac.computeModel();
		ransac.getInliers(inliers);

		ransac.getModelCoefficients(planePara);

		ransac.getModelCoefficients(planePara);
		// std::cerr<<planePara<<std::endl;



		// copies all inliers of the model computed to another PointCloud
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

		// creates the visualization object and adds either our orignial cloud or all of the inliers
		// depending on the command line arguments specified.


		return planePara;



	}



};

 const float laserMapping::scanPeriod=0.1;

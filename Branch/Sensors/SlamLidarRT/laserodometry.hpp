


#include <cmath>

#include <common.h>

#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <fstream>
using namespace ::pcl;
class laserOdometry {
public:
	laserOdometry() : cornerPointsSharp(new  PointCloud<PointType>()), cornerPointsLessSharp(new  PointCloud<PointType>()), surfPointsFlat(new  PointCloud<PointType>()),
		surfPointsLessFlat(new  PointCloud<PointType>()), laserCloudCornerLast(new  PointCloud<PointType>()), laserCloudSurfLast(new  PointCloud<PointType>()),
		laserCloudOri(new  PointCloud<PointType>()), coeffSel(new  PointCloud<PointType>()), laserCloudFullRes(new  PointCloud<PointType>()),
		imuTrans(new  PointCloud< PointXYZ>()), kdtreeCornerLast(new  KdTreeFLANN<PointType>()), kdtreeSurfLast(new  KdTreeFLANN<PointType>())
	{
		num_id = 0; num_id2 = 0; systemInited = false; timeCornerPointsSharp = 0; timeCornerPointsLessSharp = 0; timeSurfPointsFlat = 0; timeSurfPointsLessFlat = 0; timeLaserCloudFullRes = 0;
		timeImuTrans = 0;  newCornerPointsSharp = false;   newCornerPointsLessSharp = false; newSurfPointsFlat = false; newSurfPointsLessFlat = false; newLaserCloudFullRes = false; newImuTrans = false;
		imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
		imuRollLast = 0; imuPitchLast = 0; imuYawLast = 0;
		imuShiftFromStartX = 0; imuShiftFromStartY = 0; imuShiftFromStartZ = 0;
		imuVeloFromStartX = 0; imuVeloFromStartY = 0; imuVeloFromStartZ = 0;
		memset(transform, 0, sizeof(transform)); memset(transformSum, 0, sizeof(transformSum));

	}


	laserOdometryBack laserOdometryHandler(const scanRegistrationBack& scanValueBack) {

		laserCloudSharpHandler(scanValueBack.cornerPointsSharp);
		laserCloudLessSharpHandler(scanValueBack.cornerPointsLessSharp);
		laserCloudFlatHandler(scanValueBack.surfPointsFlat);
		laserCloudLessFlatHandler(scanValueBack.surfPointsLessFlat);
		laserCloudFullResHandler(scanValueBack.laserCloud);
		imuTransHandler(scanValueBack.imuTrans);

		laserOdometryBack odometryValueBack;

		std::vector<int> pointSearchInd;
		std::vector<float> pointSearchSqDis;

		PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

		bool isDegenerate = false;
		cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

		int frameCount = skipFrameNum;



		if (newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat &&
			newSurfPointsLessFlat && newLaserCloudFullRes && newImuTrans) {
			newCornerPointsSharp = false;
			newCornerPointsLessSharp = false;
			newSurfPointsFlat = false;
			newSurfPointsLessFlat = false;
			newLaserCloudFullRes = false;
			newImuTrans = false;

			if (!systemInited) {//初始化并建立kdtree
				 PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
				cornerPointsLessSharp = laserCloudCornerLast;
				laserCloudCornerLast = laserCloudTemp;
				// ROS_INFO("init");


				laserCloudTemp = surfPointsLessFlat;
				surfPointsLessFlat = laserCloudSurfLast;
				laserCloudSurfLast = laserCloudTemp;


				kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
				kdtreeSurfLast->setInputCloud(laserCloudSurfLast);



				outfile << num_id << ",cornerlast" << std::endl;
				num_id++;

				//github源代码第二帧位移没算的原因！！！！！！！！！！
				//要加上下面这些
				laserCloudCornerLastNum = laserCloudCornerLast->points.size();
				laserCloudSurfLastNum = laserCloudSurfLast->points.size();


				odometryValueBack.laserCloudCornerLast = laserCloudCornerLast;
				odometryValueBack.laserCloudSurfLast = laserCloudSurfLast;
				odometryValueBack.laserCloudFullRes.reset(new  PointCloud<PointType>());
				memset(odometryValueBack.transformSum, 0, sizeof(odometryValueBack.transformSum));

				systemInited = true;

				return odometryValueBack;

			}

			else{
			// ROS_INFO("%d,%d",laserCloudCornerLastNum,laserCloudSurfLastNum);
			if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
				std::vector<int> indices;
				 removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
				int cornerPointsSharpNum = cornerPointsSharp->points.size();
				int surfPointsFlatNum = surfPointsFlat->points.size();
				//50
				for (int iterCount = 0; iterCount < 25; iterCount++) {
					laserCloudOri->clear();
					coeffSel->clear();

					for (int i = 0; i < cornerPointsSharpNum; i++) {
						TransformToStart(&cornerPointsSharp->points[i], &pointSel);//这个位置应该是在把点投影到每一个scan开始的时候

						if (iterCount % 5 == 0) {//每隔5次迭代，重新找一次点之间的对应关系
							std::vector<int> indices;
							 removeNaNFromPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, indices);
							kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
							int closestPointInd = -1, minPointInd2 = -1;
							if (pointSearchSqDis[0] < 5) {
								closestPointInd = pointSearchInd[0];
								int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

								float pointSqDis, minPointSqDis2 = 5;
								for (int j = closestPointInd + 1; j < laserCloudCornerLast->points.size(); j++) {
									//xiugai--jk
									if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
										break;
									}

									pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
										(laserCloudCornerLast->points[j].x - pointSel.x) +
										(laserCloudCornerLast->points[j].y - pointSel.y) *
										(laserCloudCornerLast->points[j].y - pointSel.y) +
										(laserCloudCornerLast->points[j].z - pointSel.z) *
										(laserCloudCornerLast->points[j].z - pointSel.z);

									if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {//保证所处的scanID不同，对应paper
										if (pointSqDis < minPointSqDis2) {
											minPointSqDis2 = pointSqDis;
											minPointInd2 = j;
										}
									}
								}
								for (int j = closestPointInd - 1; j >= 0; j--) {
									if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
										break;
									}

									pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
										(laserCloudCornerLast->points[j].x - pointSel.x) +
										(laserCloudCornerLast->points[j].y - pointSel.y) *
										(laserCloudCornerLast->points[j].y - pointSel.y) +
										(laserCloudCornerLast->points[j].z - pointSel.z) *
										(laserCloudCornerLast->points[j].z - pointSel.z);

									if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
										if (pointSqDis < minPointSqDis2) {
											minPointSqDis2 = pointSqDis;
											minPointInd2 = j;
										}
									}
								}
							}

							pointSearchCornerInd1[i] = closestPointInd;
							pointSearchCornerInd2[i] = minPointInd2;
						}

						if (pointSearchCornerInd2[i] >= 0) {
							tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
							tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

							float x0 = pointSel.x;
							float y0 = pointSel.y;
							float z0 = pointSel.z;
							float x1 = tripod1.x;
							float y1 = tripod1.y;
							float z1 = tripod1.z;
							float x2 = tripod2.x;
							float y2 = tripod2.y;
							float z2 = tripod2.z;
							//a012=|(pointSel - tripod) corss (pointSel - tripod2)|
							float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
								* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
								+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
								* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
								+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
								* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
							//l12=distence of tripod1 and tripod2
							float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
							//la,lb,lc是距离对转移后的点位置求偏导
							float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
								+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

							float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
								- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

							float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
								+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
							//点到直线的距离是ld2
							float ld2 = a012 / l12;

							pointProj = pointSel;//这个变量没用到
							pointProj.x -= la * ld2;
							pointProj.y -= lb * ld2;
							pointProj.z -= lc * ld2;

							float s = 1;
							if (iterCount >= 5) {
								// s = 1 - 1.8 * fabs(ld2)/ sqrt(sqrt(pointSel.x * pointSel.x
								// + pointSel.y * pointSel.y + pointSel.z * pointSel.z));//就是个权重，Loss越小权重越大

								s = 1 - 1.8 * fabs(ld2);
							}

							coeff.x = s * la;
							coeff.y = s * lb;
							coeff.z = s * lc;
							coeff.intensity = s * ld2;

							if (s > 0.1 && ld2 != 0) {
								laserCloudOri->push_back(cornerPointsSharp->points[i]);
								coeffSel->push_back(coeff);
							}
						}
					}

					for (int i = 0; i < surfPointsFlatNum; i++) {
						TransformToStart(&surfPointsFlat->points[i], &pointSel);

						if (iterCount % 5 == 0) {
							kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
							int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
							if (pointSearchSqDis[0] < 5) {
								closestPointInd = pointSearchInd[0];
								int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

								float pointSqDis, minPointSqDis2 = 5, minPointSqDis3 = 5;
								for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
									if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
										break;
									}

									pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
										(laserCloudSurfLast->points[j].x - pointSel.x) +
										(laserCloudSurfLast->points[j].y - pointSel.y) *
										(laserCloudSurfLast->points[j].y - pointSel.y) +
										(laserCloudSurfLast->points[j].z - pointSel.z) *
										(laserCloudSurfLast->points[j].z - pointSel.z);

									if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
										if (pointSqDis < minPointSqDis2) {
											minPointSqDis2 = pointSqDis;
											minPointInd2 = j;
										}
									}
									else {
										if (pointSqDis < minPointSqDis3) {
											minPointSqDis3 = pointSqDis;
											minPointInd3 = j;
										}
									}
								}
								for (int j = closestPointInd - 1; j >= 0; j--) {
									if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
										break;
									}

									pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
										(laserCloudSurfLast->points[j].x - pointSel.x) +
										(laserCloudSurfLast->points[j].y - pointSel.y) *
										(laserCloudSurfLast->points[j].y - pointSel.y) +
										(laserCloudSurfLast->points[j].z - pointSel.z) *
										(laserCloudSurfLast->points[j].z - pointSel.z);

									if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
										if (pointSqDis < minPointSqDis2) {
											minPointSqDis2 = pointSqDis;
											minPointInd2 = j;
										}
									}
									else {
										if (pointSqDis < minPointSqDis3) {
											minPointSqDis3 = pointSqDis;
											minPointInd3 = j;
										}
									}
								}
							}

							pointSearchSurfInd1[i] = closestPointInd;
							pointSearchSurfInd2[i] = minPointInd2;
							pointSearchSurfInd3[i] = minPointInd3;
						}

						if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
							tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
							tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
							tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];
							//pa pb pc实际上就对应着平面法向量的三个分量
							float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
								- (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
							float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
								- (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
							float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
								- (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
							float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

							float ps = sqrt(pa * pa + pb * pb + pc * pc);
							pa /= ps;
							pb /= ps;
							pc /= ps;
							pd /= ps;

							float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

							pointProj = pointSel;
							pointProj.x -= pa * pd2;
							pointProj.y -= pb * pd2;
							pointProj.z -= pc * pd2;

							float s = 1;
							if (iterCount >= 5) {
								s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
									+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));

								// s = 1 - 1.8 * fabs(pd2) ;
							}

							coeff.x = s * pa;
							coeff.y = s * pb;
							coeff.z = s * pc;//分别是三个偏导数
							coeff.intensity = s * pd2;

							if (s > 0.1 && pd2 != 0) {
								laserCloudOri->push_back(surfPointsFlat->points[i]);
								coeffSel->push_back(coeff);
							}
						}
					}

					int pointSelNum = laserCloudOri->points.size();
					if (pointSelNum < 10) {
						continue;
					}

					cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
					cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
					cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
					cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
					cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
					cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
					for (int i = 0; i < pointSelNum; i++) {
						pointOri = laserCloudOri->points[i];
						coeff = coeffSel->points[i];

						float s = 1;

						float srx = sin(s * transform[0]);
						float crx = cos(s * transform[0]);
						float sry = sin(s * transform[1]);
						float cry = cos(s * transform[1]);
						float srz = sin(s * transform[2]);
						float crz = cos(s * transform[2]);
						float tx = s * transform[3];
						float ty = s * transform[4];
						float tz = s * transform[5];
						//loss对RT矩阵中的参数求偏导
						float arx = (-s * crx*sry*srz*pointOri.x + s * crx*crz*sry*pointOri.y + s * srx*sry*pointOri.z
							+ s * tx*crx*sry*srz - s * ty*crx*crz*sry - s * tz*srx*sry) * coeff.x
							+ (s*srx*srz*pointOri.x - s * crz*srx*pointOri.y + s * crx*pointOri.z
								+ s * ty*crz*srx - s * tz*crx - s * tx*srx*srz) * coeff.y
							+ (s*crx*cry*srz*pointOri.x - s * crx*cry*crz*pointOri.y - s * cry*srx*pointOri.z
								+ s * tz*cry*srx + s * ty*crx*cry*crz - s * tx*crx*cry*srz) * coeff.z;

						float ary = ((-s * crz*sry - s * cry*srx*srz)*pointOri.x
							+ (s*cry*crz*srx - s * sry*srz)*pointOri.y - s * crx*cry*pointOri.z
							+ tx * (s*crz*sry + s * cry*srx*srz) + ty * (s*sry*srz - s * cry*crz*srx)
							+ s * tz*crx*cry) * coeff.x
							+ ((s*cry*crz - s * srx*sry*srz)*pointOri.x
								+ (s*cry*srz + s * crz*srx*sry)*pointOri.y - s * crx*sry*pointOri.z
								+ s * tz*crx*sry - ty * (s*cry*srz + s * crz*srx*sry)
								- tx * (s*cry*crz - s * srx*sry*srz)) * coeff.z;

						float arz = ((-s * cry*srz - s * crz*srx*sry)*pointOri.x + (s*cry*crz - s * srx*sry*srz)*pointOri.y
							+ tx * (s*cry*srz + s * crz*srx*sry) - ty * (s*cry*crz - s * srx*sry*srz)) * coeff.x
							+ (-s * crx*crz*pointOri.x - s * crx*srz*pointOri.y
								+ s * ty*crx*srz + s * tx*crx*crz) * coeff.y
							+ ((s*cry*crz*srx - s * sry*srz)*pointOri.x + (s*crz*sry + s * cry*srx*srz)*pointOri.y
								+ tx * (s*sry*srz - s * cry*crz*srx) - ty * (s*crz*sry + s * cry*srx*srz)) * coeff.z;

						float atx = -s * (cry*crz - srx * sry*srz) * coeff.x + s * crx*srz * coeff.y
							- s * (crz*sry + cry * srx*srz) * coeff.z;

						float aty = -s * (cry*srz + crz * srx*sry) * coeff.x - s * crx*crz * coeff.y
							- s * (sry*srz - cry * crz*srx) * coeff.z;

						float atz = s * crx*sry * coeff.x - s * srx * coeff.y - s * crx*cry * coeff.z;

						float d2 = coeff.intensity;

						matA.at<float>(i, 0) = arx;
						matA.at<float>(i, 1) = ary;
						matA.at<float>(i, 2) = arz;
						matA.at<float>(i, 3) = atx;
						matA.at<float>(i, 4) = aty;
						matA.at<float>(i, 5) = atz;
						matB.at<float>(i, 0) = -0.05 * d2;
					}
					cv::transpose(matA, matAt);
					matAtA = matAt * matA;
					matAtB = matAt * matB;
					cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

					if (iterCount == 0) {
						cv::Mat matE(6, 1, CV_32F, cv::Scalar::all(0));
						//cv::Mat matE2(1, 6, CV_32F, cv::Scalar::all(0));

						cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
						cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
						//matE.copyTo(matE2);
						cv::eigen(matAtA, matE, matV);
						matV.copyTo(matV2);

						isDegenerate = false;
						float eignThre[6] = { 10, 10, 10, 10, 10, 10 };
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

					if (isDegenerate) {
						cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
						matX.copyTo(matX2);
						matX = matP * matX2;
					}

					// if (iterCount>20)
					// transform[0] += matX.at<float>(0, 0);
					// else
					// transform[0] = 0;

					transform[0] += matX.at<float>(0, 0);
					transform[1] += matX.at<float>(1, 0);
					transform[2] += matX.at<float>(2, 0);
					transform[3] += matX.at<float>(3, 0);
					transform[4] += matX.at<float>(4, 0);
					transform[5] += matX.at<float>(5, 0);

					// ROS_INFO("%f",transform[3]);

					for (int i = 0; i<6; i++) {
						if (isnan(transform[i]))
							transform[i] = 0;
					}
					float deltaR = sqrt(
						pow(rad2deg(matX.at<float>(0, 0)), 2) +
						pow(rad2deg(matX.at<float>(1, 0)), 2) +
						pow(rad2deg(matX.at<float>(2, 0)), 2));
					float deltaT = sqrt(
						pow(matX.at<float>(3, 0) * 100, 2) +
						pow(matX.at<float>(4, 0) * 100, 2) +
						pow(matX.at<float>(5, 0) * 100, 2));

					if (deltaR < 0.05 && deltaT < 0.05) {//迭代计算以后最有解变化过小终止迭代
						break;
					}

					if (iterCount>24) {
						cerr << "inter_count_max_odo" << iterCount << endl;
					}


				}
			}

			// ROS_INFO("%f",transform[3]);

			float rx, ry, rz, tx, ty, tz;
			transform[0] = transform[0] * 0;
			AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
				-transform[0], -transform[1] * 1, -transform[2], rx, ry, rz);

			float x1 = cos(rz) * (transform[3])
				- sin(rz) * (transform[4]);
			float y1 = sin(rz) * (transform[3])
				+ cos(rz) * (transform[4]);
			float z1 = transform[5];

			float x2 = x1;
			float y2 = cos(rx) * y1 - sin(rx) * z1;
			float z2 = sin(rx) * y1 + cos(rx) * z1;

			tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
			ty = transformSum[4] - y2;
			tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);



			transformSum[0] = rx;
			transformSum[1] = ry;
			transformSum[2] = rz;
			transformSum[3] = tx;
			transformSum[4] = ty;
			transformSum[5] = tz;




			odometryValueBack.transformSum[0] = rx;
			odometryValueBack.transformSum[1] = ry;
			odometryValueBack.transformSum[2] = rz;
			odometryValueBack.transformSum[3] = tx;
			odometryValueBack.transformSum[4] = ty;
			odometryValueBack.transformSum[5] = tz;



			outfile << num_id2 << ",odometry" << std::endl << tx << " " << ty << " " << tz << std::endl;

			num_id2++;

			int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
			for (int i = 0; i < cornerPointsLessSharpNum; i++) {
				TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
			}

			int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
			for (int i = 0; i < surfPointsLessFlatNum; i++) {
				TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
			}

			frameCount++;
			if (frameCount >= skipFrameNum + 1) {
				int laserCloudFullResNum = laserCloudFullRes->points.size();
				for (int i = 0; i < laserCloudFullResNum; i++) {
					TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
				}
			}

			 PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
			cornerPointsLessSharp = laserCloudCornerLast;
			laserCloudCornerLast = laserCloudTemp;

			laserCloudTemp = surfPointsLessFlat;
			surfPointsLessFlat = laserCloudSurfLast;
			laserCloudSurfLast = laserCloudTemp;

			laserCloudCornerLastNum = laserCloudCornerLast->points.size();
			laserCloudSurfLastNum = laserCloudSurfLast->points.size();
			if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
				kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
				kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
			}







			odometryValueBack.laserCloudCornerLast = laserCloudCornerLast;
			odometryValueBack.laserCloudFullRes = laserCloudFullRes;
			odometryValueBack.laserCloudSurfLast = laserCloudSurfLast;



			outfile << num_id << ",cornerlast" << std::endl;
			num_id++;


		}



		return odometryValueBack;
		}
	}

private:
	std::ofstream outfile;
	int num_id;
	int num_id2;


	static const float scanPeriod;

	static const int skipFrameNum = 0;
	bool systemInited;

	double timeCornerPointsSharp;
	double timeCornerPointsLessSharp;
	double timeSurfPointsFlat;
	double timeSurfPointsLessFlat;
	double timeLaserCloudFullRes;
	double timeImuTrans;

	bool newCornerPointsSharp;
	bool newCornerPointsLessSharp;
	bool newSurfPointsFlat;
	bool newSurfPointsLessFlat;
	bool newLaserCloudFullRes;
	bool newImuTrans;

	 PointCloud<PointType>::Ptr cornerPointsSharp;
	 PointCloud<PointType>::Ptr cornerPointsLessSharp;
	 PointCloud<PointType>::Ptr surfPointsFlat;
	 PointCloud<PointType>::Ptr surfPointsLessFlat;
	 PointCloud<PointType>::Ptr laserCloudCornerLast;
	 PointCloud<PointType>::Ptr laserCloudSurfLast;
	 PointCloud<PointType>::Ptr laserCloudOri;
	 PointCloud<PointType>::Ptr coeffSel;
	 PointCloud<PointType>::Ptr laserCloudFullRes;
	 PointCloud< PointXYZ>::Ptr imuTrans;
	 KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
	 KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

	int laserCloudCornerLastNum;
	int laserCloudSurfLastNum;

	int pointSelCornerInd[160000];
	float pointSearchCornerInd1[160000];
	float pointSearchCornerInd2[160000];
	//角点最近的2个点的索引矩阵
	int pointSelSurfInd[160000];
	float pointSearchSurfInd1[160000];
	float pointSearchSurfInd2[160000];
	float pointSearchSurfInd3[160000];
	//平面点最近的3个点的索引矩阵
	float transform[6];
	float transformSum[6];

	float imuRollStart, imuPitchStart, imuYawStart;
	float imuRollLast, imuPitchLast, imuYawLast;
	float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
	float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;


	void TransformToStart(PointType const * const pi, PointType * const po)
	{
		// float s = 10 * (pi->intensity - int(pi->intensity));//？？这里的10其实是跟0.1对应的，而不是单纯的为了求十分位的数
		// float rx = s * transform[0];
		// float ry = s * transform[1];
		// float rz = s * transform[2];
		// float tx = s * transform[3];
		// float ty = s * transform[4];
		// float tz = s * transform[5];

		float rx = transform[0];
		float ry = transform[1];
		float rz = transform[2];
		float tx = transform[3];
		float ty = transform[4];
		float tz = transform[5];

		float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
		float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
		float z1 = (pi->z - tz);

		float x2 = x1;
		float y2 = cos(rx) * y1 + sin(rx) * z1;
		float z2 = -sin(rx) * y1 + cos(rx) * z1;

		po->x = cos(ry) * x2 - sin(ry) * z2;
		po->y = y2;
		po->z = sin(ry) * x2 + cos(ry) * z2;
		po->intensity = pi->intensity;
	}

	void TransformToEnd(PointType const * const pi, PointType * const po)
	{
		po->x = pi->x;
		po->y = pi->y;
		po->z = pi->z;
		po->intensity = int(pi->intensity);

		// float s = 10 * (pi->intensity - int(pi->intensity));
		// float s = 1;

		// float rx = s * transform[0];
		// float ry = s * transform[1];
		// float rz = s * transform[2];
		// float tx = s * transform[3];
		// float ty = s * transform[4];
		// float tz = s * transform[5];

		// float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
		// float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
		// float z1 = (pi->z - tz);

		// float x2 = x1;
		// float y2 = cos(rx) * y1 + sin(rx) * z1;
		// float z2 = -sin(rx) * y1 + cos(rx) * z1;

		// float x3 = cos(ry) * x2 - sin(ry) * z2;
		// float y3 = y2;
		// float z3 = sin(ry) * x2 + cos(ry) * z2;

		// rx = transform[0];
		// ry = transform[1];
		// rz = transform[2];
		// tx = transform[3];
		// ty = transform[4];
		// tz = transform[5];

		// float x4 = cos(ry) * x3 + sin(ry) * z3;
		// float y4 = y3;
		// float z4 = -sin(ry) * x3 + cos(ry) * z3;

		// float x5 = x4;
		// float y5 = cos(rx) * y4 - sin(rx) * z4;
		// float z5 = sin(rx) * y4 + cos(rx) * z4;

		// float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
		// float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
		// float z6 = z5 + tz;

		// float x7 = cos(imuRollStart) * (x6 - imuShiftFromStartX) 
		//         - sin(imuRollStart) * (y6 - imuShiftFromStartY);
		// float y7 = sin(imuRollStart) * (x6 - imuShiftFromStartX) 
		//         + cos(imuRollStart) * (y6 - imuShiftFromStartY);
		// float z7 = z6 - imuShiftFromStartZ;

		// float x8 = x7;
		// float y8 = cos(imuPitchStart) * y7 - sin(imuPitchStart) * z7;
		// float z8 = sin(imuPitchStart) * y7 + cos(imuPitchStart) * z7;

		// float x9 = cos(imuYawStart) * x8 + sin(imuYawStart) * z8;
		// float y9 = y8;
		// float z9 = -sin(imuYawStart) * x8 + cos(imuYawStart) * z8;

		// float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
		// float y10 = y9;
		// float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

		// float x11 = x10;
		// float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
		// float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

		// po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
		// po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
		// po->z = z11;
		// po->intensity = int(pi->intensity);
	}


	void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
		float &ox, float &oy, float &oz)
	{
		float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
		ox = -asin(srx);

		float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz)
			+ sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
		float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy)
			- cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
		oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

		float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz)
			+ sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
		float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz)
			- cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
		oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
	}


	void laserCloudSharpHandler(const  PointCloud<PointType>& cornerPointsSharp2)
	{


		cornerPointsSharp->clear();
		*cornerPointsSharp = cornerPointsSharp2;

		std::vector<int> indices;
		 removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
		newCornerPointsSharp = true;
	}

	void laserCloudLessSharpHandler(const  PointCloud<PointType>& cornerPointsLessSharp2)
	{

		cornerPointsLessSharp->clear();
		*cornerPointsLessSharp = cornerPointsLessSharp2;

		std::vector<int> indices;
		 removeNaNFromPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, indices);
		newCornerPointsLessSharp = true;
	}

	void laserCloudFlatHandler(const  PointCloud<PointType>& surfPointsFlat2)
	{


		surfPointsFlat->clear();

		*surfPointsFlat = surfPointsFlat2;

		std::vector<int> indices;
		 removeNaNFromPointCloud(*surfPointsFlat, *surfPointsFlat, indices);
		newSurfPointsFlat = true;
	}

	void laserCloudLessFlatHandler(const  PointCloud<PointType>& surfPointsLessFlat2)
	{


		surfPointsLessFlat->clear();

		*surfPointsLessFlat = surfPointsLessFlat2;

		std::vector<int> indices;
		 removeNaNFromPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, indices);
		newSurfPointsLessFlat = true;
	}

	void laserCloudFullResHandler(const  PointCloud<PointType>::Ptr& laserCloudFullRes2)
	{


		laserCloudFullRes->clear();

		*laserCloudFullRes = *laserCloudFullRes2;
		std::vector<int> indices;
		 removeNaNFromPointCloud(*laserCloudFullRes, *laserCloudFullRes, indices);
		newLaserCloudFullRes = true;
	}

	void imuTransHandler(const  PointCloud< PointXYZ>& imuTrans2)
	{


		imuTrans->clear();
		*imuTrans = imuTrans2;

		imuPitchStart = imuTrans->points[0].x;
		imuYawStart = imuTrans->points[0].y;
		imuRollStart = imuTrans->points[0].z;

		imuPitchLast = imuTrans->points[1].x;
		imuYawLast = imuTrans->points[1].y;
		imuRollLast = imuTrans->points[1].z;

		imuShiftFromStartX = imuTrans->points[2].x;
		imuShiftFromStartY = imuTrans->points[2].y;
		imuShiftFromStartZ = imuTrans->points[2].z;

		imuVeloFromStartX = imuTrans->points[3].x;
		imuVeloFromStartY = imuTrans->points[3].y;
		imuVeloFromStartZ = imuTrans->points[3].z;

		newImuTrans = true;
	}





};
 const float laserOdometry::scanPeriod = 0.1;











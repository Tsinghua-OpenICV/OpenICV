

#include <common.h>


using std::sin;
using std::cos;
using std::atan2;
using std::cout;
using namespace std;

class scanRegistration {

public:
	scanRegistration() {
		systemInitCount = 0; systemInited = false; ang_df1 = 0.3387; ang_df2 = 0.5161;
		ang_1_max = 2 + 0.5*ang_df1; ang_1_min = -8.5 - 0.5*ang_df1; ang_2_max = 8 + 0.5*ang_df2; ang_2_min = -8 - 0.5*ang_df2;
		num_id = 0; imuPointerFront = 0; imuPointerLast = -1;

		imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
		imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;

		imuVeloXStart = 0; imuVeloYStart = 0; imuVeloZStart = 0;
		imuShiftXStart = 0; imuShiftYStart = 0; imuShiftZStart = 0;

		imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
		imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

		imuShiftFromStartXCur = 0; imuShiftFromStartYCur = 0; imuShiftFromStartZCur = 0;
		imuVeloFromStartXCur = 0; imuVeloFromStartYCur = 0; imuVeloFromStartZCur = 0;


		t[0] = 2.85*0.0254;
		t[1] = 5.5*0.0254;
		t[2] = 2.85*0.0254;
		t[3] = 3.5*0.0254;

	}

	void test_print() {
		cerr << 1.2 << endl;


	}

	scanRegistrationBack laserCloudHandler(const pcl::PointCloud<pcl::PointXYZI>& laserCloudIn2)
	{

		// cerr <<1.1<<endl;  


		std::vector<int> scanStartInd(N_SCANS, 0);
		std::vector<int> scanEndInd(N_SCANS, 0);

		pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
		laserCloudIn.clear();
		laserCloudIn += laserCloudIn2;
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
		int cloudSize = laserCloudIn.points.size();
		float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
		float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
			laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

		if (endOri - startOri > 3 * M_PI) {
			endOri -= 2 * M_PI;
		}
		else if (endOri - startOri < M_PI) {
			endOri += 2 * M_PI;
		}
		bool halfPassed = false;
		int count = cloudSize;
		PointType point;
		std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_SCANS);
		// ROS_INFO("count=%d",count);

		// ROS_INFO("2");
		int aaaa = 0;
		int aa[64] = { 0 };
		for (int i = 0; i < cloudSize; i++) {
			point.x = laserCloudIn.points[i].y;
			point.y = laserCloudIn.points[i].z;
			point.z = laserCloudIn.points[i].x;
			//point.intensity = laserCloudIn.points[i].intensity;

			int scanID;

			tmp_ang1 = atan((point.y - t[1]) / (sqrt(point.x * point.x + point.z * point.z) - t[0])) * 180 / M_PI;
			tmp_ang2 = atan((point.y - t[3]) / (sqrt(point.x * point.x + point.z * point.z) - t[2])) * 180 / M_PI + 16.9;

			if (tmp_ang1<ang_1_max && tmp_ang1>ang_1_min) {
				scanID = int((ang_1_max - tmp_ang1) / ang_df1) + 1;
				//  if (scanID - (ang_1_max-tmp_ang1)/ang_df1 >0.9 || scanID - (ang_1_max-tmp_ang1)/ang_df1 <0.1){
				//      count--;
				//      continue;
				//  }
			}

			else if (tmp_ang2<ang_2_max && tmp_ang2>ang_2_min) {
				scanID = int((ang_2_max - tmp_ang2) / ang_df2) + 32 + 1;
				// if (scanID - 32 - (ang_2_max-tmp_ang2)/ang_df2 >0.9 || scanID - 32 - (ang_2_max-tmp_ang2)/ang_df2 <0.1){
				//    count--;
				//   continue;
				// }
			}

			else {
				count--;
				continue;
			}
			if (scanID >64 || scanID<1) {
				count--;
				continue;

			}

			scanID = scanID - 1; //matlab to C++









			float ori = -atan2(point.x, point.z);
			if (!halfPassed) {
				if (ori < startOri - M_PI / 2) {
					ori += 2 * M_PI;
				}
				else if (ori > startOri + M_PI * 3 / 2) {
					ori -= 2 * M_PI;
				}

				if (ori - startOri > M_PI) {
					halfPassed = true;
				}
			}
			else {
				ori += 2 * M_PI;

				if (ori < endOri - M_PI * 3 / 2) {
					ori += 2 * M_PI;
				}
				else if (ori > endOri + M_PI / 2) {
					ori -= 2 * M_PI;
				}
			}

			// float relTime = (ori - startOri) / (endOri - startOri);
			float relTime = 1;
			point.intensity = scanID + scanPeriod * relTime;

			float ra = 1.0 / fabs(point.y)*10000.0;
			if (ra < 100000 && ra > 4000)
			{
				float uncurve = ra - sqrt(pow(ra, 2) - pow(point.x, 2) - pow(point.z, 2));
				if (point.y < 0)
					point.y += uncurve;
				else
					point.y -= uncurve;
			}

			laserCloudScans[scanID].push_back(point);
		}
		cloudSize = count;



		pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
		for (int i = 0; i < N_SCANS; i++) {
			*laserCloud += laserCloudScans[i];//所有被分层的点（不能被分层的点已经被去掉）
		}


		// std::stringstream filename;
		// filename << "/home/xiesc/testpcd_layer/"<<num_id<<".pcd";

		//  pcl::io::savePCDFileASCII (filename.str(), *laserCloud);




		// ROS_INFO("4.1");
		int scanCount = -1;
		for (int i = 5; i < cloudSize - 5; i++) {
			float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
				+ laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
				+ laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
				+ laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
				+ laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
				+ laserCloud->points[i + 5].x;
			float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
				+ laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
				+ laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
				+ laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
				+ laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
				+ laserCloud->points[i + 5].y;
			float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
				+ laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
				+ laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
				+ laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
				+ laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
				+ laserCloud->points[i + 5].z;
			cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
			cloudSortInd[i] = i;
			cloudNeighborPicked[i] = 0;
			cloudNeighborPickedForCorner[i] = 0;
			cloudLabel[i] = 0;

			if (int(laserCloud->points[i].intensity) != scanCount) {
				scanCount = int(laserCloud->points[i].intensity);

				if (scanCount > 0 && scanCount < N_SCANS) {
					scanStartInd[scanCount] = i + 5;
					scanEndInd[scanCount - 1] = i - 5;//每一线前五个点和后五个点不要
				}
			}
		}
		// ROS_INFO("4.2");
		scanStartInd[0] = 5;
		scanEndInd.back() = cloudSize - 5;

		// ROS_INFO("4");

		for (int i = 5; i < cloudSize - 6; i++) {
			float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
			float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
			float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
			float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

			if (diff > 0.1) {

				float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
					laserCloud->points[i].y * laserCloud->points[i].y +
					laserCloud->points[i].z * laserCloud->points[i].z);

				float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
					laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
					laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

				if (depth1 > depth2) {
					diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
					diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
					diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

					if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
						cloudNeighborPicked[i - 5] = 1;
						cloudNeighborPicked[i - 4] = 1;
						cloudNeighborPicked[i - 3] = 1;
						cloudNeighborPicked[i - 2] = 1;
						cloudNeighborPicked[i - 1] = 1;
						cloudNeighborPicked[i] = 1;
					}
				}
				else {
					diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
					diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
					diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

					if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
						cloudNeighborPicked[i + 1] = 1;
						cloudNeighborPicked[i + 2] = 1;
						cloudNeighborPicked[i + 3] = 1;
						cloudNeighborPicked[i + 4] = 1;
						cloudNeighborPicked[i + 5] = 1;
						cloudNeighborPicked[i + 6] = 1;
					}
				}
			}
			//xie:去除论文里提到的线特征的边缘点，如果一个点是互相遮挡间断处的点，那么他周围的至少6个点不能要，因为计算曲率的时候用的5个点。
			float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
			float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
			float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
			float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

			float dis = laserCloud->points[i].x * laserCloud->points[i].x
				+ laserCloud->points[i].y * laserCloud->points[i].y
				+ laserCloud->points[i].z * laserCloud->points[i].z;

			if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
				cloudNeighborPicked[i] = 1;
			}
			//xie:如果距离两边都很远就去掉这个点

			//用于Kitti数据集去掉扰动点

			if (laserCloud->points[i].x>-0.5 && laserCloud->points[i].x <0.5) {
				cloudNeighborPickedForCorner[i] = 1;
			}
			float temp_range;
			temp_range = sqrt(pow(laserCloud->points[i].x, 2) + pow(laserCloud->points[i].y, 2) + pow(laserCloud->points[i].z, 2));
			if (laserCloud->points[i].y>-2 && laserCloud->points[i].y <-1.3 && temp_range<8) {
				cloudNeighborPickedForCorner[i] = 1;
			}

			// if (laserCloud->points[i].x>-4 && laserCloud->points[i].x <4 && laserCloud->points[i].y<-2){
			// cloudNeighborPicked[i] = 1;
			// }
			// if (laserCloud->points[i].y<-11){
			// cloudNeighborPicked[i] = 1;
			// }    



		}


		pcl::PointCloud<PointType> cornerPointsSharp;
		pcl::PointCloud<PointType> cornerPointsLessSharp;
		pcl::PointCloud<PointType> surfPointsFlat;
		pcl::PointCloud<PointType> surfPointsLessFlat;

		for (int i = 0; i < N_SCANS; i++) {
			pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
			for (int j = 0; j < 6; j++) {
				int sp = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / 6;
				int ep = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / 6 - 1;

				for (int k = sp + 1; k <= ep; k++) {
					for (int l = k; l >= sp + 1; l--) {
						if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
							int temp = cloudSortInd[l - 1];
							cloudSortInd[l - 1] = cloudSortInd[l];
							cloudSortInd[l] = temp;
						}
					}
				}
				//cloudSortInd是对曲率排序得到的序列：这里作者将每一线划分为等间距的6段分别处理，在每一段升序排列。
				int largestPickedNum = 0;
				for (int k = ep; k >= sp; k--) {
					int ind = cloudSortInd[k];
					if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1 &&cloudNeighborPickedForCorner[ind] == 0) {


						largestPickedNum++;
						if (largestPickedNum <= 4) {
							cloudLabel[ind] = 2;
							cornerPointsSharp.push_back(laserCloud->points[ind]);
							cornerPointsLessSharp.push_back(laserCloud->points[ind]);
						}
						else if (largestPickedNum <= 40) {
							cloudLabel[ind] = 1;
							cornerPointsLessSharp.push_back(laserCloud->points[ind]);
						}
						else {
							break;
						}
						//下面两个循环让附近的5个点不能被选，如果离得很远的话就可以（break）
						cloudNeighborPicked[ind] = 1;
						for (int l = 1; l <= 5; l++) {
							float diffX = laserCloud->points[ind + l].x
								- laserCloud->points[ind + l - 1].x;
							float diffY = laserCloud->points[ind + l].y
								- laserCloud->points[ind + l - 1].y;
							float diffZ = laserCloud->points[ind + l].z
								- laserCloud->points[ind + l - 1].z;
							if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
								break;
							}

							cloudNeighborPicked[ind + l] = 1;
						}
						for (int l = -1; l >= -5; l--) {
							float diffX = laserCloud->points[ind + l].x
								- laserCloud->points[ind + l + 1].x;
							float diffY = laserCloud->points[ind + l].y
								- laserCloud->points[ind + l + 1].y;
							float diffZ = laserCloud->points[ind + l].z
								- laserCloud->points[ind + l + 1].z;
							if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
								break;
							}

							cloudNeighborPicked[ind + l] = 1;
						}
					}
				}

				int smallestPickedNum = 0;
				for (int k = sp; k <= ep; k++) {
					int ind = cloudSortInd[k];
					if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1) {


						cloudLabel[ind] = -1;
						surfPointsFlat.push_back(laserCloud->points[ind]);

						smallestPickedNum++;
						if (smallestPickedNum >= 8) {
							break;
						}

						// if (smallestPickedNum <= 8) {
						//     cloudLabel[ind] = -1;
						//     surfPointsFlat.push_back(laserCloud->points[ind]);
						//     surfPointsLessFlatScan->push_back(laserCloud->points[ind]);
						// } else if (smallestPickedNum <= 80) {
						//     cloudLabel[ind] = -1;
						//     surfPointsLessFlatScan->push_back(laserCloud->points[ind]);
						// } else {
						//     break;
						// }

						cloudNeighborPicked[ind] = 1;
						for (int l = 1; l <= 5; l++) {
							float diffX = laserCloud->points[ind + l].x
								- laserCloud->points[ind + l - 1].x;
							float diffY = laserCloud->points[ind + l].y
								- laserCloud->points[ind + l - 1].y;
							float diffZ = laserCloud->points[ind + l].z
								- laserCloud->points[ind + l - 1].z;
							if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
								break;
							}

							cloudNeighborPicked[ind + l] = 1;
						}
						for (int l = -1; l >= -5; l--) {
							float diffX = laserCloud->points[ind + l].x
								- laserCloud->points[ind + l + 1].x;
							float diffY = laserCloud->points[ind + l].y
								- laserCloud->points[ind + l + 1].y;
							float diffZ = laserCloud->points[ind + l].z
								- laserCloud->points[ind + l + 1].z;
							if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
								break;
							}

							cloudNeighborPicked[ind + l] = 1;
						}
					}
				}

				for (int k = sp; k <= ep; k++) {

					int ind = cloudSortInd[k];

					//    if( cloudNeighborPicked[ind] == 0 &&
					//      cloudCurvature[ind] < 0.05){
					//         surfPointsLessFlatScan->push_back(laserCloud->points[ind]);    
					//     }


					if (cloudLabel[k] <= 0) {
						surfPointsLessFlatScan->push_back(laserCloud->points[k]);
					}
					k++;
					k++;
					k++;
					k++;

				}
			}

			pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
			pcl::VoxelGrid<PointType> downSizeFilter;
			downSizeFilter.setInputCloud(surfPointsLessFlatScan);
			downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
			downSizeFilter.filter(surfPointsLessFlatScanDS);

			surfPointsLessFlat += surfPointsLessFlatScanDS;
			// surfPointsLessFlat += *surfPointsLessFlatScan;
		}






		scanRegistrationBack scanBackValue;

		scanBackValue.imuTrans.points.resize(4);

		scanBackValue.imuTrans.points[0].x = imuPitchStart;
		scanBackValue.imuTrans.points[0].y = imuYawStart;
		scanBackValue.imuTrans.points[0].z = imuRollStart;

		scanBackValue.imuTrans.points[1].x = imuPitchCur;
		scanBackValue.imuTrans.points[1].y = imuYawCur;
		scanBackValue.imuTrans.points[1].z = imuRollCur;

		scanBackValue.imuTrans.points[2].x = imuShiftFromStartXCur;
		scanBackValue.imuTrans.points[2].y = imuShiftFromStartYCur;
		scanBackValue.imuTrans.points[2].z = imuShiftFromStartZCur;

		scanBackValue.imuTrans.points[3].x = imuVeloFromStartXCur;
		scanBackValue.imuTrans.points[3].y = imuVeloFromStartYCur;
		scanBackValue.imuTrans.points[3].z = imuVeloFromStartZCur;


		scanBackValue.cornerPointsLessSharp = cornerPointsLessSharp;
		scanBackValue.cornerPointsSharp = cornerPointsSharp;
		scanBackValue.laserCloud = laserCloud;
		scanBackValue.surfPointsFlat = surfPointsFlat;
		scanBackValue.surfPointsLessFlat = surfPointsLessFlat;

		outfile << num_id << std::endl;




		// std::stringstream filename2;
		// filename2 << "/home/xiesc/testpcd_corner/"<<num_id<<".pcd";

		//  pcl::io::savePCDFileASCII (filename2.str(), cornerPointsSharp);



		// std::stringstream filename3;
		// filename3 << "/home/xiesc/testpcd_surf/"<<num_id<<".pcd";

		//  pcl::io::savePCDFileASCII (filename3.str(), surfPointsFlat);

		//    std::stringstream filename4;
		// filename4 << "/home/xiesc/testpcd_lesscorner/"<<num_id<<".pcd";

		//  pcl::io::savePCDFileASCII (filename4.str(), cornerPointsLessSharp);



		// std::stringstream filename5;
		// filename5 << "/home/xiesc/testpcd_lesssurf/"<<num_id<<".pcd";

		//  pcl::io::savePCDFileASCII (filename5.str(), surfPointsLessFlat);


		num_id++;
		return scanBackValue;

	}



private:

	static const float scanPeriod;

	static const int systemDelay = 0;
	int systemInitCount;
	bool systemInited;

	static const int N_SCANS = 64;

	float t[4];


	float ang_df1;
	float ang_df2;

	float ang_1_max, ang_1_min, ang_2_max, ang_2_min;



	float tmp_ang1, tmp_ang2;

	std::ofstream outfile;
	int num_id;

	float cloudCurvature[160000];
	int cloudSortInd[160000];
	int cloudNeighborPicked[160000];
	int cloudNeighborPickedForCorner[160000];
	int cloudLabel[160000];

	int imuPointerFront;
	int imuPointerLast;//front用来记录laserscanhandler处理的点云时间到哪了，last实际上用来记录的是imuhandler处理的msg时间戳已经到哪了
	static const int imuQueLength = 200;

	float imuRollStart, imuPitchStart, imuYawStart;
	float imuRollCur, imuPitchCur, imuYawCur;

	float imuVeloXStart, imuVeloYStart, imuVeloZStart;
	float imuShiftXStart, imuShiftYStart, imuShiftZStart;

	float imuVeloXCur, imuVeloYCur, imuVeloZCur;
	float imuShiftXCur, imuShiftYCur, imuShiftZCur;

	float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
	float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

	double imuTime[imuQueLength];
	float imuRoll[imuQueLength];
	float imuPitch[imuQueLength];
	float imuYaw[imuQueLength];

	float imuAccX[imuQueLength];
	float imuAccY[imuQueLength];
	float imuAccZ[imuQueLength];

	float imuVeloX[imuQueLength];
	float imuVeloY[imuQueLength];
	float imuVeloZ[imuQueLength];

	float imuShiftX[imuQueLength];
	float imuShiftY[imuQueLength];
	float imuShiftZ[imuQueLength];








	
};

const float scanRegistration::scanPeriod = 0.1;

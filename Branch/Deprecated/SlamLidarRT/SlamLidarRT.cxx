#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
//#include "OpenICV/Data/icvPrimitiveData.hxx"
#include "OpenICV/Extensions/PCL/icvPointCloudData.hxx"


#include <scanregistration.hpp>
#include <laserodometry.hpp>
#include <laserMapping.hpp>
#include <transformMaintenance.hpp>
#include <common.h>


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/visualization/cloud_viewer.h>


using namespace icv;
using namespace ::pcl;
using namespace std;
using namespace core;
constexpr const char KEY_WIDTH[] = "width";
constexpr const char KEY_HEIGHT[] = "height";
constexpr const char KEY_NAME[] = "name";
class SlamLidarFunction : public icvFunction
{
	public:
    SlamLidarFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction( info){

		if (_information.Contains(KEY_WIDTH))
			width_ = _information.GetInteger(KEY_WIDTH);
		else width_ = 23600;

		if (_information.Contains(KEY_HEIGHT))
			height_ = _information.GetInteger(KEY_HEIGHT);
		else height_ = 1;
		if (_information.Contains(KEY_NAME))
			name_ = _information.GetString(KEY_NAME);
		else height_ = 1;
		cloud_viewer_ = boost::shared_ptr<::pcl::visualization::CloudViewer>(new ::pcl::visualization::CloudViewer(name_));

		

	}
		
	 SlamLidarFunction() : SlamLidarFunction(nullptr) {}
	


	void uncurvePointcloud(::pcl::PointCloud<PointXYZI> &input, ::pcl::PointCloud<PointXYZI> &output)
	{        

           ::pcl::PointCloud<::pcl::PointXYZ>::Ptr cloud (new ::pcl::PointCloud<::pcl::PointXYZ>);
           
           ::pcl::copyPointCloud(input, *cloud);

           
           std::vector<int> inliers;

           // created RandomSampleConsensus object and compute the appropriated model

           ::pcl::SampleConsensusModelPlane<::pcl::PointXYZ>::Ptr
             model_p (new ::pcl::SampleConsensusModelPlane<::pcl::PointXYZ> (cloud));


               Eigen::VectorXf planePara;



            

                   ::pcl::RandomSampleConsensus<::pcl::PointXYZ> ransac (model_p);
                   ransac.setDistanceThreshold (.01);
                   ransac.computeModel();
                   ransac.getInliers(inliers);
                   ransac.getModelCoefficients(planePara);
                   ::pcl::PointXYZI tmpPoint;
					
                  for (int i = 0;i<input.size();i++)
                  {
                      tmpPoint.x = input.points[i].x;
                      tmpPoint.y = input.points[i].y;
                      tmpPoint.z = input.points[i].z;
                      tmpPoint.intensity = tmpPoint.z-(-planePara(3)-planePara(0)*tmpPoint.x-planePara(1)*tmpPoint.y)/planePara(2);
                      // std::cerr<<tmpPoint.intensity<<std::endl;
                      if (fabs(tmpPoint.intensity)<0.1){
						  tmpPoint.z = (-planePara(3)-planePara(0)*tmpPoint.x-planePara(1)*tmpPoint.y)/planePara(2);
						//   std::cerr<<tmpPoint.intensity<<std::endl;						  
					  }
                          
                      
					  tmpPoint.intensity =input.points[i].intensity;

                    output.push_back(tmpPoint);        
                   }
             


	}
    // virtual void ConfigurateInput(std::vector<icvSubscriber*>& inputPorts) override
	// {
	// 	//CheckDataType<icvDoubleData>(inputPorts[0]);
	// 	ini_Input(inputPorts,1);
	// //	for (icvPublisher* connection : inputPorts[0]->GetConnections())
	// 	//	connection->SetDataObject((icvDataObject*)(new icv::icvPointCloudData<::pcl::PointXYZI>(width_, height_)));


	// }
	
    // virtual void ConfigurateOutput(std::vector<icvPublisher*>& outputPorts) override
	// 		{
	// 					ini_Output(outputPorts,0);

	// 		//	CheckDataType<icvDoubleData>(outputPorts[0]);
	// 		}
	
	
    virtual void Execute() override
    {
buff_Input(inputPorts);
buff_Output(outputPorts);
			count_++;
			//icv::icvPointCloudData<::pcl::PointXYZI> points_ = *static_cast<icv::icvPointCloudData<::pcl::PointXYZI>*>(inData[0]);
			icv::data::icvPointCloudData<::pcl::PointXYZI>& points=read_Input<icv::data::icvPointCloudData<::pcl::PointXYZI>>(0);
				

						
							scanValueBack = scanner.laserCloudHandler(points);
							odometryValueBack = odometrier.laserOdometryHandler(scanValueBack);
							mappingBackValue = mapper.laserMappingHandler(odometryValueBack);
							//savelog(mappingBackValue,odometryValueBack);
							
							if (count_ == 1)
							{
								ICV_LOG_TRACE << "start to draw";
								
							}
							
							cloud_viewer_->showCloud(mappingBackValue.laserCloudSurround);//.laserCloudSurround);

				
					
			}
			 void savelog(laserMappingBack a,laserOdometryBack b) 
		 { 	
	
			outfile<<a.transformAftMapped[0]<<" "<<a.transformAftMapped[1]<<" "<<a.transformAftMapped[2]<<" "<<a.transformAftMapped[3]<<" "<<a.transformAftMapped[4]<<" "<<a.transformAftMapped[5]<<endl;
	
			outfile2<<b.transformSum[0]<<" "<<b.transformSum[1]<<" "<<b.transformSum[2]<<" "<<b.transformSum[3]<<" "<<b.transformSum[4]<<" "<<b.transformSum[5]<<endl;
	
			//outfile3<<c.transformMapped[0]<<" "<<c.transformMapped[1]<<" "<<c.transformMapped[2]<<" "<<c.transformMapped[3]<<" "<<c.transformMapped[4]<<" "<<c.transformMapped[5]<<endl; 
		}
	private:
		int count_ = 0; std::string path_;	vector<string> files;
		char filename[50] = { 0 };
		string p,name_; int width_, height_;
		std::ofstream outfile;
		std::ofstream outfile2;
		std::ofstream outfile3;
		string filename_short, filename_full;
		scanRegistration scanner;
		laserOdometry odometrier;
		laserMapping mapper;
		transformMainTenance mainer;
		scanRegistrationBack scanValueBack;
        laserOdometryBack odometryValueBack;
		laserMappingBack mappingBackValue;
		maintenanceBack maintenanceValueBack;
		//::pcl::PointCloud<PointXYZI> points;
		boost::shared_ptr<::pcl::visualization::CloudViewer> cloud_viewer_;

};

ICV_REGISTER_FUNCTION(SlamLidarFunction)


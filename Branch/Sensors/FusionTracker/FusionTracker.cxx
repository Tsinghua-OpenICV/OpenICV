#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeManager.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Core/icvTime.h"
//Optional：It depends on what kind of message you’d like to use
#include "OpenICV/structure/icvCvMatData.hxx"

#include "OpenICV/Basis/icvPrimitiveData.hxx"

#include <cstdlib>
#include <string>
#include <sstream>

#include <boost/thread/thread.hpp>

#include <eigen3/Eigen/Dense>
#include <vector>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
// Packet structure
#include "OpenICV/structure/structureIMU.h"
#include "time.h"
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include <iostream>
#include <fstream>
#include <arpa/inet.h>
#include "OpenICV/structure/structure_all_objs.h"
#include "OpenICV/structure/structure_fusion_obj.h"

// Ethernet

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

#define PI 3.1415926

#define WGS84_A         6378137.0               // major axis
#define WGS84_E         0.0818191908            // first eccentricity
#define UTM_E2          (WGS84_E * WGS84_E)       // e^2
#define UTM_K0          0.9996                  // scale factor

using namespace icv;
using namespace icv::function;


class FusionTracker:public icvFunction
{
    public:
    typedef data::icvStructureData<Imu> icvImu;
    typedef data::icvStructureData<NavSatFix> icvNavSatFix;
    typedef data::icvStructureData<TwistWithCovarianceStamped> icvTwistWithCovarianceStamped;
    typedef data::icvStructureData<OBJ_40_EQ> ICVEQ;
    typedef data::icvStructureData<OBJ_32_LRR> ICVLRR;
    typedef data::icvStructureData<OBJ_32_FSDA> ICVFSDA;
    typedef data::icvStructureData<OBJ_32_RSDA> ICVRSDA;
    
    FusionTracker(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)   //构造函数，只执行一次的放在构造函数里面
    {
        Register_Pub("obj_forward_fusion");
        Register_Pub("obj_backward_fusion"); 

        Register_Sub("obj_eq");
        Register_Sub("obj_lrr");
        Register_Sub("obj_rsda"); 
        Register_Sub("obj_fsda");  
        Register_Sub("msg_imu");
        Register_Sub("msg_fix");
        Register_Sub("msg_vel");
        
        ICV_LOG_INFO << "Fusion process Started";
    }
        
        void FusionUpdate_Forward(const OBJ_40_EQ& obj_eq_information,
        const OBJ_32_LRR& obj_lrr_information,
        const Imu &msg_imu_information, 
        const NavSatFix &msg_fix_information, 
        const TwistWithCovarianceStamped &msg_vel_information)
        {
            FUSION_ARRAY obj_fusion_forward_t;
            uint16 track_ID = 1;

            for (uint16 i=0;i<32;i++)
            {
                
                if(abs(obj_lrr_information.objs_LRR[i].lrr_dx)>0.8) 
                {
                    data::icvStructureData<OBJ_FUSION> track;
                    obj_fusion_forward_t.objs_fusion[i].Track_ID = track_ID;
                     
                    obj_fusion_forward_t.objs_fusion[i].Track_Lat_Distance = obj_lrr_information.objs_LRR[i].lrr_dx;
                    obj_fusion_forward_t.objs_fusion[i].Track_Long_Distance = obj_lrr_information.objs_LRR[i].lrr_dy; 


                    obj_fusion_forward_t.objs_fusion[i].Track_Relative_Lat_Velocity = obj_lrr_information.objs_LRR[i].lrr_vx;
                    obj_fusion_forward_t.objs_fusion[i].Track_Relative_Long_Velocity = obj_lrr_information.objs_LRR[i].lrr_vy;
                            
                    obj_fusion_forward_t.objs_fusion[i].Track_Height = 3.0;
                    obj_fusion_forward_t.objs_fusion[i].Track_Length = 3.0;
                    obj_fusion_forward_t.objs_fusion[i].Track_Width = 2.5;
                    
                    obj_fusion_forward_t.objs_fusion[i].Track_Age = 1;
                    obj_fusion_forward_t.objs_fusion[i].Track_time_since_update = 1;

                    transform(obj_fusion_forward_t.objs_fusion[i], msg_imu_information,msg_fix_information,msg_vel_information);

                    track_ID++;

                    ICV_LOG_INFO << "Forward Tracker  OBJ_ID :" <<  obj_fusion_forward_t.objs_fusion[i].Track_ID;
                    ICV_LOG_INFO << "Forward Tracker OBJ_Height :" << obj_fusion_forward_t.objs_fusion[i].Track_Height;
                    ICV_LOG_INFO << "Forward Tracker OBJ_Width :" << obj_fusion_forward_t.objs_fusion[i].Track_Width;
                    ICV_LOG_INFO << "Forward Tracker OBJ_Length :" << obj_fusion_forward_t.objs_fusion[i].Track_Length; 
                    ICV_LOG_INFO << "Forward Tracker OBJ_Long_Distance :" <<  obj_fusion_forward_t.objs_fusion[i].Track_Long_Distance ;
                    ICV_LOG_INFO << "Forward Tracker OBJ_Lat_Distance :" <<  obj_fusion_forward_t.objs_fusion[i].Track_Lat_Distance ; 

                }
                else 
                {
                    continue;
                }
            }
         
            obj_fusion_forward_information.setvalue(obj_fusion_forward_t);

        }

       void FusionUpdate_Backward(const OBJ_32_RSDA& obj_rsda_information,
        const Imu &msg_imu_information, 
        const NavSatFix &msg_fix_information, 
        const TwistWithCovarianceStamped &msg_vel_information)
        {
            //data::icvStructureData<OBJ_EQ> obj_detection;
            FUSION_ARRAY obj_fusion_backward_t;
            uint16 track_ID = 1;

            for (uint16 i=0;i<32;i++)
            {
                
               if(abs(obj_rsda_information.objs_RSDA[i].rsda_dx)>0.5) 
                {
                    data::icvStructureData<OBJ_FUSION> track;
                    obj_fusion_backward_t.objs_fusion[i].Track_ID = track_ID;
                     
                    obj_fusion_backward_t.objs_fusion[i].Track_Lat_Distance = obj_rsda_information.objs_RSDA[i].rsda_dx;
                    obj_fusion_backward_t.objs_fusion[i].Track_Long_Distance = obj_rsda_information.objs_RSDA[i].rsda_dy; 


                    obj_fusion_backward_t.objs_fusion[i].Track_Relative_Lat_Velocity = obj_rsda_information.objs_RSDA[i].rsda_vx;
                    obj_fusion_backward_t.objs_fusion[i].Track_Relative_Long_Velocity = obj_rsda_information.objs_RSDA[i].rsda_vy;
                            
                    obj_fusion_backward_t.objs_fusion[i].Track_Height = 3.0;
                    obj_fusion_backward_t.objs_fusion[i].Track_Length = 3.0;
                    obj_fusion_backward_t.objs_fusion[i].Track_Width = 2.5;
                    
                    obj_fusion_backward_t.objs_fusion[i].Track_Age = 1;
                    obj_fusion_backward_t.objs_fusion[i].Track_time_since_update = 1;

                    track_ID++;

                     transform(obj_fusion_backward_t.objs_fusion[i], msg_imu_information,msg_fix_information,msg_vel_information);
                    
                }
                else 
                {
                    continue;
                }
            }

            obj_fusion_backward_information.setvalue(obj_fusion_backward_t);

        }

        void transform(OBJ_FUSION obj_fusion, Imu msg_imu_information, NavSatFix msg_fix_information, TwistWithCovarianceStamped msg_vel_information)
        {
            Eigen::Quaterniond q(msg_imu_information.orientation.w, msg_imu_information.orientation.x, msg_imu_information.orientation.y, msg_imu_information.orientation.z);
            R_static = q.toRotationMatrix();
            
            Eigen::Vector3d xyz;
            fromMsg(msg_fix_information, xyz);
           
            T_static << xyz.x(),xyz.y(),xyz.z();


            R_ego << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0; 
            T_ego << 0.0,1.0,1.0;

            vel_static << msg_vel_information.twist.twist.linear.x, msg_vel_information.twist.twist.linear.y, msg_vel_information.twist.twist.linear.z;
        
            Eigen::Vector3d old_geo_center, new_geo_center;
            old_geo_center << obj_fusion.Track_Long_Distance, obj_fusion.Track_Lat_Distance, 1.0;
            //std::cout<<"old_geo_center:"<<std::endl<<old_geo_center<<std::endl;
            new_geo_center = R_static*(R_ego*old_geo_center+T_ego)+T_static;
            obj_fusion.Track_Long_Distance = new_geo_center(0);
            obj_fusion.Track_Lat_Distance = new_geo_center(1);

            Eigen::Vector3d old_vel, new_vel;
            old_vel << obj_fusion.Track_Relative_Long_Velocity, obj_fusion.Track_Relative_Lat_Velocity, 0.01;
            new_vel = R_static*R_ego*old_vel;
            obj_fusion.Track_Relative_Long_Velocity = new_vel(0) + vel_static(0);
            obj_fusion.Track_Relative_Lat_Velocity = new_vel(1) + vel_static(1);
        }


        void fromMsg(NavSatFix msg_fix_information, Eigen::Vector3d xyz)
        {
        double Lat = msg_fix_information.latitude;
        double Long = msg_fix_information.longitude;
        
        double a = WGS84_A;
        double eccSquared = UTM_E2;
        double k0 = UTM_K0;
        
        double LongOrigin;
        double eccPrimeSquared;
        double N, T, C, A, M;
                
        // Make sure the longitude is between -180.00 .. 179.9
        // (JOQ: this is broken for Long < -180, do a real normalize)
        double LongTemp = (Long+180)-int((Long+180)/360)*360-180;
        double LatRad = angles_from_degrees(Lat);
        double LongRad = angles_from_degrees(LongTemp);
        double LongOriginRad;

        double to_altitude = msg_fix_information.altitude;
        int to_zone = int((LongTemp + 180)/6) + 1;
        
        if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
            to_zone = 32;

        // Special zones for Svalbard
        if( Lat >= 72.0 && Lat < 84.0 ) 
            {
            if(      LongTemp >= 0.0  && LongTemp <  9.0 ) to_zone = 31;
            else if( LongTemp >= 9.0  && LongTemp < 21.0 ) to_zone = 33;
            else if( LongTemp >= 21.0 && LongTemp < 33.0 ) to_zone = 35;
            else if( LongTemp >= 33.0 && LongTemp < 42.0 ) to_zone = 37;
            }
        // +3 puts origin in middle of zone
        LongOrigin = (to_zone - 1)*6 - 180 + 3; 
        LongOriginRad = angles_from_degrees(LongOrigin);
        
        // compute the UTM band from the latitude
       // to.band = UTMBand(Lat, LongTemp);
        #if 0
        if (to.band == ' ')
            throw std::range_error;
        #endif
        
        eccPrimeSquared = (eccSquared)/(1-eccSquared);
        
        N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
        T = tan(LatRad)*tan(LatRad);
        C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
        A = cos(LatRad)*(LongRad-LongOriginRad);
        
        M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
                - 5*eccSquared*eccSquared*eccSquared/256) * LatRad 
                - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
                    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
                + (15*eccSquared*eccSquared/256
                    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad) 
                - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));
                
        double to_easting = (double)
            (k0*N*(A+(1-T+C)*A*A*A/6
                    + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
            + 500000.0);
        
        double to_northing = (double)
            (k0*(M+N*tan(LatRad)
                *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                    + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
        
        if(Lat < 0)
            {
            //10000000 meter offset for southern hemisphere
            to_northing += 10000000.0;
            }
        
        xyz<<to_easting, to_northing, to_altitude;

        }
        
        float64 angles_from_degrees(float64 degree )
        {
            return degree*PI/180;
        }

        virtual void Execute() override 
        {   
            icvSubscribe("obj_eq",&obj_eq_information);
            icvSubscribe("obj_lrr",&obj_lrr_information);
            icvSubscribe("obj_rsda",&obj_rsda_information);
            icvSubscribe("obj_fsda",&obj_fsda_information);
            icvSubscribe("obj_fsda",&obj_fsda_information);
            icvSubscribe("msg_imu", &IMU_DATA);
            icvSubscribe("msg_fix", &FIX_DATA);
            icvSubscribe("msg_vel", &VEL_DATA);

            obj_eqs = obj_eq_information.getvalue();
            obj_lrrs = obj_lrr_information.getvalue();
            obj_fsdas = obj_fsda_information.getvalue();
            obj_rsdas = obj_rsda_information.getvalue();
            msg_imu = IMU_DATA.getvalue();
            msg_fix = FIX_DATA.getvalue();
            msg_vel = VEL_DATA.getvalue();


            //FusionPredict(obj_fusion_t);
            FusionUpdate_Forward(obj_eqs, obj_lrrs,msg_imu,msg_fix,msg_vel);
            FusionUpdate_Backward(obj_rsdas,msg_imu,msg_fix,msg_vel);

            icvPublish("obj_forward_fusion",&obj_fusion_forward_information);
            icvPublish("obj_backward_fusion",&obj_fusion_backward_information);
        }


    private:
        ICVEQ obj_eq_information;
        ICVLRR obj_lrr_information;
        ICVRSDA obj_rsda_information;
        ICVFSDA obj_fsda_information;
        icvImu IMU_DATA;
        icvNavSatFix FIX_DATA;
        icvTwistWithCovarianceStamped VEL_DATA;

        OBJ_40_EQ obj_eqs;
        OBJ_32_LRR obj_lrrs;
        OBJ_32_FSDA obj_fsdas;
        OBJ_32_RSDA obj_rsdas;
        Imu msg_imu;
        NavSatFix msg_fix;
        TwistWithCovarianceStamped msg_vel;

        Eigen::Matrix3d R_static;  // rotation gps - 大地
        Eigen::Vector3d T_static;  // 自车gps 到 大地

        Eigen::Matrix3d R_ego;   // lidar - 自车  标定关系  固定关系
        Eigen::Vector3d T_ego;   // lidar - 自车  标定关系

        Eigen::Vector3d vel_static;  // 自车相对大地 速度  矢量

        data::icvStructureData<FUSION_ARRAY> obj_fusion_forward_information;
        data::icvStructureData<FUSION_ARRAY> obj_fusion_backward_information;
        bool send_ = true;
        //data::icvStructureData<OBJ_FUSION> obj_fusion_t;
};
ICV_REGISTER_FUNCTION(FusionTracker)

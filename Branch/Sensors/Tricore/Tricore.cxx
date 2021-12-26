//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//

#ifndef _Tricore_H
#define _Tricore_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeManager.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Net/icvUdpReceiverSource.h"

#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Net/icvUdpReceiverSource.h"

#include <cstdlib>
#include <string>
#include <sstream>

#include <boost/thread/thread.hpp>

#define PI 3.1415926

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

using namespace icv;
using namespace icv::function;

#include <eigen3/Eigen/Dense>
#include <string>
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

#include "OpenICV/structure/Can_SCU_IPC_1_0x174.h"
#include "OpenICV/structure/Can_SCU_IPC_2_0x175.h"
#include "OpenICV/structure/Can_SCU_IPC_3_0x176.h"
#include "OpenICV/structure/Can_SCU_IPC_4_0x177.h"
#include "OpenICV/structure/Can_SCU_IPC_5_0x178.h"
#include "OpenICV/structure/Can_SCU_IPC_6_0x179.h"
#include "OpenICV/structure/Can_SCU_IPC_7_0x17A.h"
#include "OpenICV/structure/structureCanP7.h"

//#include "OpenICV/structure/CanFrame.h"
//#include "OpenICV/structure/inputmessage.h"

// Ethernet
#include <arpa/inet.h>

#include "OpenICV/structure/structure_all_objs.h"

//#include "structure_obj_eq.h"

// UINT16_MAX is not defined by default in Ubuntu Saucy
#ifndef UINT16_MAX
#define UINT16_MAX (65535)
#endif

class Tricore : public icvUdpReceiverSource
{
public:
    typedef data::icvStructureData<Imu> icvImu;
    typedef data::icvStructureData<NavSatFix> icvNavSatFix;
    typedef data::icvStructureData<TwistWithCovarianceStamped> icvTwistWithCovarianceStamped;
    typedef data::icvStructureData<Odometry> icvOdometry;

    typedef data::icvStructureData<CanFrame_SCU_IPC_1_0x174> SCU_IPC_0x174;
    typedef data::icvStructureData<CanFrame_SCU_IPC_2_0x175> SCU_IPC_0x175;
    typedef data::icvStructureData<CanFrame_SCU_IPC_3_0x176> SCU_IPC_0x176;
    typedef data::icvStructureData<CanFrame_SCU_IPC_4_0x177> SCU_IPC_0x177;
    typedef data::icvStructureData<CanFrame_SCU_IPC_5_0x178> SCU_IPC_0x178;
    typedef data::icvStructureData<CanFrame_SCU_IPC_6_0x179> SCU_IPC_0x179;
    typedef data::icvStructureData<CanFrame_SCU_IPC_7_0x17A> SCU_IPC_0x17A;

    Tricore(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info)
    {

        Register_Pub("obj_eq");
        Register_Pub("obj_lrr"); 
        Register_Pub("obj_rsda"); 
        Register_Pub("obj_fsda");  
        Register_Pub("sls_information"); 
        Register_Pub("tfl_eq"); 
        Register_Pub("path_prediction"); 
        Register_Pub("Host_lines"); 

        Register_Sub("data_174");
        Register_Sub("data_175");
        Register_Sub("data_176");
        Register_Sub("data_177");
        Register_Sub("data_178");
        Register_Sub("data_179");
        Register_Sub("data_17A"); 

        Register_Sub("msg_imu");

        icv_thread loop(icv_bind(&Tricore::processsend, this));
        icv_thread_guard g(loop);
    };

    void processsend()
    {
        //temp_ag = 0;
        while (send_)
        {
            icvSubscribe("data_174", &ANGL_DATA);
            icvSubscribe("data_175", &WHEL_DATA);
            icvSubscribe("data_176", &SPED_DATA);
            icvSubscribe("data_177", &YAW_DATA);
            icvSubscribe("data_178", &LAMP_DATA);
            icvSubscribe("data_179", &GEAR_DATA);
            icvSubscribe("data_17A", &WARN_DATA);

            icvSubscribe("msg_imu", &IMU_DATA);

            angl_data = ANGL_DATA.getvalue();
            whel_data = WHEL_DATA.getvalue();
            sped_data = SPED_DATA.getvalue();
            yaw_data = YAW_DATA.getvalue();
            lamp_data = LAMP_DATA.getvalue();
            gear_data = GEAR_DATA.getvalue();
            warn_data = WARN_DATA.getvalue();

            msg_imu = IMU_DATA.getvalue();
/*
            ICV_LOG_DEBUG << "lamp_data.DriverDoorLockSt :" << (int16)lamp_data.DriverDoorLockSt;
            ICV_LOG_DEBUG << "lamp_data.DriverDoorAjarSt :" << (int16)lamp_data.DriverDoorAjarSt;
            ICV_LOG_DEBUG << "lamp_data.PsngrDoorAjarSt：" << (int16)lamp_data.PsngrDoorAjarSt;
            ICV_LOG_DEBUG << "lamp_data.RLDoorAjarSt :" << (int16)lamp_data.RLDoorAjarSt;
            ICV_LOG_DEBUG << "lamp_data.RRDoorAjarSt :" << (int16)lamp_data.RRDoorAjarSt;
            ICV_LOG_DEBUG << "whel_data.FLWheelSpd :" << whel_data.FLWheelSpd;
            ICV_LOG_DEBUG << "whel_data.FRWheelSpd :" << whel_data.FRWheelSpd;
            ICV_LOG_DEBUG << "whel_data.RLWheelSpd :" << whel_data.RLWheelSpd;
            ICV_LOG_DEBUG << "whel_data.RRWheelSpd :" << whel_data.RRWheelSpd;
            ICV_LOG_DEBUG << " whel_data.FLWheelSpdVD :" << (int16)whel_data.FLWheelSpdVD;
            ICV_LOG_DEBUG << "whel_data.FRWheelSpdVD :" << (int16)whel_data.FRWheelSpdVD;
            ICV_LOG_DEBUG << "whel_data.RLWheelSpdVD :" << (int16)whel_data.RLWheelSpdVD;
            ICV_LOG_DEBUG << "whel_data.RRWheelSpdVD :" << (int16)whel_data.RRWheelSpdVD;
*/
            send_297(sped_data.VehSpd, sped_data.VehSpdVD, lamp_data.LTurnLampOutputSt, lamp_data.RTurnLampOutputSt, gear_data.CurrentGearLev, gear_data.CurrentGearLevVD,
                     angl_data.SteeringAngle, angl_data.SteeringAngleSpd, angl_data.SteeringAngleVD, angl_data.SteeringAngleSpdVD,
                     yaw_data.YAW, yaw_data.YAWVD, (float32)msg_imu.angular_velocity.y, (float32)msg_imu.linear_acceleration.x, lamp_data.DriverDoorLockSt);
            send_micro(lamp_data.DriverDoorLockSt, lamp_data.DriverDoorAjarSt, lamp_data.PsngrDoorAjarSt, lamp_data.RLDoorAjarSt, lamp_data.RRDoorAjarSt,
                       whel_data.FLWheelSpd, whel_data.FLWheelSpd, whel_data.RLWheelSpd, whel_data.RRWheelSpd, whel_data.FLWheelSpdVD, whel_data.FRWheelSpdVD, whel_data.RLWheelSpdVD, whel_data.RRWheelSpdVD);
            usleep(50000);
            //ICV_LOG_INFO << "A cycle\n";
        }
    }
    void send_micro(uint8_t DriverDoorLockSt, uint8_t DriverDoorAjarSt, uint8_t PsngrDoorAjarSt, uint8_t RLDoorAjarSt, uint8_t RRDoorAjarSt,
                    float32 FLWheelSpd, float32 FRWheelSpd, float32 RLWheelSpd, float32 RRWheelSpd, uint8_t FLWheelSpdVD, uint8_t FRWheelSpdVD, uint8_t RLWheelSpdVD, uint8_t RRWheelSpdVD)
    {
        uint8_t data_199[40] = {0};
        uint8_t temp[4];
        //net send type ----------------------

        data_199[0] = 0x88;
        data_199[1] = 0x00;
        data_199[2] = 0x00;
        data_199[3] = 0x01;
        data_199[4] = 0x99;

        uint8_t drvdooropenst;
        if (DriverDoorLockSt == 0 && DriverDoorAjarSt == 1)
            drvdooropenst = 3;
        else if (DriverDoorLockSt == 0 && DriverDoorAjarSt == 0)
            drvdooropenst = 0;
        else if (DriverDoorLockSt == 1 && DriverDoorAjarSt == 0)
            drvdooropenst = 2;
        else if (DriverDoorLockSt == 1 && DriverDoorAjarSt == 1)
            drvdooropenst = 1;

        data_199[5] = drvdooropenst;
        data_199[6] = PsngrDoorAjarSt;
        data_199[7] = RLDoorAjarSt;
        data_199[8] = RRDoorAjarSt;

        float32 vehspddrvn = (RLWheelSpd + RRWheelSpd) * 0.05625 / 2.0; // km/h
        float32 vehspdnondrvn = (FLWheelSpd + FRWheelSpd) * 0.05625 / 2.0;

        memcpy(temp, &vehspddrvn, sizeof(temp));
        data_199[9] = temp[0];
        data_199[10] = temp[1];
        data_199[11] = temp[2];
        data_199[12] = temp[3];

        memcpy(temp, &vehspdnondrvn, sizeof(temp));
        data_199[13] = temp[0];
        data_199[14] = temp[1];
        data_199[15] = temp[2];
        data_199[16] = temp[3];

        uint8_t vehspdnondrvnVD, vehspddrvnVD;
        if (FLWheelSpdVD == 1 && FRWheelSpdVD == 1)
            vehspdnondrvnVD = 1;
        else
            vehspdnondrvnVD = 0;

        if (RLWheelSpdVD == 1 && RRWheelSpdVD == 1)
            vehspddrvnVD = 1;
        else
            vehspddrvnVD = 0;

        data_199[17] = vehspddrvnVD;
        data_199[18] = vehspdnondrvnVD;

        send(data_199, 20);
    }

    void send_297(float32 vehicelspeed, uint8_t vehiclespeedVD, uint8_t leftlamp, uint8_t rightlamp, uint8_t gearstatus, uint8_t gearstatusVD,
                  float32 angle, float32 anglespd, uint8_t SteeringAngleVD, uint8_t SteeringAngleSpdVD,
                  float32 yawrate, uint8_t yawrateVD, float32 acc_rate_lon, float32 acc_lat, uint8_t lockstatus)
    {

        float32 vehicelspeed_ = vehicelspeed * 0.05625; // km/h
        uint8_t gearstatus_ = 80;
        switch (gearstatus)
        {
        case 0x1: //D
        {
            gearstatus_ = 68;
            break;
        }
        case 0x2: //N
        {
            gearstatus_ = 78;
            break;
        }
        case 0x3: //R
        {
            gearstatus_ = 82;
            break;
        }
        case 0x4: //P
        {
            gearstatus_ = 80;
            break;
        }
        default:
            break;
        } // end switch

        float32 angle_;    //deg
        float32 anglespd_; //deg/s

        angle_ = angle * 0.1 - 780; // P7 type tranform
        anglespd_ = (float32)anglespd;

        float32 yawrate_; //deg/s
        yawrate_ = yawrate * 0.0625 - 93;
        uint8_t temp[4];

        uint8_t data_188[40] = {0};
        //net send type ----------------------

        data_188[0] = 0x88;
        data_188[1] = 0x00;
        data_188[2] = 0x00;
        data_188[3] = 0x01;
        data_188[4] = 0x88;

        memcpy(temp, &vehicelspeed_, sizeof(temp));
        data_188[5] = temp[0];
        data_188[6] = temp[1];
        data_188[7] = temp[2];
        data_188[8] = temp[3];

        data_188[9] = vehiclespeedVD;

        data_188[10] = leftlamp;
        data_188[11] = rightlamp;

        data_188[12] = gearstatus_;
        data_188[13] = gearstatusVD;

        memcpy(temp, &angle_, sizeof(temp));
        data_188[14] = temp[0];
        data_188[15] = temp[1];
        data_188[16] = temp[2];
        data_188[17] = temp[3];

        memcpy(temp, &anglespd, sizeof(temp));
        data_188[18] = temp[0];
        data_188[19] = temp[1];
        data_188[20] = temp[2];
        data_188[21] = temp[3];

        data_188[22] = SteeringAngleVD;
        data_188[23] = SteeringAngleSpdVD;

        memcpy(temp, &yawrate_, sizeof(temp));
        data_188[24] = temp[0];
        data_188[25] = temp[1];
        data_188[26] = temp[2];
        data_188[27] = temp[3];

        data_188[28] = yawrateVD;

        memcpy(temp, &acc_rate_lon, sizeof(temp));
        data_188[29] = temp[0];
        data_188[30] = temp[1];
        data_188[31] = temp[2];
        data_188[32] = temp[3];

        memcpy(temp, &acc_lat, sizeof(temp));
        data_188[33] = temp[0];
        data_188[34] = temp[1];
        data_188[35] = temp[2];
        data_188[36] = temp[3];

        data_188[37] = lockstatus;

        send(data_188, 40);
    }

    OBJ_EQ get_one_object(uint16 start_point)
    {
        OBJ_EQ one_object;
        uint8 id_Uint8_array;
        uint8 F_id;
        float32 temp_F32;
        uint8 temp_array[4];

        //  uint16 temp_U16;
        uint8 temp_U16_array[2];

        one_object.OBJ_ID = getpdu_TC[start_point + 2];
        one_object.OBJ_Object_Class = getpdu_TC[start_point + 3];

        F_id = 1;
        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Class_Probability = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Lat_Distance = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Long_Distance = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Relative_Lat_Velocity = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Relative_Long_Velocity = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Relative_Long_Acc = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Abs_Lat_Acc = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Abs_Long_Acc = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Abs_Lat_Velocity = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Abs_Long_Velocity = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Abs_Acceleration = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Height = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Height_STD = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Length = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Length_STD = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Width = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_object.OBJ_Width_STD = temp_F32;
        F_id++;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + F_id * 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        //memcpy(&one_object.OBJ_Existence_Probability,&temp_array,sizeof(temp_array));
        one_object.OBJ_Existence_Probability = temp_F32;
        F_id++;

        one_object.OBJ_Age_Seconds = getpdu_TC[start_point + F_id * 4];
        one_object.OBJ_Motion_Orientation = getpdu_TC[start_point + F_id * 4 + 1];
        one_object.OBJ_Motion_Status = getpdu_TC[start_point + F_id * 4 + 2];
        one_object.OBJ_Measuring_Status = getpdu_TC[start_point + F_id * 4 + 3];
        one_object.OBJ_Motion_Category = getpdu_TC[start_point + F_id * 4 + 4];

        temp_U16_array[0] = getpdu_TC[start_point + F_id * 4 + 5];
        temp_U16_array[1] = getpdu_TC[start_point + F_id * 4 + 6];
        memcpy(&one_object.OBJ_Object_Age, &temp_U16_array, 2);

        return one_object;
    }

    OBJ_LRR get_one_object_LRR(uint16 startpoint)
    {
        OBJ_LRR one_object_LRR;
        uint8 temp_U16_array[2];

        one_object_LRR.LRR_ID = getpdu_TC[startpoint + 2];
        temp_U16_array[0] = getpdu_TC[startpoint + 3];
        temp_U16_array[1] = getpdu_TC[startpoint + 4];
        memcpy(&one_object_LRR.LRR_dx, &temp_U16_array, 2);
        one_object_LRR.lrr_dx= one_object_LRR.LRR_dx * 0.1248;

        temp_U16_array[0] = getpdu_TC[startpoint + 5];
        temp_U16_array[1] = getpdu_TC[startpoint + 6];
        memcpy(&one_object_LRR.LRR_dy, &temp_U16_array, 2);
        one_object_LRR.lrr_dy= one_object_LRR.LRR_dy * 0.1248 - 128;

        temp_U16_array[0] = getpdu_TC[startpoint + 7];
        temp_U16_array[1] = getpdu_TC[startpoint + 8];
        memcpy(&one_object_LRR.LRR_Vx, &temp_U16_array, 2);
        one_object_LRR.lrr_vx= one_object_LRR.LRR_Vx * 0.0625 - 128;

        temp_U16_array[0] = getpdu_TC[startpoint + 9];
        temp_U16_array[1] = getpdu_TC[startpoint + 10];
        memcpy(&one_object_LRR.LRR_Vy, &temp_U16_array, 2);
        one_object_LRR.lrr_vy= one_object_LRR.LRR_Vy * 0.0625 - 128;

        one_object_LRR.LRR_Sync = getpdu_TC[startpoint + 11];

        one_object_LRR.LRR_wExist = getpdu_TC[startpoint + 12];
        one_object_LRR.lrr_wexist = one_object_LRR.LRR_wExist * 0.0625;

        one_object_LRR.LRR_wObstacle = getpdu_TC[startpoint + 13];
        one_object_LRR.lrr_wobstacle = one_object_LRR.LRR_wObstacle * 0.0625;

        return one_object_LRR;
    }
    //offset & coefficient ?

    OBJ_RSDA get_one_object_RSDA(uint16 startpoint)
    {
        OBJ_RSDA one_object_RSDA;
        uint8 temp_U16_array[2];

        one_object_RSDA.RSDA_ID = getpdu_TC[startpoint + 386];

        temp_U16_array[0] = getpdu_TC[startpoint + 387];
        temp_U16_array[1] = getpdu_TC[startpoint + 388];
        memcpy(&one_object_RSDA.RSDA_xPosition, &temp_U16_array, 2);
        one_object_RSDA.rsda_dx = one_object_RSDA.RSDA_xPosition * 0.064 - 262.144;

        temp_U16_array[0] = getpdu_TC[startpoint + 389];
        temp_U16_array[1] = getpdu_TC[startpoint + 390];
        memcpy(&one_object_RSDA.RSDA_yPosition, &temp_U16_array, 2);
        one_object_RSDA.rsda_dy = one_object_RSDA.RSDA_yPosition * 0.064 - 262.144;

        temp_U16_array[0] = getpdu_TC[startpoint + 391];
        temp_U16_array[1] = getpdu_TC[startpoint + 392];
        memcpy(&one_object_RSDA.RSDA_xSpeed, &temp_U16_array, 2);
        one_object_RSDA.rsda_vx = one_object_RSDA.RSDA_xSpeed * 0.1 - 102.4;

        temp_U16_array[0] = getpdu_TC[startpoint + 393];
        temp_U16_array[1] = getpdu_TC[startpoint + 394];
        memcpy(&one_object_RSDA.RSDA_ySpeed, &temp_U16_array, 2);
        one_object_RSDA.rsda_vy = one_object_RSDA.RSDA_ySpeed * 0.1 - 102.4;


        if (getpdu_TC[startpoint + 395] == 1)
        {
            one_object_RSDA.RSDA_ObjMovStatic = 1;
        }
        else
        {
            one_object_RSDA.RSDA_ObjMovStatic = 0;
        }

        one_object_RSDA.RSDA_SNR = getpdu_TC[startpoint + 396];
        
        one_object_RSDA.RSDA_obj_Class = getpdu_TC[startpoint + 397];
        
        return one_object_RSDA;
    }
    //offset & coefficient ?

    OBJ_FSDA get_one_object_FSDA(uint16 startpoint)
    {
        OBJ_FSDA one_object_FSDA;
        uint8 temp_U16_array[2];

        one_object_FSDA.FSDA_ID = getpdu_TC[startpoint + 770];
        
        temp_U16_array[0] = getpdu_TC[startpoint + 771];
        temp_U16_array[1] = getpdu_TC[startpoint + 772];
        memcpy(&one_object_FSDA.FSDA_xPosition, &temp_U16_array, 2);
        one_object_FSDA.fsda_dx = one_object_FSDA.FSDA_xPosition * 0.125 -5;

        temp_U16_array[0] = getpdu_TC[startpoint + 773];
        temp_U16_array[1] = getpdu_TC[startpoint + 774];
        memcpy(&one_object_FSDA.FSDA_yPosition, &temp_U16_array, 2);
        one_object_FSDA.fsda_dy = one_object_FSDA.FSDA_yPosition * 0.125 -128;
        
        temp_U16_array[0] = getpdu_TC[startpoint + 775];
        temp_U16_array[1] = getpdu_TC[startpoint + 776];
        memcpy(&one_object_FSDA.FSDA_xSpeed, &temp_U16_array, 2);
        one_object_FSDA.fsda_vx = one_object_FSDA.FSDA_xSpeed * 0.1 -51.15;
        
        temp_U16_array[0] = getpdu_TC[startpoint + 777];
        temp_U16_array[1] = getpdu_TC[startpoint + 778];
        memcpy(&one_object_FSDA.FSDA_ySpeed, &temp_U16_array, 2);
        one_object_FSDA.fsda_vy = one_object_FSDA.FSDA_ySpeed * 0.06 -61.41;
        
        if (getpdu_TC[startpoint + 779] == 1)
        {
            one_object_FSDA.FSDA_ObjMovStatic = 1;
        }
        else
        {
            one_object_FSDA.FSDA_ObjMovStatic = 0;
        }
        one_object_FSDA.FSDA_SNR = getpdu_TC[startpoint + 780];
        one_object_FSDA.FSDA_obj_Class = getpdu_TC[startpoint + 781];
        return one_object_FSDA;
    }
    //offset & coefficient ?

    Sematic_lines_info get_Sematiclines_info()
    {
        Sematic_lines_info Sematic_lns_info;
        // uint16 id_sematic_line;
        for (uint16 id_sematic_line = 0; id_sematic_line < 10; id_sematic_line++)
        {

            if (getpdu_TC[3450 + id_sematic_line] == 1)
            {
                Sematic_lns_info.SL_Close_To_Junc[id_sematic_line] = 1;
            }
            else
            {
                Sematic_lns_info.SL_Close_To_Junc[id_sematic_line] = 0;
            }
        }
        Sematic_lns_info.SLD_Num_Of_Lanes_Close_Left = getpdu_TC[3500];
        Sematic_lns_info.SLD_Num_Of_Lanes_Close_Right = getpdu_TC[3501];
        return Sematic_lns_info;
    }

    stHeader get_stHeader()
    {
        stHeader Header_info;

        Header_info.FrameID = getpdu_TC[3485];

        uint8 temp_U16_array[2];
        temp_U16_array[0] = getpdu_TC[3486];
        temp_U16_array[1] = getpdu_TC[3487];
        memcpy(&Header_info.Counter, &temp_U16_array, 2);

        uint8 temp_32_array[4];
        float32 temp_F32;

        for (uint16 id_32_array = 0; id_32_array < 4; id_32_array++)
        {
            temp_32_array[id_32_array] = getpdu_TC[3488 + id_32_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        Header_info.Timestamp[0] = temp_F32;

        for (uint16 id_32_array = 0; id_32_array < 4; id_32_array++)
        {
            temp_32_array[id_32_array] = getpdu_TC[3492 + id_32_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        Header_info.Timestamp[1] = temp_F32;
        return Header_info;
    }

    Freespace_point get_one_FSP(uint16 start_point)
    {
        Freespace_point one_Freespace_point;
        uint8 id_Uint8_array;
        //uint8 Pt_id;
        float32 temp_F32;
        uint8 temp_array[4];


        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_Freespace_point.FSP_Range = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + 4 + id_Uint8_array];
        } 
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_Freespace_point.FSP_Range_STD = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + 8 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_Freespace_point.FSP_Azimuth_Angle = temp_F32;

        return one_Freespace_point;
    }

    Trafic_light get_one_TFL(uint16 start_point)
    {
        Trafic_light one_TFL;
        uint8 id_Uint8_array;
       // uint8 TFL_id;
        float32 temp_F32;
        uint8 temp_array[4];

        one_TFL.TFL_Object_ID   = getpdu_TC[start_point];
        one_TFL.TFL_LightBox_ID = getpdu_TC[start_point+1];

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + 2 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_TFL.TFL_Long_Distance = temp_F32;        

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + 6 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_TFL.TFL_Long_Distance_STD = temp_F32;     

        one_TFL.TFL_Color = getpdu_TC[start_point+10];
        one_TFL.TFL_Mode  = getpdu_TC[start_point+11];

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_array[id_Uint8_array] = getpdu_TC[start_point + 12 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_array, sizeof(temp_array));
        one_TFL.TFL_Existence_Probability = temp_F32;     

        return one_TFL;
    }

    Path_pred get_Path_pred()
    {
        Path_pred Path_pred_all;
        uint8 id_Uint8_array;
       
        float32 temp_F32;
        float32 temp_ch32;
        float64 temp_F64;
        uint8 temp_32_array[4];
        uint8 temp_64_array[8];

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[ 4550 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        Path_pred_all.LS_Path_Prediction_VR_End = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[ 4554 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        Path_pred_all.LS_Path_Pred_Half_Width = temp_F32;                       

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[ 4558 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        Path_pred_all.LS_Path_Pred_Conf = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[ 4562 + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        Path_pred_all.LS_Path_Pred_C3 = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[ 4570 + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        Path_pred_all.LS_Path_Pred_C2 = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[ 4574 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        Path_pred_all.LS_Path_Pred_C1 = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[ 4578 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        Path_pred_all.LS_Path_Pred_C0 = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[ 4582 + id_Uint8_array];
        }
        memcpy(&temp_ch32, &temp_32_array, sizeof(temp_32_array));
        Path_pred_all.LS_Path_Pred_CRC = temp_ch32;

        if (getpdu_TC[4586] == 1)
        {
            Path_pred_all.LS_Path_Pred_Available = 1;
        }
        else
        {
            Path_pred_all.LS_Path_Pred_Available = 0;
        }

        if (getpdu_TC[4587] == 1)
        {
            Path_pred_all.LS_Is_Highway_Exit_Right = 1;
        }
        else
        {
            Path_pred_all.LS_Is_Highway_Exit_Right = 0;
        }

        if (getpdu_TC[4588] == 1)
        {
            Path_pred_all.LS_Is_Highway_Exit_Left = 1;
        }
        else
        {
            Path_pred_all.LS_Is_Highway_Exit_Left = 0;
        }

        if (getpdu_TC[4589] == 1)
        {
            Path_pred_all.LS_Is_Highway_Merge_Right = 1;
        }
        else
        {
            Path_pred_all.LS_Is_Highway_Merge_Right = 0;
        }

        if (getpdu_TC[4590] == 1)
        {
            Path_pred_all.LS_Is_Highway_Merge_Left = 1;
        }
        else
        {
            Path_pred_all.LS_Is_Highway_Merge_Left = 0;
        }

        if (getpdu_TC[4591] == 1)
        {
            Path_pred_all.LS_Is_Exit_Right_Valid = 1;
        }
        else
        {
            Path_pred_all.LS_Is_Exit_Right_Valid = 0;
        }

        if (getpdu_TC[4592] == 1)
        {
            Path_pred_all.LS_Is_Exit_Left_Valid = 1;
        }
        else
        {
            Path_pred_all.LS_Is_Exit_Left_Valid = 0;
        }

        if (getpdu_TC[4593] == 1)
        {
            Path_pred_all.LS_Is_Merge_Right_Valid = 1;
        }
        else
        {
            Path_pred_all.LS_Is_Merge_Right_Valid = 0;
        }

        if (getpdu_TC[4594] == 1)
        {
            Path_pred_all.LS_Is_Merge_Left_Valid = 1;
        }
        else
        {
            Path_pred_all.LS_Is_Merge_Left_Valid = 0;
        }

        if (getpdu_TC[4595] == 1)
        {
            Path_pred_all.LS_Exit_Merge_Available = 1;
        }
        else
        {
            Path_pred_all.LS_Exit_Merge_Available = 0;
        }

        if (getpdu_TC[4596] == 1)
        {
            Path_pred_all.LS_CA_Is_Construction_Area = 1;
        }
        else
        {
            Path_pred_all.LS_CA_Is_Construction_Area = 0;
        }

        return Path_pred_all;
    }

    Road_Edge get_one_Edge(uint16 start_point)
    {
        Road_Edge one_Edge;
        uint8 id_Uint8_array;

        float32 temp_F32;
        float32 temp_ch32;
        float64 temp_F64;
        uint8 temp_32_array[4];
        uint8 temp_64_array[8];


        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Edge.LS_Road_Edge_Line_C0 = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Edge.LS_Road_Edge_Line_C0_STD = temp_F32;        

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 8 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Edge.LS_Road_Edge_Line_C1_STD = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 12 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Edge.LS_Road_Edge_Line_C0 = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point + 16  + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Edge.LS_Road_Edge_Line_C2 = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 24 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Edge.LS_Road_Edge_Line_C2_STD = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point + 28  + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Edge.LS_Road_Edge_Line_C3_STD = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point + 36  + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Edge.LS_Road_Edge_Line_C3 = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 44 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Edge.LS_Road_Edge_Measured_VR_End = temp_F32;

        one_Edge.LS_Road_Edge_Type=getpdu_TC[start_point + 48];

        return one_Edge;
    }

    Host_Line get_one_Host_line(uint16 start_point)
    {
        Host_Line one_Host_line;
        uint8 id_Uint8_array;

        float32 temp_F32;
        float32 temp_ch32;
        float64 temp_F64;
        uint8 temp_32_array[4];
        uint8 temp_64_array[8];

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point  + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Host_line.LS_Host_Line_C3 = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point + 8  + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Host_line.LS_Host_Line_C3_STD = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point + 16 + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Host_line.LS_Host_Line_C2_STD = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point + 24 + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Host_line.LS_Host_Line_C2 = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 32 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Host_line.LS_Host_Line_C1 = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 36 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Host_line.LS_Host_Line_C1_STD = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 40 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Host_line.LS_Host_Line_C0_STD = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 44 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Host_line.LS_Host_Line_C0 = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 48 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Host_line.LS_Host_Measured_VR_End = temp_F32;

        one_Host_line.LS_Host_Type_Classification=getpdu_TC[start_point + 52];

        return one_Host_line;
    }

    Adjacent_Line get_one_Adjacent_line(uint16 start_point)
    {
        Adjacent_Line one_Adjacent_line;
        uint8 id_Uint8_array;

        float32 temp_F32;
        float32 temp_ch32;
        float64 temp_F64;
        uint8 temp_32_array[4];
        uint8 temp_64_array[8];

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point  + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Adjacent_line.LS_Adjacent_Line_C0_STD = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 4 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Adjacent_line.LS_Adjacent_Line_C0 = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 8 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Adjacent_line.LS_Adjacent_Line_C1 = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 12 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Adjacent_line.LS_Adjacent_Line_C1_STD = temp_F32;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point + 16 + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Adjacent_line.LS_Adjacent_Line_C2_STD = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point + 24 + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Adjacent_line.LS_Adjacent_Line_C2 = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point  + 32 + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Adjacent_line.LS_Adjacent_Line_C3 = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 8; id_Uint8_array++)
        {
            temp_64_array[id_Uint8_array] = getpdu_TC[start_point + 40  + id_Uint8_array];
        }
        memcpy(&temp_F64, &temp_64_array, sizeof(temp_64_array));
        one_Adjacent_line.LS_Adjacent_Line_C3_STD = temp_F64;

        for (id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
        {
            temp_32_array[id_Uint8_array] = getpdu_TC[start_point + 48 + id_Uint8_array];
        }
        memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
        one_Adjacent_line.LS_Adjacent_Measured_VR_End = temp_F32;

        one_Adjacent_line.LS_Adjacent_Type_Classification=getpdu_TC[start_point + 52];


        return one_Adjacent_line;
    }



    virtual void Process(std::istream &stream) override
    {

        ICV_LOG_INFO << "buffer size----------------------------------------------------:";

        ICV_LOG_INFO << "buffer size:" << _buffer.size();
        
        //if (_buffer.size()%72 == 0)
        //{
        //ICV_LOG_INFO<<"JK";
        // boost::asio::streambuf::const_buffers_type cbt = _buffer.data();
        // std::string request_data(boost::asio::buffers_begin(cbt), boost::asio::buffers_end(cbt));
        // //
        // std::cout << request_data << std::endl;
        // stream>>packet;
        while (!stream.eof())
        {

            stream.read(getpdu_TC, 5888);
            OBJ_EQ obj_EQ[40];
            OBJ_LRR obj_LRR[32];
            OBJ_FSDA obj_FSDA[32];
            OBJ_RSDA obj_RSDA[32];

            if (getpdu_TC[0] == 3 && getpdu_TC[1] == 4)
            {
                /*             for (uint16 j = 0; j < 40; j++)
            {
                obj_EQ[j] = get_one_object(j * 85);
                obj_t.setvalue(obj_EQ[j]);
                icvPublish("obj_eq", &obj_t);
            } */

                for (uint16 j = 0; j < 40; j++)
                {
                    obj_EQ[j] = get_one_object(j * 85);
                }
/*                 OBJ_40_EQ obj_EQs_40;
                for (int i = 0; i < 40; i++)
                {
                    obj_EQs_40.obj_s[i] = obj_EQ[i];
                }
                obj_t.setvalue(obj_EQs_40);
                icvPublish("obj_eq", &obj_t); */
                OBJ_40_EQ obj_EQs_40;
                for (int i = 0; i < 40; i++)
                {
                    obj_EQs_40.obj_s[i] = obj_EQ[i];
                }
                obj_t.setvalue(obj_EQs_40);
                //ICV_LOG_INFO << "obj_EQs_40.obj_s[0]  OBJ_ID :" << obj_EQs_40.obj_s[0].OBJ_ID;
/*                 ICV_LOG_INFO << "obj_EQs_40.obj_s[0]  OBJ_Height :" << obj_EQs_40.obj_s[0].OBJ_Height;
                ICV_LOG_INFO << "obj_EQs_40.obj_s[0]  OBJ_Width :" << obj_EQs_40.obj_s[0].OBJ_Width;
                ICV_LOG_INFO << "obj_EQs_40.obj_s[0]  OBJ_Length :" << obj_EQs_40.obj_s[0].OBJ_Length; */
           /*     ICV_LOG_INFO << "obj_EQs_40.obj_s[0]  OBJ_Long_Distance :" << obj_EQs_40.obj_s[0].OBJ_Long_Distance;
                ICV_LOG_INFO << "obj_EQs_40.obj_s[0]  OBJ_Lat_Distance :" << obj_EQs_40.obj_s[0].OBJ_Lat_Distance; */

               // ICV_LOG_INFO << "obj_EQs_40.obj_s[1]  OBJ_ID :" << obj_EQs_40.obj_s[1].OBJ_ID;
/*                 ICV_LOG_INFO << "obj_EQs_40.obj_s[1]  OBJ_Height :" << obj_EQs_40.obj_s[1].OBJ_Height;
                ICV_LOG_INFO << "obj_EQs_40.obj_s[1]  OBJ_Width :" << obj_EQs_40.obj_s[1].OBJ_Width;
                ICV_LOG_INFO << "obj_EQs_40.obj_s[1]  OBJ_Length :" << obj_EQs_40.obj_s[1].OBJ_Length; */
           /*     ICV_LOG_INFO << "obj_EQs_40.obj_s[1]  OBJ_Long_Distance :" <<obj_EQs_40.obj_s[1].OBJ_Long_Distance;
                ICV_LOG_INFO << "obj_EQs_40.obj_s[1]  OBJ_Lat_Distance :" << obj_EQs_40.obj_s[1].OBJ_Lat_Distance; 

                ICV_LOG_INFO << "obj_EQs_40.obj_s[2]  OBJ_Long_Distance :" <<obj_EQs_40.obj_s[2].OBJ_Long_Distance;
                ICV_LOG_INFO << "obj_EQs_40.obj_s[2]  OBJ_Lat_Distance :" << obj_EQs_40.obj_s[2].OBJ_Lat_Distance; */

                ICV_LOG_INFO << "-------------------------------------------------------------------------------------:";
                icvPublish("obj_eq", &obj_t);
                /********************************************************************/

                Sematic_lines_info SLs_info;
                SLs_info=get_Sematiclines_info();
                sls_inf.setvalue(SLs_info);
             /*    ICV_LOG_INFO << "Slines_info  SLD_Num_Of_Lanes_Close_Left :" <<(int) SLs_info.SLD_Num_Of_Lanes_Close_Left;
                ICV_LOG_INFO << "Slines_info  SLD_Num_Of_Lanes_Close_Right :" << (int)SLs_info.SLD_Num_Of_Lanes_Close_Right; */

                icvPublish("sls_information",&sls_inf);
                /********************************************************************/
                stHeader st_Header;
                st_Header=get_stHeader();
                
                /********************************************************************/
                FSP_72 FS_72_pt;
                for (uint16 j=0; j<72 ; j++)
                {
                    FS_72_pt.Freespace_pt[j]=get_one_FSP(3510+j*12);
                }
                
                /********************************************************************/
                TFL_10 TF_10_Light;

                //TFL_Number_Of_Objects=getpdu_TC[4380];

                for (uint16 j=0; j<10 ; j++)
                {
                    TF_10_Light.TF_Lt[j]=get_one_TFL(4381 +j*16);
                }

                TF_10_Light.TFL_num=getpdu_TC[4380]; 

                tfl.setvalue(TF_10_Light);  

/*                 ICV_LOG_INFO << "TF_10_Light.TF_Lt[0]  TFL_Color :" << TF_10_Light.TF_Lt[0].TFL_Color;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[0]  TFL_Mode :" << TF_10_Light.TF_Lt[0].TFL_Mode;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[0]  TFL_Long_Distance :" << TF_10_Light.TF_Lt[0].TFL_Long_Distance;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[0]  TFL_Object_ID :" << TF_10_Light.TF_Lt[0].TFL_Object_ID;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[0]  TFL_LightBox_ID :" << TF_10_Light.TF_Lt[0].TFL_LightBox_ID;

                ICV_LOG_INFO << "TF_10_Light.TF_Lt[1]  TFL_Color :" << TF_10_Light.TF_Lt[1].TFL_Color;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[1]  TFL_Mode :" << TF_10_Light.TF_Lt[1].TFL_Mode;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[1]  TFL_Long_Distance :" << TF_10_Light.TF_Lt[1].TFL_Long_Distance;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[1]  TFL_Object_ID :" << TF_10_Light.TF_Lt[1].TFL_Object_ID;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[1]  TFL_LightBox_ID :" << TF_10_Light.TF_Lt[1].TFL_LightBox_ID;

                ICV_LOG_INFO << "TF_10_Light.TF_Lt[2]  TFL_Color :" << TF_10_Light.TF_Lt[2].TFL_Color;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[2]  TFL_Mode :" << TF_10_Light.TF_Lt[2].TFL_Mode;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[2]  TFL_Long_Distance :" << TF_10_Light.TF_Lt[2].TFL_Long_Distance;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[2]  TFL_Object_ID :" << TF_10_Light.TF_Lt[2].TFL_Object_ID;
                ICV_LOG_INFO << "TF_10_Light.TF_Lt[2]  TFL_LightBox_ID :" << TF_10_Light.TF_Lt[2].TFL_LightBox_ID;     */            

                icvPublish("tfl_eq",&tfl);

                /********************************************************************/
                Path_pred Path_prediction;
                Path_prediction=get_Path_pred();

                path_pre.setvalue(Path_prediction);

       /*          ICV_LOG_INFO << "Path_prediction.LS_Path_Pred_C0 :" << Path_prediction.LS_Path_Pred_C0;  
                ICV_LOG_INFO << "Path_prediction.LS_Path_Pred_C1 :" << Path_prediction.LS_Path_Pred_C1;
                ICV_LOG_INFO << "Path_prediction.LS_Path_Pred_C2 :" << Path_prediction.LS_Path_Pred_C2;
                ICV_LOG_INFO << "Path_prediction.LS_Path_Pred_C3 :" << Path_prediction.LS_Path_Pred_C3;  
                ICV_LOG_INFO << "Path_prediction.LS_Path_Pred_Half_Width :" << Path_prediction.LS_Path_Pred_Half_Width;   */

                icvPublish("path_prediction",&path_pre);

                /********************************************************************/

                Road_Edge_4 Road_4_Edge;

                for (uint16 j=0; j<4 ; j++)
                {
                    Road_4_Edge.Rd_Eg[j]=get_one_Edge(4600+j*49);
                }
                
                uint32 temp_ch32;
                uint8 temp_32_array[4];
                for (uint16 id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
                {
                    temp_32_array[id_Uint8_array] = getpdu_TC[4796+id_Uint8_array];
                }
                memcpy(&temp_ch32, &temp_32_array, sizeof(temp_32_array));
                Road_4_Edge.LS_Rd_Eg_CRC = temp_ch32;

                /********************************************************************/

                Host_Line_2 Host_2_line;
                for (uint16 j = 0; j < 2; j++)
                {
                    Host_2_line.Ht_Ln[j]=get_one_Host_line(4800 + j *53);
                }

                hst_ln.setvalue(Host_2_line);

             //   ICV_LOG_INFO << "Host_2_line.LS_Host_Estimate_Width :" << Host_2_line.LS_Host_Est_Width;  

                icvPublish("Host_lines", &hst_ln);

                /********************************************************************/

                Adjacent_Line_4 Adjacent_4_line;
                for (uint16 j = 0; j < 4; j++)
                {
                    Adjacent_4_line.Adj_Ln[j]=get_one_Adjacent_line(4910 + j *53);
                }

                float32 temp_F32;
                //uint8 temp_32_array[4];
                for (uint16 id_Uint8_array = 0; id_Uint8_array < 4; id_Uint8_array++)
                {
                    temp_32_array[id_Uint8_array] = getpdu_TC[5122+id_Uint8_array];
                }
                memcpy(&temp_F32, &temp_32_array, sizeof(temp_32_array));
                Host_2_line.LS_Host_Est_Width = temp_F32;               

                
            }
            else if (getpdu_TC[0] == 9 && getpdu_TC[1] == 10)
            {
                for (uint16 j = 0; j < 32; j++)
                {
                    obj_LRR[j] = get_one_object_LRR(j * 12);
                }
                for (uint16 j = 0; j < 32; j++)
                {
                    obj_FSDA[j] = get_one_object_FSDA(j * 12);
                }
                for (uint16 j = 0; j < 32; j++)
                {
                    obj_RSDA[j] = get_one_object_RSDA(j * 12);
                }

                OBJ_32_LRR obj_LRRs_32;

                for (int i = 0; i < 32; i++)
                {
                    obj_LRRs_32.objs_LRR[i] = obj_LRR[i];
                }
                obj_l.setvalue(obj_LRRs_32);

                OBJ_32_RSDA obj_RSDAs_32;

                for (int i = 0; i < 32; i++)
                {
                    obj_RSDAs_32.objs_RSDA[i] = obj_RSDA[i];
                }
                obj_r.setvalue(obj_RSDAs_32);

                OBJ_32_FSDA obj_FSDAs_32;

                for (int i = 0; i < 32; i++)
                {
                    obj_FSDAs_32.objs_FSDA[i] = obj_FSDA[i];
                }
                obj_f.setvalue(obj_FSDAs_32);

/*                 icvPublish("obj_lrr", &obj_l);
                icvPublish("obj_rsda", &obj_r);
                icvPublish("obj_fsda", &obj_f); */
           /*     for (uint16 j = 0; j < 32; j++)
                {
                    if ((abs(obj_LRRs_32.objs_LRR[j].lrr_dx) < 0.01) && (abs(obj_LRRs_32.objs_LRR[j].lrr_dy) < 0.1))
                    {
                        ICV_LOG_INFO << "--------------------LRR----------------RIEN-------------------------------------------------:";
                    }
                    else
                    {
                        ICV_LOG_INFO << "OBJ_LRR_ index  :" << j;
                        ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[j]    LRR_xPosition :" << obj_LRRs_32.objs_LRR[j].lrr_dx;
                        ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[j]    LRR_yPosition :" << obj_LRRs_32.objs_LRR[j].lrr_dy;
                        ICV_LOG_INFO << "-------------------------------------------------------------------------------------:";
                    }
                }
*/
    /*            ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[0]  LRR_dx :" << obj_LRRs_32.objs_LRR[0].lrr_dx;
                ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[0]  LRR_dy :" << obj_LRRs_32.objs_LRR[0].lrr_dy;

                ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[1]  LRR_dx :" << obj_LRRs_32.objs_LRR[1].lrr_dx;
                ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[1]  LRR_dy :" << obj_LRRs_32.objs_LRR[1].lrr_dy;

                ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[2]  LRR_dx :" << obj_LRRs_32.objs_LRR[2].lrr_dx;
                ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[2]  LRR_dy :" << obj_LRRs_32.objs_LRR[2].lrr_dy;

                ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[3]  LRR_dx :" << obj_LRRs_32.objs_LRR[3].lrr_dx;
                ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[3]  LRR_dy :" << obj_LRRs_32.objs_LRR[3].lrr_dy;

                ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[4]  LRR_dx :" << obj_LRRs_32.objs_LRR[4].lrr_dx;
                ICV_LOG_INFO << "obj_LRRs_32.objs_LRR[4]  LRR_dy :" << obj_LRRs_32.objs_LRR[4].lrr_dy;*/
                icvPublish("obj_lrr", &obj_l);
                ICV_LOG_INFO << "-------------------------------------------------------------------------------------:";

                for (uint16 j=0; j<32;j++)
                {
                        if ( (  abs(  obj_RSDAs_32.objs_RSDA[j].rsda_dx ) < 0.01 )  &&  ( abs ( obj_RSDAs_32.objs_RSDA[j].rsda_dy ) < 0.01 )  ) 
                        {
                          //       ICV_LOG_INFO << "----------------RSDA--------------------RIEN-------------------------------------------------:";
                        }
                        else
                        {
                            ICV_LOG_INFO << "OBJ_RSDA index  :" << j;
                            ICV_LOG_INFO << "obj_RSDAs_32.objs_RSDA[j]    RSDA_xPosition :" << obj_RSDAs_32.objs_RSDA[j].rsda_dx;
                            ICV_LOG_INFO << "obj_RSDAs_32.objs_RSDA[j]    RSDA_yPosition :" << obj_RSDAs_32.objs_RSDA[j].rsda_dy;
                            ICV_LOG_INFO << "-------------------------------------------------------------------------------------:";
                        }
                        
                }

             /*   ICV_LOG_INFO << "obj_RSDAs_32.objs_RSDA[1]    RSDA_xPosition :" << obj_RSDAs_32.objs_RSDA[1].rsda_dx;
                ICV_LOG_INFO << "obj_RSDAs_32.objs_RSDA[1]    RSDA_yPosition :" << obj_RSDAs_32.objs_RSDA[1].rsda_dy;

                ICV_LOG_INFO << "obj_RSDAs_32.objs_RSDA[2]    RSDA_xPosition :" << obj_RSDAs_32.objs_RSDA[2].rsda_dx;
                ICV_LOG_INFO << "obj_RSDAs_32.objs_RSDA[2]    RSDA_yPosition :" << obj_RSDAs_32.objs_RSDA[2].rsda_dy;*/

                icvPublish("obj_rsda", &obj_r);

                for (uint16 j=0; j<32;j++)
                {   
                      //  if ( (obj_RSDAs_32.objs_RSDA[j].rsda_dx =0 )  &&  (obj_RSDAs_32.objs_RSDA[j].rsda_dy=0)  ) 
                   if ( (  abs(  obj_FSDAs_32.objs_FSDA[j].fsda_dx ) < 0.01 )  &&  ( abs ( obj_FSDAs_32.objs_FSDA[j].fsda_dy ) < 0.01 )  ) 
                        {
                          //   ICV_LOG_INFO << "---------------------FSDA---------------RIEN-------------------------------------------------:";
                        }
                        else
                        {
                            ICV_LOG_INFO << "OBJ_FSDA index  :" << j;
                            ICV_LOG_INFO << "obj_FSDAs_32.objs_FSDA[j]    FSDA_xPosition :" << obj_FSDAs_32.objs_FSDA[j].fsda_dx;
                            ICV_LOG_INFO << "obj_FSDAs_32.objs_FSDA[j]    FSDA_yPosition :" << obj_FSDAs_32.objs_FSDA[j].fsda_dy;
                            ICV_LOG_INFO << "-------------------------------------------------------------------------------------:";
                        }
                        
                }
/*
                ICV_LOG_INFO << "obj_FSDAs_32.objs_FSDA[0]    FSDA_xPosition :" << obj_FSDAs_32.objs_FSDA[0].fsda_dx;
                ICV_LOG_INFO << "obj_FSDAs_32.objs_FSDA[0]    FSDA_yPosition :" << obj_FSDAs_32.objs_FSDA[0].fsda_dy;

                ICV_LOG_INFO << "obj_FSDAs_32.objs_FSDA[1]    FSDA_xPosition :" << obj_FSDAs_32.objs_FSDA[1].fsda_dx;
                ICV_LOG_INFO << "obj_FSDAs_32.objs_FSDA[1]    FSDA_yPosition :" << obj_FSDAs_32.objs_FSDA[1].fsda_dy;

                ICV_LOG_INFO << "obj_FSDAs_32.objs_FSDA[2]    FSDA_xPosition :" << obj_FSDAs_32.objs_FSDA[2].fsda_dx;
                ICV_LOG_INFO << "obj_FSDAs_32.objs_FSDA[2]    FSDA_yPosition :" << obj_FSDAs_32.objs_FSDA[2].fsda_dy; */
                icvPublish("obj_fsda", &obj_f);
                ICV_LOG_INFO << "-------------------------------------------------------------------------------------:";
                /* code */
            }
            // msg_imu_t.setvalue(msg_imu);
            // msg_fix_t.setvalue(msg_fix);
            // msg_vel_t.setvalue(msg_vel);
            // msg_odom_t.setvalue(msg_odom);
            // icvPublish("IMU_data",&msg_imu_t);
            // icvPublish("msg_fix",&msg_fix_t);
            // icvPublish("msg_vel",&msg_vel_t);
            // icvPublish("msg_odom",&msg_odom_t);
            /*Deprecated
        Send_Out(msg_imu_t,0);
        Send_Out(msg_fix_t,1);
        Send_Out(msg_vel_t,2);
        Send_Out(msg_odom_t,3);
*/

            //}
        }
    }

private:
    NavSatFix msg_fix;
    TwistWithCovarianceStamped msg_vel;
    Imu msg_imu;
    Odometry msg_odom;
    char getpdu_TC[5888];
    //uint8 getpdu_TC[5888u];
    data::icvStructureData<OBJ_40_EQ> obj_t;
    data::icvStructureData<OBJ_32_LRR> obj_l;
    data::icvStructureData<OBJ_32_FSDA> obj_f;
    data::icvStructureData<OBJ_32_RSDA> obj_r;

    data::icvStructureData<TFL_10> tfl;

    data::icvStructureData<Path_pred> path_pre;
    data::icvStructureData<Host_Line_2> hst_ln;
    data::icvStructureData<Sematic_lines_info> sls_inf;


    icvNavSatFix msg_fix_t;
    icvTwistWithCovarianceStamped msg_vel_t;
    icvImu IMU_DATA;
    icvOdometry msg_odom_t;

    SCU_IPC_0x174 ANGL_DATA;
    CanFrame_SCU_IPC_1_0x174 angl_data;
    SCU_IPC_0x175 WHEL_DATA;
    CanFrame_SCU_IPC_2_0x175 whel_data;
    SCU_IPC_0x176 SPED_DATA;
    CanFrame_SCU_IPC_3_0x176 sped_data;
    SCU_IPC_0x177 YAW_DATA;
    CanFrame_SCU_IPC_4_0x177 yaw_data;
    SCU_IPC_0x178 LAMP_DATA;
    CanFrame_SCU_IPC_5_0x178 lamp_data;
    SCU_IPC_0x179 GEAR_DATA;
    CanFrame_SCU_IPC_6_0x179 gear_data;
    SCU_IPC_0x17A WARN_DATA;
    CanFrame_SCU_IPC_7_0x17A warn_data;

    bool send_ = true;

    // Variables
    std::string frame_id = "gps";
    std::string frame_id_vel = "utm";
    Imu_Packet *packet;
};
ICV_REGISTER_FUNCTION(Tricore)

#endif //

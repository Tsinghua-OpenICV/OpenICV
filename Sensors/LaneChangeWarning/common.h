#ifndef _COMMEN_H_
#define _COMMEN_H_

#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/structure/structureFusionData.h"
#include "OpenICV/structure/structureLaneData.h"
#include <iostream>

#define CAR_WIDTH 2.97
#define LANE_WIDTH 3.5
#define LANE_BOUNDARY_WIDTH 0.2
#define PI 3.1415926          

class OBSVector3d 
{
    public:
        float _x;
        float _y;
        float _z;
};

struct TARGET_INFO{ // 单个目标信息
    int _id;
    OBSVector3d _obs_position;
    OBSVector3d _velocity;
    OBSVector3d _acceleration;
    float _dist;
    float _safe_dist;
    int repeat_num ;
};

struct TARGETS_INFO{ // 多个目标信息
    double _header;
    std::vector<TARGET_INFO> _target_info;
    bool _warning_flag = 0;  
    int _warning_id;     
};


bool compare(TARGET_INFO a,TARGET_INFO b);
void Select_Min_TTC(TARGETS_INFO &targets);

struct lanecoeff{ //车道线方程系数
    float a ;
    float b ;
    float c ;
    float d ;
};

class CCommon{
    public:
    CCommon();
    ~CCommon(){}
    
    void Target_Classified(TARGET_INFO target); //目标分区域存储到_Vec_TargetsInfo_Classfied中
    void Update_Target_Status(TrackArray msg); //根据曲率半径等转换到实际的位置 速度 加速度
  
    void LaneParam_set(ld_Frame msg);
    void CarParam_set(OBSVector3d VehSpeed,float YawVelocity, OBSVector3d VehAcceleration);

    public:
    TARGETS_INFO _TargetsInfo_Raw;  //RAW 
    std::vector<TARGETS_INFO> _Vec_TargetsInfo_Classfied; //分类后的目标信息

    OBSVector3d _VehSpeed; //自车车速
    float _YawVelocity; //横摆角速度
    OBSVector3d _VehAcceleration; //自车加速度
    float _CurRadRoad;  //曲率半径
    float _LaneDepValue; //偏离车道中心距离
    float _LaneWidth=3.5;  //车道宽度
    float _LaneAngle;  //道路与车中线夹角

    std::vector<lanecoeff> lane_coeff ;//车道线方程系数
    bool left_lane_flag ;    // 车道存在标签
    bool right_lane_flag ;
    std::vector<float> hist_left_lane_flag ;
    std::vector<float> hist_right_lane_flag ;
};

class CFunc_lcw{
    public:
    CFunc_lcw(){}
    ~CFunc_lcw(){}
    
    void init(TARGETS_INFO Target_Frame,std::vector<TARGETS_INFO> Target_Classfied); //初始化类内参数

    void CarParam_set(OBSVector3d VehSpeed,float YawVelocity,OBSVector3d VehAcceleration);

    void TargetLaneFront_SafeDist(TARGETS_INFO &_Targets); // 距离小于安全距离的所有危险目标
    void TargetLaneRear_SafeDist(TARGETS_INFO &_Targets);  // 距离小于安全距离的所有危险目标
    void SelfLaneFront_SafeDist(TARGETS_INFO &_Targets);   // 距离小于安全距离的所有危险目标


    int Find_Risk_Target(TARGETS_INFO _Targets);     //return 最危险的目标id

    void Run();
    public:
    OBSVector3d _VehSpeed; //自车车速
    float _YawVelocity; //横摆角速度
    OBSVector3d _VehAcceleration; //自车加速度
    public:
    TARGETS_INFO _TargetInfo_Raw;  //RAW 
    std::vector<TARGET_INFO> _Vec_TargetInfo_Classfied; //分类后的目标信息
    TARGETS_INFO _TargetInfo_TLFront;  //目标车道前车所有目标
    TARGETS_INFO _TargetInfo_TLRear;   //目标车道后车所有目标
    TARGETS_INFO _TargetInfo_SLFront;  //自车道前车所有目标
    TARGETS_INFO _TargetInfo_Danger_TLFront;  //目标车道前车所有危险目标
    TARGETS_INFO _TargetInfo_Danger_TLRear;   //目标车道后车所有危险目标
    TARGETS_INFO _TargetInfo_Danger_SLFront;  //自车道前车所有危险目标
    
};

#endif

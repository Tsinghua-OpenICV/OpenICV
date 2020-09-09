#include "common.h"
#include <cmath>

#define time_lanechange 1.5  // 经验换道时间
#define time_reaction  1.8  // 驾驶员反应时间
#define distance_safe_heart  3  // 紧急制动结束最小心理安全距离
#define acceleration_max 7  // 最大制动减速度
#define acceleration_adjust 2  //跟车调整减速度


void CFunc_lcw::CarParam_set(OBSVector3d VehSpeed,float YawVelocity,OBSVector3d VehAcceleration){
     _VehSpeed = VehSpeed; //自车车速
     _YawVelocity = YawVelocity; //横摆角速度
     _VehAcceleration = VehAcceleration;//自车加速度
}

void CFunc_lcw::SelfLaneFront_SafeDist(TARGETS_INFO &_Targets) // 自车道前车安全距离
{
    _TargetInfo_Danger_SLFront._target_info.clear();
    int targets_num=_Targets._target_info.size() ; // 目标车辆数
    float distance_safe_min [targets_num] ;

    for (int ii=0 ; ii<targets_num ; ii++) // 计算每个目标的最小安全换道距离
    {
         distance_safe_min[ii]=distance_safe_heart+(_VehSpeed._x*time_lanechange+0.5*_VehAcceleration._x*pow(time_lanechange,2))\
        -_Targets._target_info[ii]._velocity._x*time_lanechange+0.5*acceleration_max*pow(time_lanechange,2) ;

        if (_Targets._target_info[ii]._dist< distance_safe_min[ii]) // 判断， 如果当前距离小于最小安全距离，则添加到危险目标容器内，记录安全距离
        {
            _Targets._target_info[ii]._safe_dist = distance_safe_min[ii] ;          
            _TargetInfo_Danger_SLFront._target_info.push_back(_Targets._target_info[ii]);
        }
    }

    int tmp_id = Find_Risk_Target(_TargetInfo_Danger_SLFront);
    if(tmp_id!=-1){
        _Targets._warning_id =tmp_id; 
        _Targets._warning_flag = 1;
    }else{
        _Targets._warning_flag = 0;
    }

}

void CFunc_lcw::TargetLaneFront_SafeDist(TARGETS_INFO &_Targets) // 目标车道前车安全距离
{
    _TargetInfo_Danger_TLFront._target_info.clear();
    int targets_num=_Targets._target_info.size() ;
    float distance_safe_min[targets_num] ;
    for (int ii=0 ; ii<targets_num ;ii++)
    {
         if (_Targets._target_info[ii]._velocity._x < 0)
        {           
            distance_safe_min[ii] = 100;
 
            _Targets._target_info[ii]._safe_dist = distance_safe_min[ii] ;
            if (_Targets._target_info[ii]._dist < 50)
            {
                _TargetInfo_Danger_TLFront._target_info.push_back(_Targets._target_info[ii]);
            }
            continue;
        }
        // 换道完成时刻目标车辆前车车速大于自车车速， 此时即为最小相对距离， 该距离应大于最小跟车距离
        if ((_Targets._target_info[ii]._velocity._x+_Targets._target_info[ii]._acceleration._x*time_lanechange)\
        >(_VehSpeed._x+_VehAcceleration._x*time_lanechange))
        {
            distance_safe_min[ii]=distance_safe_heart\
                +(_VehSpeed._x*time_lanechange+0.5*_VehAcceleration._x*pow(time_lanechange,2)) \
                -(_Targets._target_info[ii]._velocity._x*time_lanechange+0.5*_Targets._target_info[ii]._acceleration._x*pow(time_lanechange,2)) ;

        }
        // 换道完成时刻目标车辆车速大于自车车速，最危险情况为前车紧急制动，最小距离为两车制动静止时的相对距离，该距离应大于最小心理安全距离
        else if ((_Targets._target_info[ii]._velocity._x+_Targets._target_info[ii]._acceleration._x*time_lanechange) <= \
        (_VehSpeed._x+_VehAcceleration._x*time_lanechange))
        {
            distance_safe_min[ii]=distance_safe_heart +(_VehSpeed._x*time_lanechange+0.5*_VehAcceleration._x*pow(time_lanechange,2)) \
                -(_Targets._target_info[ii]._velocity._x*time_lanechange+0.5*_Targets._target_info[ii]._acceleration._x*pow(time_lanechange,2)) \
                +((_VehSpeed._x+_VehAcceleration._x*time_lanechange)*time_reaction+pow(((_VehSpeed._x+_VehAcceleration._x*time_lanechange)),2)/2/acceleration_max) \
                -pow(((_Targets._target_info[ii]._velocity._x+_Targets._target_info[ii]._acceleration._x*time_lanechange)),2)/2/acceleration_max ;      
        } 

        float ttc = fabs(_Targets._target_info[ii]._dist/(_Targets._target_info[ii]._velocity._x-_VehSpeed._x));
        if (_Targets._target_info[ii]._dist < distance_safe_min[ii]) // 判断， 如果当前距离小于最小安全距离，则添加到危险目标容器内
        {
            if (_Targets._target_info[ii]._dist < 30 || ttc < 7.5)
            {
                _Targets._target_info[ii]._safe_dist = distance_safe_min[ii] ;
                _TargetInfo_Danger_TLFront._target_info.push_back(_Targets._target_info[ii]);
            }      
        }
    }
    int tmp_id = Find_Risk_Target(_TargetInfo_Danger_TLFront);

    if(tmp_id!=-1){
        _Targets._warning_id =tmp_id; 
        _Targets._warning_flag = 1;
    }else{
        _Targets._warning_flag = 0;
    }
}



void CFunc_lcw::TargetLaneRear_SafeDist(TARGETS_INFO &_Targets) // 目标车辆后车
{
    _TargetInfo_Danger_TLRear._target_info.clear();
    int targets_num=_Targets._target_info.size() ;
    float distance_safe_min[targets_num] ;
    for (int ii=0 ; ii<targets_num ;ii++)
    {
        // 换道完成时刻目标后车车速小于自车，此时即为最小相对距离，该距离应大于最小跟车距离
        if ((_Targets._target_info[ii]._velocity._x+_Targets._target_info[ii]._acceleration._x*time_lanechange)< \
            (_VehSpeed._x+_VehAcceleration._x*time_lanechange))
            {
                distance_safe_min[ii]=distance_safe_heart \
                    -(_VehSpeed._x*time_lanechange+0.5*_VehAcceleration._x*pow(time_lanechange,2)) \
                    +(_Targets._target_info[ii]._velocity._x*time_lanechange+0.5*_Targets._target_info[ii]._acceleration._x*pow(time_lanechange,2)) ;
            }
        
        // 换道完成时刻目标后车车速大于自车，后车开始减速调整，两车车速相等时为最小相对距离，该距离应大于最小跟车距离
        else if ((_Targets._target_info[ii]._velocity._x+_Targets._target_info[ii]._acceleration._x*time_lanechange) >= \
            (_VehSpeed._x+_VehAcceleration._x*time_lanechange))
            {
                float time_brake ;
                time_brake=((_Targets._target_info[ii]._velocity._x+_Targets._target_info[ii]._acceleration._x*time_lanechange) \
                -(_VehSpeed._x+_VehAcceleration._x*time_lanechange ))/acceleration_adjust ;

                distance_safe_min[ii]=distance_safe_heart \
                    -((_VehSpeed._x*time_lanechange + 0.5*_VehAcceleration._x*pow(time_lanechange ,2)) + (_VehSpeed._x + _VehAcceleration._x * time_lanechange)*(time_brake+time_reaction))\
                    +(_Targets._target_info[ii]._velocity._x*time_lanechange + 0.5*_Targets._target_info[ii]._acceleration._x*pow(time_lanechange,2)) \               
                    + (_Targets._target_info[ii]._velocity._x+_Targets._target_info[ii]._acceleration._x*time_lanechange)*(time_brake + time_reaction)\
                    -  0.5*acceleration_adjust*pow(time_brake,2) ;

            }
        float ttc = _Targets._target_info[ii]._dist /(_Targets._target_info[ii]._velocity._x-_VehSpeed._x);
 
        if (_Targets._target_info[ii]._dist < distance_safe_min[ii]) // 判断， 如果当前距离小于最小安全距离，则添加到危险目标容器内
            {
            if (_Targets._target_info[ii]._dist < 30 || ttc < 7.5)
            {
                _Targets._target_info[ii]._safe_dist = distance_safe_min[ii] ;
                _TargetInfo_Danger_TLRear._target_info.push_back(_Targets._target_info[ii]);
            }
                
            }
        
    }
    int tmp_id = Find_Risk_Target(_TargetInfo_Danger_TLRear);
    if(tmp_id!=-1){
        _Targets._warning_id =tmp_id; 
        _Targets._warning_flag = 1;
    }else{
        _Targets._warning_flag = 0;
    }
}


int CFunc_lcw::Find_Risk_Target(TARGETS_INFO _Targets) // 返回最危险的目标id, 最危险车辆为纵向最近的目标
{
    int danger_car = 0 ;
    int targets_num=_Targets._target_info.size();
    if (targets_num==0) 
    {
        danger_car=-1 ;
    }
    else if (targets_num!=0)
    {
        float minx ;

        danger_car = _Targets._target_info[0]._id ;
        minx = abs(_Targets._target_info[0]._obs_position._x) ;

        for(int ii=1 ; ii<targets_num ; ii++)
        {
            if (abs(_Targets._target_info[ii]._obs_position._x) < minx)
            {
                minx=abs(_Targets._target_info[ii]._obs_position._x) ;
                danger_car=_Targets._target_info[ii]._id ;
            }
            else 
            {
                continue ;
            }
        }
    }

    return danger_car ;
}
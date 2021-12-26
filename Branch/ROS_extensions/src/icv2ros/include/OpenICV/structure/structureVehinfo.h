

#ifndef _VEHINFO_H
#define _VEHINFO_H
#include <stdint.h>
#include <msgpack.hpp>

#include <string>


typedef struct
{
float Speed;
float ax;
int data;
float YawRate;
int YawDir;

int steerDir;
float steerAngle;
float steerSpeed;
MSGPACK_DEFINE( Speed,ax,data,YawRate,YawDir,steerDir,steerAngle,steerSpeed);


}veh_info;
//=====================================================

typedef struct
{
float y_vehicle;
float x_vehicle;
float y_vehicle_gps;
float x_vehicle_gps;
float y_now_idea;
float vehicle_wheel_angle;
float vehicle_wheel_speed;
float y_preview;
float x_preview;
float vehicle_theta;
float d_dis;
float x_path;
float y_path;
float yaw_ang_speed;
float yaw_change_theta;
float predict_theta;
MSGPACK_DEFINE( y_vehicle, x_vehicle, y_vehicle_gps, x_vehicle_gps, y_now_idea, vehicle_wheel_angle, vehicle_wheel_speed, y_preview, x_preview, vehicle_theta, d_dis, x_path, y_path, yaw_ang_speed, yaw_change_theta, predict_theta);


}veh_info_control;


typedef struct
{
float pos_x;
float pos_y;
float orientation_angle;
float imu_angle;
MSGPACK_DEFINE(pos_x,pos_y,orientation_angle,imu_angle);
}veh_pos;








#endif // _DISPATCH_H

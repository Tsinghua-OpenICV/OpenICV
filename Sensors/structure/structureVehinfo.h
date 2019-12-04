

#ifndef _VEHINFO_H
#define _VEHINFO_H
#include <stdint.h>

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


}veh_info_control;


typedef struct
{
float pos_x;
float pos_y;
float orientation_angle;
float imu_angle;

}veh_pos;








#endif // _DISPATCH_H

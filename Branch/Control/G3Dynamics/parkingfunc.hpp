#ifndef PARKINGFUNC_H
#define PARKINGFUNC_H
#include <stdio.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>

typedef unsigned char uint8;
typedef float  float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

#define WW 2.45     //停车位长度
#define LLL 5    //停车位宽度
#define L_a 4.747    //Car length 车长度
#define W_a 1.834 //car width  车宽度
//#define HH 1.659     //car height 车高
#define W_bf 1.675 //front tire distance 前轮距
#define W_br 1.675 //rear tire distance   后轮距
#define h_m 2.802  //wheelbase           轴距
#define L_f 0.884  //front coerhang     前悬长度
#define L_r 1.072  //rear overhang      后悬长度
#define R_r 0.5  //wheel radius      轮子半径（不准确，不重要）
#define R_min 4.3  //turning radius    最小转弯半径，从车的后轴中心算起
#define delta_1 0.2     //控制量  距离车库低端距离
#define delta_2 0.2     // 进车哭时与内测的距离
#define delta_3 0.1    // 与通道顶端的距离 （开扩场地暂时没有加约束）
#define pi_aps 3.14159
#define DD      10.00   //通道款


//int control =0;
//point struct
struct RoadPoint{
  float x;
  float y;
  float Road_angle; 
  float curve; 
};

struct Car_Status{
    float x;
    float y;
    float angle;
    int status;
};

//float traceGeneration( float *ControlPoint ){};
//void traceGeneration2left( float *ControlPoint2 ,float startx,float starty,float startangle);
float getangle (float pointAx, float pointAy, float pointBx,float pointBy  );
float gedis (float pointAx, float pointAy, float pointBx,float pointBy );
float drawArc(float x1, float y1, float alpha1 , float x2, float y2,float alpha2, float &x,float &y,std::vector<RoadPoint> &partpointList);
float drawLine(float pointAx, float pointAy, float pointBx, float pointBy, std::vector<RoadPoint> &partpointList);
float drawBezier(float x1, float y1, float alpha1 , float x2, float y2,float alpha2 ,std::vector<RoadPoint> &partpointList);
float drawBezier2(float x1, float y1, float alpha1, float x2, float y2, float alpha2, float indexL ,std::vector<RoadPoint> &partpointList);
void generateCurveList(int control,float  ControlPoint[2][3], Car_Status &startPoint,  std::vector<RoadPoint>  &apspointList);
void generateCurveList2(int control,float  ControlPoint2[8], Car_Status &startPoint,  std::vector<RoadPoint>  &apspointList);
void generateCurveList2left(int control,float  ControlPoint2[8], Car_Status &startPoint,  std::vector<RoadPoint>  &apspointList);
int changecarstatis( int status);
void YuanXin(float x1,float y1,float alpha1, float x2,float y2,float alpha2,float &x,float &y);
float getR(float x1,float y1,float alpha1,float x2,float y2,float alpha2);
void controlstr(std::vector<RoadPoint>& apsroadpoint,float x ,float y,float heading ,float& strangle ,int& count_now);

float ControlStr2(std::vector<RoadPoint> &apspoint, float _In_X_Position, float _In_Y_Position, float _In_Heading_Vehicle, int &count_now, float _In_Speed_Ego_Vehicle);

 //1——21日 增加预测函数，输入为当前的反馈值back，反馈延时时间T_delay，输出频率rate，过去的输入,输出为角度
float controldelay(float back,float Tdelay,int rate , float wheelout[10]);//float back , float T_delay,i nt rate，float wheelout[10]);
float Cij(int n, int j);
void OptArconlineright(float *ArcKMap);
void OptArconlineleft(float *ArcKMap);
#endif



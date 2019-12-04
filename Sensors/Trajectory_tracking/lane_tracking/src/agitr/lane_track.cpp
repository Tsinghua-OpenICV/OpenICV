#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "agitr/vehicleout.h"
#include "math.h"
#include "canmsg.h"
#include "agitr/veh_pos.h"
#include <fstream>
using namespace std;
float pi=3.14159265;  //定义ｐｉ
float ACC_v; //the actual speed
float wheel_angle_vehicle;  //方向盘转角
int   vehicle_wheel_dre;
float vehicle_wheel_speed;
float yaw_speed;            //横摆角速度
int   yaw_dre;                  //横摆角速度方向
float yaw_angle;
float predict_angle;
void chatcallback1(const agitr::can::ConstPtr& canmsg)//回调函数  
{  
    ROS_INFO("I heared Speed is:[%f]",canmsg->Speed); 
    ACC_v=canmsg->Speed;
    ACC_v=ACC_v/3.6;
    ROS_INFO("I heared wheel angle is:[%f]",canmsg->LWI_StrWhlAngleSize); 
    wheel_angle_vehicle=canmsg->LWI_StrWhlAngleSize;
    vehicle_wheel_dre=canmsg->LWI_StrWhlAngleDrt; 
    vehicle_wheel_speed=canmsg->LWI_StrWhlSpeedSize; 
    if (vehicle_wheel_dre==0)
    {wheel_angle_vehicle=wheel_angle_vehicle;}
    else 
    {wheel_angle_vehicle=-1*wheel_angle_vehicle;}
    yaw_speed=canmsg->YawRate;           //横摆角速度,需要实车测试一下
    yaw_dre=canmsg->YawToRight;          //横摆角速度参数   
    if (yaw_dre==1)                      //向右转,顺时针,为负
       {
        yaw_speed=-1*yaw_speed;
          }
    else
       {
       yaw_speed=yaw_speed;             //角度
         }
} 

float x_vehicle;                   //车辆的x坐标
float y_vehicle;                   //车辆的y坐标
float x_vehicle_gps;               //gps收到车辆的坐标
float y_vehicle_gps;  
float heading_vehicle;             //车辆航向角,角度
void chatcallback2(const agitr::veh_pos::ConstPtr& vehicle_p)//回调函数  
{  
    ROS_INFO("I heared x is:[%f]",vehicle_p->pos_x); 
    x_vehicle_gps=vehicle_p->pos_x;
    ROS_INFO("I heared y is:[%f]",vehicle_p->pos_y); 
    y_vehicle_gps=vehicle_p->pos_y;
    ROS_INFO("I heared y is:[%f]",vehicle_p->orientation_angle); 
    heading_vehicle=vehicle_p->orientation_angle;
    //yaw_angle=(yaw_speed*pi/180.0)*(0.2*ACC_v)/(2.0*ACC_v)*180/pi;                 //角度         
    yaw_angle=0;    
    predict_angle=heading_vehicle+yaw_angle;
    x_vehicle=x_vehicle_gps+ACC_v*0.7*cos(predict_angle*pi/180);                  //向前推一个周期，消除滞后误差
    y_vehicle=y_vehicle_gps+ACC_v*0.7*sin(predict_angle*pi/180);                  //0.2*ACC_v 为一个周期内向前走的距离
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "lane_tracking");
    ros::NodeHandle nh;


    //ros::Rate rate(100);    //frequency is 100ms
    ros::Rate rate(5);   //frequency is 100ms
    ros::Subscriber sub1=nh.subscribe("touranmsg",1000,chatcallback1);
    ros::Subscriber sub2=nh.subscribe("vehicle_position",1000,chatcallback2);

    float m=1705;  //the mass of vehicle
    float a=1.255;   //the distance from centre of mass to front axle
    float b=1.535;    //the distance from centre of mass to rear axle
    float Cf=116000;  //the front tyre cornering stiffness
    float Cr=187000;  //the rear tyre cornering stiffness
    float G;     //the steering gear ratio (hand wheel angle/road wheel angle)
    float U; //the velocity of vehicle in the direction of vehicle’s longitudinal axis, km/h to m/s
    float delta_T=1.0/5.0; //time interval
    //float delta_T=0.01; //time interval
    float t_preview=2.8;   //预瞄时间，开始版本前面V未除以3.6,所以t比较小，这一版本修改过来了  7.2-2.8，误差20cm
    float t_preview2=3;   //预瞄时间     7.2--1.15  误差为1m    3.6--1.5,两点预瞄
    
    float d_0=0.5;  //起始距离.

    float wheel_angle_degree_max=470;  //方向盘最大的转角
    float wheel_angle_degree_sec=200;   //一秒中方向盘最大的转角
    int flag=1;


    //计算量的定义声明
    float theta_ss;
    float d_dis;
    float d_dis2;  //第二个预瞄点距离
    float T;          //分母中的参数
    float Oss1;        //距离偏差
    float Oss2;        //第二个预瞄点误差
    float Oss_back;   //反馈的现阶段距离偏差
    float denominator;
    float denominator2;    //第二个预瞄点分母
    float molecular;
    float delta_y_back;
    //float wheel_angle_radian=0;   //方向盘转角，弧度
    float wheel_angle_radian_next;
    float wheel_angle_degree;
    float wheel_angle_degree_next;
    float basepoint_x=0.0;
    float basepoint_y=0.0;
    float x_preview;   //预瞄处的x坐标
    float y_preview;   //预瞄处的y坐标
    float x_preview2;   //第二个预瞄点处的x坐标
    float y_preview2;   //第二个预瞄点处的y坐标
    float y_preview_actual;  //预瞄x值处的实际y值
    float delta_y_preview;  // 预瞄位置处的偏差值
    float y_now_idea;      //车辆理想位置
    float path_x[10240];   //路径x坐标
    float path_y[10240];   //路径y坐标

    float p1_neg=-0.001498;  //轮胎转角为负的情况
    float p2_neg=-0.02241;
    float p3_neg=14.99;
    float p1_post=-0.0008035;//轮胎转角为正的情况
    float p2_post=-0.01031;
    float p3_post=15.33;
   


    int count=0;           //用于存储路径
    int count_cycle=0; 
    int flag_end=0;   
    float L;
    float L_back;
    float dis_lilun;
    float dis_yumiao;
    float p_feedback=0;    //反馈的参数   3.6--0.1  7.2--0.056
    int flag_cycle; 
    int count_cycle2=0;
    float delta_y_preview2;
    float L2;
    
    ros::Publisher pubdraw=nh.advertise<agitr::vehicleout>("y_draw",1000);
    ros::Publisher pub=nh.advertise<std_msgs::Float32>("steer_angle_chatter",1000);    //can
    ros::Publisher pubacc=nh.advertise<std_msgs::Int8>("ACC_brake",1000);    //can
    std_msgs::Float32 vehicle_attitude;
    std_msgs::Int8 acc_off;
    agitr::vehicleout vehicle_state;
    

    ifstream ifile1("/home/icv002/Desktop/lane_data/x.txt");
    ifstream ifile2("/home/icv002/Desktop/lane_data/y.txt");

    while ((ifile1.good())&&(ifile2.good()))                              //读入路径数据
    {
	ifile1>>path_x[count];
	ifile2>>path_y[count];
	ROS_INFO("x %f",path_x[count]);
	ROS_INFO("y %f",path_y[count]);
	count=count+1;
    }
    ros::spinOnce();
    while (ros::ok())
    {   
        U=ACC_v;
	d_dis=t_preview*U+d_0;  //公式分母中的ｄ
        d_dis2=t_preview2*U+d_0;  //第二个预瞄点距离

	if (flag==1)   
	{
	    Oss1=0.1;
	    Oss_back=0.1;
            Oss2=0.1;
	}
        else
	{
	    Oss1=delta_y_preview;
	    Oss_back=delta_y_back;
            Oss2=delta_y_preview2;
	}  
	T=b-((a*m*(U*U))/(Cr*(a+b)));  //分母中的T
	denominator=d_dis*(d_dis+2*T);         //分母
        denominator2=d_dis2*(d_dis2+2*T);         //分母
	molecular=2*(a+b-(m*(a*Cf-b*Cr))/((a+b)*Cf*Cr));   //分子
	//theta_ss=0.7*molecular/denominator*Oss1+0.3*molecular/denominator2*Oss2+p_feedback*Oss_back;    //多点预瞄
        theta_ss=molecular/denominator*Oss1+p_feedback*Oss_back;
        //theta_ss=p_feedback*Oss_back; 
	if (theta_ss>=0)
	    G=p1_post*(theta_ss*(180/pi))*(theta_ss*(180/pi))+p2_post*(theta_ss*(180/pi))+p3_post;
	else
	    G=p1_neg*(theta_ss*(180/pi))*(theta_ss*(180/pi))+p2_neg*(theta_ss*(180/pi))+p3_neg;
     	wheel_angle_radian_next=theta_ss*G;    //计算出来的方向盘转角，弧度
	//wheel_angle_radian=theta_ss*G;
	//弧度转角度
	wheel_angle_degree_next=wheel_angle_radian_next*(180/pi);   //弧度转角度
	if (wheel_angle_degree_next>=wheel_angle_degree)
	    {
		if (wheel_angle_degree_next-wheel_angle_degree>wheel_angle_degree_sec*delta_T)
			{wheel_angle_degree_next=wheel_angle_degree+wheel_angle_degree_sec*delta_T;}
		else
			{wheel_angle_degree_next=wheel_angle_degree_next;}

		if (wheel_angle_degree_next>=wheel_angle_degree_max)
			{wheel_angle_degree_next=wheel_angle_degree_max;}
		else
			{wheel_angle_degree_next=wheel_angle_degree_next;}
	    }
	else
	    {
		if (fabs((wheel_angle_degree_next-wheel_angle_degree)/delta_T)>wheel_angle_degree_sec)
			{wheel_angle_degree_next=wheel_angle_degree-wheel_angle_degree_sec*delta_T;}
		else
			{wheel_angle_degree_next=wheel_angle_degree_next;}

		if (wheel_angle_degree_next<=-wheel_angle_degree_max)
			{wheel_angle_degree_next=-wheel_angle_degree_max;}
		else
			{wheel_angle_degree_next=wheel_angle_degree_next;}
	    }
	ROS_INFO("the wheel_angle is:[%f]",wheel_angle_degree_next);
        if (flag<30)
        {
           wheel_angle_degree=wheel_angle_vehicle;              //前几时刻，以当前转角为下一时刻参考转角
           }
        else
        {       
         wheel_angle_degree=wheel_angle_degree_next;           //保存当前理论转角为下一时刻参考转角
            }
	vehicle_attitude.data=wheel_angle_degree_next;

	//计算预瞄处的x和y坐标
	x_preview=x_vehicle+d_dis*cos(heading_vehicle*pi/180);
	y_preview=y_vehicle+d_dis*sin(heading_vehicle*pi/180);

        x_preview2=x_vehicle+d_dis2*cos(heading_vehicle*pi/180);        //第二个预瞄点位置
	y_preview2=y_vehicle+d_dis2*sin(heading_vehicle*pi/180);

        float L_min=20.0;
	float x_min=-1.0;
        float y_min=-1.0;
	int count_design=0;

	for (count_cycle=0;count_cycle<count-1;count_cycle++)
	    {
	     //L=sqrt(pow((path_x[count_cycle]-x_preview),2)+pow((path_y[count_cycle]-y_preview),2));
	     L=sqrt((path_x[count_cycle]-x_preview)*(path_x[count_cycle]-x_preview)+(path_y[count_cycle]-y_preview)*(path_y[count_cycle]-y_preview));
             //ROS_INFO("L:[%f]",L);
             if (L<=L_min)
                {
                 L_min=L;
                 x_min=path_x[count_cycle];
                 y_min=path_y[count_cycle];
		 count_design=count_cycle;
                 }
              else
                 {
                 L_min=L_min;
                 x_min=x_min;
                 y_min=y_min;
		 count_design=count_design;
                 }
             ROS_INFO("count_design:[%d]",count_design);
	    }
	//判断方向盘的方向
	if ((x_preview-x_vehicle)*(y_min-y_vehicle)-(y_preview-y_vehicle)*(x_min-x_vehicle)>=0)   delta_y_preview=L_min;
	else   delta_y_preview=-1.0*L_min;

       
	ROS_INFO("the actual y is:[%f]",y_vehicle);
	//ROS_INFO("the ideal y is:[%f]",y_now_idea);
	//ROS_INFO("the wheel y is:[%f]",wheel_angle_radian);
	//ROS_INFO("the x is:[%f]",yaw_angle_next);


	float L_min_back=20;
	float x_min_back=-1;
	float y_min_back=-1;
	flag_cycle=1;
	for (count_cycle=0;count_cycle<count-1;count_cycle++)
	{
	    L_back=sqrt((path_x[count_cycle]-x_vehicle)*(path_x[count_cycle]-x_vehicle)+(path_y[count_cycle]-y_vehicle)*(path_y[count_cycle]-y_vehicle));

	    if (L_back<L_min_back)
	    {
		L_min_back=L_back;
		x_min_back=path_x[count_cycle];
		y_min_back=path_y[count_cycle];
		flag_cycle=count_cycle;
	    }
	    else
	    {
		L_min_back=L_min_back;
		x_min_back=x_min_back;
		y_min_back=y_min_back;
		flag_cycle=flag_cycle;
	    }

	}
	
	if ((x_vehicle-path_x[flag_cycle-20])*(y_min_back-path_y[flag_cycle-20])-(x_min_back-path_x[flag_cycle-20])*(y_vehicle-path_y[flag_cycle-20])>=0)   delta_y_back=1.0*L_min_back;
	else   delta_y_back=-1.0*L_min_back;
	ROS_INFO("the delta_y_back is:[%f]",delta_y_back);
        float L_min2=20.0;                 //计算第二个预瞄点误差
	float x_min2=-1.0;
        float y_min2=-1.0;
       
        for (count_cycle2=0;count_cycle2<count-1;count_cycle2++)
	    {
	     L2=sqrt((path_x[count_cycle2]-x_preview2)*(path_x[count_cycle2]-x_preview2)+(path_y[count_cycle2]-y_preview2)*(path_y[count_cycle2]-y_preview2));
             //ROS_INFO("L:[%f]",L);
             if (L2<L_min2)
                {
                 L_min2=L2;
                 x_min2=path_x[count_cycle2];
                 y_min2=path_y[count_cycle2];
                 }
              else
                 {
                 L_min2=L_min2;
                 x_min2=x_min2;
                 y_min2=y_min2;
                 }
	    }
	//判断方向盘的方向
	if ((x_preview2-x_vehicle)*(y_min2-y_vehicle)-(y_preview2-y_vehicle)*(x_min2-x_vehicle)>=0)   delta_y_preview2=L_min2;
	else   delta_y_preview2=-1.0*L_min2;


	//用来判断是否到达终点
	if (count_design>=count-600)
	{
	    flag_end=flag_end+1;
	    
	}
	else
	{
	    flag_end=flag_end;
	}
        
	if (flag_end>=5)
	{
	    vehicle_attitude.data=0;
	}
        ROS_INFO("flag_end:[%d]",flag_end);
	flag=flag+1;

	//topic:ACC_brake
        acc_off.data=flag_end;
        //topic:y_draw
        vehicle_state.y_vehicle_gps=y_vehicle_gps;      //车辆实际位置
        vehicle_state.x_vehicle_gps=x_vehicle_gps; 
        vehicle_state.y_vehicle=y_vehicle;              //向前推一个周期的位置
        vehicle_state.x_vehicle=x_vehicle; 
        vehicle_state.y_preview=y_preview;              //预瞄位置
        vehicle_state.x_preview=x_preview;          
	vehicle_state.vehicle_theta=heading_vehicle;    //车辆航向角
        vehicle_state.d_dis=delta_y_preview;            //预瞄误差
	vehicle_state.vehicle_wheel_angle=wheel_angle_vehicle;   //车辆的方向盘转角
        vehicle_state.vehicle_wheel_speed=vehicle_wheel_speed;   //车辆方向盘转角速度
        vehicle_state.yaw_ang_speed=yaw_speed;
        vehicle_state.yaw_change_theta=yaw_angle;
        vehicle_state.predict_theta=predict_angle;

	pub.publish(vehicle_attitude);
        pubdraw.publish(vehicle_state);
	pubacc.publish(acc_off);

	ros::spinOnce();
	rate.sleep();
	
    }
    return 0;

}

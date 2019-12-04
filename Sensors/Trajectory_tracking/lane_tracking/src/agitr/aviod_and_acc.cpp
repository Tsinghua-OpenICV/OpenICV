#include <ros/ros.h>
#include "canmsg.h"
#include "std_msgs/Float32.h"
#include "agitr/aviod.h"

float ACC_v; //the actual speed
float ACC_ax;//the actual accel

void chatcallback(const agitr::can::ConstPtr& canmsg)//回调函数  
{  
    ROS_INFO("I heared Speed is:[%f]",canmsg->Speed); 
    ROS_INFO("I heared ax is:[%f]",canmsg->ax);
    ACC_v=canmsg->Speed;
    ACC_ax=canmsg->ax;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ACC_control");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<std_msgs::Float32>("ACC_control_chatter",1000);
    ros::Publisher pub1=nh.advertise<agitr::aviod>("aviod_show",1000);
    ros::Rate rate(10);
    ros::Subscriber sub=nh.subscribe("touranmsg",1000,chatcallback);
    
    float V_s;          //costant number,known,
    float V_set;        //costant number,known
    float a_accel_max;  //costant number,known
    float a_decel_desired;  //costant number,known
    float constant;   //costant number,known
    float constant_aviod;
    float delta_T;    //costant number,known,related to the number of rate
    float K_SVE;     //costant number,known
    float d_SVE;     //costant number,known
    float k_v_set;


    V_set=25;      //set the cruise speed
    V_set=V_set/3.6;
    a_accel_max=2.0;   //postive number
    a_decel_desired=5.0;  //neagtive number
    constant=3.0;  //the change rate of a
    constant_aviod=10.0;
    delta_T=1.0/10.0;  //1/10,related to rate number
    //delta_T=0.01;  //1/10,related to rate number
    K_SVE=-0.002;
    d_SVE=1.025; 
    k_v_set=0.5583; 
    
    float d_safe;    //the constant para in the brake system
    if (V_set<=7.3/3.6)  d_safe=2;
    else if ((V_set>7.3/3.6)&&(V_set<=10/3.6))  d_safe=3;
    else if ((V_set>10/3.6)&&(V_set<=15/3.6))   d_safe=6;
    else if ((V_set>15/3.6)&&(V_set<=20/3.6))   d_safe=8;
    else if ((V_set>20/3.6)&&(V_set<=25/3.6))   d_safe=16;
    else d_safe=25;

    float inv_SVE;  //unknown
    float SVE; //unknown
    float accel;  //unknown
    float accel_pre;  //unknown,storage last moment accel

    int flag=0;//determine the position 
 

    //Some parameters during collision avoidance
    float d_brake1;   //the distance of brake in different suitation
    float d_brake2;
    float d_brake3;
    float d_init;    //the init distance
    float t_sys;     //the reacting time of the vehicle system
    float d_actual;    //the distance two cars
    float d_actual_show;
    float a_brake_situ1;
    float a_brake_situ2;
    float a_brake_situ3;  //three situtation of brake
    float v_front;
    int brake_mode;   
    int flag_v;
    int flag_situ1;
    int flag_situ2;
    int flag_situ3;
    int flag_mode1;
    int flag_mode2;
    int flag_mode3;

    d_actual=80;  //  m
    d_actual_show=d_actual;
    t_sys=0.5;
    a_brake_situ1=4;
    a_brake_situ2=5;
    a_brake_situ3=6;
    v_front=0;
    flag_situ1=0;
    flag_situ2=0;
    flag_situ3=0;
    flag_mode1=1;
    flag_mode2=1;
    flag_mode3=1;
    //brake_mode=0;
    

    ros::spinOnce();

    agitr::aviod show;
    while(ros::ok())
    {
	V_s=ACC_v;
	V_s=V_s/3.6;
	accel_pre=ACC_ax;
	d_actual_show=d_actual_show-(V_s-v_front)*delta_T;
	if ((flag_situ1==0)&&(flag_situ2==0)&&(flag_situ3==0))
	{
	    d_brake1=0.5*(V_s*V_s/a_brake_situ1)+V_s*t_sys+d_safe;
	    d_brake2=0.5*(V_s*V_s/a_brake_situ2)+V_s*t_sys+d_safe;
	    d_brake3=0.5*(V_s*V_s/a_brake_situ3)+V_s*t_sys+d_safe;
	    d_actual=d_actual-(V_s-v_front)*delta_T;
	}
	else
	{
	    d_brake1=d_brake1;
	    d_brake2=d_brake2;
	    d_brake3=d_brake3;
	    d_actual=d_actual;
	}
	
	
	
     
	if (v_front<=0.2) flag_v=1;
	else flag_v=0;
	
	if ((d_actual<=d_brake1)&&(d_actual>d_brake2)&&(d_actual>d_brake3)) 
	{
	    flag_situ1=1;
	}
	else if ((d_actual<=d_brake2)&&(d_actual>d_brake3))
	{
	    flag_situ1=1;
	    flag_situ2=1;
	}
	else if ((d_actual<=d_brake3))
	{
	    flag_situ1=1;
	    flag_situ2=1;
	    flag_situ3=1;
	}


	if ((flag_v==1)&&(flag_situ1==1)&&(flag_mode1==1))
	{
	    accel=-a_brake_situ1;
	    if (accel>accel_pre)  //the speed is increase
	    {
		if ((accel-accel_pre)/delta_T>constant_aviod)  accel=accel_pre+delta_T*constant_aviod;
		else  accel=accel;
		if  (accel>a_accel_max)  accel=a_accel_max;
		else  accel=accel;
	    }
	    else     //the speed is decrease
	    {
		if (abs((accel-accel_pre)/delta_T)>constant_aviod)  accel=accel_pre+delta_T*(-constant_aviod);
		else  accel=accel;
		if  (accel<-a_decel_desired)  accel=-a_decel_desired;
		else  accel=accel;
	    }
	    flag_mode2==0;
	    flag_mode3==0;
	}
	else if ((flag_v==1)&&(flag_situ2==1)&&(flag_mode2==1))
	{
	    accel=-a_brake_situ2;
	    if (accel>accel_pre)  //the speed is increase
	    {
		if ((accel-accel_pre)/delta_T>constant_aviod)  accel=accel_pre+delta_T*constant_aviod;
		else  accel=accel;
		if  (accel>a_accel_max)  accel=a_accel_max;
		else  accel=accel;
	    }
	    else     //the speed is decrease
	    {
		if (abs((accel-accel_pre)/delta_T)>constant_aviod)  accel=accel_pre+delta_T*(-constant_aviod);
		else  accel=accel;
		if  (accel<-a_decel_desired)  accel=-a_decel_desired;
		else  accel=accel;
	    }
	    flag_mode1==0;
	    flag_mode3==0;
	}
	else if ((flag_v==1)&&(flag_situ3==1)&&(flag_mode3==1))
	{
	    accel=-a_brake_situ2;
	    if (accel>accel_pre)  //the speed is increase
	    {
		if ((accel-accel_pre)/delta_T>constant_aviod)  accel=accel_pre+delta_T*constant_aviod;
		else  accel=accel;
		if  (accel>a_accel_max)  accel=a_accel_max;
		else  accel=accel;
	    }
	    else     //the speed is decrease
	    {
		if (abs((accel-accel_pre)/delta_T)>constant_aviod)  accel=accel_pre+delta_T*(-constant_aviod);
		else  accel=accel;
		if  (accel<-a_decel_desired)  accel=-a_decel_desired;
		else  accel=accel;
	    }
	    flag_mode1==0;
	    flag_mode2==0;
	}
	else
	{
	    inv_SVE=K_SVE*V_s+d_SVE;
	    SVE=1/inv_SVE;
	    accel=SVE*k_v_set*(V_set-V_s); //ideal accel
	    if (accel>accel_pre)  //the speed is increase
	    {
		if ((accel-accel_pre)/delta_T>constant)  accel=accel_pre+delta_T*constant;
		else  accel=accel;
		if  (accel>a_accel_max)  accel=a_accel_max;
		else  accel=accel;
	    }
	    else     //the speed is decrease
	    {
		if (abs((accel-accel_pre)/delta_T)>constant)  accel=accel_pre+delta_T*(-constant);
		else  accel=accel;
		if  (accel<-a_decel_desired)  accel=-a_decel_desired;
		else  accel=accel;
	    }
	}

    flag=flag+1;
    std_msgs::Float32 ACC_msg;
    ACC_msg.data=accel;
    show.a_ide=accel;
    show.a_act=ACC_ax;
    show.d_act=d_actual_show;
    show.d_bra=d_brake1;
    show.v_act=V_s;
    pub.publish(ACC_msg);
    pub1.publish(show);
    ros::spinOnce();
    rate.sleep();

    }
    return 0;
}

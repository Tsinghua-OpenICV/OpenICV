#include <ros/ros.h>
#include "canmsg.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"


float ACC_v; //the actual speed
float ACC_ax;//the actual accel

void chatcallback(const agitr::can::ConstPtr& canmsg)//回调函数  
{  
    ROS_INFO("I heared Speed is:[%f]",canmsg->Speed); 
    ROS_INFO("I heared ax is:[%f]",canmsg->ax);
    ACC_v=canmsg->Speed	;
    ACC_ax=canmsg->ax;
}

int acc_off;
void accoffcallback(const std_msgs::Int8::ConstPtr& accoffmsg)//回调函数 
{
    acc_off=accoffmsg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ACC_control");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<std_msgs::Float32>("ACC_control_chatter",1000);
    ros::Rate rate(5);
    //ros::Rate rate(100);
    ros::Subscriber sub=nh.subscribe("touranmsg",1000,chatcallback);
    ros::Subscriber sub_accoff=nh.subscribe("ACC_brake",1000,accoffcallback);
    
    float V_s;          //costant number,known,
    float V_set;        //costant number,known
    float a_accel_max;  //costant number,known
    float a_decel_desired;  //costant number,known
    float constant;   //costant number,known
    float delta_T;    //costant number,known,related to the number of rate
    float K_SVE;     //costant number,known
    float d_SVE;     //costant number,known
    float k_v_set;

    //V_s=ACC_v;
    //V_s=V_s/3.6;
    V_set=7.2;      //set the cruise speed
    V_set=V_set/3.6;
    a_accel_max=2.0;   //postive number
    a_decel_desired=5.0;  //neagtive number
    constant=3.0;  //the change rate of a
    delta_T=1.0/5.0;  //1/10,related to rate number
    //delta_T=0.01;  //1/10,related to rate number
    K_SVE=-0.002;
    d_SVE=1.025; 
    k_v_set=0.5583;
    

    float inv_SVE;  //unknown
    float SVE; //unknown
    float accel;  //unknown
    float accel_pre;  //unknown,storage last moment accel

    int flag=0;//determine the position 

    ros::spinOnce();
    while(ros::ok())
    {
	V_s=ACC_v;
        V_s=V_s/3.6;
	accel_pre=ACC_ax;
        inv_SVE=K_SVE*V_s+d_SVE;
  	SVE=1/inv_SVE;
	accel=SVE*k_v_set*(V_set-V_s); //ideal accel
	if (flag==0)  //the first beginning part
	{
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
	else
	{ 
		if (accel>accel_pre)  //the speed is increase
		{
			if ((accel-accel_pre)/delta_T>constant)  accel=accel_pre+delta_T*constant;
			else  accel=accel;
			if  (accel>a_accel_max)  accel=a_accel_max;
			else  accel=accel;
		}	
		else  //the speed is decrease
		{
			if (abs((accel-accel_pre)/delta_T)>constant)  accel=accel_pre+delta_T*(-constant);
			else  accel=accel;
			if  (accel<-a_decel_desired)  accel=-a_decel_desired;
			else  accel=accel;
		}
	}

	if (acc_off>=5)   accel=-1.0;
	
	ROS_INFO("the subject car speed is:[%f]",V_s);
  	flag=flag+1;
	std_msgs::Float32 ACC_msg;
        ACC_msg.data=accel;
	pub.publish(ACC_msg);
	ros::spinOnce();
	rate.sleep();
    } 
    return 0;
}


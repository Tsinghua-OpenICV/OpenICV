#include <ros/ros.h>
//#include <dynamic_reconfigure/server.h>
//#include <eps_steer/eps_steerConfig.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "agitr/vehicleout.h"
#include "math.h"
#include "stdio.h"
#include "canmsg.h"
//#include "agitr/veh_pos.h"

//回调函数方向盘转角
/*float theta_expect_can;//发送的期望方向盘转角
void messageCallbackacc(const std_msgs::Float32::ConstPtr& acc_msg)
{
	ROS_INFO("expect angle:[%f]",acc_msg->data);
	theta_expect_can=acc_msg->data;
}*/

float theta_expect_can;//发送的期望方向盘转角
float wheel_angle_degree;  //方向盘转角
int   vehicle_wheel_dre;
void messageCallbackacc(const agitr::can::ConstPtr& acc_msg)
{
	ROS_INFO("expect angle:[%f]",acc_msg->LWI_StrWhlAngleSize);
	wheel_angle_degree=acc_msg->LWI_StrWhlAngleSize;
	vehicle_wheel_dre=acc_msg->LWI_StrWhlAngleDrt;
	if (vehicle_wheel_dre==0)
        {wheel_angle_degree=wheel_angle_degree;}
        else 
        {wheel_angle_degree=-1*wheel_angle_degree;}
	theta_expect_can=wheel_angle_degree;
}

//回调函数can信息
float theta_real_can;   //发送实际方向盘转角
float theta_real_canvalue;//方向盘转角大小
float theta_real_cansign;//方向盘转动方向
float velocity_can;//发送车辆速度
float Wr_can;//发送质心处横摆角速度

void messageCallbackcan(const agitr::can::ConstPtr& can_msg)
{
	ROS_INFO("ACC speed:[%f]",can_msg->Speed);
	velocity_can=can_msg->Speed;
	ROS_INFO("yaw rate:[%f]",can_msg->YawRate);
	Wr_can=can_msg->YawRate;
	ROS_INFO("steer anglevalue:[%f]",can_msg->LWI_StrWhlAngleSize);
	theta_real_canvalue=can_msg->LWI_StrWhlAngleSize;
	ROS_INFO("steer anglesign:[%d]",can_msg->LWI_StrWhlAngleDrt);
	theta_real_cansign=can_msg->LWI_StrWhlAngleDrt;
	if (theta_real_cansign==0)
	    theta_real_can=theta_real_canvalue;
	else
	    theta_real_can=-1*theta_real_canvalue;//将返回转角及方向转化成实际转角
}

//查表插值函数
float chatterseek(float kp[][2], float theta,int row)
{
	int i;
	float kp_eva;
	for(i=0;i<row;i++)
    {
		if (theta>=kp[i][1] && theta<kp[i+1][1])
		    {kp_eva=(kp[i+1][2]-kp[i][2])*(theta-kp[i][1])/(kp[i+1][1]-kp[i][1])+kp[i][2];
			break;
			}
	}
    return kp_eva;
}
	


int main(int argc,char **argv)
{
	//定义ros发布节点
	ros::init(argc,argv,"eps_steer");
	ros::NodeHandle nl;//启动句柄并设置名称nl
	ros::Publisher Moment=nl.advertise<std_msgs::Float32>("eps_moment_expectvalue",1000); 
	ros::Publisher Moment2=nl.advertise<std_msgs::Float32>("eps_angle_expect_value",1000);
	ros::Publisher Moment3=nl.advertise<std_msgs::Float32>("eps_angle_real_value",1000);
	//ros::Publisher Moment_2=nl.advertise<std_msgs::Int8>("eps_moment_expectsign",1000);//发布节点定义
	//ros::Rate loop_rate(50);//ros节点发送频率
	ros::Rate loop_rate(20);//ros节点发送频率
	//ros接收节点
	//ros::Subscriber sub1=nl.subscribe("steer_angle_chatter",1000,messageCallbackacc);//期望方向盘转角
        ros::Subscriber sub1=nl.subscribe("touranmsg_steer",1000,messageCallbackacc);//期望方向盘转角
	ros::Subscriber sub2=nl.subscribe("touranmsg",1000,messageCallbackcan);//can信息
	
	//变量定义
	float theta_expect;  //期望方向盘转角
   	float theta_real;   //实际方向盘转角
	float theta_err;    //方向盘转角误差
    float theta_Err_integral=0;  //方向盘转角误差积分
    float Moment_Expect;  //期望输出力矩
	float Moment_Expect1; 
 
   //前馈部分变量定义
	float X1P_1 = 0;           //前馈变量X1P(k-1)周期输入
	float X2P_1 = 0;           //前馈变量X2P(k-1)周期输入
	float X3P_1 = 0;        //前馈误差（k-1）周期输入
	float X1P;         //前馈变量X1P(k)周期输入
	float X2P;          //前馈变量X2P(k)周期输入
	float X3P;          //前馈误差（k-1）周期输入
    float h=0.000009;  //前馈参数修正变量
	float l1,l2,l3;  //定义前馈修正参数
	float Est_err;//前馈估计误差
	float Est_err_abs;//前馈估计误差abs
	float Est_sgn;//sgn(error)
	//float Ts = 1/50;      //系统周期
	float Ts = 1.0/20.0;      //系统周期
	float k_steer=18000;  //轮胎侧偏刚度
	float d_steer=0.006;  //轮胎拖距
	//float i_steer = 1/(20*16);//轮胎到电机的传动比
	float beta_car = 0;//质心侧偏角
	float velocity;//车辆速度
	//float v_y;//车辆横向速度
	//float v_x;//车辆纵向速度
	float a = 1.2; //质心到前轴的距离
	float Wr;//质心处横摆角速度
	//float Jm=4.44;//等效转动惯量
	//float Bm=3;//等效阻尼系数
	float Jk = 0.1; //修正等效转动惯量Jk=Jm*i^2*ie
	float Bk = 1.5; //修正等效阻尼系数Bk=Bm*i*ie
	float ie = 0.02;//转向系到电机传动比
	float ii = 0.06;//传动比
	float Est_err_abssqr;//误差估计^1/2
	float Est_err_absqua;//误差估计^1/4
	float pi=3.1415926;//定义pi
	int flag;
	float const_kp[46][2]=
	{{-450,3},{-405,2.75},{-360,2.75},{-315,2.75},{-270,2.58},{-225,2.57},{-210,2.58},{-180,2.57},{-150,2.59},
	 {-135,2.72},{-120,2.78},{-110,2.8},{-100,3.01},{-90,3.15},{-80,3.15},{-70,3.15},{-60,3.15},{-50,3.15},
	 {-40,3.15},{-30,3.15},{-20,3.15},{-10,3.15},{0,3},{10,2.95},{20,2.95},{30,2.95},{40,2.95},{50,2.95},{60,2.95},
	 {70,2.95},{80,2.95},{90,2.95},{100,2.91},{120,2.85},{135,2.81},{150,2.7},{180,2.84},{210,2.85},{225,2.89},
	 {270,2.9},{315,2.9},{360,2.9},{405,3},{450,3}
	};
	float Ki;
	float Kp;
	
	std_msgs::Float32 Moment_Expect_value;//定义输出期望输入力距的topic信息
	//std_msgs::Int8 Moment_Expect_sign;//定义输出期望力距
						   
	//控制器主程序
	while (ros::ok())
	{
		//车辆参数估计
		//beta_car = v_y / v_x;//计算车辆的质心侧偏角
		beta_car = 0;
		//Jk = Jm*i*i*ie;
		//Bk = Bm*i*ie;//计算车辆的等效系数
		theta_expect=theta_expect_can*(pi/180);//获取期望方向盘转角弧度
		theta_real=theta_real_can*(pi/180);   //获取实际方向盘转角
		velocity=velocity_can/3.6;//获取车辆速度
		Wr=Wr_can*(pi/180);//获取车辆的横摆角速度

		//PID主程序
		theta_err = theta_expect - theta_real;//计算方向盘转角误差
		ROS_INFO("steer err:[%f]",theta_err);
		theta_Err_integral += theta_err;         //计算方向盘转角误差积分
		ROS_INFO("theta_Err_integral:[%f]",theta_Err_integral);
		Kp=chatterseek(const_kp,theta_expect_can,46);
		if (theta_expect_can>=0)
		    Ki=0.005;
		else if (theta_expect_can<0 && theta_expect_can>-360)
			    Ki=0.006;
		else
			Ki=0.003;
		//Moment_Expect = Kp*theta_err + Ki*theta_Err_integral + X3P_1; //PID+前馈控制输出计算
		Moment_Expect = Kp*theta_err + Ki*theta_Err_integral;
		if (Moment_Expect>3)
		    Moment_Expect1=3;
		else if (Moment_Expect<-3)
		    Moment_Expect1=-3;
		else
		    Moment_Expect1=Moment_Expect;//限制幅值
		Moment_Expect_value.data=Moment_Expect1;
		ROS_INFO("Moment_Expect_value:[%f]",Moment_Expect_value.data);
		//if (Moment_Expect>0) Moment_Expect_sign=1;else Moment_Expect_sign=-1;//判断左转右转，左转为1，右转为-1
		//flag=1;
		//Moment_Expect_sign.data=flag;//激活EPS
		//前馈控制主程序
		l1 = 1 / h;
		l2 = 1 / (3 * h*h);
		l3 = 1 / (32 * h*h*h);//计算前馈系数
		Est_err = X1P_1 - theta_real; //估计误差
		Est_err_abs = abs(Est_err);//估计误差绝对值
		if (Est_err_abs > 0.01)
			Est_sgn = Est_err / Est_err_abs;
		else
			Est_sgn = Est_err / 0.01;//误差稳定函数计算
		Est_err_abssqr = sqrt(Est_err_abs); //误差估计^1/2
		Est_err_absqua = pow(Est_err_abs,1 / 4.0);//误差估计^1/4
		//前馈原理
		X1P = X1P_1 + Ts*(X2P_1 - l1*Est_err);
		//X2P = X2P_1 + Ts*(X3P_1 - l2*Est_err_abssqr*Est_sgn + (1 / Jk)*Moment_Expect - 2 * k_steer*d_steer*i_steer*(beta_car + (a*Wr / velocity - i*X1P_1)) - Bk*X2P_1);
		X2P = X2P_1 + Ts*(X3P_1 - l2*Est_err_abssqr*Est_sgn + (1 / (Jk*ii*ii*ie))*Moment_Expect - 2 * k_steer*d_steer*ii*ie*(beta_car + (a*Wr / velocity - ii*X1P_1)) - Bk*ii*ie*X2P_1);
		X3P = X3P_1 + Ts*(-l3*Est_err_absqua*Est_sgn);
		//前馈变量下一个周期初始化
		X1P_1 = X1P;
		X2P_1 = X2P;
		X3P_1 = X3P;
		
		std_msgs::Float32 angle_expect_value;
		std_msgs::Float32 angle_real_value;
		angle_expect_value.data = theta_expect;
		angle_real_value.data = theta_real_can;
		//Moment.publish(Moment_Expect);
		Moment.publish(Moment_Expect_value);
		Moment2.publish(angle_expect_value);
		Moment3.publish(angle_real_value);
		//Moment_2.publish(Moment_Expect_sign);//发布节点
		//Moment_2.publish(flag);//发布节点
		ros::spinOnce();//ros更新并读取所有主题
		loop_rate.sleep();//程序按照发送频率挂起
	}
	//ros::spin();
	return 0;
}



//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _IMUEthernet_H
#define _IMUEthernet_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
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

// Ethernet
#include <arpa/inet.h>

// UINT16_MAX is not defined by default in Ubuntu Saucy
#ifndef UINT16_MAX
#define UINT16_MAX (65535)
#endif



static inline double SQUARE(double x)
{
  return x * x;
}

#ifndef OXFORD_DISPLAY_INFO
#define OXFORD_DISPLAY_INFO 0
#endif
static inline void handlePacket(const Imu_Packet *packet,  NavSatFix &msg_fix, TwistWithCovarianceStamped &msg_vel, 
                                Imu &msg_imu, Odometry &msg_odom, const std::string &frame_id,const std::string &frame_id_vel)
{
  static uint8_t fix_status = STATUS_FIX;
  static uint8_t position_covariance_type = COVARIANCE_TYPE_UNKNOWN;
  static double position_covariance[3];
  static uint8_t velocity_covariance_type = COVARIANCE_TYPE_UNKNOWN;
  static double velocity_covariance[3];
  static uint8_t orientation_covariance_type = COVARIANCE_TYPE_UNKNOWN;
  static double orientation_covariance[3];
  // ICV_LOG_INFO<<"nav_status:"<<(int)packet->nav_status<<" channel"<<(int)packet->channel;
  if (packet->nav_status == 4) {
    time_t stamp; 
    stamp =time(NULL); 
    // ros::Time stamp = ros::Time::now();

    switch (packet->channel) {
      case 0:
        switch (packet->chan.chan0.position_mode) {
          case MODE_DIFFERENTIAL:
          case MODE_DIFFERENTIAL_PP:
          case MODE_RTK_FLOAT:
          case MODE_RTK_INTEGER:
          case MODE_RTK_FLOAT_PP:
          case MODE_RTK_INTEGER_PP:
          case MODE_DOPLER_PP:
          case MODE_SPS_PP:
            fix_status = STATUS_GBAS_FIX;
            break;
          case MODE_OMNISTAR_VBS:
          case MODE_OMNISTAR_HP:
          case MODE_OMNISTAR_XP:
          case MODE_WAAS:
          case MODE_CDGPS:
            fix_status = STATUS_SBAS_FIX;
            break;
          case MODE_SPS:
            fix_status = STATUS_FIX;
            break;
          case MODE_NONE:
          case MODE_SEARCH:
          case MODE_DOPLER:
          case MODE_NO_DATA:
          case MODE_BLANKED:
          case MODE_NOT_RECOGNISED:
          case MODE_UNKNOWN:
          default:
            fix_status = STATUS_NO_FIX;
            break;
        }
#if OXFORD_DISPLAY_INFO
        // ROS_INFO("Num Sats: %u, Position mode: %u, Velocity mode: %u, Orientation mode: %u",
        //          packet->chan.chan0.num_sats,
        //          packet->chan.chan0.position_mode,
        //          packet->chan.chan0.velocity_mode,
        //          packet->chan.chan0.orientation_mode);
#endif
        break;
      case 3:
        if (packet->chan.chan3.age < 150) {
          position_covariance[0] = SQUARE((double)packet->chan.chan3.acc_position_east * 1e-3);
          position_covariance[1] = SQUARE((double)packet->chan.chan3.acc_position_north * 1e-3);
          position_covariance[2] = SQUARE((double)packet->chan.chan3.acc_position_down * 1e-3);
          position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
#if OXFORD_DISPLAY_INFO
          // ROS_INFO("Position accuracy: North: %umm, East: %umm, Down: %umm",
          //          packet->chan.chan3.acc_position_north,
          //          packet->chan.chan3.acc_position_east,
          //          packet->chan.chan3.acc_position_down);
#endif
        } else {
          position_covariance_type = COVARIANCE_TYPE_UNKNOWN;
        }
        break;
      case 4:
        if (packet->chan.chan4.age < 150) {
          velocity_covariance[0] = SQUARE((double)packet->chan.chan4.acc_velocity_east * 1e-3);
          velocity_covariance[1] = SQUARE((double)packet->chan.chan4.acc_velocity_north * 1e-3);
          velocity_covariance[2] = SQUARE((double)packet->chan.chan4.acc_velocity_down * 1e-3);
          velocity_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
#if OXFORD_DISPLAY_INFO
          // ROS_INFO("Velocity accuracy: North: %umm/s, East: %umm/s, Down: %umm/s",
          //          packet->chan.chan4.acc_velocity_north,
          //          packet->chan.chan4.acc_velocity_east,
          //          packet->chan.chan4.acc_velocity_down);
#endif
        } else {
          velocity_covariance_type =COVARIANCE_TYPE_UNKNOWN;
        }
        break;

      case 5:
        if (packet->chan.chan5.age < 150) {
          orientation_covariance[0] = SQUARE((double)packet->chan.chan5.acc_roll * 1e-5);
          orientation_covariance[1] = SQUARE((double)packet->chan.chan5.acc_pitch * 1e-5);
          orientation_covariance[2] = SQUARE((double)packet->chan.chan5.acc_heading * 1e-5);
          orientation_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
#if OXFORD_DISPLAY_INFO
          // ROS_INFO("Velocity accuracy: Heading: %frad, Pitch: %frad, Roll: %frad",
          //          (double)packet->chan.chan5.acc_heading * 1e-5,
          //          (double)packet->chan.chan5.acc_pitch * 1e-5,
          //          (double)packet->chan.chan5.acc_roll * 1e-5);
#endif
        } else {
          orientation_covariance_type = COVARIANCE_TYPE_UNKNOWN;
        }
        break;
      case 23:
#if OXFORD_DISPLAY_INFO
        // ROS_INFO("Delay: %ums", packet->chan.chan23.delay_ms);
#endif
        break;
      case 27:
#if OXFORD_DISPLAY_INFO
        // ROS_INFO("Heading quality: %u", packet->chan.chan27.heading_quality);
#endif
        break;
      case 37:
#if OXFORD_DISPLAY_INFO
        if (packet->chan.chan37.valid) {
          // ROS_INFO("Heading Misalignment: Angle: %frad, Accuracy: %frad",
          //          (double)packet->chan.chan37.heading_misalignment_angle * 1e-4,
          //          (double)packet->chan.chan37.heading_misalignment_accuracy * 1e-4);
        }
#endif
        break;
      case 48:
#if OXFORD_DISPLAY_INFO
        // ROS_INFO("HDOP: %0.1f, PDOP: %0.1f",
        //          (double)packet->chan.chan48.HDOP * 1e-1,
        //          (double)packet->chan.chan48.PDOP * 1e-1);
#endif
        break;
    }

    
    msg_fix.header.stamp = stamp;
   // strcpy(msg_fix.header.frame_id,frame_id.c_str());
    msg_fix.latitude = packet->latitude * (180 / M_PI);
    msg_fix.longitude = packet->longitude * (180 / M_PI);
    msg_fix.altitude = packet->altitude;
    msg_fix.status.status = fix_status;
    msg_fix.status.service = SERVICE_GPS;
    msg_fix.position_covariance_type = position_covariance_type;
    if (position_covariance_type > COVARIANCE_TYPE_UNKNOWN) {
      msg_fix.position_covariance[0] = position_covariance[0]; // x
      msg_fix.position_covariance[4] = position_covariance[1]; // y
      msg_fix.position_covariance[8] = position_covariance[2]; // z
    }
    // pub_fix.publish(msg_fix);
  //strcpy( msg_vel.header.frame_id , frame_id_vel.c_str());
    
    msg_vel.header.stamp = stamp;

    msg_vel.twist.twist.linear.x = (double)packet->vel_east * 1e-4;
    msg_vel.twist.twist.linear.y = (double)packet->vel_north * 1e-4;
    msg_vel.twist.twist.linear.z = (double)packet->vel_down * -1e-4;
    if (velocity_covariance_type > COVARIANCE_TYPE_UNKNOWN) {
      msg_vel.twist.covariance[0] = velocity_covariance[0]; // x
      msg_vel.twist.covariance[7] = velocity_covariance[1]; // y
      msg_vel.twist.covariance[14]= velocity_covariance[2]; // z
    }
    // pub_vel.publish(msg_vel);

    //EulerAngles to RotationMatrix
    Eigen::Vector3d ea0((double)packet->heading * -1e-6,(double)packet->pitch * 1e-6,(double)packet->roll * 1e-6);
    Eigen::Matrix3d R;
    R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
        * ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
        * ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
 

    Eigen::Quaterniond q(R);
    // q.setRPY((double)packet->roll * 1e-6, (double)packet->pitch * 1e-6, (double)packet->heading * -1e-6);
    
    msg_imu.header.stamp = stamp;
    strcpy(msg_imu.header.frame_id ,frame_id.c_str());
    msg_imu.linear_acceleration.x = (double)packet->accel_x * 1e-4;
    msg_imu.linear_acceleration.y = (double)packet->accel_y * 1e-4;
    msg_imu.linear_acceleration.z = (double)packet->accel_z * -1e-4;
    msg_imu.angular_velocity.x = (double)packet->gyro_x * 1e-5;
    msg_imu.angular_velocity.y = (double)packet->gyro_y * 1e-5;
    msg_imu.angular_velocity.z = (double)packet->gyro_z * -1e-5;
    msg_imu.orientation.w = q.w();
    msg_imu.orientation.x = q.x();
    msg_imu.orientation.y = q.y();
    msg_imu.orientation.z = q.z();
    if (orientation_covariance_type > COVARIANCE_TYPE_UNKNOWN) {
      msg_imu.orientation_covariance[0] = orientation_covariance[0]; // x
      msg_imu.orientation_covariance[4] = orientation_covariance[1]; // y
      msg_imu.orientation_covariance[8] = orientation_covariance[2]; // z
    } else {
      msg_imu.orientation_covariance[0] = 0.0174532925; // x
      msg_imu.orientation_covariance[4] = 0.0174532925; // y
      msg_imu.orientation_covariance[8] = 0.0174532925; // z
    }
    msg_imu.angular_velocity_covariance[0] = 0.000436332313; // x
    msg_imu.angular_velocity_covariance[4] = 0.000436332313; // y
    msg_imu.angular_velocity_covariance[8] = 0.000436332313; // x
    msg_imu.linear_acceleration_covariance[0] = 0.0004; // x
    msg_imu.linear_acceleration_covariance[4] = 0.0004; // y
    msg_imu.linear_acceleration_covariance[8] = 0.0004; // z
    
    
    msg_odom.header.stamp = stamp;
    strcpy(msg_odom.header.frame_id , frame_id_vel.c_str());
    strcpy(msg_odom.child_frame_id , "base_link");
    msg_odom.twist = msg_vel.twist;
    // pub_odom.publish(msg_odom);
#if OXFORD_DISPLAY_INFO
  // } else {
  //   ROS_WARN("Nav Status: %u", packet->nav_status);
#endif
  }
}

class IMUEthernet: public icvUdpReceiverSource
{
public:

    typedef data::icvStructureData<Imu>    icvImu;
    typedef data::icvStructureData<NavSatFix>    icvNavSatFix;
    typedef data::icvStructureData<TwistWithCovarianceStamped>    icvTwistWithCovarianceStamped;
    typedef data::icvStructureData<Odometry>    icvOdometry;

  IMUEthernet(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info){


    packet=new Imu_Packet;
    Register_Pub("msg_imu");
    Register_Pub("msg_fix");
    Register_Pub("msg_vel");
    Register_Pub("msg_odom");
  
    /*msg_imu_t=new icvImu();
    msg_fix_t=new icvNavSatFix();
    msg_vel_t=new icvTwistWithCovarianceStamped();
    msg_odom_t=new icvOdometry(); */
	    };

int transintoint(char *pointer_data, int byte_sta,int numofbyte)
{
vector<int> temp;
temp.resize(numofbyte);
int total=0;

for(int j=0;j<numofbyte;j++)
{
  temp[j]=(int)(pointer_data[j]&0xFF);

  total+=temp[j]*pow(256,j);

}
return total;

}


  Imu_Packet* protocol_dec(char *pointer_data,Imu_Packet* result_p)
  {

    (*result_p).sync=transintoint(pointer_data,0,1);;

    result_p->time=transintoint(pointer_data,1,2);

    result_p->accel_x=transintoint(pointer_data,3,3);

    result_p->accel_y =transintoint(pointer_data,6,3);

    result_p->accel_z =transintoint(pointer_data,9,3);

    result_p->gyro_x =transintoint(pointer_data,12,3);

    result_p->gyro_y =transintoint(pointer_data,15,3);

    result_p->gyro_z =transintoint(pointer_data,18,3);

    result_p->nav_status =(int)pointer_data[21];

    result_p->chksum1 =(int)pointer_data[22];

    double temp_d1,temp_d2;float temp_f1;
    memcpy(&temp_d1,pointer_data+23,8);
    memcpy(&temp_d2,pointer_data+31,8);
    memcpy(&temp_f1,pointer_data+39,4);

    result_p->latitude =temp_d1;

    result_p->longitude =temp_d2;

    result_p->altitude =temp_f1;

    result_p->vel_north =transintoint(pointer_data,43,3);

    result_p->vel_east =transintoint(pointer_data,46,3);

    result_p->vel_down =transintoint(pointer_data,49,3);    

    result_p->heading =transintoint(pointer_data,52,3);

    result_p->pitch =transintoint(pointer_data,55,3);

    result_p->roll =transintoint(pointer_data,58,3);

    result_p->chksum2 =(int)pointer_data[61];

    result_p->channel =(int)pointer_data[62];

  for(int i=0;i<8;i++)result_p->chan.bytes[i]=(int)pointer_data[63+i];

    result_p->chksum3=(int)pointer_data[71];

    return result_p;
  }
  virtual void Process(std::istream& stream) override
  {

    //  ICV_LOG_INFO<< "buffer size:"<<_buffer.size() ;
	  if (_buffer.size()%72 == 0)
	  {
      //ICV_LOG_INFO<<"JK";
		
			// stream>>packet;
        while (!stream.eof())
		  {
              stream.read(pointer_imu,72);

            //packet=reinterpret_cast<Packet*>(pointer_imu);
            protocol_dec(pointer_imu,packet);
            handlePacket( packet,  msg_fix, msg_vel,   msg_imu, msg_odom, frame_id, frame_id_vel);
          
          // ICV_LOG_INFO<<"  imu data az:"<<msg_imu.linear_acceleration.z;
          // ICV_LOG_INFO<<"  imu data ax:"<<msg_imu.linear_acceleration.x;
          // ICV_LOG_INFO<<"  imu data angle:"<<msg_imu.angular_velocity.z;
          // ICV_LOG_INFO<<"  IMU DATA VELO"<<msg_vel.twist.twist.linear.x;


          // ICV_LOG_INFO<<"  imu data latitude:"<<msg_fix.latitude;
          // ICV_LOG_INFO<<"  imu data longititude:"<<msg_fix.longitude;



              
        msg_imu_t.setvalue(msg_imu);
        msg_fix_t.setvalue(msg_fix);
        msg_vel_t.setvalue(msg_vel);
        msg_odom_t.setvalue(msg_odom);
        icvPublish("IMU_data",&msg_imu_t);
        icvPublish("msg_fix",&msg_fix_t);
        icvPublish("msg_vel",&msg_vel_t);
        icvPublish("msg_odom",&msg_odom_t);
/*Deprecated
        Send_Out(msg_imu_t,0);
        Send_Out(msg_fix_t,1);
        Send_Out(msg_vel_t,2);
        Send_Out(msg_odom_t,3);
*/

      }



	  }
	  else
	  {
		  printf("Error: have not received packet data, or byte number is wrong!\n");
	  }
  

  }


	
	
private:
    NavSatFix msg_fix;
    TwistWithCovarianceStamped msg_vel;
    Imu msg_imu;
    Odometry msg_odom;
    char pointer_imu[72];

    icvNavSatFix msg_fix_t;
    icvTwistWithCovarianceStamped msg_vel_t;
    icvImu msg_imu_t;
    icvOdometry msg_odom_t;

    // Variables
    	std::string frame_id = "gps";
      std::string frame_id_vel = "utm";
    Imu_Packet* packet;
   
};
ICV_REGISTER_FUNCTION(IMUEthernet)

#endif  //

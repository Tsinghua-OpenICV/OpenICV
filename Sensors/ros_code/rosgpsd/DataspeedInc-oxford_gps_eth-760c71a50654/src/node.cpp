/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2017, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// ROS
#include <ros/ros.h>

// ROS messages
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

// Tf
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

// Packet structure
#include "dispatch.h"

// UTM conversion
#include <gps_common/conversions.h>

// Ethernet
#include <arpa/inet.h>

// UINT16_MAX is not defined by default in Ubuntu Saucy
#ifndef UINT16_MAX
#define UINT16_MAX (65535)
#endif

// GPS time to UTC time parameters
#define GPS_LEAP_SECONDS 18         // Offset to account for UTC leap seconds (need to increment when UTC changes)
#define GPS_EPOCH_OFFSET 315964800  // Offset to account for GPS / UTC epoch difference


static inline bool openSocket(const std::string &interface, const std::string &ip_addr, uint16_t port, int *fd_ptr, sockaddr_in *sock_ptr)
{
  // Create UDP socket
  int fd;
  fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd != -1) {
    if (interface.length()) {
      if (!setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, interface.c_str(), interface.length()) == 0) {
        close(fd);
        return false;
      }
    }
    memset(sock_ptr, 0, sizeof(sockaddr_in));
    sock_ptr->sin_family = AF_INET;
    sock_ptr->sin_port = htons(port);
    if (!inet_aton(ip_addr.c_str(), &sock_ptr->sin_addr)) {
      sock_ptr->sin_addr.s_addr = INADDR_ANY; // Invalid address, use ANY
    }
    if (bind(fd, (sockaddr*)sock_ptr, sizeof(sockaddr)) == 0) {
      *fd_ptr = fd;
      return true;
    }
  }
  return false;
}

static inline int readSocket(int fd, unsigned int timeout, void *data, size_t size, sockaddr *source_ptr = NULL)
{
  if (fd >= 0) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    // Set up timeout
    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout * 1000) % 1000000;

    if (select(fd + 1, &fds, NULL, NULL, &tv) > 0) {
      socklen_t socklen = source_ptr ? sizeof(*source_ptr) : 0;
      socklen_t *socklen_ptr = source_ptr ? &socklen : NULL;
      return recvfrom(fd, data, size, 0, source_ptr, socklen_ptr);
    }

    // Timeout
    return 0;
  }
  return -1;
}

static inline double SQUARE(double x) { return x * x; }

#ifndef OXFORD_DISPLAY_INFO
#define OXFORD_DISPLAY_INFO 0
#endif

static const std::map<uint8_t, std::string> POS_MODE_MAP = {
  {MODE_NONE, "NONE"},
  {MODE_NO_DATA, "NONE"},
  {MODE_BLANKED, "NONE"},
  {MODE_SEARCH, "SEARCHING"},
  {MODE_NOT_RECOGNISED, "NONE"},
  {MODE_UNKNOWN, "NONE"},
  {MODE_DOPPLER, "DOPPLER"},
  {MODE_DOPPLER_PP, "DOPPLER"},
  {MODE_DOPPLER_GX, "DOPPLER"},
  {MODE_DOPPLER_IX, "DOPPLER"},
  {MODE_SPS, "POINT_POSITION"},
  {MODE_SPS_PP, "POINT_POSITION"},
  {MODE_SPS_GX, "POINT_POSITION"},
  {MODE_SPS_IX, "POINT_POSITION"},
  {MODE_DIFFERENTIAL, "DIFF_PSEUDORANGE"},
  {MODE_DIFFERENTIAL_PP, "DIFF_PSEUDORANGE"},
  {MODE_DIFFERENTIAL_GX, "DIFF_PSEUDORANGE"},
  {MODE_DIFFERENTIAL_IX, "DIFF_PSEUDORANGE"},
  {MODE_RTK_FLOAT, "RTK_FLOAT"},
  {MODE_RTK_FLOAT_PP, "RTK_FLOAT"},
  {MODE_RTK_FLOAT_GX, "RTK_FLOAT"},
  {MODE_RTK_FLOAT_IX, "RTK_FLOAT"},
  {MODE_RTK_INTEGER, "RTK_INTEGER"},
  {MODE_RTK_INTEGER_PP, "RTK_INTEGER"},
  {MODE_RTK_INTEGER_GX, "RTK_INTEGER"},
  {MODE_RTK_INTEGER_IX, "RTK_INTEGER"},
  {MODE_WAAS, "WAAS"},
  {MODE_OMNISTAR_VBS, "OMNISTAR_VBS"},
  {MODE_OMNISTAR_HP, "OMNISTAR_HP"},
  {MODE_OMNISTAR_XP, "OMNISTAR_XP"},
  {MODE_CDGPS, "CANADA_DGPS"},
  {MODE_PPP_CONVERGING, "PPP_CONVERGING"},
  {MODE_PPP, "PPP"}
};

static const std::map<uint8_t, std::string> NAV_STATUS_MAP = {
  {0, "INVALID"},
  {1, "IMU_ONLY"},
  {2, "INITIALIZING"},
  {3, "LOCKING"},
  {4, "READY"},
};

static inline void mapLookup(const std::map<uint8_t, std::string>& map, uint8_t key, std::string& val) {
  std::map<uint8_t, std::string>::const_iterator it = map.find(key);
  if (it != map.end()) {
    val = map.at(key);
  } else {
    val = "UNKNOWN";
  }
}

static inline double getZoneMeridian(const std::string& utm_zone)
{
  int zone_number = std::atoi(utm_zone.substr(0,2).c_str());
  return (zone_number == 0) ? 0.0 : (zone_number - 1) * 6.0 - 177.0;
}

static inline double toUtcTime(uint32_t gps_minutes, uint16_t gps_ms)
{
  return GPS_EPOCH_OFFSET - GPS_LEAP_SECONDS + 60.0 * (double)gps_minutes + 0.001 * (double)gps_ms;
}

static inline void handlePacket(const Packet *packet, ros::Publisher &pub_fix, ros::Publisher &pub_vel,
                                ros::Publisher &pub_imu, ros::Publisher &pub_odom, ros::Publisher &pub_pos_type,
                                ros::Publisher &pub_nav_status, ros::Publisher &pub_gps_time_ref, const std::string & frame_id_gps,
                                const std::string &frame_id_vel, const std::string &frame_id_odom)
{
  static uint8_t fix_status = sensor_msgs::NavSatStatus::STATUS_FIX;
  static uint8_t position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  static double position_covariance[3];
  static uint8_t velocity_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  static double velocity_covariance[3];
  static uint8_t orientation_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  static double orientation_covariance[3];
  static uint8_t pos_mode = 0;
  static int none_type_count = 0;

  ros::Time stamp = ros::Time::now();

  switch (packet->channel) {
    case 0:
      if (packet->chan.chan0.position_mode != MODE_NONE) {
        pos_mode = packet->chan.chan0.position_mode;
        none_type_count = 0;
      } else {
        if (none_type_count > 2) {
          none_type_count = 0;
          pos_mode = MODE_NONE;
        } else {
          none_type_count++;
        }
      }

      if (packet->chan.chan0.gps_minutes > 1000) { // Documentation says invalid if < 1000
        sensor_msgs::TimeReference gps_time_ref_msg;
        gps_time_ref_msg.source = "gps";
        gps_time_ref_msg.header.stamp = stamp;
        gps_time_ref_msg.time_ref = ros::Time(toUtcTime(packet->chan.chan0.gps_minutes, packet->time));
        pub_gps_time_ref.publish(gps_time_ref_msg);
      }

      switch (pos_mode) {
        case MODE_DIFFERENTIAL:
        case MODE_DIFFERENTIAL_PP:
        case MODE_DIFFERENTIAL_GX:
        case MODE_DIFFERENTIAL_IX:
        case MODE_RTK_FLOAT:
        case MODE_RTK_FLOAT_PP:
        case MODE_RTK_FLOAT_GX:
        case MODE_RTK_FLOAT_IX:
        case MODE_RTK_INTEGER:
        case MODE_RTK_INTEGER_PP:
        case MODE_RTK_INTEGER_GX:
        case MODE_RTK_INTEGER_IX:
          fix_status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
          break;
        case MODE_OMNISTAR_VBS:
        case MODE_OMNISTAR_HP:
        case MODE_OMNISTAR_XP:
        case MODE_WAAS:
        case MODE_CDGPS:
          fix_status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
          break;
        case MODE_SPS:
        case MODE_SPS_PP:
        case MODE_SPS_GX:
        case MODE_SPS_IX:
          fix_status = sensor_msgs::NavSatStatus::STATUS_FIX;
          break;
        case MODE_NONE:
        case MODE_SEARCH:
        case MODE_DOPPLER:
        case MODE_NO_DATA:
        case MODE_BLANKED:
        case MODE_NOT_RECOGNISED:
        case MODE_UNKNOWN:
        default:
          fix_status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
          break;
      }
#if OXFORD_DISPLAY_INFO
    ROS_INFO("Num Sats: %u, Position mode: %u, Velocity mode: %u, Orientation mode: %u",
             packet->chan.chan0.num_sats,
             packet->chan.chan0.position_mode,
             packet->chan.chan0.velocity_mode,
             packet->chan.chan0.orientation_mode);
#endif
      break;
    case 3:
      if (packet->chan.chan3.age < 150) {
        position_covariance[0] = SQUARE((double)packet->chan.chan3.acc_position_east * 1e-3);
        position_covariance[1] = SQUARE((double)packet->chan.chan3.acc_position_north * 1e-3);
        position_covariance[2] = SQUARE((double)packet->chan.chan3.acc_position_down * 1e-3);
        position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
#if OXFORD_DISPLAY_INFO
        ROS_INFO("Position accuracy: North: %umm, East: %umm, Down: %umm",
                 packet->chan.chan3.acc_position_north,
                 packet->chan.chan3.acc_position_east,
                 packet->chan.chan3.acc_position_down);
#endif
      } else {
        position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      }
      break;
    case 4:
      if (packet->chan.chan4.age < 150) {
        velocity_covariance[0] = SQUARE((double)packet->chan.chan4.acc_velocity_east * 1e-3);
        velocity_covariance[1] = SQUARE((double)packet->chan.chan4.acc_velocity_north * 1e-3);
        velocity_covariance[2] = SQUARE((double)packet->chan.chan4.acc_velocity_down * 1e-3);
        velocity_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
#if OXFORD_DISPLAY_INFO
        ROS_INFO("Velocity accuracy: North: %umm/s, East: %umm/s, Down: %umm/s",
                 packet->chan.chan4.acc_velocity_north,
                 packet->chan.chan4.acc_velocity_east,
                 packet->chan.chan4.acc_velocity_down);
#endif
      } else {
        velocity_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      }
      break;

    case 5:
      if (packet->chan.chan5.age < 150) {
        orientation_covariance[0] = SQUARE((double)packet->chan.chan5.acc_roll * 1e-5);
        orientation_covariance[1] = SQUARE((double)packet->chan.chan5.acc_pitch * 1e-5);
        orientation_covariance[2] = SQUARE((double)packet->chan.chan5.acc_heading * 1e-5);
        orientation_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
#if OXFORD_DISPLAY_INFO
        ROS_INFO("Velocity accuracy: Heading: %frad, Pitch: %frad, Roll: %frad",
                 (double)packet->chan.chan5.acc_heading * 1e-5,
                 (double)packet->chan.chan5.acc_pitch * 1e-5,
                 (double)packet->chan.chan5.acc_roll * 1e-5);
#endif
      } else {
        orientation_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      }
      break;
    case 23:
#if OXFORD_DISPLAY_INFO
      ROS_INFO("Delay: %ums", packet->chan.chan23.delay_ms);
#endif
      break;
    case 27:
#if OXFORD_DISPLAY_INFO
      ROS_INFO("Heading quality: %u", packet->chan.chan27.heading_quality);
#endif
      break;
    case 37:
#if OXFORD_DISPLAY_INFO
      if (packet->chan.chan37.valid) {
          ROS_INFO("Heading Misalignment: Angle: %frad, Accuracy: %frad",
                   (double)packet->chan.chan37.heading_misalignment_angle * 1e-4,
                   (double)packet->chan.chan37.heading_misalignment_accuracy * 1e-4);
        }
#endif
      break;
    case 48:
#if OXFORD_DISPLAY_INFO
      ROS_INFO("HDOP: %0.1f, PDOP: %0.1f",
               (double)packet->chan.chan48.HDOP * 1e-1,
               (double)packet->chan.chan48.PDOP * 1e-1);
#endif
      break;
  }
  std_msgs::String str_msg;
  mapLookup(POS_MODE_MAP, pos_mode, str_msg.data);
  pub_pos_type.publish(str_msg);
  mapLookup(NAV_STATUS_MAP, packet->nav_status, str_msg.data);
  pub_nav_status.publish(str_msg);

  if (packet->nav_status == 4) {
    // Convert lat/lon into UTM x, y, and zone
    double utm_x;
    double utm_y;
    std::string utm_zone;
    gps_common::LLtoUTM(180.0 / M_PI * packet->latitude, 180.0 / M_PI * packet->longitude, utm_y, utm_x, utm_zone);

    // Compute convergence angle and heading in ENU and UTM grid
    double central_meridian = M_PI / 180.0 * getZoneMeridian(utm_zone);
    double convergence_angle = atan(tan(packet->longitude - central_meridian) * sin(packet->latitude));
    double enu_heading = M_PI_2 - (double)packet->heading * 1e-6;
    double grid_heading = enu_heading + convergence_angle;

    // Compute local frame velocity for odometry message
    double east_vel = (double)packet->vel_east * 1e-4;
    double north_vel = (double)packet->vel_north * 1e-4;
    double local_x_vel = east_vel * cos(enu_heading) + north_vel * sin(enu_heading);
    double local_y_vel = -east_vel * sin(enu_heading) + north_vel * cos(enu_heading);

    sensor_msgs::NavSatFix msg_fix;
    msg_fix.header.stamp = stamp;
    msg_fix.header.frame_id = frame_id_gps;
    msg_fix.latitude = packet->latitude * (180 / M_PI);
    msg_fix.longitude = packet->longitude * (180 / M_PI);
    msg_fix.altitude = packet->altitude;
    msg_fix.status.status = fix_status;
    msg_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    msg_fix.position_covariance_type = position_covariance_type;
    if (position_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      msg_fix.position_covariance[0] = position_covariance[0]; // x
      msg_fix.position_covariance[4] = position_covariance[1]; // y
      msg_fix.position_covariance[8] = position_covariance[2]; // z
    }
    pub_fix.publish(msg_fix);

    geometry_msgs::TwistWithCovarianceStamped msg_vel;
    msg_vel.header.stamp = stamp;
    msg_vel.header.frame_id = frame_id_vel;
    msg_vel.twist.twist.linear.x = east_vel;
    msg_vel.twist.twist.linear.y = north_vel;
    msg_vel.twist.twist.linear.z = (double)packet->vel_down * -1e-4;
    if (velocity_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      msg_vel.twist.covariance[0] = velocity_covariance[0]; // x
      msg_vel.twist.covariance[7] = velocity_covariance[1]; // y
      msg_vel.twist.covariance[14] = velocity_covariance[2]; // z
    }
    pub_vel.publish(msg_vel);

    tf::Quaternion q;
    q.setRPY((double)packet->roll * 1e-6, (double)packet->pitch * 1e-6, enu_heading);
    sensor_msgs::Imu msg_imu;
    msg_imu.header.stamp = stamp;
    msg_imu.header.frame_id = frame_id_gps;
    msg_imu.linear_acceleration.x = (double)packet->accel_x *  1e-4;
    msg_imu.linear_acceleration.y = (double)packet->accel_y *  1e-4;
    msg_imu.linear_acceleration.z = (double)packet->accel_z * -1e-4;
    msg_imu.angular_velocity.x = (double)packet->gyro_x *  1e-5;
    msg_imu.angular_velocity.y = (double)packet->gyro_y *  1e-5;
    msg_imu.angular_velocity.z = (double)packet->gyro_z * -1e-5;
    msg_imu.orientation.w = q.w();
    msg_imu.orientation.x = q.x();
    msg_imu.orientation.y = q.y();
    msg_imu.orientation.z = q.z();
    if (orientation_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      msg_imu.orientation_covariance[0] = orientation_covariance[0]; // x
      msg_imu.orientation_covariance[4] = orientation_covariance[1]; // y
      msg_imu.orientation_covariance[8] = orientation_covariance[2]; // z
    }
    pub_imu.publish(msg_imu);

    nav_msgs::Odometry msg_odom;
    msg_odom.header.stamp = stamp;
    msg_odom.header.frame_id = "utm";
    msg_odom.child_frame_id = frame_id_odom;
    msg_odom.pose.pose.position.x = utm_x;
    msg_odom.pose.pose.position.y = utm_y;
    msg_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(grid_heading);

    // Project position standard deviations into UTM frame, accounting for convergence angle
    double std_east = (double)packet->chan.chan3.acc_position_east * 1e-3;
    double std_north = (double)packet->chan.chan3.acc_position_north * 1e-3;
    double std_x = std_east * cos(convergence_angle) - std_north * sin(convergence_angle);
    double std_y = std_north * sin(convergence_angle) + std_east * cos(convergence_angle);

    // Project velocity standard deviations into local frame
    double std_east_vel = (double)packet->chan.chan4.acc_velocity_east * 1e-3;
    double std_north_vel = (double)packet->chan.chan4.acc_velocity_north * 1e-3;
    double std_x_vel = std_east_vel * cos(enu_heading) + std_north_vel * sin(enu_heading);
    double std_y_vel = -std_east_vel * cos(enu_heading) + std_north_vel * sin(enu_heading);

    msg_odom.pose.covariance[0*6 + 0] = SQUARE(std_x);
    msg_odom.pose.covariance[1*6 + 1] = SQUARE(std_y);
    msg_odom.pose.covariance[5*6 + 5] = orientation_covariance[2];
    msg_odom.twist.twist.linear.x = local_x_vel;
    msg_odom.twist.twist.linear.y = local_y_vel;
    msg_odom.twist.twist.angular.z = msg_imu.angular_velocity.z;
    msg_odom.twist.covariance[0*6 + 0] = SQUARE(std_x_vel);
    msg_odom.twist.covariance[1*6 + 1] = SQUARE(std_y_vel);
    pub_odom.publish(msg_odom);

#if OXFORD_DISPLAY_INFO
  } else {
    ROS_WARN("Nav Status: %u", packet->nav_status);
#endif
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oxford_gps");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  std::string interface = "";
  priv_nh.getParam("interface", interface);

  std::string ip_addr = "";
  priv_nh.getParam("ip_address", ip_addr);

  int port = 3000;
  priv_nh.getParam("port", port);

  std::string frame_id_gps = "gps";
  priv_nh.getParam("frame_id_gps", frame_id_gps);

  std::string frame_id_vel = "enu";
  priv_nh.getParam("frame_id_vel", frame_id_vel);

  std::string frame_id_odom = "base_footprint";
  priv_nh.getParam("frame_id_odom", frame_id_odom);

  if (port > UINT16_MAX) {
    ROS_ERROR("Port %u greater than maximum value of %u", port, UINT16_MAX);
  }

  if (interface.length() && ip_addr.length()) {
    ROS_INFO("Preparing to listen on interface %s port %u and IP %s", interface.c_str(), port, ip_addr.c_str());
  } else if (interface.length()) {
    ROS_INFO("Preparing to listen on interface %s port %u", interface.c_str(), port);
  } else if (ip_addr.length()) {
    ROS_INFO("Preparing to listen on %s:%u", ip_addr.c_str(), port);
  } else {
    ROS_INFO("Preparing to listen on port %u", port);
  }

  int fd;
  sockaddr_in sock;
  if (openSocket(interface, ip_addr, port, &fd, &sock)) {
    // Setup Publishers
    ros::Publisher pub_fix = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 2);
    ros::Publisher pub_vel = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("gps/vel", 2);
    ros::Publisher pub_imu = node.advertise<sensor_msgs::Imu>("imu/data", 2);
    ros::Publisher pub_odom = node.advertise<nav_msgs::Odometry>("gps/odom", 2);
    ros::Publisher pub_pos_type = node.advertise<std_msgs::String>("gps/pos_type", 2);
    ros::Publisher pub_nav_status = node.advertise<std_msgs::String>("gps/nav_status", 2);
    ros::Publisher pub_gps_time_ref = node.advertise<sensor_msgs::TimeReference>("gps/time_ref", 2);

    // Variables
    Packet packet;
    sockaddr source;
    bool first = true;

    // Loop until shutdown
    while (ros::ok()) {
      if (readSocket(fd, 10, &packet, sizeof(packet), &source) >= sizeof(packet)) {
        if (validatePacket(&packet)) {
          if (first) {
            first = false;
            ROS_INFO("Connected to Oxford GPS at %s:%u", inet_ntoa(((sockaddr_in*)&source)->sin_addr), htons(((sockaddr_in*)&source)->sin_port));
          }
          handlePacket(&packet, pub_fix, pub_vel, pub_imu, pub_odom, pub_pos_type, pub_nav_status, pub_gps_time_ref, frame_id_gps, frame_id_vel, frame_id_odom);
        }
      }

      // Handle callbacks
      ros::spinOnce();
    }

    // Close socket
    close(fd);
  } else {
    ROS_FATAL("Failed to open socket");
    ros::WallDuration(1.0).sleep();
  }

  return 0;
}

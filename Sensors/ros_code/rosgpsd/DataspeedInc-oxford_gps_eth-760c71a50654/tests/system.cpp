/*********************************************************************
 * C++ unit test for oxford_gps_eth
 * Verify correct handling of UDP packets
 *********************************************************************/

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

// Tf Quaternions
#include <tf/LinearMath/Quaternion.h>

// Ethernet
#include <arpa/inet.h>
#include <sys/socket.h>

// Helper classes
#include "MsgRx.h"

// Packet structure
#include "../src/dispatch.h"

// Parameters
std::string g_ip_address = "";
int g_port = 3000;
std::string g_frame_id_gps = "gps";
std::string g_frame_id_vel = "enu";
std::string g_frame_id_odom = "base_footprint";

// Subscribed topics
ros::Subscriber g_sub_fix;
ros::Subscriber g_sub_vel;
ros::Subscriber g_sub_imu;

// Received messages
MsgRx<sensor_msgs::NavSatFix> g_msg_fix;
MsgRx<geometry_msgs::TwistWithCovarianceStamped> g_msg_vel;
MsgRx<sensor_msgs::Imu> g_msg_imu;
void recvClear() {
  g_msg_fix.clear();
  g_msg_vel.clear();
  g_msg_imu.clear();
}

// Subscriber receive callbacks
void recvGpsFix(const sensor_msgs::NavSatFix::ConstPtr& msg) { g_msg_fix.set(*msg); }
void recvGpsVel(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg) { g_msg_vel.set(*msg); }
void recvImuData(const sensor_msgs::Imu::ConstPtr& msg) { g_msg_imu.set(*msg); }

// Set sync byte and apply checksums
static void fixPacket(Packet &packet) {
  packet.sync = 0xE7;
  const uint8_t *ptr = ((uint8_t*)&packet) + 1;
  uint8_t chksum = 0;
  for (; ptr < &packet.chksum1; ptr++) {
    chksum += *ptr;
  }
  packet.chksum1 = chksum;
  for (; ptr < &packet.chksum2; ptr++) {
    chksum += *ptr;
  }
  packet.chksum2 = chksum;
  for (; ptr < &packet.chksum3; ptr++) {
    chksum += *ptr;
  }
  packet.chksum3 = chksum;
}

// Open socket and send UDP packet
static bool sendPacket(const Packet &packet) {
  recvClear();
  static int fd = -1;
  if (fd == -1) {
    fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // Create UDP socket
  }
  if (fd != -1) {
    struct sockaddr_in si_other;
    memset(&si_other, 0x00, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(g_port);
    if (inet_aton(g_ip_address.c_str(), &si_other.sin_addr)) {
      if (sendto(fd, &packet, sizeof(packet), 0, (struct sockaddr *) &si_other, sizeof(si_other)) == sizeof(packet)) {
        return true;
      }
    }
  }
  return false;
}

// Helper wait functions
static inline double SQUARE(double x) { return x * x; }
static bool waitForTopics(ros::WallDuration dur) {
  const ros::WallTime start = ros::WallTime::now();
  while (true) {
    if ((g_sub_fix.getNumPublishers() == 1) &&
        (g_sub_vel.getNumPublishers() == 1) &&
        (g_sub_imu.getNumPublishers() == 1)) {
      return true;
    }
    if ((ros::WallTime::now() - start) > dur) {
      return false;
    }
    ros::WallDuration(0.001).sleep();
    ros::spinOnce();
  }
}
static bool waitForMsgs(ros::WallDuration dur) {
  const ros::WallTime start = ros::WallTime::now();
  while (true) {
    if (g_msg_fix.valid() && g_msg_vel.valid() && g_msg_imu.valid()) {
      return true;
    }
    if ((ros::WallTime::now() - start) > dur) {
      return false;
    }
    ros::WallDuration(0.001).sleep();
    ros::spinOnce();
  }
}

// Wait for node to startup and verify connections
TEST(Main, topics)
{
  EXPECT_TRUE(waitForTopics(ros::WallDuration(2.0)));
  ASSERT_EQ(1, g_sub_fix.getNumPublishers());
  ASSERT_EQ(1, g_sub_vel.getNumPublishers());
  ASSERT_EQ(1, g_sub_imu.getNumPublishers());
  EXPECT_FALSE(g_msg_fix.valid());
  EXPECT_FALSE(g_msg_vel.valid());
  EXPECT_FALSE(g_msg_imu.valid());
}

// Verify correct handling of packets
TEST(Main, packets)
{
  // Variables
  Packet packet;
  const ros::WallDuration DURATION(0.2);
  memset(&packet, 0x00, sizeof(packet));

  // Empty packet should fail the sync byte and checksum tests
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_FALSE(waitForMsgs(DURATION));
  EXPECT_FALSE(g_msg_fix.valid());
  EXPECT_FALSE(g_msg_vel.valid());
  EXPECT_FALSE(g_msg_imu.valid());

  // Empty packet should fail because nav_status is missing
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_FALSE(waitForMsgs(DURATION));
  EXPECT_FALSE(g_msg_fix.valid());
  EXPECT_FALSE(g_msg_vel.valid());
  EXPECT_FALSE(g_msg_imu.valid());

  // Just enough data for a successful packet
  packet.nav_status = 4;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
}

// Verify message frame_ids and timestamps
TEST(Main, header)
{
  // Variables
  Packet packet;
  const ros::WallDuration DURATION(0.2);
  memset(&packet, 0x00, sizeof(packet));

  // Verify that all messages have the same non-zero timestamp and that the frame_ids are correct
  ros::Time stamp = ros::Time::now();
  packet.nav_status = 4;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_fix.get().header.frame_id.c_str(), g_frame_id_gps.c_str());
  EXPECT_STREQ(g_msg_vel.get().header.frame_id.c_str(), g_frame_id_vel.c_str());
  EXPECT_STREQ(g_msg_imu.get().header.frame_id.c_str(), g_frame_id_gps.c_str());
  EXPECT_EQ(g_msg_fix.get().header.stamp, g_msg_vel.get().header.stamp);
  EXPECT_EQ(g_msg_fix.get().header.stamp, g_msg_imu.get().header.stamp);
  EXPECT_NE(g_msg_fix.get().header.stamp, ros::Time(0));
  EXPECT_NEAR((g_msg_fix.get().header.stamp - stamp).toSec(), 0, 0.01);
}

// Verify correct fields in the sensor_msgs::NavSatFix message
TEST(Main, fix)
{
  // Variables
  Packet packet;
  const ros::WallDuration DURATION(0.2);
  memset(&packet, 0x00, sizeof(packet));

  // Send valid packet
  packet.nav_status = 4;
  packet.latitude = 1.23456789012345;
  packet.longitude = 9.87654321098765;
  packet.altitude = 1.234567;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_fix.get().header.frame_id.c_str(), g_frame_id_gps.c_str());
  EXPECT_EQ(g_msg_fix.get().status.status, sensor_msgs::NavSatStatus::STATUS_NO_FIX);
  EXPECT_EQ(g_msg_fix.get().status.service, sensor_msgs::NavSatStatus::SERVICE_GPS);
  EXPECT_EQ(g_msg_fix.get().latitude, packet.latitude * (180 / M_PI));
  EXPECT_EQ(g_msg_fix.get().longitude, packet.longitude * (180 / M_PI));
  EXPECT_EQ(g_msg_fix.get().altitude, packet.altitude);
  EXPECT_EQ(g_msg_fix.get().position_covariance_type, sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
  EXPECT_EQ(g_msg_fix.get().position_covariance[0], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[1], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[2], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[3], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[4], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[5], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[6], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[7], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[8], 0);

  // Set position mode on channel 0
  memset(&packet.chan, 0x00, sizeof(packet.chan));
  packet.channel = 0;
  packet.chan.chan0.position_mode = MODE_RTK_INTEGER;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_fix.get().header.frame_id.c_str(), g_frame_id_gps.c_str());
  EXPECT_EQ(g_msg_fix.get().status.status, sensor_msgs::NavSatStatus::STATUS_GBAS_FIX);
  EXPECT_EQ(g_msg_fix.get().status.service, sensor_msgs::NavSatStatus::SERVICE_GPS);
  EXPECT_EQ(g_msg_fix.get().latitude, packet.latitude * (180 / M_PI));
  EXPECT_EQ(g_msg_fix.get().longitude, packet.longitude * (180 / M_PI));
  EXPECT_EQ(g_msg_fix.get().altitude, packet.altitude);
  EXPECT_EQ(g_msg_fix.get().position_covariance_type, sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
  EXPECT_EQ(g_msg_fix.get().position_covariance[0], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[1], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[2], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[3], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[4], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[5], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[6], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[7], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[8], 0);

  // Set position covariance on channel 3
  memset(&packet.chan, 0x00, sizeof(packet.chan));
  packet.channel = 3;
  packet.chan.chan3.age = 10;
  packet.chan.chan3.acc_position_east  = 123;
  packet.chan.chan3.acc_position_north = 1234;
  packet.chan.chan3.acc_position_down  = 12345;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_fix.get().header.frame_id.c_str(), g_frame_id_gps.c_str());
  EXPECT_EQ(g_msg_fix.get().status.status, sensor_msgs::NavSatStatus::STATUS_GBAS_FIX);
  EXPECT_EQ(g_msg_fix.get().status.service, sensor_msgs::NavSatStatus::SERVICE_GPS);
  EXPECT_EQ(g_msg_fix.get().latitude, packet.latitude * (180 / M_PI));
  EXPECT_EQ(g_msg_fix.get().longitude, packet.longitude * (180 / M_PI));
  EXPECT_EQ(g_msg_fix.get().altitude, packet.altitude);
  EXPECT_EQ(g_msg_fix.get().position_covariance_type, sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
  EXPECT_EQ(g_msg_fix.get().position_covariance[0], SQUARE(packet.chan.chan3.acc_position_east * 1e-3)); // x
  EXPECT_EQ(g_msg_fix.get().position_covariance[1], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[2], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[3], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[4], SQUARE(packet.chan.chan3.acc_position_north * 1e-3)); // y
  EXPECT_EQ(g_msg_fix.get().position_covariance[5], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[6], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[7], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[8], SQUARE(packet.chan.chan3.acc_position_down * 1e-3)); // z

  // Set position covariance on channel 3 with old age
  memset(&packet.chan, 0x00, sizeof(packet.chan));
  packet.channel = 3;
  packet.chan.chan3.age = 190;
  packet.chan.chan3.acc_position_east  = 123;
  packet.chan.chan3.acc_position_north = 1234;
  packet.chan.chan3.acc_position_down  = 12345;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_fix.get().header.frame_id.c_str(), g_frame_id_gps.c_str());
  EXPECT_EQ(g_msg_fix.get().status.status, sensor_msgs::NavSatStatus::STATUS_GBAS_FIX);
  EXPECT_EQ(g_msg_fix.get().status.service, sensor_msgs::NavSatStatus::SERVICE_GPS);
  EXPECT_EQ(g_msg_fix.get().latitude, packet.latitude * (180 / M_PI));
  EXPECT_EQ(g_msg_fix.get().longitude, packet.longitude * (180 / M_PI));
  EXPECT_EQ(g_msg_fix.get().altitude, packet.altitude);
  EXPECT_EQ(g_msg_fix.get().position_covariance_type, sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
  EXPECT_EQ(g_msg_fix.get().position_covariance[0], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[1], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[2], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[3], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[4], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[5], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[6], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[7], 0);
  EXPECT_EQ(g_msg_fix.get().position_covariance[8], 0);
}

// Verify correct fields in the geometry_msgs::TwistWithCovarianceStamped message
TEST(Main, vel)
{
  // Variables
  Packet packet;
  const ros::WallDuration DURATION(0.2);
  memset(&packet, 0x00, sizeof(packet));

  // Send valid packet
  packet.nav_status = 4;
  packet.vel_east  =  1234567;
  packet.vel_north = -7654321;
  packet.vel_down  =  3456789;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_vel.get().header.frame_id.c_str(), g_frame_id_vel.c_str());
  EXPECT_EQ(g_msg_vel.get().twist.twist.linear.x, (double)packet.vel_east  * 1e-4);
  EXPECT_EQ(g_msg_vel.get().twist.twist.linear.y, (double)packet.vel_north * 1e-4);
  EXPECT_EQ(g_msg_vel.get().twist.twist.linear.z, (double)packet.vel_down * -1e-4);
  for (size_t i = 0; i < g_msg_vel.get().twist.covariance.size(); i++) {
    EXPECT_EQ(g_msg_vel.get().twist.covariance[i], 0);
  }

  // Set velocity covariance on channel 4
  memset(&packet.chan, 0x00, sizeof(packet.chan));
  packet.channel = 4;
  packet.chan.chan4.age = 10;
  packet.chan.chan4.acc_velocity_east  = 123;
  packet.chan.chan4.acc_velocity_north = 1234;
  packet.chan.chan4.acc_velocity_down  = 12345;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_vel.get().header.frame_id.c_str(), g_frame_id_vel.c_str());
  EXPECT_EQ(g_msg_vel.get().twist.twist.linear.x, (double)packet.vel_east  * 1e-4);
  EXPECT_EQ(g_msg_vel.get().twist.twist.linear.y, (double)packet.vel_north * 1e-4);
  EXPECT_EQ(g_msg_vel.get().twist.twist.linear.z, (double)packet.vel_down * -1e-4);
  EXPECT_EQ(g_msg_vel.get().twist.covariance[0], SQUARE(packet.chan.chan4.acc_velocity_east  * 1e-3)); // x
  EXPECT_EQ(g_msg_vel.get().twist.covariance[7], SQUARE(packet.chan.chan4.acc_velocity_north * 1e-3)); // y
  EXPECT_EQ(g_msg_vel.get().twist.covariance[14], SQUARE(packet.chan.chan4.acc_velocity_down * 1e-3)); // z

  // Set velocity covariance on channel 4 with old age
  memset(&packet.chan, 0x00, sizeof(packet.chan));
  packet.channel = 4;
  packet.chan.chan4.age = 190;
  packet.chan.chan4.acc_velocity_east  = 123;
  packet.chan.chan4.acc_velocity_north = 1234;
  packet.chan.chan4.acc_velocity_down  = 12345;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_vel.get().header.frame_id.c_str(), g_frame_id_vel.c_str());
  EXPECT_EQ(g_msg_vel.get().twist.twist.linear.x, (double)packet.vel_east  * 1e-4);
  EXPECT_EQ(g_msg_vel.get().twist.twist.linear.y, (double)packet.vel_north * 1e-4);
  EXPECT_EQ(g_msg_vel.get().twist.twist.linear.z, (double)packet.vel_down * -1e-4);
  for (size_t i = 0; i < g_msg_vel.get().twist.covariance.size(); i++) {
    EXPECT_EQ(g_msg_vel.get().twist.covariance[i], 0);
  }
}

// Verify correct fields in the sensor_msgs::Imu message
TEST(Main, imu)
{
  // Variables
  Packet packet;
  const ros::WallDuration DURATION(0.2);
  memset(&packet, 0x00, sizeof(packet));
  tf::Quaternion q;

  // Send valid packet
  packet.nav_status = 4;
  packet.accel_x =  1234567;
  packet.accel_y = -7654321;
  packet.accel_z =  3456789;
  packet.gyro_x  =  2345678;
  packet.gyro_y  = -6543210;
  packet.gyro_z  =  4567890;
  packet.heading =  5678901;
  packet.pitch   =  -123456;
  packet.roll    =   234567;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_imu.get().header.frame_id.c_str(), g_frame_id_gps.c_str());
  q.setRPY((double)packet.roll * 1e-6, (double)packet.pitch * 1e-6, M_PI_2 - (double)packet.heading * 1e-6);
  EXPECT_EQ(g_msg_imu.get().orientation.w, q.w());
  EXPECT_EQ(g_msg_imu.get().orientation.x, q.x());
  EXPECT_EQ(g_msg_imu.get().orientation.y, q.y());
  EXPECT_EQ(g_msg_imu.get().orientation.z, q.z());
  EXPECT_EQ(g_msg_imu.get().linear_acceleration.x, (double)packet.accel_x *  1e-4);
  EXPECT_EQ(g_msg_imu.get().linear_acceleration.y, (double)packet.accel_y *  1e-4);
  EXPECT_EQ(g_msg_imu.get().linear_acceleration.z, (double)packet.accel_z * -1e-4);
  EXPECT_EQ(g_msg_imu.get().angular_velocity.x, (double)packet.gyro_x *  1e-5);
  EXPECT_EQ(g_msg_imu.get().angular_velocity.y, (double)packet.gyro_y *  1e-5);
  EXPECT_EQ(g_msg_imu.get().angular_velocity.z, (double)packet.gyro_z * -1e-5);
  EXPECT_EQ(g_msg_imu.get().angular_velocity_covariance[0], 0.0); // x
  EXPECT_EQ(g_msg_imu.get().angular_velocity_covariance[4], 0.0); // y
  EXPECT_EQ(g_msg_imu.get().angular_velocity_covariance[8], 0.0); // z
  EXPECT_EQ(g_msg_imu.get().linear_acceleration_covariance[0], 0.0); // x
  EXPECT_EQ(g_msg_imu.get().linear_acceleration_covariance[4], 0.0); // y
  EXPECT_EQ(g_msg_imu.get().linear_acceleration_covariance[8], 0.0); // z
  EXPECT_EQ(g_msg_imu.get().orientation_covariance[0], 0.0); // x
  EXPECT_EQ(g_msg_imu.get().orientation_covariance[4], 0.0); // y
  EXPECT_EQ(g_msg_imu.get().orientation_covariance[8], 0.0); // z

  // Set orientation covariance on channel 5
  memset(&packet.chan, 0x00, sizeof(packet.chan));
  packet.channel = 5;
  packet.chan.chan5.age = 10;
  packet.chan.chan5.acc_heading = 123;
  packet.chan.chan5.acc_pitch   = 1234;
  packet.chan.chan5.acc_roll    = 12345;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_imu.get().header.frame_id.c_str(), g_frame_id_gps.c_str());
  q.setRPY((double)packet.roll * 1e-6, (double)packet.pitch * 1e-6, M_PI_2 - (double)packet.heading * 1e-6);
  EXPECT_EQ(g_msg_imu.get().orientation.w, q.w());
  EXPECT_EQ(g_msg_imu.get().orientation.x, q.x());
  EXPECT_EQ(g_msg_imu.get().orientation.y, q.y());
  EXPECT_EQ(g_msg_imu.get().orientation.z, q.z());
  EXPECT_EQ(g_msg_imu.get().linear_acceleration.x, (double)packet.accel_x *  1e-4);
  EXPECT_EQ(g_msg_imu.get().linear_acceleration.y, (double)packet.accel_y *  1e-4);
  EXPECT_EQ(g_msg_imu.get().linear_acceleration.z, (double)packet.accel_z * -1e-4);
  EXPECT_EQ(g_msg_imu.get().angular_velocity.x, (double)packet.gyro_x *  1e-5);
  EXPECT_EQ(g_msg_imu.get().angular_velocity.y, (double)packet.gyro_y *  1e-5);
  EXPECT_EQ(g_msg_imu.get().angular_velocity.z, (double)packet.gyro_z * -1e-5);
  EXPECT_EQ(g_msg_imu.get().angular_velocity_covariance[0], 0.0); // x
  EXPECT_EQ(g_msg_imu.get().angular_velocity_covariance[4], 0.0); // y
  EXPECT_EQ(g_msg_imu.get().angular_velocity_covariance[8], 0.0); // z
  EXPECT_EQ(g_msg_imu.get().linear_acceleration_covariance[0], 0.0); // x
  EXPECT_EQ(g_msg_imu.get().linear_acceleration_covariance[4], 0.0); // y
  EXPECT_EQ(g_msg_imu.get().linear_acceleration_covariance[8], 0.0); // z
  EXPECT_EQ(g_msg_imu.get().orientation_covariance[0], SQUARE(packet.chan.chan5.acc_roll    * 1e-5)); // x
  EXPECT_EQ(g_msg_imu.get().orientation_covariance[4], SQUARE(packet.chan.chan5.acc_pitch   * 1e-5)); // y
  EXPECT_EQ(g_msg_imu.get().orientation_covariance[8], SQUARE(packet.chan.chan5.acc_heading * 1e-5)); // z

  // Set orientation covariance on channel 5 with old age
  memset(&packet.chan, 0x00, sizeof(packet.chan));
  packet.channel = 5;
  packet.chan.chan5.age = 190;
  packet.chan.chan5.acc_heading = 123;
  packet.chan.chan5.acc_pitch   = 1234;
  packet.chan.chan5.acc_roll    = 12345;
  fixPacket(packet);
  ASSERT_TRUE(sendPacket(packet));
  EXPECT_TRUE(waitForMsgs(DURATION));
  ASSERT_TRUE(g_msg_fix.valid());
  ASSERT_TRUE(g_msg_vel.valid());
  ASSERT_TRUE(g_msg_imu.valid());
  EXPECT_STREQ(g_msg_imu.get().header.frame_id.c_str(), g_frame_id_gps.c_str());
  q.setRPY((double)packet.roll * 1e-6, (double)packet.pitch * 1e-6, M_PI_2 - (double)packet.heading * 1e-6);
  EXPECT_EQ(g_msg_imu.get().orientation.w, q.w());
  EXPECT_EQ(g_msg_imu.get().orientation.x, q.x());
  EXPECT_EQ(g_msg_imu.get().orientation.y, q.y());
  EXPECT_EQ(g_msg_imu.get().orientation.z, q.z());
  EXPECT_EQ(g_msg_imu.get().linear_acceleration.x, (double)packet.accel_x *  1e-4);
  EXPECT_EQ(g_msg_imu.get().linear_acceleration.y, (double)packet.accel_y *  1e-4);
  EXPECT_EQ(g_msg_imu.get().linear_acceleration.z, (double)packet.accel_z * -1e-4);
  EXPECT_EQ(g_msg_imu.get().angular_velocity.x, (double)packet.gyro_x *  1e-5);
  EXPECT_EQ(g_msg_imu.get().angular_velocity.y, (double)packet.gyro_y *  1e-5);
  EXPECT_EQ(g_msg_imu.get().angular_velocity.z, (double)packet.gyro_z * -1e-5);
  EXPECT_EQ(g_msg_imu.get().angular_velocity_covariance[0], 0.0); // x
  EXPECT_EQ(g_msg_imu.get().angular_velocity_covariance[4], 0.0); // y
  EXPECT_EQ(g_msg_imu.get().angular_velocity_covariance[8], 0.0); // z
  EXPECT_EQ(g_msg_imu.get().linear_acceleration_covariance[0], 0.0); // x
  EXPECT_EQ(g_msg_imu.get().linear_acceleration_covariance[4], 0.0); // y
  EXPECT_EQ(g_msg_imu.get().linear_acceleration_covariance[8], 0.0); // z
  EXPECT_EQ(g_msg_imu.get().orientation_covariance[0], 0.0); // x
  EXPECT_EQ(g_msg_imu.get().orientation_covariance[4], 0.0); // x
  EXPECT_EQ(g_msg_imu.get().orientation_covariance[8], 0.0); // x
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test");

  // Parameters
  ros::NodeHandle priv_nh("~");
  priv_nh.getParam("ip_address", g_ip_address);
  priv_nh.getParam("port", g_port);
  priv_nh.getParam("frame_id_gps", g_frame_id_gps);
  priv_nh.getParam("frame_id_vel", g_frame_id_vel);
  priv_nh.getParam("frame_id_odom", g_frame_id_odom);
  if (g_ip_address.empty()) {
    g_ip_address = "127.0.0.1";
  }

  // Subscribers
  ros::NodeHandle nh;
  g_sub_fix = nh.subscribe("gps/fix", 2, recvGpsFix);
  g_sub_vel = nh.subscribe("gps/vel", 2, recvGpsVel);
  g_sub_imu = nh.subscribe("imu/data", 2, recvImuData);

  return RUN_ALL_TESTS();
}


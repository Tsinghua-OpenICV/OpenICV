#include <ros/ros.h>
#include <mutex>
#include <signal.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64.h>
#include <cv_bridge/cv_bridge.h>
#include "icv2ros/ivsensorlrrobj.h"
#include "icv2ros/ivsensorsrrobj.h"
#include "icv2ros/radarlrrobject.h"
#include "icv2ros/radarsrrobject.h"
#include <sstream>
#include "OpenICV/Core/icvDataObject.h"
#include <time.h>
#include <boost/thread/thread.hpp>
#include <array>
#include "OpenICV/Core/icvZmq.h"

#define MAX_THREAD_COUNT 10

class icv2rosCore
{
private:
  //ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_number_;
  std::array<boost::shared_ptr<boost::thread>, MAX_THREAD_COUNT> threadsList_;
  void* context_zmq_uniq=nullptr;
  std::map<std::string,icv::core::ZMQ_Socket*>* zmq_subs=new std::map<std::string,icv::core::ZMQ_Socket*>();

public:
  icv2rosCore(ros::NodeHandle &nh);
  void Register_Sub_Remote(const std::string &sub_name, int port,int sendpip,int recvpip,int timeout);
  void icvSubscribe_Remote(std::string sub,icv::core::icvDataObject* datatosend);
  ~icv2rosCore();
  void Spin();
  void Demo();
};
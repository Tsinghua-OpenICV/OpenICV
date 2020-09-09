#pragma once

#include <iostream>
#include <sstream>
#include <assert.h>
#include <unistd.h>
#include <time.h>
// for thread and timer:
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>
// for zmq:
#include <zmq.h>
//#include <zmq_api.h>
// for json:
#include <json/json.h>
#include <queue>

#define SENSOR_LIST_SIZE 10

struct Zone{
  double xLeft = 0;
  double xRight = 0;
  double yTop = 0;
  double yTopWider = 0;
  double yTopEvenWider = 0;
  double angleLeft = 0;
  double angleRight = 0;
  bool b_isValid = false;
  bool b_leftCandidate = false;
  bool b_rightCandidate = false;
};

struct LidarData{
  std::vector<uint32_t> blockedAreaIndex;
  boost::posix_time::ptime lastPacketTime;
  bool b_isAlive;
  bool b_isValid;
  Zone zone;
};

struct RadarData{
  std::vector<uint32_t> blockedAreaIndex;
  boost::posix_time::ptime lastPacketTime;
  bool b_isAlive;
  bool b_isValid;
  Zone zone;
};

//struct VisionData{
//  uint8_t b_isValid[4];
//  uint8_t laneType;
//  uint8_t laneCount;
//  double leftError;
//  double rightError;
//  double angleError;
//  boost::posix_time::ptime lastPacketTime;
//  bool b_isAlive;
//};

struct LaneParameter{
  bool b_isSolid;
  uint8_t laneType;
  int32_t realParameter1;
  int32_t realParameter2;
  int32_t realParameter3;
  int32_t realLineK;
  int32_t realLineB;
  int32_t latOffset;
  int32_t yawAngle;
  int32_t latOffsetFiltered;
  int32_t yawAngleFiltered;
};

struct LaneData{
  uint8_t laneCount;
  uint8_t laneStatus[4];
  int32_t offsetRight;
  int32_t offsetLeft;
  int32_t angleError;
  LaneParameter leftLaneParameter;
  LaneParameter rightLaneParameter;
};

struct TrafficLightData{
  bool b_leftIsValid;
  uint8_t leftPassable;
  bool b_straightIsValid;
  uint8_t straightPassable;
  bool b_rightIsValid;
  uint8_t rightPassable;
  uint8_t start;
};

struct TrafficSignData{
  bool b_isValid;
  uint16_t pattern;
};

struct VisionData{
  LaneData laneData;
  TrafficLightData trafficLightData;
  TrafficSignData trafficSignData;

  boost::posix_time::ptime lastPacketTime;
  bool b_isAlive;
};


struct GpsData{
  double longitude;
  double latitude;
  double gaussX = 0.0;
  double gaussY = 0.0;
  double yaw = 0.0;
  uint16_t gpsQuality = 0;
  double utc_second = 0.0;

  bool b_isValid;
  bool b_isDiffitial;

  boost::posix_time::ptime lastPacketTime;
  bool b_isAlive;
};

struct ImuData{
  double time;
  double velocity;
  double roll;
  double yaw;
  double pitch;
  bool b_isValid;
  bool b_gpsValid = false;
  double latitude = 0.0;
  double longitude = 0.0;
  double gaussX = 0.0;
  double gaussY = 0.0;
  double yawRate = 0.0;

  boost::posix_time::ptime lastPacketTime;
  bool b_isAlive;
};

struct PostureData{
  GpsData gpsData;
  ImuData imuData;
//  uint32_t dataTime;

//  double longitude;
//  double latitude;
//  double velocity;
//
//  double roll;
//  double pitch;
//  double yaw;

  boost::posix_time::ptime lastPacketTime;
  bool b_isAlive;
};

struct ActuatorData{
//  bool b_epsEnabled;
//  bool b_torqueEnabled;
//  bool b_decEnabled;
//  uint8_t aebStatus;
//  uint8_t epsStatus;
//  uint8_t torqueStatus;
//  uint8_t decStatus;
//  uint8_t systemStatus;
//  uint8_t gearControlStatus;
//  uint8_t breakPedalStatus;
//  uint8_t cruiseStatus;
//  uint8_t gearPositionStatus;

  double vehicleSpeed = 0;
  double steeringAngle = 0;
  double steeringVelocity = 0;
  double targetSteeringAngle = 0;
  double targetSteeringVelocity = 0;
  double targetTorque = 0;
  double targetBrakePressure = 0;
  double brakePressure = 0;
  double steeringTorqueVoltage = 0;
  double steeringDAC_A = 0;
  double steeringDAC_B = 0;

  boost::posix_time::ptime lastPacketTime;
  bool b_isAlive;
};

//struct IntersectionData{
//  uint16_t intersectionType = 0;
//  bool b_isValid = false;
//};

struct UiData{
//  uint16_t nextId = 0;
  bool b_stopFlag = true;
  bool b_speedOff = true;
  bool b_steeringOff = true;
//  bool b_smallCircleSelected = true;
  bool b_isValid = false;
  bool b_config1 = false;

  boost::posix_time::ptime firstStoppedTime;
//  bool b_readyToGo = false;  // send to ui
//  uint16_t lastId = 0;

  boost::posix_time::ptime lastPacketTime;
  bool b_isAlive;
};

struct DecisionData{
  double targetSteeringAngle = 0;
  double targetSpeed = 0;
//  uint8_t gearPosition = 0;
//  int16_t accLevel = 2;
//  int16_t currentState = 0;
  bool b_isValid = false;

  int32_t currentSegId = 0;
  int32_t currentIndex = 0;
  //
  int32_t currentTotalIndex = 0;
  int32_t currentPreviewTotalIndex = 0;
  uint16_t maneuver = 0;
//  bool obstacle = 0;
  bool currentState = 0;  // 停止点减速中：1；停止点停车后或其他：0
  uint16_t postureSensorMode = 0;

  uint16_t targetWorkMode = 0;
  boost::posix_time::ptime lastPacketTime;
  bool b_isAlive;
 int accRequest=0;
};

/*
struct IovData{
  bool b_isValid = false;
  bool b_isAlive = false;
  boost::posix_time::ptime lastPacketTime;

  uint32_t phase = 0;
  uint32_t time = 0;
  double advSpd = 0;

  uint32_t fakeLightCounter = 0;

};
*/

struct IovData{
  bool b_isValid = false;
  bool b_isAlive = false;
  boost::posix_time::ptime lastPacketTime;

  bool b_vehicleValid = false;
  uint32_t vehicleId = 0;
  uint32_t vehicleType = 0;
  double vehicleGaussX = 0.0;
  double vehicleGaussY = 0.0;
  double vehicleSpeed = 0.0;
  double vehicleTime = 0.0;
  double vehicleAdvisedSpeed = 0.0;

  bool b_lightValid = false;
  uint32_t lightPhase = 0;
  double lightTime = 0.0;
  double lightAdvisedSpeed = 0.0;
};

struct CvData{
  bool b_isValid = false;
  bool b_isAlive = false;
  boost::posix_time::ptime lastPacketTime;

  bool b_vehicleValid = false;
  uint32_t vehicleId = 0;
  uint32_t vehicleType = 0;
  double vehicleGaussX = 0.0;
  double vehicleGaussY = 0.0;
  double vehicleSpeed = 0.0;
  double vehicleTime = 0.0;
  double vehicleAdvisedSpeed = 0.0;

  bool b_lightValid = false;
  uint32_t lightPhase = 0;
  double lightTime = 0.0;
  double lightAdvisedSpeed = 0.0;
};


class Jobs{
  public:
    explicit Jobs (boost::asio::io_service& io, std::vector<void*>& receivingSocketList, std::vector<void*>& sendingSocketList);
    virtual ~Jobs();

    virtual void subscriber();
    virtual void handler();
    virtual void publisher();

    virtual void lidarHandler(Json::Value values);
    virtual void visionHandler(Json::Value values);
//    virtual void postureHandler(Json::Value values);

    virtual void gpsHandler(Json::Value values);
    virtual void imuHandler(Json::Value values);
    virtual void actuatorHandler(Json::Value values);
    virtual void iovHandler(Json::Value values);
    virtual void cvHandler(Json::Value values);
    virtual void uiHandler(Json::Value values);
    virtual void decisionHandler(Json::Value values);
    virtual void radarHandler(Json::Value values);
    virtual void checkSensorHeartBeat();

  private:
    boost::asio::strand strand1;
    boost::asio::strand strand2;
    boost::asio::strand strand3;
    boost::asio::deadline_timer timer1;
    boost::asio::deadline_timer timer2;
    boost::asio::deadline_timer timer3;

    std::vector<void*> rSocketList;
    std::vector<void*> sSocketList;

    std::vector<std::string> rStringList;
    std::vector<std::string> sStringList;
//    std::stringstream rSS; // receving string stream
//    std::string sString; // sending string

    boost::mutex mutex;

    std::deque<LidarData> lidarDataQueue;
    std::deque<PostureData> postureDataQueue;
    std::deque<RadarData> radarDataQueue;

    LidarData lidarData;
    VisionData visionData;
    PostureData postureData;
    ActuatorData actuatorData;
    UiData uiData;
    DecisionData decisionData;
    IovData iovData;
  	// IovData iovData_sub;
    RadarData radarData;
    CvData cvData;


//    bool heartBeat[SENSOR_LIST_SIZE] = {false};
};

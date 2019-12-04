//
//  functions.h
//  decision
//
//  Created by QingXu on 5/19/15.
//  Copyright (c) 2015 Qing Xu. All rights reserved.
//

#ifndef __decision__functions__
#define __decision__functions__

#include <stdio.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>

#include "qingColor.h"

//#define MAX_ID 1
#define MINIMAL_ID 1

#define MIN(a,b) ((a)<=(b)?:(a),(b))

//#define OFFSET_X -57291.78 - 2
//#define OFFSET_X -57309.491 - 2
//#define OFFSET_X -57291.027872297
//#define OFFSET_X -57291.72787
#define OFFSET_X 0 
//#define OFFSET_Y 4453577.394 + 3
//#define OFFSET_Y 4453587.50552676
#define OFFSET_Y 0

//#define STATE_READY 0
//#define STATE_START 1
//#define STATE_ROADMAP 2
//#define STATE_VISION 3
//#define STATE_LIDAR 4

#define X_SIZE_MIN -20.0 // lateral
#define X_SIZE_MAX 20.0 // X-symmetric
#define Y_SIZE_MIN 0.0
#define Y_SIZE_MAX 80.0 // longitudinal
#define GRID_X_RESOLUTION 0.2
#define GRID_Y_RESOLUTION 0.2
#define GRID_INDEX_SIZE ((X_SIZE_MAX-X_SIZE_MIN)/GRID_X_RESOLUTION)*((Y_SIZE_MAX-Y_SIZE_MIN)/GRID_Y_RESOLUTION)
#define GRID_INDEX_X_SIZE (X_SIZE_MAX-X_SIZE_MIN)/GRID_X_RESOLUTION

#define LIDAR_SAFE_ZONE_MIN 2.2 //2.4
#define LIDAR_SAFE_ZONE_MAX 3.0 
#define LIDAR_SAFE_DISTANCE_MIN 10.0 //20 
#define LIDAR_SAFE_DISTANCE_MAX 25.0 // 25

// curves vertical check
#define CURVE_POINT_RESOLUTION 0.2
#define GRID_X_RESOLUTION_CHECK GRID_X_RESOLUTION/2
#define GRID_Y_RESOLUTION_CHECK GRID_Y_RESOLUTION/2
#define LIDAR_SAFE_ZONE_MAX_R_NUM LIDAR_SAFE_ZONE_MAX/2/MIN(GRID_X_RESOLUTION,GRID_Y_RESOLUTION)
#define LIDAR_SAFE_ZONE_MIN_R_NUM LIDAR_SAFE_ZONE_MIN/2/MIN(GRID_X_RESOLUTION,GRID_Y_RESOLUTION)



#define LIDAR_MAX_SPEED 14 / 3.6 // m/s
#define PREVIEW_DISTANCE_RATIO 0.75
//take over
#define TAKE_OVER_PERIOD 1500 //ms
#define OVER_TAKE_SPEED_THRESHOLD 1.5
#define TAKE_OVER_MAX_SPEED 1 //old = 2.8 // m/s

//#define CURVES_SIZE 7
//#define CURVE_LENGTH 100

//#define MAX_ABS_STEERING_RATE 5
#define MAX_ABS_STEERING_RATE 25
#define MAX_ABS_STEERING_RATE_TAKE_OVER 15
//#define MAX_ABS_STEERING_RATE 2.5
#define MAX_ABS_STEERING_ANGLE_HIGH_SPEED 45
#define MAX_ABS_STEERING_ANGLE_LOW_SPEED 450


#define STATE_INITIAL 0
#define STATE_FOLLOW_ROADMAP 1
#define STATE_PARKING 2

//curve related.
#define CP_DISTANCE 3
#define CURVE_POINT_NUM 50
#define CURVE_NUM 2
#define CURVE_DISTANCE 4.0


//path number
#define MAX_PATH 10
#define MAX_PATH_NUMBER 100

#define ALMOST_ZERO 0.01

struct Zone{
  double xLeft = 0;
  double xRight = 0;
  double yTop = 0;
  double yTopWider = 0.0;
  double yTopEvenWider = 0.0;
  double angleLeft = 0;
  double angleRight = 0;
  double pathAngle = 0;
  bool b_isValid = false;
  bool b_leftCandidate = false;
  bool b_rightCandidate = false;
};

struct LidarData{
  std::vector<uint32_t> blockedAreaIndex;
  bool b_isAlive;
  bool b_isValid;
  Zone zone;
};

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
//  int32_t offsetRight;
//  int32_t offsetLeft;
//  int32_t angleError;
  double offsetRight;
  double offsetLeft;
  double angleError;
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
  
  bool b_isAlive;
};


struct GpsData{
  double longitude;
  double latitude;
  double heading;
  bool b_isValid;
  bool b_isDifferential;
};

struct ImuData{
  double velocity;
  double roll;
  double yaw;
  double pitch;
  bool b_isValid;
  
  double yawRate = 0.0;

  bool b_gpsValid = false;
  double longitude = 0.0;
  double latitude = 0.0;
};

struct PostureData{
  GpsData gpsData;
  ImuData imuData;
  
  bool b_isAlive;
};

struct ActuatorData{
  uint8_t aebStatus;
  uint8_t epsStatus;
  uint8_t torqueStatus;
  uint8_t decStatus;
  uint8_t systemStatus;
  uint8_t gearControlStatus;
  uint8_t breakPedalStatus;
  uint8_t cruiseStatus;
  uint8_t gearPositionStatus;
  double vehicleSpeedStatus = 0;
  double vehicleSteerStatus = 0;
  double vehicleSpeed = 0;
  uint8_t wsuSceneStatus = 0;
  uint8_t wsuAlarmLevelStatus = 0;
  
  bool b_isAlive;
};

struct UiData{
//  uint16_t nextId = 0;
  bool b_stopFlag = true;
  bool b_speedOff = true;
  bool b_steeringOff = true;
  bool b_config1 = false;
  uint16_t pathNumber = 0;
//  bool b_smallCircleSelected = true;
  bool b_isValid = false;
  
//  bool b_readyToGo = false;
//  uint16_t lastId = 0;
  
  bool b_isAlive;
};

/*
struct IovData{
  bool b_isValid = false;
  uint32_t phase = 0;
  uint32_t time = 0;
  double advSpd = 0;
  
  bool b_isAlive;
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

struct StartSignal{
  uint8_t data = 0;
  bool b_isValid = 0;
};

struct Point{
  double x;
  double y;
  double angle;
};

// parameters for road point: 1 road type, 2 curvature, 3 maxSpeed
// parameters for stop point: 1 point type, 2 effective area, 3 maxSpeed
struct RoadPoint{
  uint32_t index = 0;
  uint32_t roadId = 0;
  uint32_t totalIndex = 0;
  double longitude = 0.0;
  double latitude = 0.0;
  double courseAngle = 0.0;
  int16_t roadType = 0;
  double height = 0.0;
  int32_t parameter1 = 0;
  double parameter2 = 0.0;
  double parameter3 = 0.0;
  double distance = 0.0;
  bool b_isValid = false;
};



struct Curve{
  int32_t index = 0;
  Point points[CURVE_POINT_NUM];
  std::vector<Point> pointList;
};

class DecisionData{
public:
  std::vector<Curve> curveList;
  RoadPoint currentPosture; // real point
  RoadPoint currentPoint; // point in roadmap
  RoadPoint previewPoint;
  double targetSteeringAngle = 0.0;
  double targetSpeed = 0.0;
  double targetAccReq = 0.0;
  double previewDistance = 0.0;
  uint8_t targetGearPosition = 0;
  int16_t targetAccLevel = 2;
  int32_t currentState = 0;
  int32_t currentId = 0;
  int32_t currentIndex = 0;
  int32_t currentTotalIndex = 0;
  int32_t roadType = 0;
  int32_t nextId = 0;
  int32_t nextNextId = 0;

  uint32_t targetWorkMode = 0;
  uint16_t postureSensorMode = 0;
  bool b_takeOverEnable = false;
  bool b_takingOver = false;
  bool b_comingBack = false;
  bool b_redLightStopped = false;
  bool b_onRoadPoint = false;
  
  uint16_t pathNumber = 0;
  boost::posix_time::ptime takingOverTimeCounter;
  
  bool b_isValid = false;
  bool b_isAlive;
 
public:
  uint32_t updateStateMachine();
};

//struct RoadMap{
//  uint32_t roadId = 0;
//  double longitude = 0.0;
//  double latitude = 0.0;
//  double courseAngle = 0.0;
//  double distance = 0.0;
//};

class PidController{
private:
  bool b_hasStarted, b_threshIsOn;
  double kp, ki, kd, interval, frequency, lastError, errorThresh, integral, fractionValue;
  
public:
  explicit PidController (double kpInput, double kiInput, double kdInput, double errorThreshInput, bool b_threshInOnInput, double intervalInput);
  ~PidController ();
  double update (double error);
  void modifyGains(double kp, double ki, double kd);
  void resetI ();
};


bool angleDiffLessThan(double angle1, double angle2, double diff);
//RoadPoint movePreviewPointForLaneChange (RoadPoint originalRoadPoint, double yaw);
void gaussConvert(double longitude1,double latitude1,double& x1,double& y1);
uint32_t loadRoadMapSize(const std::string fileName);
void loadRoadMap(const std::string fileName, std::vector<std::vector<RoadPoint>>& rawRoadPointss);
void loadRoadMapConverted(const std::string fileName, std::vector<std::vector<RoadPoint>>& rawRoadPointss, double offsetX, double offsetY);
uint32_t loadStopPointsSize(const std::string fileName);
void loadStopPoints(const std::string fileName, std::vector<RoadPoint>& rawStopPoints, double offsetX, double offsetY);
void loadTerminalPoint(const std::string fileName, RoadPoint& terminalPoint);
//void loadPathNumber(const std::string fileName, std::vector<std::vector<uint32_t>>& pathNumberss);
void loadPathNumber(const std::string fileName, std::vector<std::vector<uint32_t>>& pathNumberLists);

void buildFinalRoadMap(std::vector<std::vector<RoadPoint>>& roadPointss, const std::vector<RoadPoint>& stopPoints);

void pathSelection(DecisionData& decisionData, const UiData& uiData, const uint16_t maxPathNumber);
uint32_t getNextRoadId (uint32_t currentId, uint32_t size, const DecisionData& decisionData, std::vector<std::vector<uint32_t>>& pathNumberLists);
uint32_t getLastRoadId (uint32_t currentId, uint32_t size);

void getCoordinate(std::vector<RoadPoint>& rawRoadPoints, std::vector<RoadPoint>& roadPoints);
double pointDistance(double x1, double x2, double y1, double y2);
//void generateCandidates(RoadPoint previewPoint, uint16_t leftCandidateNumbers, uint16_t rightCandidateNumbers, double candidateDistance);

// velocity
void velocityDecision(const LidarData& lidarData, bool b_stopFromUI, const PostureData& postureData, DecisionData& decisionData);
// end of velocity


RoadPoint getCurrentPosture(double longitude, double latitude, double yaw, std::vector<std::vector<RoadPoint>>& roadPoints, int32_t currentId);
RoadPoint getCurrentPoint (double longitude, double latitude, double yaw, std::vector<std::vector<RoadPoint>>& roadPointss, int32_t lastId, int32_t nextId);
//RoadPoint getPreviewPoint (double velocity, double curvature, RoadPoint& currentPosture, int32_t nextId, int32_t nextNextId, std::vector<std::vector<RoadPoint>>& roadPointss);
double getPreviewDistance(double velocity, double curvature, RoadPoint& currentPoint, DecisionData& decisionData);
//RoadPoint getPreviewPoint (double distance, double velocity, double curvature, RoadPoint& currentPosture, double yaw, int32_t nextId, int32_t nextNextId, std::vector<std::vector<RoadPoint>>& roadPointss);
RoadPoint getPreviewPoint (double distance, double velocity, double curvature, RoadPoint& currentPosture, double yaw, int32_t nextId, int32_t nextNextId, std::vector<std::vector<RoadPoint>>& roadPointss);
double limitSpeed(DecisionData& decisionData, double lastTargetSpeed);
double limitTargetSpeed(DecisionData& decisionData, double targetSpeed);
double limitSteeringAngle(double steeringAngle, double currentSteeringAngle, double velocity, double yaw, bool overTakeEnable);
double limitPreviewDistance(double targetDistance, double lastTargetDistance);
  
RoadPoint getTargetPoint (RoadPoint currentPosture, std::vector<RoadPoint>& roadPoints);
double getAngle (double x0, double y0, double x1, double y1);
double getAngleDiff(double ang0, double ang1);
double getYawDiff (double yawVehicle, double yawRoad);
//double followVision();
void followLidar(double refAngle, const PostureData& postureData, DecisionData& decisionData);
void followLane(const VisionData& visionData, const PostureData& postureData, DecisionData& decisionData);
void followRoadMap (double longitude, double latitude, double yaw, double velocity, std::vector<std::vector<RoadPoint>>& roadPointss, DecisionData& decisionData, std::vector<std::vector<uint32_t>>& pathNumberLists);
void followLidarAndLane (const LidarData& lidarData, const VisionData& visionData, const PostureData& postureData,DecisionData& decisionData);

void trafficLightJudge(const PostureData& postureData, const CvData& cvData, const std::vector<RoadPoint>& stopPoints, DecisionData& decisionData);
void terminalPointJudge(const PostureData& postureData, const RoadPoint& terminalPoint, DecisionData& decisionData);
//void stopPointJudge(const PostureData& postureData, const RoadPoint& terminalPoint, DecisionData& decisionData);
void stopPointJudge(const PostureData& postureData, const CvData& cvData, const std::vector<RoadPoint>& stopPoints,  DecisionData& decisionData);
void mapJudge(DecisionData& decisionData, const CvData& cvData, const VisionData& visionData);

//bool checkGrids (LidarData lidarData);
//int32_t curveSelection (std::vector<Curve>& curves, LidarData lidarData);
double curvesLidarCheck (Curve& curve, std::vector<uint32_t>& blockedAreaIndex, double previewDistance);
bool gridOccupied(Point point, uint32_t gridIndex, double previewDistance);
bool curvePointOccupied(Point& point, bool*& b_obstacle);


Point pointOnCubicBezier (std::vector<Point> cp, double t);
void generateCurve (const RoadPoint& startPoint, const RoadPoint& endPoint, Curve& curve);
void generateCurveList (const RoadPoint& startPoint, const std::vector<RoadPoint>& endPointList, std::vector<Curve>& curveList);
void clearCurveList(std::vector<Curve>& curveList);

void generateCurvePreviewPoint(DecisionData& decisionData, const PostureData& postureData);
void generateLidarCurvePreviewPoint (DecisionData& decisionData, const std::vector<std::vector<RoadPoint>>& roadPointss, const PostureData& postureData, DecisionData& lidarDecisionData);

double cruiseController(const PostureData& postureData, const IovData& iovData, const ActuatorData actuatorData);

void lidarJudge(DecisionData& decisionData, PostureData& postureData, LidarData& lidarData);
//void lidarJudgeLongerPreview(DecisionData& lidarDecisionData, PostureData& postureData, LidarData& lidarData);
void lidarJudgeLongerPreview(DecisionData& decisionData, DecisionData& lidarDecisionData, PostureData& postureData, LidarData& lidarData, std::vector<std::vector<RoadPoint>>& roadPointss);
//CXY
/*
void speedPlanWithLidar(DecisionData& decisionData, double advisedSpeed);
void lidarJudge(LidarData& lidarData, DecisionData& decisionData, PostureData& postureData);*/
#endif /* defined(__decision__functions__) */

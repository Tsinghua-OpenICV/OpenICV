#include "dataFusion.hpp"
#include "qingColor.h"

#include <fstream>
#define doNothing()

#define DECISION_TIMEOUT_FLAG true

#define DEFAULT_TIMER1_START 35
#define DEFAULT_TIMER2_START 172
#define DEFAULT_TIMER3_START 200

#define DEFAULT_TIMER1_PERIOD 20
#define DEFAULT_TIMER2_PERIOD 100
#define DEFAULT_TIMER3_PERIOD 100

// Keys
#define KEY_LIDAR "Lidar"

#define KEY_VISION "Vision"
#define KEY_VISION_LANE "LaneModel"
#define KEY_VISION_TRAFFIC_LIGHT "TrafficLight"
#define KEY_VISION_TRAFFIC_SIGN "TrafficSign"

#define KEY_POSTURE "Posture"
#define KEY_POSTURE_GPS "Gps"
#define KEY_POSTURE_IMU "Imu"

#define KEY_GPS "Gps"
#define KEY_IMU "Imu"

#define KEY_ACTUATOR "Actuator"
#define KEY_START "StartSignal"

// traffic light stop point
#define MY_CV_SWITCH true
#define KEY_CV "Cv"
#define TRAFFIC_LIGHT_X -57282.9
#define TRAFFIC_LIGHT_Y 4453608.3
#define TRAFFIC_LIGHT_AREA 15

// intersection stop point
#define KEY_IOV "Iov"
#define KEY_IOV_E30 "IovE30"
#define KEY_IOV_RAETON "IovRaeton"
#define INTERSECTION_X -57304.30
#define INTERSECTION_Y 4453596.15
#define INTERSECTION_AREA 3

#define KEY_UI "Ui"

#define KEY_DECISION "Decision"

#define KEY_RADAR "Radar"

#define KEY_TIME "Time"

#define HEARTBEAT_PERIOD 500 //in millisecond
#define STOP_PERIOD 2000

#define STOP_DEAD_ZONE 0.5

#define LIDAR_VALID_PITCH 10 // in degree
#define LIDAR_VALID_PITCH_FLAG false

#define FUSION_FLAG false

#define LIDAR_QUEUE_SIZE 3
#define RADAR_QUEUE_SIZE 3
#define POSTURE_QUEUE_SIZE 1 // posture freq 50ms, 3 times

#define YAW_ADJ_DEGREE_IMU -0.5 // positive value for left side compensation
#define YAW_ADJ_DEGREE_GPS 0.7

#define SAVE_DATA_FLAG false 
#define SAVE_ROAD_FLAG true

//#define SAVEDATA(foo, list, fs, flag) saveData(#foo, foo, list, fs, flag)
#define SAVEDATA(foo) saveData(#foo, foo, list, fs, firstTimeFlag)
//for fake light:
#define BOOST_DATE_TIME_SOURCE
//end fake light.

boost::posix_time::ptime startTime;
boost::posix_time::time_duration duration;

uint32_t sendingCounter = 0;
bool b_postureInitialized = false;
//lidar:
//#define X_SIZE_MIN -20.0
//#define X_SIZE_MAX 20.0 // X-symmetric
//#define Y_SIZE_MIN 0.0
//#define Y_SIZE_MAX 80.0
//#define GRID_X_RESOLUTION 0.2
//#define GRID_Y_RESOLUTION 0.2
//#define GRID_INDEX_SIZE ((X_SIZE_MAX-X_SIZE_MIN)/GRID_X_RESOLUTION)*((Y_SIZE_MAX-Y_SIZE_MIN)/GRID_Y_RESOLUTION)

template <class T>
void saveData(std::string name, T value, std::ofstream& list, std::ofstream& fs, bool firstTimeFlag){
  if (true == firstTimeFlag) {
    list << name << " ";
  }else;
  fs << value << " ";
}

void gaussConvert(double longitude1,double latitude1,double& x1,double& y1)
{
  double lon;
  double lat;
  int a=6378137;
  double lon0;
  double e=0.0066943799013;
  double e2=0.006739496742270;
  double deltL;
  double t;
  double n;
  double C1;
  double C2;
  double C3;
  double C4;
  double C5;
  double N;
  double X;
  double A;
  lon=longitude1*2*M_PI/360;
  lat=latitude1*2*M_PI/360;
  lon0=117*2*M_PI/360;
  deltL=lon-lon0;
  t=tan(lat);
  n=e2*cos(lat);
  C1=1+3*pow(e,2)/4+45*pow(e,4)/64+175*pow(e,6)/256+11025*pow(e,8)/16384;
  C2=3*pow(e,2)/4+15*pow(e,4)/16+525*pow(e,6)/512+2205*pow(e,8)/2048;
  C3=15*pow(e,4)/64+105*pow(e,6)/256+2205*pow(e,8)/4096;
  C4=35*pow(e,6)/512+315*pow(e,8)/2048;
  C5=315*pow(e,8)/131072;
  N=a/sqrt(1-pow(e,2)*pow(sin(lat),2));
  X=a*(1-pow(e,2))*(C1*lat-0.5*C2*sin(2*lat)+C3*sin(4*lat)/4-C4*sin(6*lat)/6+C5*sin(8*lat));
  A=1+pow(deltL,2)/12*pow(cos(lat),2)*(5-pow(t,2)+9*pow(n,2)+4*pow(n,4))+pow(deltL,4)/360*pow(cos(lat),4)*(61-58*pow(t,2)+pow(t,4));
  y1=X+0.5*N*sin(lat)*cos(lat)*pow(deltL,2)*A;//正北方向
  x1=N*cos(lat)*deltL*(1+pow(deltL,2)/6*pow(cos(lat),2)*(1-pow(t,2)+pow(n,2))+pow(deltL,4)/120*pow(cos(lat),4)*(5-18*pow(t,2)+pow(t,4)-14*pow(n,2)-58*pow(n,2)*pow(t,2)));//正东方向
}

bool inArea(double longitude, double latitude, double targetX, double targetY, double distance){
  bool ret = false;
  double longitudeX = 0.0, latitudeY = 0.0;
  gaussConvert (longitude, latitude, longitudeX, latitudeY);

//  std::cout << std::setprecision(15) << "lon: " << longitudeX << ", lat: " << latitudeY << std::endl;
//  std::cout << std::setprecision(15) << "tx : " << targetX << ", ty : " << targetY << std::endl;


  if (std::abs(longitudeX - targetX) < distance && std::abs(latitudeY - targetY) < distance){
    ret = true;
  }
  else{
    ret = false;
  }

//  std::cout << "inarea?: " << ret << std::endl;
  return ret;
}

uint32_t getCurrentSec(){
  boost::posix_time::ptime secTime = boost::posix_time::second_clock::local_time();
//  std::cout << secTime << std::endl;

  std::stringstream ss;
  ss << secTime;
  std::string timeString = ss.str();
  uint16_t secStrPos = timeString.length() - 2 ;
  std::string secStr = timeString.substr(secStrPos, 2);
//  std::cout << secStr << std::endl;
  uint32_t currentSec = std::stoul(secStr, 0, 10);
//  std::cout << currentSec << std::endl;

  return currentSec;
}

PostureData postureFilter (std::deque<PostureData>& postureDataQueue, PostureData postureDataIn){
  if (( true == postureDataIn.gpsData.b_isValid) && (true == postureDataIn.imuData.b_isValid)){
    postureDataQueue.pop_front();
    postureDataQueue.push_back(postureDataIn);
  }
  PostureData pd;
  for ( int i = 0; i < POSTURE_QUEUE_SIZE; i++){
    pd.gpsData.longitude += postureDataQueue[i].gpsData.longitude / POSTURE_QUEUE_SIZE;
    pd.gpsData.latitude += postureDataQueue[i].gpsData.latitude / POSTURE_QUEUE_SIZE;
    pd.imuData.velocity += postureDataQueue[i].imuData.velocity / POSTURE_QUEUE_SIZE;
//    pd.roll += postureDataQueue[i].roll / POSTURE_QUEUE_SIZE;
//    pd.pitch += postureDataQueue[i].pitch / POSTURE_QUEUE_SIZE;
//    pd.yaw += postureDataQueue[i].yaw / POSTURE_QUEUE_SIZE;
  }
  pd.gpsData.b_isValid = postureDataIn.gpsData.b_isValid;
  pd.gpsData.gaussX = postureDataIn.gpsData.gaussX;
  pd.gpsData.gaussY = postureDataIn.gpsData.gaussY;
  pd.gpsData.b_isDiffitial = postureDataIn.gpsData.b_isDiffitial;

  pd.imuData.roll = postureDataIn.imuData.roll;
  pd.imuData.yaw = (postureDataIn.imuData.yaw + YAW_ADJ_DEGREE_IMU) > 0? postureDataIn.imuData.yaw + YAW_ADJ_DEGREE_IMU: postureDataIn.imuData.yaw + YAW_ADJ_DEGREE_IMU + 360;
  pd.imuData.pitch = postureDataIn.imuData.pitch;
  pd.imuData.b_isValid = postureDataIn.imuData.b_isValid;

//  std::ofstream fs("pd.txt", std::ofstream::app);
//  fs << std::setprecision(15) <<pd.gpsData.longitude <<", " <<pd.gpsData.latitude << std::endl;;
//  fs.close();

//  pd.dataTime = postureDataIn.dataTime;
  return pd;
}


void Jobs::lidarHandler (Json::Value values){
//  std::cout << "lidar!" << std::endl;
//  uint32_t data = values["Lidar"].asInt();
//  std::cout << "This is the data of \"Lidar\": " << data <<std::endl;  //  lidarData.blockedAreaIndex.clear();
//  lidarData.blockedAreaIndex.push_back(data);

  lidarData.blockedAreaIndex.clear();
  lidarData.zone.xLeft = values["Lidar"]["Zone"]["XLeft"].asDouble();
  lidarData.zone.xRight = values["Lidar"]["Zone"]["XRight"].asDouble();
  lidarData.zone.yTop = values["Lidar"]["Zone"]["YTop"].asDouble();
  lidarData.zone.yTopWider = values["Lidar"]["Zone"]["YTopWider"].asDouble();
  lidarData.zone.yTopEvenWider = values["Lidar"]["Zone"]["YTopEvenWider"].asDouble();
  lidarData.zone.angleLeft = values["Lidar"]["Zone"]["AngleLeft"].asDouble();
  lidarData.zone.angleRight = values["Lidar"]["Zone"]["AngleRight"].asDouble();
  lidarData.zone.b_isValid = values["Lidar"]["Zone"]["IsValid"].asInt();
  lidarData.zone.b_leftCandidate = values["Lidar"]["Zone"]["LeftCandidate"].asInt();
  lidarData.zone.b_rightCandidate = values["Lidar"]["Zone"]["RightCandidate"].asInt();

  Json::Value::Members members (values["Lidar"]["Data"].getMemberNames());
  if(members.begin() != members.end() - 1){
    for (Json::Value::Members::iterator mit = members.begin(); mit != members.end(); mit++){
      lidarData.blockedAreaIndex.push_back(values["Lidar"]["Data"][*mit].asInt());
//      std::cout << "This is the data of \"Lidar\": " << values["Lidar"]["Data"][*mit].asInt() <<std::endl;
    }
  }
  else{
    std::cout << RED << "Lidar ERROR!" << RESET << std::endl;
  }

  if(true == FUSION_FLAG){
    lidarDataQueue.pop_front();
    lidarDataQueue.push_back(lidarData);
    lidarData.blockedAreaIndex.clear();
    lidarData.zone.angleLeft = 0.0;
    lidarData.zone.angleRight = 0.0;
    lidarData.zone.xLeft = 0.0;
    lidarData.zone.yTop = 0.0;
    lidarData.zone.yTopWider = 0.0;
    lidarData.zone.yTopEvenWider = 0.0;
    lidarData.zone.xRight = 0.0;
//    for (int i = 0; i < LIDAR_QUEUE_SIZE; i ++){
      for (std::deque<LidarData>::iterator it = lidarDataQueue.begin(); it != lidarDataQueue.end(); it++){
        lidarData.blockedAreaIndex.insert(lidarData.blockedAreaIndex.end(), lidarDataQueue[it - lidarDataQueue.begin()].blockedAreaIndex.begin(), lidarDataQueue[it - lidarDataQueue.begin()].blockedAreaIndex.end());
//    lidarData.blockedAreaIndex += lidarDataQueue[it - lidarDataQueue.begin()].blockedAreaIndex;

      }
//    }
      for (int i = 0; i < LIDAR_QUEUE_SIZE; i ++){
        lidarData.zone.angleLeft += lidarDataQueue[i].zone.angleLeft/LIDAR_QUEUE_SIZE;
        lidarData.zone.angleRight += lidarDataQueue[i].zone.angleRight/LIDAR_QUEUE_SIZE;
        lidarData.zone.xLeft += lidarDataQueue[i].zone.xLeft/LIDAR_QUEUE_SIZE;
        lidarData.zone.yTop += lidarDataQueue[i].zone.yTop/LIDAR_QUEUE_SIZE;
        lidarData.zone.yTopWider += lidarDataQueue[i].zone.yTopWider/LIDAR_QUEUE_SIZE;
        lidarData.zone.yTopEvenWider += lidarDataQueue[i].zone.yTopEvenWider/LIDAR_QUEUE_SIZE;
        lidarData.zone.xRight += lidarDataQueue[i].zone.xRight/LIDAR_QUEUE_SIZE;
      }
  }
  else{
    doNothing();
  }

  //Fusion//
  if (true == LIDAR_VALID_PITCH_FLAG &&
      true == FUSION_FLAG && (
      false == postureData.b_isAlive ||
      false == postureData.imuData.b_isValid ||
      LIDAR_VALID_PITCH < std::abs(postureData.imuData.pitch))){
    lidarData.b_isValid = false;
  }
  else{
    lidarData.b_isValid = true;
  }


//  // display
//  bool b_passable[400][200] = {true};
//  for(uint32_t i = 0; i < 400; i ++){
//    for (uint32_t j = 0; j < 200; j ++){
//      b_passable[i][j] = true;
//    }
//  }
//  for (uint32_t i = 0; i < lidarData.blockedAreaIndex.size(); i ++){
//    if (lidarData.blockedAreaIndex[i] > GRID_INDEX_SIZE){
//      std::cout << RED << "GRID INDEX OUT OF RANGE!" << RESET << std::endl;
//    }
//    else{
//      b_passable[(int)(lidarData.blockedAreaIndex[i]/((X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION))][(int)lidarData.blockedAreaIndex[i]%(int)((X_SIZE_MAX - X_SIZE_MIN)/GRID_X_RESOLUTION)] = false;
//    }
//  }
//  for (int32_t j = 50; j >= 0; j-- ){
//    std::stringstream ss;
//    for (uint32_t i = 50; i < 150; i++ ){
//      if (false == b_passable[j][i]){
//        ss << "#";
//      }
//      else{
//        ss << " ";
//      }
//    }
//    std::cout << YELLOW << ss.str() << RESET << std::endl;
//  }
//

  lidarData.b_isAlive = true;
  lidarData.lastPacketTime = boost::posix_time::microsec_clock::local_time();

}

void Jobs::visionHandler ( Json::Value values){
//  visionData.b_isValid[0] = values[KEY_VISION]["IsValid"]["RR"].asInt();
//  visionData.b_isValid[1] = values[KEY_VISION]["IsValid"]["R"].asInt();
//  visionData.b_isValid[2] = values[KEY_VISION]["IsValid"]["L"].asInt();
//  visionData.b_isValid[3] = values[KEY_VISION]["IsValid"]["LL"].asInt();
//
//  visionData.laneType = values[KEY_VISION]["LaneType"].asInt();
//  visionData.laneCount = values[KEY_VISION]["LaneCount"].asInt();
//  visionData.leftError = values[KEY_VISION]["OffsetLeft"].asFloat();
//  visionData.rightError = values[KEY_VISION]["OffsetRight"].asFloat();
//  visionData.angleError = values[KEY_VISION]["AngleError"].asFloat();

  visionData.laneData.laneCount = values[KEY_VISION][KEY_VISION_LANE]["LaneCount"].asInt();
  visionData.laneData.laneStatus[0] = values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["RR"].asInt();
  visionData.laneData.laneStatus[1] = values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["R"].asInt();
  visionData.laneData.laneStatus[2] = values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["L"].asInt();
  visionData.laneData.laneStatus[3] = values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["LL"].asInt();
  visionData.laneData.offsetRight = values[KEY_VISION][KEY_VISION_LANE]["OffsetRight"].asInt();
  visionData.laneData.offsetLeft = values[KEY_VISION][KEY_VISION_LANE]["OffsetLeft"].asInt();
  visionData.laneData.angleError = values[KEY_VISION][KEY_VISION_LANE]["AngleError"].asInt();

  visionData.laneData.leftLaneParameter.b_isSolid = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["IsSolid"].asInt();
  visionData.laneData.leftLaneParameter.laneType = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["LaneType"].asInt();
  visionData.laneData.leftLaneParameter.realParameter1 = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealPara1"].asInt();
  visionData.laneData.leftLaneParameter.realParameter2 = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealPara2"].asInt();
  visionData.laneData.leftLaneParameter.realParameter3 = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealPara3"].asInt();
  visionData.laneData.leftLaneParameter.realLineK = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealLineK"].asInt();
  visionData.laneData.leftLaneParameter.realLineB = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealLineB"].asInt();
  visionData.laneData.leftLaneParameter.latOffset = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["LatOffset"].asInt();
  visionData.laneData.leftLaneParameter.yawAngle = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["YawAngle"].asInt();
  visionData.laneData.leftLaneParameter.latOffsetFiltered = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["LatOffset_Filtered"].asInt();
  visionData.laneData.leftLaneParameter.yawAngleFiltered = values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["YawAngle_Filtered"].asInt();


  visionData.laneData.rightLaneParameter.b_isSolid = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["IsSolid"].asInt();
  visionData.laneData.rightLaneParameter.laneType = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["LaneType"].asInt();
  visionData.laneData.rightLaneParameter.realParameter1 = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealPara1"].asInt();
  visionData.laneData.rightLaneParameter.realParameter2 = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealPara2"].asInt();
  visionData.laneData.rightLaneParameter.realParameter3 = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealPara3"].asInt();
  visionData.laneData.rightLaneParameter.realLineK = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealLineK"].asInt();
  visionData.laneData.rightLaneParameter.realLineB = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealLineB"].asInt();
  visionData.laneData.rightLaneParameter.latOffset = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["LatOffset"].asInt();
  visionData.laneData.rightLaneParameter.yawAngle = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["YawAngle"].asInt();
  visionData.laneData.rightLaneParameter.latOffsetFiltered = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["LatOffset_Filtered"].asInt();
  visionData.laneData.rightLaneParameter.yawAngleFiltered = values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["YawAngle_Filtered"].asInt();

  visionData.trafficLightData.b_leftIsValid = values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["LeftIsValid"].asInt();
  visionData.trafficLightData.leftPassable = values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["LeftPassable"].asInt();
  visionData.trafficLightData.b_straightIsValid = values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["StraightIsValid"].asInt();
  visionData.trafficLightData.straightPassable = values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["StraightPassable"].asInt();
  visionData.trafficLightData.b_rightIsValid = values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["RightIsValid"].asInt();
  visionData.trafficLightData.rightPassable = values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["RightPassable"].asInt();
  visionData.trafficLightData.start = values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["Start"].asInt();

  visionData.trafficSignData.b_isValid = values[KEY_VISION][KEY_VISION_TRAFFIC_SIGN]["IsValid"].asInt();
  visionData.trafficSignData.pattern = values[KEY_VISION][KEY_VISION_TRAFFIC_SIGN]["Pattern"].asInt();

  visionData.b_isAlive = true;
  visionData.lastPacketTime = boost::posix_time::microsec_clock::local_time();
}

void Jobs::gpsHandler (Json::Value values){
  postureData.gpsData.longitude = values[KEY_GPS]["Longitude"].asDouble();
  postureData.gpsData.latitude = values[KEY_GPS]["Latitude"].asDouble();
  postureData.gpsData.yaw = values[KEY_GPS]["Yaw"].asDouble();
  postureData.gpsData.utc_second = values[KEY_GPS]["UtcSecond"].asDouble();

  uint16_t gpsQuality = values[KEY_GPS]["Status"].asInt();
  postureData.gpsData.gpsQuality = gpsQuality;
//  std::cout<< std::setprecision(15) << postureData.gpsData.longitude << ", " << postureData.gpsData.latitude << ", " << gpsQuality << std::endl;
//  std::cout << gpsQuality << std::endl;
  if ( gpsQuality == 4 ||gpsQuality == 5 ||gpsQuality == 6  ){
    postureData.gpsData.b_isDiffitial = true;
  }
  else{
    postureData.gpsData.b_isDiffitial = false;
  }
//  postureData.gpsData.b_isDiffitial =  ((gpsQuality== 4)? true: false);

  gaussConvert(postureData.gpsData.longitude, postureData.gpsData.latitude, postureData.gpsData.gaussX, postureData.gpsData.gaussY);

//  std::cout << "x: " << postureData.gpsData.gaussX << " , y: " << postureData.gpsData.gaussY << std::endl;

//  postureData.dataTime = values[KEY_POSTURE]["Time"].asInt();
//  std::cout << "time: " << postureData.dataTime << std::endl;
//
//  if (true == FUSION_FLAG){
//    if (false == b_postureInitialized){
//      for ( int i = 0; i < POSTURE_QUEUE_SIZE; i++){
//        postureDataQueue.push_back(postureData);
//      }
//      b_postureInitialized = true;
//    }
//    else{
//      doNothing();
//    }
//    postureData = postureFilter(postureDataQueue, postureData);
//  }
//  else{
//    doNothing();
//  }

  postureData.gpsData.b_isValid = values[KEY_GPS]["IsValid"].asInt();

  postureData.gpsData.b_isAlive = true;
  postureData.gpsData.lastPacketTime = boost::posix_time::microsec_clock::local_time();
//  std::cout << postureData.lastPacketTime << std::endl;



//  std::string strTime = boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
//  int pos = strTime.find("T");

}

void Jobs::imuHandler (Json::Value values){
//temp gps
//  postureData.gpsData.b_isValid = true;
//  postureData.gpsData.b_isAlive = true;
//imu
  postureData.imuData.time = values[KEY_IMU]["Time"].asDouble();
  postureData.imuData.velocity = values[KEY_IMU]["Velocity"].asDouble();
  postureData.imuData.roll = values[KEY_IMU]["Roll"].asDouble();
  postureData.imuData.pitch = values[KEY_IMU]["Pitch"].asDouble();
  postureData.imuData.yaw = values[KEY_IMU]["Yaw"].asDouble();

  postureData.imuData.yawRate = values[KEY_IMU]["YawRate"].asDouble();

  postureData.imuData.b_gpsValid = values[KEY_IMU]["GpsValid"].asInt();
  postureData.imuData.longitude = values[KEY_IMU]["Longitude"].asDouble();
  postureData.imuData.latitude = values[KEY_IMU]["Latitude"].asDouble();

  gaussConvert(postureData.imuData.longitude, postureData.imuData.latitude, postureData.imuData.gaussX, postureData.imuData.gaussY);

//  postureData.dataTime = values[KEY_POSTURE]["Time"].asInt();
//  std::cout << "time: " << postureData.dataTime << std::endl;

//  if (true == FUSION_FLAG){
//    if (false == b_postureInitialized){
//      for ( int i = 0; i < POSTURE_QUEUE_SIZE; i++){
//        postureDataQueue.push_back(postureData);
//      }
//      b_postureInitialized = true;
//    }
//    else{
//      doNothing();
//    }
//    postureData = postureFilter(postureDataQueue, postureData);
//  }
//  else{
//    doNothing();
//  }

  postureData.imuData.b_isValid = values[KEY_IMU]["IsValid"].asInt();

  postureData.imuData.b_isAlive = true;
  postureData.imuData.lastPacketTime = boost::posix_time::microsec_clock::local_time();
//  std::cout << postureData.lastPacketTime << std::endl;
}

/*
void Jobs::postureHandler (Json::Value values){

  postureData.gpsData.longitude = values[KEY_GPS]["Longitude"].asDouble();
  postureData.gpsData.latitude = values[KEY_GPS]["Latitude"].asDouble();
  uint16_t gpsQuality = values[KEY_GPS]["Status"].asInt();
//  std::cout << gpsQuality << std::endl;
  if ( gpsQuality == 4 ||gpsQuality == 5 ||gpsQuality == 6  ){
    postureData.gpsData.b_isDiffitial = true;
  }
  else{
    postureData.gpsData.b_isDiffitial = false;
  }
//  postureData.gpsData.b_isDiffitial =  ((gpsQuality== 4)? true: false);

  gaussConvert(postureData.gpsData.longitude, postureData.gpsData.latitude, postureData.gpsData.gaussX, postureData.gpsData.gaussY);

//  std::cout << "x: " << postureData.gpsData.gaussX << " , y: " << postureData.gpsData.gaussY << std::endl;

  postureData.imuData.velocity = values[KEY_POSTURE][KEY_POSTURE_IMU]["Velocity"].asDouble();
  postureData.imuData.roll = values[KEY_POSTURE][KEY_POSTURE_IMU]["Roll"].asDouble();
  postureData.imuData.pitch = values[KEY_POSTURE][KEY_POSTURE_IMU]["Pitch"].asDouble();
  postureData.imuData.yaw = values[KEY_POSTURE][KEY_POSTURE_IMU]["Yaw"].asDouble();
//  postureData.dataTime = values[KEY_POSTURE]["Time"].asInt();
//  std::cout << "time: " << postureData.dataTime << std::endl;
  if (true == FUSION_FLAG){
    if (false == b_postureInitialized){
      for ( int i = 0; i < POSTURE_QUEUE_SIZE; i++){
        postureDataQueue.push_back(postureData);
      }
      b_postureInitialized = true;
    }
    else{
      doNothing();
    }
    postureData = postureFilter(postureDataQueue, postureData);
  }
  else{
    doNothing();
  }

  postureData.gpsData.b_isValid = values[KEY_POSTURE][KEY_POSTURE_GPS]["IsValid"].asInt();
  postureData.imuData.b_isValid = values[KEY_POSTURE][KEY_POSTURE_IMU]["IsValid"].asInt();

  postureData.b_isAlive = true;
  postureData.lastPacketTime = boost::posix_time::microsec_clock::local_time();
//  std::cout << postureData.lastPacketTime << std::endl;



//  std::string strTime = boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
//  int pos = strTime.find("T");

}
*/

void Jobs::actuatorHandler (Json::Value values){
//  actuatorData.aebStatus = values[KEY_ACTUATOR]["AebStatus"].asInt();
//  actuatorData.epsStatus = values[KEY_ACTUATOR]["EpsStatus"].asInt();
//  actuatorData.torqueStatus = values[KEY_ACTUATOR]["TorqueStatus"].asInt();
//  actuatorData.decStatus = values[KEY_ACTUATOR]["DecStatus"].asInt();
//  actuatorData.systemStatus = values[KEY_ACTUATOR]["SystemStatus"].asInt();
//  actuatorData.gearControlStatus = values[KEY_ACTUATOR]["GearControlStatus"].asInt();
//  actuatorData.breakPedalStatus = values[KEY_ACTUATOR]["BreakPedalStatus"].asInt();
//  actuatorData.cruiseStatus = values[KEY_ACTUATOR]["CruiseStatus"].asInt();
//  actuatorData.gearPositionStatus = values[KEY_ACTUATOR]["GearPositionStatus"].asInt();

  actuatorData.vehicleSpeed = values[KEY_ACTUATOR]["VehicleSpeed"].asDouble();
  actuatorData.steeringAngle = values[KEY_ACTUATOR]["SteeringAngle"].asDouble();
  actuatorData.steeringVelocity = values[KEY_ACTUATOR]["SteeringVelocity"].asDouble();
  actuatorData.targetSteeringAngle = values[KEY_ACTUATOR]["TargetSteeringAngle"].asDouble();
  actuatorData.targetSteeringVelocity = values[KEY_ACTUATOR]["TargetSteeringVelocity"].asDouble();
  actuatorData.targetTorque = values[KEY_ACTUATOR]["TargetTorque"].asDouble();
  actuatorData.targetBrakePressure = values[KEY_ACTUATOR]["targetBrakePressure"].asDouble();
  actuatorData.brakePressure = values[KEY_ACTUATOR]["BrakePressure"].asDouble();
  actuatorData.steeringTorqueVoltage = values[KEY_ACTUATOR]["SteeringTorqueVoltage"].asDouble();
  actuatorData.steeringDAC_A = values[KEY_ACTUATOR]["SteeringDAC_A"].asDouble();
  actuatorData.steeringDAC_B = values[KEY_ACTUATOR]["SteeringDAC_B"].asDouble();

  actuatorData.b_isAlive = true;
  actuatorData.lastPacketTime = boost::posix_time::microsec_clock::local_time();
}

void Jobs::uiHandler (Json::Value values){
  uiData.b_isValid = values[KEY_UI]["IsValid"].asInt();
  uiData.b_stopFlag = values[KEY_UI]["StopFlag"].asInt();
//  std::cout << "UI StopFlag = " << uiData.b_stopFlag << std::endl;
  uiData.b_speedOff = values[KEY_UI]["SpeedOff"].asInt();
  uiData.b_steeringOff = values[KEY_UI]["SteeringOff"].asInt();
  uiData.b_config1 = values[KEY_UI]["Config1"].asInt();
//  uiData.b_smallCircleSelected = values[KEY_UI]["SmallCircleSelected"].asInt();
  //uiData.lastId = values[KEY_UI]["LastId"].asInt();
//  uint16_t tempId = values[KEY_UI]["NextId"].asInt();
//  if (tempId != uiData.nextId){
//    uiData.lastId = uiData.nextId;
//    uiData.nextId = tempId;
//  }
//  else;

  // check if stopped
  boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration diff;

//  if (true == uiData.b_readyToGo){
//    doNothing();
//  }
//  else{
//    if (std::abs(postureData.imuData.velocity) > STOP_DEAD_ZONE){
//      uiData.firstStoppedTime = now;
//      uiData.b_readyToGo = false;
//    }
//    else{
//      diff = now - uiData.firstStoppedTime;
//
//      if (diff.total_milliseconds() > STOP_PERIOD){
//        uiData.b_readyToGo = true;
//      }
//      else;
//    }
//  }

  uiData.b_isAlive = true;
  uiData.lastPacketTime = boost::posix_time::microsec_clock::local_time();
}

void Jobs::decisionHandler(Json::Value values){
  decisionData.targetSteeringAngle = values[KEY_DECISION]["TargetSteeringAngleOut"].asDouble();
  decisionData.targetSpeed = values[KEY_DECISION]["TargetSpeedOut"].asDouble();
//  decisionData.gearPosition = values[KEY_DECISION]["GearPosition"].asInt();
//  decisionData.accLevel = values[KEY_DECISION]["AccLevel"].asInt();
  decisionData.targetWorkMode = values[KEY_DECISION]["TargetWorkMode"].asInt();
  decisionData.currentState = values[KEY_DECISION]["CurrentState"].asInt();
  decisionData.b_isValid = values[KEY_DECISION]["IsValid"].asInt();

  // 150710
  decisionData.currentSegId = values[KEY_DECISION]["CurrentId"].asInt();

  decisionData.currentIndex = values[KEY_DECISION]["CurrentIndex"].asInt();
  decisionData.postureSensorMode = values[KEY_DECISION]["PostureSensorMode"].asInt();

  decisionData.currentTotalIndex = values[KEY_DECISION]["CurrentTotalIndex"].asInt();
  decisionData.currentPreviewTotalIndex = values[KEY_DECISION]["CurrentPreviewTotalIndex"].asInt();
	//decisionData.maneuver = values[KEY_DECISION]["Maneuver"].asInt();  
decisionData.accRequest=values[KEY_DECISION]["TargetAccReq"].asInt();// changed
  decisionData.b_isAlive = true;
  decisionData.lastPacketTime = boost::posix_time::microsec_clock::local_time();
}

void Jobs::radarHandler (Json::Value values){
//  std::cout << "radar!" << std::endl;

  radarData.blockedAreaIndex.clear();
  radarData.zone.xLeft = values["Radar"]["Zone"]["XLeft"].asDouble();
  radarData.zone.xRight = values["Radar"]["Zone"]["XRight"].asDouble();
  radarData.zone.yTop = values["Radar"]["Zone"]["YTop"].asDouble();
  radarData.zone.angleLeft = values["Radar"]["Zone"]["AngleLeft"].asDouble();
  radarData.zone.angleRight = values["Radar"]["Zone"]["AngleRight"].asDouble();
  radarData.zone.b_isValid = values["Radar"]["Zone"]["IsValid"].asInt();
  radarData.zone.b_leftCandidate = values["Radar"]["Zone"]["LeftCandidate"].asInt();
  radarData.zone.b_rightCandidate = values["Radar"]["Zone"]["RightCandidate"].asInt();

//  std::cout << "here." << std::endl;

  Json::Value::Members members (values["Radar"]["Data"].getMemberNames());
  if(members.begin() != members.end() - 1){
//    std::cout << "Size: " << members.size() << std::endl;
    for (Json::Value::Members::iterator mit = members.begin(); mit != members.end(); mit++){
      radarData.blockedAreaIndex.push_back(values["Radar"]["Data"][*mit]["Index"].asInt());
//      std::cout << "This is the data of \"Lidar\": " << values["Lidar"]["Data"][*mit].asInt() <<std::endl;
    }
  }
  else{
    std::cout << RED << "Radar ERROR!" << RESET << std::endl;
  }

  if(true == FUSION_FLAG){
    radarDataQueue.pop_front();
    radarDataQueue.push_back(radarData);
    radarData.blockedAreaIndex.clear();
    radarData.zone.angleLeft = 0.0;
    radarData.zone.angleRight = 0.0;
    radarData.zone.xLeft = 0.0;
    radarData.zone.yTop = 0.0;
    radarData.zone.xRight = 0.0;
//    for (int i = 0; i < LIDAR_QUEUE_SIZE; i ++){
      for (std::deque<RadarData>::iterator it = radarDataQueue.begin(); it != radarDataQueue.end(); it++){
        radarData.blockedAreaIndex.insert(radarData.blockedAreaIndex.end(), radarDataQueue[it - radarDataQueue.begin()].blockedAreaIndex.begin(), radarDataQueue[it - radarDataQueue.begin()].blockedAreaIndex.end());
//    lidarData.blockedAreaIndex += lidarDataQueue[it - lidarDataQueue.begin()].blockedAreaIndex;

      }
//    }
      for (int i = 0; i < RADAR_QUEUE_SIZE; i ++){
        radarData.zone.angleLeft += radarDataQueue[i].zone.angleLeft/RADAR_QUEUE_SIZE;
        radarData.zone.angleRight += radarDataQueue[i].zone.angleRight/RADAR_QUEUE_SIZE;
        radarData.zone.xLeft += radarDataQueue[i].zone.xLeft/RADAR_QUEUE_SIZE;
        radarData.zone.yTop += radarDataQueue[i].zone.yTop/RADAR_QUEUE_SIZE;
        radarData.zone.xRight += radarDataQueue[i].zone.xRight/RADAR_QUEUE_SIZE;
      }
  }
  else{
    doNothing();
  }

//always valid//
  radarData.b_isValid = true;

  radarData.b_isAlive = true;
  radarData.lastPacketTime = boost::posix_time::microsec_clock::local_time();
}

void Jobs::iovHandler (Json::Value values){
  /*
	iovData_sub.b_isValid = values[KEY_IOV]["IsValid"].asInt();
	iovData_sub.phase = values[KEY_IOV]["Phase"].asInt();
  iovData_sub.time =values[KEY_IOV]["Time"].asInt();
  */

  iovData.b_isValid = values[KEY_IOV]["IsValid"].asInt();
  iovData.vehicleId = values[KEY_IOV]["Vehicle"]["VehicleID"].asInt();
  iovData.vehicleType = values[KEY_IOV]["Vehicle"]["vehicleType"].asInt();

  iovData.vehicleGaussX = values[KEY_IOV]["Vehicle"]["GaussX"].asDouble();
  iovData.vehicleGaussY = values[KEY_IOV]["Vehicle"]["GaussY"].asDouble();
  iovData.vehicleSpeed = values[KEY_IOV]["Vehicle"]["Speed"].asDouble();
  iovData.vehicleTime = values[KEY_IOV]["Vehicle"]["Time"].asDouble();
  
//    std::cout << std::setprecision(15) <<iovData.vehicleGaussY << std::endl;

  iovData.lightPhase = values[KEY_IOV]["Light"]["Phase"].asInt();
  iovData.lightTime = values[KEY_IOV]["Light"]["Time"].asDouble();
  iovData.lightAdvisedSpeed = values[KEY_IOV]["Light"]["AdvisedSpeed"].asDouble();

//  std::cout << "iovData_sub.b_isValid: " << iovData_sub.b_isValid <<std::endl;
//  std::cout << "iovData_sub.phase:  " << iovData_sub.phase <<std::endl;
  iovData.b_isAlive = true;
  iovData.lastPacketTime = boost::posix_time::microsec_clock::local_time();

}

void Jobs::cvHandler (Json::Value values){
  /*
	iovData_sub.b_isValid = values[KEY_IOV]["IsValid"].asInt();
	iovData_sub.phase = values[KEY_IOV]["Phase"].asInt();
  iovData_sub.time =values[KEY_IOV]["Time"].asInt();
  */

  cvData.b_isValid = values[KEY_CV]["IsValid"].asInt();
  cvData.vehicleId = values[KEY_CV]["Vehicle"]["VehicleID"].asInt();
  cvData.vehicleType = values[KEY_CV]["Vehicle"]["vehicleType"].asInt();

  cvData.vehicleGaussX = values[KEY_CV]["Vehicle"]["GaussX"].asDouble();
  cvData.vehicleGaussY = values[KEY_CV]["Vehicle"]["GaussY"].asDouble();
  cvData.vehicleSpeed = values[KEY_CV]["Vehicle"]["Speed"].asDouble();
  cvData.vehicleTime = values[KEY_CV]["Vehicle"]["Time"].asDouble();
  
  cvData.lightPhase = values[KEY_CV]["Light"]["Phase"].asInt();
  cvData.lightTime = values[KEY_CV]["Light"]["Time"].asDouble();
  cvData.lightAdvisedSpeed = values[KEY_CV]["Light"]["AdvisedSpeed"].asDouble();
  

//  std::cout << "iovData_sub.b_isValid: " << iovData_sub.b_isValid <<std::endl;
//  std::cout << "iovData_sub.phase:  " << iovData_sub.phase <<std::endl;
  cvData.b_isAlive = true;
  cvData.lastPacketTime = boost::posix_time::microsec_clock::local_time();
}

void Jobs::checkSensorHeartBeat () {
  boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration diff;

  //lidar
  diff = now - lidarData.lastPacketTime;
//  std::cout << "lidar total time: " <<  diff.total_milliseconds() << std::endl;
  if(diff.total_milliseconds() > HEARTBEAT_PERIOD){
    lidarData.b_isAlive = false;
    std::cout << RED << "Lidar Sensor DIED!!" << RESET << std::endl;
  }
  else;

  //vision
  diff = now - visionData.lastPacketTime;
//  std::cout << "vision total time: " <<  diff.total_milliseconds() << std::endl;

  if(diff.total_milliseconds() > HEARTBEAT_PERIOD){
    visionData.b_isAlive = false;
    //    tbd qing 0829
//    std::cout << RED << "Vision Sensor DIED!!" << RESET << std::endl;
  }
  else;

  //gps
  diff = now - postureData.gpsData.lastPacketTime;
//  std::cout << "gps total time: " <<  diff.total_milliseconds() << std::endl;

  if(diff.total_milliseconds() > HEARTBEAT_PERIOD){
    postureData.gpsData.b_isAlive = false;
    std::cout << RED << "Posture Sensor DIED!!" << RESET << std::endl;
  }
  else;

  //imu
  diff = now - postureData.imuData.lastPacketTime;
//  std::cout << "imu total time: " <<  diff.total_milliseconds() << std::endl;

  if(diff.total_milliseconds() > HEARTBEAT_PERIOD){
    postureData.imuData.b_isAlive = false;
    std::cout << RED << "Posture Sensor DIED!!" << RESET << std::endl;
  }
  else;
  //actuator
  diff = now - actuatorData.lastPacketTime;
  if(diff.total_milliseconds() > HEARTBEAT_PERIOD){
    actuatorData.b_isAlive = false;
    std::cout << RED << "Actuator Feed-back DIED!!" << RESET << std::endl;
  }
  else;

  //ui
  diff = now - uiData.lastPacketTime;
  if (diff.total_milliseconds() > HEARTBEAT_PERIOD){
    uiData.b_isAlive = false;
    std::cout << RED << "UI Communication DIED!!" << RESET << std::endl;
  }
  else;

  //decision
  diff = now - decisionData.lastPacketTime;
  if (DECISION_TIMEOUT_FLAG == true)
  if (diff.total_milliseconds() > HEARTBEAT_PERIOD){
    decisionData.b_isAlive = false;
    std::cout << RED << "Decision Feed-back DIED!!" << RESET << std::endl;
  }
  else;

  //radar
  diff = now - radarData.lastPacketTime;
  if (diff.total_milliseconds() > HEARTBEAT_PERIOD){
    radarData.b_isAlive = false;
//    std::cout << RED << "Radar Feed-back DIED!!" << RESET << std::endl;
//    qing 0829
  }
  else;

  //iov
  diff = now - iovData.lastPacketTime;
  if (diff.total_milliseconds() > HEARTBEAT_PERIOD){
    iovData.b_isAlive = false;
    std::cout << RED << "iov Feed-back DIED!!" << RESET << std::endl;
  }
  else;

  //cv
  diff = now - cvData.lastPacketTime;
  if (diff.total_milliseconds() > 1500){
    cvData.b_isAlive = false;
    std::cout << RED << "cv Feed-back DIED!!" << RESET << std::endl;
  }
  else;


}

Jobs::Jobs (boost::asio::io_service& io, std::vector<void*>& receivingSocketList, std::vector<void*>& sendingSocketList) : strand1(io), strand2(io), strand3(io), timer1(io, boost::posix_time::milliseconds(DEFAULT_TIMER1_START)), timer2(io, boost::posix_time::milliseconds(DEFAULT_TIMER2_START)), timer3(io, boost::posix_time::milliseconds(DEFAULT_TIMER3_START)), rSocketList(receivingSocketList), sSocketList(sendingSocketList){
  for (int i = 0; i < LIDAR_QUEUE_SIZE; i ++){
    lidarDataQueue.push_back(lidarData);
  }
  for (int i = 0; i < RADAR_QUEUE_SIZE; i ++){
    radarDataQueue.push_back(radarData);
  }
  timer1.async_wait(strand1.wrap(boost::bind(&Jobs::subscriber, this)));
  timer2.async_wait(strand2.wrap(boost::bind(&Jobs::handler, this)));
  timer3.async_wait(strand3.wrap(boost::bind(&Jobs::publisher, this)));
}

Jobs::~Jobs (){
  ;
}

void Jobs::subscriber (){
  //std::cout << "Subscribing ..." << std::endl;


  zmq_pollitem_t items [9];
  items[0].socket = rSocketList[0];
  items[0].events = ZMQ_POLLIN;
  items[1].socket = rSocketList[1];
  items[1].events = ZMQ_POLLIN;
  items[2].socket = rSocketList[2];
  items[2].events = ZMQ_POLLIN;
  items[3].socket = rSocketList[3];
  items[3].events = ZMQ_POLLIN;
  items[4].socket = rSocketList[4];
  items[4].events = ZMQ_POLLIN;
  //decision
  items[5].socket = rSocketList[5];
  items[5].events = ZMQ_POLLIN;
  //radar
  items[6].socket = rSocketList[6];
  items[6].events = ZMQ_POLLIN;
  //iov
  items[7].socket = rSocketList[7];
  items[7].events = ZMQ_POLLIN;
  //imu
  items[8].socket = rSocketList[8];
  items[8].events = ZMQ_POLLIN;
  //cv
  items[9].socket = rSocketList[9];
  items[9].events = ZMQ_POLLIN;
  

  //std::cout << "Polling Information ..." << std::endl;
  zmq_poll (items, 10, 0);

  if (items [0].revents & ZMQ_POLLIN) {
    char msg[81920] = {0};
    int size = zmq_recv (rSocketList[0], msg, 81920, 0);
    if (size != -1) {
      // Lidar
//      std::cout << "Got Lidar Information." << std::endl;

      std::stringstream ss;
      ss << msg;
//      std::cout << "I got this: " << ss.str() << std::endl;
      if(mutex.try_lock ()){
        rStringList.push_back(ss.str());
        mutex.unlock ();
      }
      else{
        std::cout << RED << "failed locking rStringList! (item 0)" << RESET << std::endl;
      }
    }
  }

  if (items [1].revents & ZMQ_POLLIN) {
    char msg[8192] = {0};
    int size = zmq_recv (rSocketList[1], msg, 8192, 0);
    if (size != -1) {
        // vision
//      std::cout << "Got Vision Information." << std::endl;

      std::stringstream ss;
      ss << msg;

//      std::cout << "I got this: " << ss.str() << std::endl;
      if(mutex.try_lock ()){
        rStringList.push_back(ss.str());
        mutex.unlock ();
      }
      else{
        std::cout << RED << "failed locking rStringList! (item 1)" << RESET << std::endl;
      }
    }
  }

  // item 2
  if (items [2].revents & ZMQ_POLLIN) {
    char msg[8192] = {0};
    int size = zmq_recv (rSocketList[2], msg, 8192, 0);
    if (size != -1) {
        // posture
//      std::cout << "Got Posture Information." << std::endl;

      std::stringstream ss;
      ss << msg;

//      std::cout << "I got this: " << ss.str() << std::endl;
//      if(mutex.try_lock ()){
        mutex.lock();
        rStringList.push_back(ss.str());
        mutex.unlock ();
//      }
//      else{
//        std::cout << RED << "failed locking rStringList! (item 2)" << RESET << std::endl;
//      }
    }
  }

  //item 3
  if (items[3].revents & ZMQ_POLLIN){
    char msg[8192] = {0};
    int size = zmq_recv (rSocketList[3], msg, 8192, 0);
    if (size != -1) {
        // Actuator
//      std::cout << "Got Posture Information." << std::endl;

      std::stringstream ss;
      ss << msg;
// 
//     std::cout << msg << std::endl;
//      std::cout << "I got this: " << ss.str() <<"That's it."<< std::endl;
      if(mutex.try_lock ()){
        rStringList.push_back(ss.str());
        mutex.unlock ();
      }
      else{
        std::cout << RED << "failed locking rStringList! (item 3)" << RESET << std::endl;
      }
    }
  }

  //item 4
  if (items[4].revents & ZMQ_POLLIN){
    char msg[8192] = {0};
    int size = zmq_recv (rSocketList[4], msg, 8192, 0);
    if (size != -1) {
        // Actuator
//      std::cout << "Got Posture Information." << std::endl;

      std::stringstream ss;
      ss << msg;

//      std::cout << msg << std::endl;
//      std::cout << "I got this: \"" << ss.str() <<"\" That's it."<< std::endl;
//      if(mutex.try_lock ()){
        mutex.lock();
        rStringList.push_back(ss.str());
        mutex.unlock ();
//      }
//      else{
//        std::cout << RED << "failed locking rStringList! (item 4)" << RESET << std::endl;
//      }
    }
  }

  //item 5
  if (items[5].revents & ZMQ_POLLIN){
    char msg[8192] = {0};
    int size = zmq_recv (rSocketList[5], msg, 8192, 0);
    if (size != -1) {
        // Actuator
//      std::cout << "Got Posture Information." << std::endl;

      std::stringstream ss;
      ss << msg;

//      std::cout << msg << std::endl;
//      std::cout << "I got this: " << ss.str() <<"That's it."<< std::endl;
      if(mutex.try_lock ()){
        rStringList.push_back(ss.str());
        mutex.unlock ();
      }
      else{
        std::cout << RED << "failed locking rStringList! (item 4)" << RESET << std::endl;
      }
    }
  }

  if (items [6].revents & ZMQ_POLLIN) {
    char msg[81920] = {0};
    int size = zmq_recv (rSocketList[6], msg, 81920, 0);
    if (size != -1) {
      // Radar
      std::cout << "Got Radar Information." << std::endl;

      std::stringstream ss;
      ss << msg;
      std::cout << "I got this: " << ss.str() << std::endl;
      if(mutex.try_lock ()){
        rStringList.push_back(ss.str());
        mutex.unlock ();
      }
      else{
        std::cout << RED << "failed locking rStringList! (item 6)" << RESET << std::endl;
      }
    }
  }

// item 7
  if (items [7].revents & ZMQ_POLLIN) {
    char msg[81920] = {0};
    int size = zmq_recv (rSocketList[7], msg, 81920, 0);
    if (size != -1) {

      std::stringstream ss;
      ss << msg;

      std::cout << "Got iov." << std::endl;
      std::cout << "I got this: " << ss.str() << std::endl;

      if(mutex.try_lock ()){
        rStringList.push_back(ss.str());
        mutex.unlock ();
      }
      else{
        std::cout << RED << "failed locking rStringList! (item 7)" << RESET << std::endl;
      }
    }
  }
// item 8
  if (items [8].revents & ZMQ_POLLIN) {
    char msg[81920] = {0};
    int size = zmq_recv (rSocketList[8], msg, 81920, 0);
    if (size != -1) {
      // imu
      //std::cout << "Got imu." << std::endl;

      std::stringstream ss;
      ss << msg;

      if(mutex.try_lock ()){
        rStringList.push_back(ss.str());
        mutex.unlock ();
      }
      else{
        std::cout << RED << "failed locking rStringList! (item 8)" << RESET << std::endl;
      }
    }
  }

// item 9
  if (items [9].revents & ZMQ_POLLIN) {
    char msg[81920] = {0};
    int size = zmq_recv (rSocketList[9], msg, 81920, 0);
    if (size != -1) {
      //cv
      //std::cout << "Got cv." << std::endl;

      std::stringstream ss;
      ss << msg;
//      std::cout << ss.str() <<std::endl;

      if(mutex.try_lock ()){
        rStringList.push_back(ss.str());
        mutex.unlock ();
      }
      else{
        std::cout << RED << "failed locking rStringList! (item 9)" << RESET << std::endl;
      }
    }
  }
//  std::cout << "sub done." << std::endl;

//      while(1){sleep(1);}

  timer1.expires_at(timer1.expires_at() + boost::posix_time::milliseconds(DEFAULT_TIMER1_PERIOD));
  timer1.async_wait(strand1.wrap(boost::bind(&Jobs::subscriber, this)));
}

void Jobs::handler(){
//  std::cout << "Handling ..." << std::endl;

  Json::Value values;
  Json::Reader reader;

  std::cout<<std::endl;

  if(mutex.try_lock ()){
    for(std::vector<std::string>::iterator it = rStringList.begin(); it < rStringList.end(); it++){
//      std::cout << "goint to parse..." << std::endl;
      if(true == reader.parse(rStringList[it-rStringList.begin()], values)){
        Json::Value::Members members (values.getMemberNames());

//        std::string key = "Null";
        if(members.begin() != members.end()){
          const std::string &key = *members.begin();
//          std::string key = *members.begin();

          std::cout << GREEN << "key : " << key << RESET << std::endl;
          if (0 == key.compare(KEY_LIDAR)){
  //          std::cout << "lidar"  << std::endl;
            lidarHandler(values);
  //          uint16_t data = values["Lidar"].asInt();
  //          std::cout << "This is the data of \"Lidar\": " << data <<std::endl;
          }
          else if (0 == key.compare(KEY_VISION)){
            visionHandler(values);
          }
          else if (0 == key.compare(KEY_GPS)){
            gpsHandler(values);
          }
          else if (0 == key.compare(KEY_IMU)){
            imuHandler(values);
          }
          else if (0 == key.compare(KEY_ACTUATOR)){
            actuatorHandler(values);
          }
          else if (0 == key.compare(KEY_UI)){
            uiHandler(values);
          }
          else if (0 == key.compare(KEY_DECISION)){
            decisionHandler(values);
          }
          else if ( 0 == key.compare(KEY_RADAR)){
            radarHandler(values);
          }
		      else if ( 0 == key.compare(KEY_IOV)){
			      iovHandler(values);
		      }
          else if ( 0 == key.compare(KEY_CV)){
			      cvHandler(values);
		      }
          else{
            std::cout << RED << "No matching KEY!" << RESET << std::endl;
          }
        }

        else{
          std::cout << "members is null" << std::endl;
        }
      }

      else{
        std::cout << "Parse error  :(" << std::endl;

      }
    }
    rStringList.clear ();
    checkSensorHeartBeat();
    mutex.unlock ();
  }
  else{
    std::cout << RED << "failed locking shared stream" << RESET << std::endl;
  }

  //save Data
  if (true == SAVE_ROAD_FLAG){
//    std::ofstream fs("savedData.txt", std::ofstream::app);
//    fs << std::setprecision(15) << boost::posix_time::microsec_clock::local_time() << ", " << postureData.gpsData.longitude << ", " << postureData.gpsData.latitude << std::endl;
//    std::ofstream fs("roadMap.txt", std::ofstream::app);
    static std::ofstream fsRoad("rawMap.txt", std::ofstream::trunc);
//    fsRoad << std::setprecision(15) << 1 << " " << postureData.gpsData.longitude << " " << postureData.gpsData.latitude << " " << postureData.gpsData.yaw << " " << 4 << " " << 0 << std::endl;
    fsRoad << std::setprecision(15) << 1 << " " << postureData.gpsData.longitude << " " << postureData.gpsData.latitude << " " << postureData.gpsData.yaw << " " << 4 << " " << 0 << std::endl;
//    fsRoad.close();
  }
  else;

  if (true  == SAVE_DATA_FLAG){
    static bool firstTimeFlag = true;
    std::stringstream ss1, ss2;
    ss1 << boost::posix_time::second_clock::local_time() << " savedData" << ".txt";
    ss2 << boost::posix_time::second_clock::local_time() <<  " list" << ".txt";
    std::string fsName = ss1.str();
    std::string listName = ss2.str();
//    std::ofstream fs("savedData.txt", std::ofstream::app);
    static std::ofstream list(listName, std::ofstream::trunc);
    static std::ofstream fs(fsName, std::ofstream::trunc);
//    for (static bool firstTimeFlag = true; firstTimeFlag; firstTimeFlag = false) {

    list << " ";
    fs << std::setprecision(15) << boost::posix_time::microsec_clock::local_time() << " ";

//    SAVEDATA(postureData.gpsData.longitude, list, fs, firstTimeFlag);
    SAVEDATA(postureData.gpsData.longitude);
    SAVEDATA(postureData.gpsData.latitude);
    SAVEDATA(postureData.gpsData.yaw);
    SAVEDATA(postureData.gpsData.gpsQuality);
    SAVEDATA(postureData.gpsData.gaussX);
    SAVEDATA(postureData.gpsData.gaussY);
    SAVEDATA(postureData.gpsData.utc_second);
    SAVEDATA(postureData.imuData.roll);
    SAVEDATA(postureData.imuData.yaw);
    SAVEDATA(postureData.imuData.pitch);
    SAVEDATA(postureData.imuData.longitude);
    SAVEDATA(postureData.imuData.latitude);
    SAVEDATA(postureData.imuData.gaussX);
    SAVEDATA(postureData.imuData.gaussY);
    SAVEDATA(postureData.imuData.velocity);
    SAVEDATA(postureData.imuData.time);

    SAVEDATA(actuatorData.vehicleSpeed);
    SAVEDATA(actuatorData.steeringAngle);
    SAVEDATA(actuatorData.steeringVelocity);
    SAVEDATA(actuatorData.targetSteeringAngle);
    SAVEDATA(actuatorData.targetSteeringVelocity);
    SAVEDATA(actuatorData.targetTorque);
    SAVEDATA(actuatorData.targetBrakePressure);
    SAVEDATA(actuatorData.brakePressure);
    SAVEDATA(actuatorData.steeringTorqueVoltage);
    SAVEDATA(actuatorData.steeringDAC_A);
    SAVEDATA(actuatorData.steeringDAC_B);

    SAVEDATA(decisionData.targetSteeringAngle);
    SAVEDATA(decisionData.targetSpeed);
    SAVEDATA(decisionData.targetWorkMode);
    SAVEDATA(decisionData.currentState);
    SAVEDATA(decisionData.currentSegId);
    SAVEDATA(decisionData.currentIndex);
    SAVEDATA(decisionData.postureSensorMode);
    SAVEDATA(decisionData.currentTotalIndex);
    SAVEDATA(decisionData.currentPreviewTotalIndex);
    //SAVEDATA(decisionData.maneuver);

    SAVEDATA(iovData.b_isAlive);
    SAVEDATA(iovData.vehicleGaussX);
    SAVEDATA(iovData.vehicleGaussY);
    SAVEDATA(iovData.vehicleSpeed);


    if (firstTimeFlag) {
      list << std::endl;
    }else;
    firstTimeFlag = false;

    fs << std::endl;
//    fs.close();
  }

  else;



  // fake traffic light WORK
  //fake iov work
  /*
  uint32_t currentSec = getCurrentSec();
  int redLightTime = 10;
  int greenLightTime = 15;
  int totalLightTime = redLightTime + greenLightTime;

  iovData.time = totalLighTime - currentSec % totalLightTime;
  if ( iovData.time < greenLightTime ){
    iovData.phase = 1;
    iovData.advSpd = 2;
  }
  else {
    iovData.phase = 0;
    iovData.advSpd = 0;
  }
  */

//fake cv work
  /*
  uint32_t currentSec = getCurrentSec();
  cvData.lightTime = 15 - currentSec % 15;
//  if ((0 <= currentSec && 15 > currentSec)||(30 <= currentSec && 45 > currentSec) || (60 == currentSec )){
  if (( 15 > currentSec)||(30 <= currentSec && 45 > currentSec) || (60 == currentSec )){
    cvData.lightPhase = 1;
  }
  else {
    cvData.lightPhase = 0;
  }
*/
if( MY_CV_SWITCH == true){
  time_t tt;
  time(&tt);
  uint32_t currentSec = tt;
  int redLightTime = 15;
  int greenLightTime = 15;
  int totalLightTime = redLightTime + greenLightTime;

  int cntTime = totalLightTime - currentSec % totalLightTime;
  if ( cntTime < greenLightTime ){
    cvData.lightTime = cntTime;
    cvData.lightPhase = 1;
  }
  else {
    cvData.lightTime = cntTime - greenLightTime;
    cvData.lightPhase = 0;
  }
  std::cout << "lightTime = " << cvData.lightTime << std::endl;
  std::cout << "lightPhase = " << cvData.lightPhase << std::endl;
}
  // disabled 20180305
//IOV Publisher Work
//  iovData.b_isValid = true;
//  iovData.b_isAlive = true;
//  iovData.vehicleGaussX = postureData.imuData.gaussX;
//  iovData.vehicleGaussY = postureData.imuData.gaussY;
//  iovData.vehicleSpeed = postureData.imuData.velocity;


// old counter for iov
//  iovData.fakeLightCounter++;
//  iovData.time = 15 - static_cast<int>(std::floor(iovData.fakeLightCounter/10.0));
//  if ( 150 == iovData.fakeLightCounter) {
//    iovData.fakeLightCounter = 0;
//
//    if ( 0 == iovData.phase ){
//      iovData.phase = 1;
//      iovData.advSpd = 2;
//    }
//    else {
//      iovData.phase = 0;
//      iovData.advSpd = 0;
//    }
//  }
//  else;
//  end old counter for iov


  timer2.expires_at(timer2.expires_at() + boost::posix_time::milliseconds(DEFAULT_TIMER2_PERIOD));
  timer2.async_wait(strand2.wrap(boost::bind(&Jobs::handler, this)));
}

void Jobs::publisher(){
  //  std::cout << "Broadcasting ..." << std::endl;

  Json::Value values;
  Json::Value iovValues;

  mutex.lock();
  //lidar
  if(0 == lidarData.blockedAreaIndex.size() || false == lidarData.b_isAlive){
//    std::cout << "nothing blocked." << std::endl;
    values["Lidar"]["IsAlive"] = lidarData.b_isAlive;

  }
  else{
    values["Lidar"]["IsAlive"] = lidarData.b_isAlive;
    values["Lidar"]["IsValid"] = lidarData.b_isValid;
//    std::cout << "to send lidar data" << std::endl;
    for (uint32_t i = 0; i < lidarData.blockedAreaIndex.size(); i++){
      std::stringstream ss;
      ss << i;
      values["Lidar"]["Data"][ss.str()] = lidarData.blockedAreaIndex[i];
    }
    values["Lidar"]["Zone"]["XLeft"] = lidarData.zone.xLeft;
    values["Lidar"]["Zone"]["XRight"] = lidarData.zone.xRight;
    values["Lidar"]["Zone"]["AngleLeft"] = lidarData.zone.angleLeft;
    values["Lidar"]["Zone"]["AngleRight"] = lidarData.zone.angleRight;
    values["Lidar"]["Zone"]["YTop"] = lidarData.zone.yTop;
    values["Lidar"]["Zone"]["YTopWider"] = lidarData.zone.yTopWider;
    values["Lidar"]["Zone"]["YTopEvenWider"] = lidarData.zone.yTopEvenWider;
    values["Lidar"]["Zone"]["IsValid"] = lidarData.zone.b_isValid;
    values["Lidar"]["Zone"]["LeftCandidate"] = lidarData.zone.b_leftCandidate;
    values["Lidar"]["Zone"]["RightCandidate"] = lidarData.zone.b_rightCandidate;

    values["Lidar"]["Zone"]["PathAngle"] = (lidarData.zone.angleLeft + lidarData.zone.angleRight)/2;
//    std::cout << "lidar sending done" << std::endl;
//    values["Lidar"] = lidarData.blockedAreaIndex[0];
//    values["Alive"] = heartBeat[0];
  }

  //vision
  if(true == visionData.b_isAlive){
//    values["LaneModel"]["LaneType"] = visionData.laneType;
//    values["LaneModel"]["LaneCount"] = visionData.laneCount;
//    values["LaneModel"]["IsValid"]["RR"] = visionData.b_isValid[0];
//    values["LaneModel"]["IsValid"]["R"] = visionData.b_isValid[1];
//    values["LaneModel"]["IsValid"]["L"] = visionData.b_isValid[2];
//    values["LaneModel"]["IsValid"]["LL"] = visionData.b_isValid[3];
//
//    values["LaneModel"]["OffsetRight"] = visionData.rightError;
//    values["LaneModel"]["OffsetLeft"] = visionData.leftError;
//    values["LaneModel"]["AngleError"] = visionData.angleError;
    values[KEY_VISION][KEY_VISION_LANE]["LaneCount"] = visionData.laneData.laneCount;
    values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["RR"] = visionData.laneData.laneStatus[0];
    values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["R"] = visionData.laneData.laneStatus[1];
    values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["L"] = visionData.laneData.laneStatus[2];
    values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["LL"] = visionData.laneData.laneStatus[3];
    values[KEY_VISION][KEY_VISION_LANE]["OffsetRight"] = visionData.laneData.offsetRight;
    values[KEY_VISION][KEY_VISION_LANE]["OffsetLeft"] = visionData.laneData.offsetLeft;
    values[KEY_VISION][KEY_VISION_LANE]["AngleError"] =  visionData.laneData.angleError;

    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["IsSolid"] = visionData.laneData.leftLaneParameter.b_isSolid;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["LaneType"] = visionData.laneData.leftLaneParameter.laneType;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealPara1"] = visionData.laneData.leftLaneParameter.realParameter1;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealPara2"] = visionData.laneData.leftLaneParameter.realParameter2;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealPara3"] = visionData.laneData.leftLaneParameter.realParameter3;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealLineK"] = visionData.laneData.leftLaneParameter.realLineK;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["RealLineB"] = visionData.laneData.leftLaneParameter.realLineB;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["LatOffset"] = visionData.laneData.leftLaneParameter.latOffset;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["YawAngle"] = visionData.laneData.leftLaneParameter.yawAngle;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["LatOffset_Filtered"] = visionData.laneData.leftLaneParameter.latOffsetFiltered;
    values[KEY_VISION][KEY_VISION_LANE]["LeftLane"]["YawAngle_Filtered"] = visionData.laneData.leftLaneParameter.yawAngleFiltered;

    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["IsSolid"] = visionData.laneData.rightLaneParameter.b_isSolid;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["LaneType"] = visionData.laneData.rightLaneParameter.laneType;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealPara1"] = visionData.laneData.rightLaneParameter.realParameter1;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealPara2"] = visionData.laneData.rightLaneParameter.realParameter2;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealPara3"] = visionData.laneData.rightLaneParameter.realParameter3;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealLineK"] = visionData.laneData.rightLaneParameter.realLineK;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["RealLineB"] = visionData.laneData.rightLaneParameter.realLineB;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["LatOffset"] = visionData.laneData.rightLaneParameter.latOffset;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["YawAngle"] = visionData.laneData.rightLaneParameter.yawAngle;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["LatOffset_Filtered"] = visionData.laneData.rightLaneParameter.latOffsetFiltered;
    values[KEY_VISION][KEY_VISION_LANE]["RightLane"]["YawAngle_Filtered"] = visionData.laneData.rightLaneParameter.yawAngleFiltered;

    values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["LeftIsValid"] = visionData.trafficLightData.b_leftIsValid;
    values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["LeftPassable"] = visionData.trafficLightData.leftPassable;
    values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["StraightIsValid"] = visionData.trafficLightData.b_straightIsValid;
    values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["StraightPassable"] = visionData.trafficLightData.straightPassable;
    values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["RightIsValid"] = visionData.trafficLightData.b_rightIsValid;
    values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["RightPassable"] = visionData.trafficLightData.rightPassable;
    values[KEY_VISION][KEY_VISION_TRAFFIC_LIGHT]["Start"] = visionData.trafficLightData.start;

    values[KEY_VISION][KEY_VISION_TRAFFIC_SIGN]["IsValid"] = visionData.trafficSignData.b_isValid;
    values[KEY_VISION][KEY_VISION_TRAFFIC_SIGN]["Pattern"] = visionData.trafficSignData.pattern;

    values[KEY_VISION]["IsAlive"] = visionData.b_isAlive;
    //values["LaneModel"]["IsValid"] = true;
  }
  else{
    values[KEY_VISION]["IsAlive"] = visionData.b_isAlive;
  }


  //posture
//  if ( true == postureData.b_isAlive){

//  if (true == postureData.gpsData.b_isAlive && true  == postureData.imuData.b_isAlive){
  values[KEY_POSTURE]["IsAlive"] = false;
  if ( true  == postureData.gpsData.b_isAlive){
    values[KEY_POSTURE][KEY_POSTURE_GPS]["Longitude"] = postureData.gpsData.longitude;
    values[KEY_POSTURE][KEY_POSTURE_GPS]["Latitude"] = postureData.gpsData.latitude;
    values[KEY_POSTURE][KEY_POSTURE_GPS]["UtcSecond"] = postureData.gpsData.utc_second;
    values[KEY_POSTURE][KEY_POSTURE_GPS]["IsValid"] = postureData.gpsData.b_isValid;
    values[KEY_POSTURE][KEY_POSTURE_GPS]["IsDifferential"] = postureData.gpsData.b_isDiffitial;
    values[KEY_POSTURE][KEY_POSTURE_GPS]["Status"] = postureData.gpsData.gpsQuality;
//    if (postureData.gpsData.b_isDiffitial == 1)
//    {
//      std::cout << "here"<< std::endl;
//      std::cout << postureData.gpsData.b_isDiffitial << std::endl;
//    }
//    else{
//      std::cout << "else" << std::endl;
//    }
    values[KEY_POSTURE][KEY_POSTURE_GPS]["GaussX"] = postureData.gpsData.gaussX;
    values[KEY_POSTURE][KEY_POSTURE_GPS]["GaussY"] = postureData.gpsData.gaussY;
    values[KEY_POSTURE][KEY_POSTURE_GPS]["Yaw"] = postureData.gpsData.yaw + YAW_ADJ_DEGREE_GPS;
std::cout<< values[KEY_POSTURE][KEY_POSTURE_GPS]["Yaw"]<<std::endl;
    values[KEY_POSTURE]["IsAlive"] = true;
  }else;
  values[KEY_POSTURE][KEY_POSTURE_GPS]["IsAlive"] = postureData.gpsData.b_isAlive;

  if ( true  == postureData.imuData.b_isAlive){
    values[KEY_POSTURE][KEY_POSTURE_IMU]["Velocity"] = postureData.imuData.velocity;
    values[KEY_POSTURE][KEY_POSTURE_IMU]["Roll"] = postureData.imuData.roll;
    values[KEY_POSTURE][KEY_POSTURE_IMU]["Yaw"] = postureData.imuData.yaw + YAW_ADJ_DEGREE_IMU;
    values[KEY_POSTURE][KEY_POSTURE_IMU]["Pitch"] = postureData.imuData.pitch;
    values[KEY_POSTURE][KEY_POSTURE_IMU]["IsValid"] = postureData.imuData.b_isValid;

    values[KEY_POSTURE][KEY_POSTURE_IMU]["YawRate"] = postureData.imuData.yawRate;

    values[KEY_POSTURE][KEY_POSTURE_IMU]["GpsValid"] = postureData.imuData.b_gpsValid;
    values[KEY_POSTURE][KEY_POSTURE_IMU]["GaussX"] = postureData.imuData.gaussX;
    values[KEY_POSTURE][KEY_POSTURE_IMU]["GaussY"] = postureData.imuData.gaussY;
//    values[KEY_POSTURE]["IsAlive"] = postureData.b_isAlive;
    values[KEY_POSTURE]["IsAlive"] = true;

//    std::cout << postureData.dataTime<< std::endl;
  }
  else{
//    values[KEY_POSTURE]["IsAlive"] = false;
//    std::cout << RED << "!!!POSTURE DIED!!!" << RESET << std::endl;
//    std::cout <<std::endl;
//    std::cout <<std::endl;
//    std::cout <<std::endl;
//    std::cout <<std::endl;
//    std::cout <<std::endl;
//    std::cout <<std::endl;
  }
  values[KEY_POSTURE][KEY_POSTURE_IMU]["IsAlive"] = postureData.imuData.b_isAlive;

  //actuator
  if (true == actuatorData.b_isAlive){
//    values["Actuator"]["AebStatus"] = actuatorData.aebStatus;
//    values["Actuator"]["EpsStatus"] = actuatorData.epsStatus;
//    values["Actuator"]["TorqueStatus"] = actuatorData.torqueStatus;
//    values["Actuator"]["DecStatus"] = actuatorData.decStatus;
//    values["Actuator"]["SystemStatus"] = actuatorData.systemStatus;
//    values["Actuator"]["GearControlStatus"] = actuatorData.gearControlStatus;
//    values["Actuator"]["BreakPedalStatus"] = actuatorData.breakPedalStatus;
//    values["Actuator"]["CruiseStatus"] = actuatorData.cruiseStatus;
//    values["Actuator"]["GearPositionStatus"] = actuatorData.gearPositionStatus;

    values[KEY_ACTUATOR]["VehicleSpeed"] = actuatorData.vehicleSpeed;
    values[KEY_ACTUATOR]["SteeringAngle"] = actuatorData.steeringAngle;
    values[KEY_ACTUATOR]["SteeringVelocity"] = actuatorData.steeringVelocity;
    values[KEY_ACTUATOR]["TargetSteeringAngle"] = actuatorData.targetSteeringAngle;
    values[KEY_ACTUATOR]["TargetSteeringVelocity"] = actuatorData.targetSteeringVelocity;
    values[KEY_ACTUATOR]["TargetTorque"] = actuatorData.targetTorque;
    values[KEY_ACTUATOR]["targetBrakePressure"] = actuatorData.targetBrakePressure;
    values[KEY_ACTUATOR]["BrakePressure"] = actuatorData.brakePressure;
    values[KEY_ACTUATOR]["SteeringTorqueVoltage"] = actuatorData.steeringTorqueVoltage;
    values[KEY_ACTUATOR]["SteeringDAC_A"] = actuatorData.steeringDAC_A;
    values[KEY_ACTUATOR]["SteeringDAC_B"] = actuatorData.steeringDAC_B;

    //by bianyougang 
    values[KEY_ACTUATOR]["IsAlive"] = true;
  }
  else{
    values[KEY_ACTUATOR]["IsAlive"] = actuatorData.b_isAlive;
  }

  //start
  values[KEY_START]["IsValid"] = 1;
//  values[KEY_START]["Data"] = ((actuatorData.systemStatus==0)||(visionData.trafficLightData.start == 1))?1:0;
  values[KEY_START]["Data"] = 1;

  //intersection
  //values[KEY_INTERSECTION]["IsValid"] = intersectionData.b_isValid;
  //values[KEY_INTERSECTION]["IntersectionType"] = intersectionData.intersectionType;

  //ui
  if (true == uiData.b_isAlive){
//    values[KEY_UI]["LastId"] = uiData.lastId;
//    values[KEY_UI]["NextId"] = uiData.nextId;
    values[KEY_UI]["StopFlag"] = uiData.b_stopFlag;
    values[KEY_UI]["SpeedOff"] = uiData.b_speedOff;
    values[KEY_UI]["SteeringOff"] = uiData.b_steeringOff;
    values[KEY_UI]["Config1"] = uiData.b_config1;
//    values[KEY_UI]["StopFlag"] = false;
//    values[KEY_UI]["SmallCircleSelected"] = uiData.b_smallCircleSelected;
//    values[KEY_UI]["ReadyToGo"] = uiData.b_readyToGo;
//    values[KEY_UI]["ReadyToGo"] = true;
    values[KEY_UI]["IsValid"] = uiData.b_isValid;
    values[KEY_UI]["IsAlive"] = uiData.b_isAlive;
  }
  else{
    values[KEY_UI]["IsAlive"] = uiData.b_isAlive;
  }

  //decision
  if (true == decisionData.b_isAlive){
    values[KEY_DECISION]["TargetSteeringAngleOut"] = decisionData.targetSteeringAngle;
    values[KEY_DECISION]["TargetSpeedOut"] = decisionData.targetSpeed;
    values[KEY_DECISION]["TargetWorkMode"] = decisionData.targetWorkMode;
//    values[KEY_DECISION]["GearPosition"] = decisionData.gearPosition;
//    values[KEY_DECISION]["AccLevel"] = decisionData.accLevel;
    values[KEY_DECISION]["CurrentState"] = decisionData.currentState;
    values[KEY_DECISION]["IsValid"] = decisionData.b_isValid;
    values[KEY_DECISION]["CurrentId"] = decisionData.currentSegId;
    values[KEY_DECISION]["CurrentIndex"] = decisionData.currentIndex;
    //values[KEY_DECISION]["Maneuver"] = decisionData.maneuver;
     values[KEY_DECISION]["TargetAccReq"] =decisionData.accRequest;//changed
  }
  else{
    values[KEY_DECISION]["IsAlive"] = decisionData.b_isAlive;
  }


  //CV
  if ( MY_CV_SWITCH == true && inArea( postureData.imuData.longitude, postureData.imuData.latitude, TRAFFIC_LIGHT_X, TRAFFIC_LIGHT_Y, TRAFFIC_LIGHT_AREA)){
//    std::cout << "light valid." << std::endl;
    cvData.b_lightValid = true;
    cvData.b_isAlive = true;
    cvData.b_isValid = true;
  }
  else{
    cvData.b_lightValid = false;
    cvData.b_isAlive = false;
    cvData.b_isValid = false;
  }

  //iov
  if (inArea( postureData.gpsData.longitude, postureData.gpsData.latitude, INTERSECTION_X, INTERSECTION_Y, INTERSECTION_AREA)){
    iovData.b_vehicleValid = true;
  }
  else{
    iovData.b_vehicleValid = false;
  }

  {
//    values[KEY_IOV]["IsAlive"] = true;
//161201
    // values[KEY_IOV]["IsAlive"] = false;
    // values[KEY_IOV]["Phase"] = iovData.phase;
    // values[KEY_IOV]["Time"] = iovData.time;
    // values[KEY_IOV]["AdvSpd"] = iovData.advSpd;

    values[KEY_IOV]["IsAlive"] = iovData.b_isAlive;
    values[KEY_IOV]["IsValid"] = iovData.b_isValid;

    values[KEY_IOV]["Vehicle"]["Valid"] = iovData.b_vehicleValid;
    values[KEY_IOV]["Vehicle"]["Id"] = iovData.vehicleId;
    values[KEY_IOV]["Vehicle"]["Type"] = iovData.vehicleType;
    values[KEY_IOV]["Vehicle"]["GaussX"] = iovData.vehicleGaussX;
    values[KEY_IOV]["Vehicle"]["GaussY"] = iovData.vehicleGaussY;
    values[KEY_IOV]["Vehicle"]["Speed"] = iovData.vehicleSpeed;
    values[KEY_IOV]["Vehicle"]["Time"] = iovData.vehicleTime;
    values[KEY_IOV]["Vehicle"]["AdvisedSpeed"] = iovData.vehicleAdvisedSpeed;
//    std::cout << std::setprecision(15) << "gaussY: " << iovData.vehicleGaussY << std::endl;

    values[KEY_IOV]["Light"]["Valid"] = iovData.b_lightValid;
    values[KEY_IOV]["Light"]["Time"] = iovData.lightTime;
    values[KEY_IOV]["Light"]["Phase"] = iovData.lightPhase;
    values[KEY_IOV]["Light"]["AdvisedSpeed"] = iovData.lightAdvisedSpeed;

    values[KEY_CV]["IsAlive"] = cvData.b_isAlive;
    values[KEY_CV]["IsValid"] = cvData.b_isValid;

    values[KEY_CV]["Vehicle"]["Valid"] = cvData.b_vehicleValid;
    values[KEY_CV]["Vehicle"]["Id"] = cvData.vehicleId;
    values[KEY_CV]["Vehicle"]["Type"] = cvData.vehicleType;
    values[KEY_CV]["Vehicle"]["GaussX"] = cvData.vehicleGaussX;
    values[KEY_CV]["Vehicle"]["GaussY"] = cvData.vehicleGaussY;
    values[KEY_CV]["Vehicle"]["Speed"] = cvData.vehicleSpeed;
    values[KEY_CV]["Vehicle"]["Time"] = cvData.vehicleTime;
    values[KEY_CV]["Vehicle"]["AdvisedSpeed"] = cvData.vehicleAdvisedSpeed;

    values[KEY_CV]["Light"]["Valid"] = cvData.b_lightValid;
    values[KEY_CV]["Light"]["Time"] = cvData.lightTime;
    values[KEY_CV]["Light"]["Phase"] = cvData.lightPhase;
    //values[KEY_CV]["Light"]["Phase"] = 1;
    values[KEY_CV]["Light"]["AdvisedSpeed"] = cvData.lightAdvisedSpeed;


    //for tablet
    if(true == cvData.b_isAlive){
    values["UiLight"]["IsValid"] = true;
    values["UiLight"]["IsAlive"] = true;
    values["UiLight"]["Time"] = cvData.lightTime;
    values["UiLight"]["Phase"] = cvData.lightPhase;
    //values["UiLight"]["Phase"] = 1;
    }
    else{
      values["UiLight"]["IsAlive"] = false;
    }

//    values[KEY_IOV]["Gw"]["Phase"] = iovData.phase;
//    values[KEY_IOV]["Gw"]["Time"] = iovData.time;
//    values[KEY_IOV]["Gw"]["AdvSpd"] = iovData.advSpd;

  }

  {
    iovValues[KEY_IOV]["IsAlive"] = true;
    iovValues[KEY_IOV]["IsValid"] = true;

    iovValues[KEY_IOV]["Vehicle"]["GaussX"] = iovData.vehicleGaussX;
    iovValues[KEY_IOV]["Vehicle"]["GaussY"] = iovData.vehicleGaussY;
    iovValues[KEY_IOV]["Vehicle"]["Speed"] = iovData.vehicleSpeed;

  }

//  {
//    values[KEY_IOV_E30]["IsAlive"] = true;
//    values[KEY_IOV_E30]["IsValid"] = true;
//
//    values[KEY_IOV_E30]["Vehicle"]["Valid"] = true;
//    values[KEY_IOV_E30]["Vehicle"]["Id"] = 1;
//    values[KEY_IOV_E30]["Vehicle"]["Type"] = 1;
//    values[KEY_IOV_E30]["Vehicle"]["GaussX"] = postureData.imuData.gaussX;
//    values[KEY_IOV_E30]["Vehicle"]["GaussY"] = postureData.imuData.gaussY;
//    values[KEY_IOV_E30]["Vehicle"]["Speed"] = postureData.imuData.velocity;
//
//    values[KEY_IOV_E30]["Light"]["Valid"] = iovData.b_lightValid;
//    values[KEY_IOV_E30]["Light"]["Time"] = iovData.lightTime;
//    values[KEY_IOV_E30]["Light"]["Phase"] = iovData.lightPhase;
//    values[KEY_IOV_E30]["Light"]["AdvisedSpeed"] = iovData.lightAdvisedSpeed;
//  }



//  values[KEY_IOV]["IsValid"] = true;
//  values[KEY_IOV]["IsAlive"] = true;
//
//  if (1 == iovData_sub.b_isValid && iovData_sub.b_isAlive == 1)
//  {
//		values[KEY_IOV]["Phase"] = iovData_sub.phase;
//    values[KEY_IOV]["Time"] = iovData_sub.time;
//    values[KEY_IOV]["Gw"]["Phase"] = iovData_sub.phase == 0? 1: 0;
//    values[KEY_IOV]["Gw"]["Time"] = iovData_sub.time;
//
//  }
//  else if (1 == visionData.trafficLightData.b_straightIsValid)
//  {
//	  if (1 == visionData.trafficLightData.straightPassable)
//	  {
//		  values[KEY_IOV]["Phase"] = 0;
//		  values[KEY_IOV]["Gw"]["Phase"] = 1;
//      values[KEY_IOV]["Time"] = 0;
//      values[KEY_IOV]["Gw"]["Time"] = 0;
//	  }
//	  else
//	  {
//		  values[KEY_IOV]["Phase"] = 1;
//		  values[KEY_IOV]["Gw"]["Phase"] = 0;
//      values[KEY_IOV]["Time"] = 99;
//      values[KEY_IOV]["Gw"]["Time"] = 99;
//	  }
//  }
//  else
//  {
//	  values[KEY_IOV]["Phase"] = 0;
//	  values[KEY_IOV]["Gw"]["Phase"] = 1;
//    values[KEY_IOV]["Time"] = 0;
//    values[KEY_IOV]["Gw"]["Time"] = 0;
//  }



//  std::cout <<"light phase: " <<values[KEY_IOV]["Phase"].asInt()<< std::endl;
//  std::cout <<"light time: " <<values[KEY_IOV]["Time"].asInt()<< std::endl;

  if(0 == radarData.blockedAreaIndex.size() || false == radarData.b_isAlive){
//    std::cout << "nothing blocked." << std::endl;
    values["Radar"]["IsAlive"] = radarData.b_isAlive;
  }
  else{
    values["Radar"]["IsAlive"] = radarData.b_isAlive;
    values["Radar"]["IsValid"] = radarData.b_isValid;
//    std::cout << "to send lidar data" << std::endl;
    for (uint32_t i = 0; i < radarData.blockedAreaIndex.size(); i++){
      std::stringstream ss;
      ss << i;
      values["Radar"]["Data"][ss.str()] = radarData.blockedAreaIndex[i];
    }
    values["Radar"]["Zone"]["XLeft"] = radarData.zone.xLeft;
    values["Radar"]["Zone"]["XRight"] = radarData.zone.xRight;
    values["Radar"]["Zone"]["AngleLeft"] = radarData.zone.angleLeft;
    values["Radar"]["Zone"]["AngleRight"] = radarData.zone.angleRight;
    values["Radar"]["Zone"]["YTop"] = radarData.zone.yTop;
    values["Radar"]["Zone"]["IsValid"] = radarData.zone.b_isValid;
    values["Radar"]["Zone"]["LeftCandidate"] = radarData.zone.b_leftCandidate;
    values["Radar"]["Zone"]["RightCandidate"] = radarData.zone.b_rightCandidate;

    values["Radar"]["Zone"]["PathAngle"] = (radarData.zone.angleLeft + radarData.zone.angleRight)/2;
//    std::cout << "lidar sending done" << std::endl;
//    values["Lidar"] = lidarData.blockedAreaIndex[0];
//    values["Alive"] = heartBeat[0];
  }
  mutex.unlock();
  //time
//  boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
//  duration = now - startTime;
//  std::stringstream ss;
//  ss << duration.total_milliseconds();
//  values[KEY_TIME] = postureData.dataTime;
//  values[KEY_TIME] = ss.str();
  //lidar
  Json::FastWriter fastWriter;
  std::string jsonString = fastWriter.write(values);
  //jsonString.erase (std::remove(jsonString.begin(), jsonString.end(), '\n'), jsonString.end());
  zmq_send(sSocketList[0], jsonString.c_str(), strlen(jsonString.c_str()), 0);
  // print json
//  std::cout << "Sent: " << jsonString << std::endl;
//  std::cout << "Length: " << jsonString.length() << std::endl;
//  sendingCounter++;
//  std::cout << "sending counter: " << sendingCounter <<std::endl;


  //iov stuff
  Json::FastWriter iovFastWriter;
  std::string iovJsonString = iovFastWriter.write(iovValues);
  zmq_send(sSocketList[1], iovJsonString.c_str(), strlen(iovJsonString.c_str()), 0);

//  std::cout << "Sent iov: " << iovJsonString << std::endl;

  timer3.expires_at(timer3.expires_at() + boost::posix_time::milliseconds(DEFAULT_TIMER3_PERIOD));
  timer3.async_wait(strand3.wrap(boost::bind(&Jobs::publisher, this)));
}

int main(){
  startTime = boost::posix_time::microsec_clock::local_time();

  void* context = zmq_ctx_new ();

// zmq receiving sockets
  void* lidarSocket = zmq_socket (context, ZMQ_SUB);
  void* visionSocket = zmq_socket (context, ZMQ_SUB);
  void* gpsSocket = zmq_socket (context, ZMQ_SUB);
  void* imuSocket = zmq_socket (context, ZMQ_SUB);
  void* actuatorSocket = zmq_socket (context, ZMQ_SUB);
  void* iovSocket = zmq_socket (context, ZMQ_SUB);
  void* uiSocket = zmq_socket (context, ZMQ_SUB);
  void* decisionSocket = zmq_socket (context, ZMQ_SUB);
  void* radarSocket = zmq_socket (context, ZMQ_SUB);
  void* cvSocket = zmq_socket (context, ZMQ_SUB);


  zmq_connect (lidarSocket, "tcp://127.0.0.1:5555");
  zmq_setsockopt (lidarSocket, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect (visionSocket, "tcp://192.168.1.32:5556");
  zmq_setsockopt (visionSocket, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect (gpsSocket, "tcp://127.0.0.1:5557");
  zmq_setsockopt (gpsSocket, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect(imuSocket, "tcp://127.0.0.1:5558");
  zmq_setsockopt (imuSocket, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect (actuatorSocket, "tcp://127.0.0.1:6974");
  zmq_setsockopt (actuatorSocket, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect (uiSocket, "tcp://192.168.1.45:8484");
  zmq_setsockopt (uiSocket, ZMQ_SUBSCRIBE, "", 0);
//  zmq_connect (decisionSocket, "tcp://192.168.1.217:6970");
  zmq_connect (decisionSocket, "tcp://127.0.0.1:6970");
//  zmq_connect (decisionSocket, "tcp://127.0.0.1:6931");
  zmq_setsockopt (decisionSocket, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect (radarSocket, "tcp://192.168.1.33:5559");
  zmq_setsockopt (radarSocket, ZMQ_SUBSCRIBE, "", 0);
  //zmq_connect (iovSocket, "tcp://192.168.1.51:6980");
//  zmq_connect (iovSocket, "tcp://127.0.0.1:6666"); // for "imu" iov receive
  zmq_setsockopt (iovSocket, ZMQ_SUBSCRIBE, "", 0);
//  zmq_connect (cvSocket, "tcp://192.168.1.52:6991");
  zmq_connect (cvSocket, "tcp://127.0.0.1:6666");
  zmq_setsockopt (cvSocket, ZMQ_SUBSCRIBE, "", 0);

  std::vector<void*> receivingSocketList;
  receivingSocketList.push_back(lidarSocket);     //0
  receivingSocketList.push_back(visionSocket);    //1
  receivingSocketList.push_back(gpsSocket);       //2
  receivingSocketList.push_back(actuatorSocket);  //3
  receivingSocketList.push_back(uiSocket);        //4
  receivingSocketList.push_back(decisionSocket);  //5
  receivingSocketList.push_back(radarSocket);     //6
  receivingSocketList.push_back(iovSocket);       //7 for vehicle
  receivingSocketList.push_back(imuSocket);       //8
  receivingSocketList.push_back(cvSocket);        //9 for light

// zmq sending sockets

  void* publisherSocket = zmq_socket (context, ZMQ_PUB);
  zmq_bind(publisherSocket, "tcp://*:6931");

  void* iovPublisherSocket = zmq_socket (context, ZMQ_PUB);
  zmq_bind(iovPublisherSocket, "tcp://*:6932");

  std::vector<void*> sendingSocketList;
  sendingSocketList.push_back(publisherSocket);
  sendingSocketList.push_back(iovPublisherSocket);

// thread initial
  boost::asio::io_service io;
  Jobs jobs(io, receivingSocketList, sendingSocketList);
  boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
  io.run();
  t.join();

  return 0;
}

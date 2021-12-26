#include "decision.hpp"
#include "functions.hpp"
#include "qingColor.h"

#define ROADMAP_FILE_NAME "/home/nvidia/Software/autopliot/roadMap.txt"
#define STOP_POINT_FILE_NAME "/home/nvidia/Software/autopliot/stopPoint.txt"
#define TERMINAL_POINT_FILE_NAME "/home/nvidia/Software/autopliot/terminalPoint.txt"
#define PATH_NUMBER_FILE_NAME "/home/nvidia/Software/autopliot/pathNumber.txt"


#define NO_UI_MODE true 

// Keys
#define KEY_LIDAR "Lidar"
#define KEY_VISION "Vision"
#define KEY_VISION_LANE "LaneModel"
#define KEY_VISION_TRAFFIC_LIGHT "TrafficLight"
#define KEY_VISION_TRAFFIC_SIGN "TrafficSign"
#define KEY_POSTURE "Posture"
#define KEY_POSTURE_GPS "Gps"
#define KEY_POSTURE_IMU "Imu"
#define KEY_ACTUATOR "Actuator"
#define KEY_START "StartSignal"
#define KEY_UI "Ui"
#define KEY_DECISION "Decision"
#define KEY_IOV "Iov"
#define KEY_CV "Cv"

#define DEFAULT_TIMER1_START 50
#define DEFAULT_TIMER2_START 100
#define DEFAULT_TIMER3_START 150
#define DEFAULT_TIMER4_START 200

#define TIMER1_PERIOD 40
#define TIMER2_PERIOD 100
#define TIMER3_PERIOD 100
#define TIMER4_PERIOD 100


// obstacle distance (m)
#define OBSTACLE_DIS_MAX 20
#define OBSTACLE_DIS_MID 10
#define OBSTACLE_DIS_MIN 5

// speed definition (m/s)  change to wbd, which max speed is 2.7m/s
#define SPEED_ULTRA 2.77
#define SPEED_HIGH 2.7
#define SPEED_MID 2
#define SPEED_LOW 1
#define SPEED_ZERO 0
#define SPEED_TEST 1
#define SPEED_ACC_MAX 10/3.6//8.5

// acc definitions
#define ACC_ULTRA 4
#define ACC_HIGH 3 
#define ACC_MID 2
#define ACC_LOW 1

#define FOLLOW_ROAD_MAP_FLAG 1
#define FOLLOW_VISION_AND_LIDAR_FLAG 1
#define FOLLOW_VISION_FLAG 1
#define FOLLOW_LIDAR_FLAG 1

#define POSTURE_SENSOR_FLAG 2 // 0: both, 1: IMU, 2: GPS

#define LIDAR_INSTALLED_FLAG true 


#define OVERTAKE_DIST_THR 40.0

#define DEAD_ZONE 2


Jobs::Jobs(boost::asio::io_service& io, std::vector<void*>& receivingSocketList, std::vector<void*>& sendingSocketList, std::vector<std::vector<RoadPoint>>& rPointss, std::vector<RoadPoint>& sPoints, std::vector<std::vector<uint32_t>>& pathNLists) : strand1(io), strand2(io), strand3(io), strand4(io), timer1(io, boost::posix_time::milliseconds(DEFAULT_TIMER1_START)), timer2(io, boost::posix_time::milliseconds(DEFAULT_TIMER2_START)), timer3(io, boost::posix_time::milliseconds(DEFAULT_TIMER3_START)), timer4(io, boost::posix_time::milliseconds(DEFAULT_TIMER4_START)),  rSocketList(receivingSocketList), sSocketList(sendingSocketList), roadPointss(rPointss), stopPoints(sPoints), pathNumberLists(pathNLists){
  decisionData.postureSensorMode = POSTURE_SENSOR_FLAG;
  timer1.async_wait(strand1.wrap(boost::bind(&Jobs::subscriber, this)));
  timer2.async_wait(strand2.wrap(boost::bind(&Jobs::handler, this)));
  timer3.async_wait(strand3.wrap(boost::bind(&Jobs::publisher, this)));
  timer4.async_wait(strand4.wrap(boost::bind(&Jobs::processor, this)));
}

Jobs::~Jobs(){
  ;
}

void Jobs::subscriber(){
//  std::cout << "Subscribing ..." << std::endl;

  char msg[204800] = {0};

//  zmq_pollitem_t items [2];
  zmq_pollitem_t items [1];
  items[0].socket = rSocketList[0];
  items[0].events = ZMQ_POLLIN;
//  items[1].socket = rSocketList[1];
//  items[1].events = ZMQ_POLLIN;

//  std::cout << "Polling Information ..." << std::endl;
  
//  zmq_poll (items, 2, 0);
  zmq_poll (items, 1, 0);

  if (items [0].revents & ZMQ_POLLIN) {
    int size = zmq_recv (rSocketList[0], msg, 204800, 0);
    if (size != -1) {
      // Lidar
//      std::cout << "Got Data Fusion Information." << std::endl;

      std::stringstream ss;
      ss << msg;

//      std::cout << "Got json string: " << ss.str() << std::endl;
      Json::Value values;
      Json::Reader reader;
      if(true == reader.parse(ss.str(), values)){
      	//std::cout<< ss.str()<<std::endl;
        // Lidar
//        std::cout << "lidar Alive: " << lidarData.b_isAlive << std::endl;
        lidarData.b_isAlive = values[KEY_LIDAR]["IsAlive"].asInt();

        if (true == lidarData.b_isAlive){
          lidarData.b_isValid = values[KEY_LIDAR]["IsValid"].asInt();
          lidarData.blockedAreaIndex.clear();
          lidarData.zone.pathAngle = values[KEY_LIDAR]["Zone"]["PathAngle"].asDouble();
          lidarData.zone.xLeft = values[KEY_LIDAR]["Zone"]["XLeft"].asDouble();
          lidarData.zone.xRight = values[KEY_LIDAR]["Zone"]["XRight"].asDouble();
          lidarData.zone.yTop = values[KEY_LIDAR]["Zone"]["YTop"].asDouble();
          lidarData.zone.yTopWider = values[KEY_LIDAR]["Zone"]["YTopWider"].asDouble();
          lidarData.zone.yTopEvenWider = values[KEY_LIDAR]["Zone"]["YTopEvenWider"].asDouble();
          lidarData.zone.angleLeft = values[KEY_LIDAR]["Zone"]["AngleLeft"].asDouble();
          lidarData.zone.angleRight = values[KEY_LIDAR]["Zone"]["AngleRight"].asDouble();
          lidarData.zone.b_isValid = values[KEY_LIDAR]["Zone"]["IsValid"].asInt();
          lidarData.zone.b_leftCandidate = values[KEY_LIDAR]["Zone"]["LeftCandidate"].asInt();
          lidarData.zone.b_rightCandidate = values[KEY_LIDAR]["Zone"]["RightCandidate"].asInt();
          
          Json::Value::Members members (values[KEY_LIDAR]["Data"].getMemberNames());
          if(members.begin() != members.end() - 1){
            for (Json::Value::Members::iterator mit = members.begin(); mit != members.end(); mit++){
              lidarData.blockedAreaIndex.push_back(values[KEY_LIDAR]["Data"][*mit].asInt());
              //      std::cout << "This is the data of \"Lidar\": " << values["Lidar"]["Data"][*mit].asInt() << std::endl;
            }
            std::ofstream fs("lidarData.txt", std::ofstream::trunc);
            for (uint32_t i = 0; i < lidarData.blockedAreaIndex.size(); i++) {
              fs << std::setprecision(15) << lidarData.blockedAreaIndex[i]<< std::endl;
            }
            fs.close();
  
          }
          else{
            std::cout << RED << "Lidar ERROR!" << RESET << std::endl;
          }

        }
        else;
        

        
        // vision
        visionData.b_isAlive = values[KEY_VISION]["IsAlive"].asInt();
        if (true == visionData.b_isAlive){
          visionData.laneData.laneCount = values[KEY_VISION][KEY_VISION_LANE]["LaneCount"].asInt();
          visionData.laneData.laneStatus[0] = values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["RR"].asInt();
          visionData.laneData.laneStatus[1] = values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["R"].asInt();
          visionData.laneData.laneStatus[2] = values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["L"].asInt();
          visionData.laneData.laneStatus[3] = values[KEY_VISION][KEY_VISION_LANE]["LaneStatus"]["LL"].asInt();
          visionData.laneData.offsetRight = values[KEY_VISION][KEY_VISION_LANE]["OffsetRight"].asInt()/1000.0;
          visionData.laneData.offsetLeft = values[KEY_VISION][KEY_VISION_LANE]["OffsetLeft"].asInt()/1000.0;
          visionData.laneData.angleError = values[KEY_VISION][KEY_VISION_LANE]["AngleError"].asInt()/10.0;
          
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
          
        }
        else;
        
        // posture
        postureData.b_isAlive = values[KEY_POSTURE]["IsAlive"].asInt();
      //std::cout << "Posture data status: " << postureData.b_isAlive << std::endl;
        if (true == postureData.b_isAlive) {
          postureData.imuData.b_isValid = values[KEY_POSTURE][KEY_POSTURE_IMU]["IsValid"].asInt();
          postureData.gpsData.b_isValid = values[KEY_POSTURE][KEY_POSTURE_GPS]["IsValid"].asInt();
          
          postureData.imuData.velocity = values[KEY_POSTURE][KEY_POSTURE_IMU]["Velocity"].asDouble();
          postureData.imuData.roll = values[KEY_POSTURE][KEY_POSTURE_IMU]["Roll"].asDouble();
          postureData.imuData.pitch = values[KEY_POSTURE][KEY_POSTURE_IMU]["Pitch"].asDouble();
std::cout<<"using gps"<<values[KEY_POSTURE][KEY_POSTURE_GPS]["GaussX"].asDouble()<<std::endl;
          if (0 == decisionData.postureSensorMode || 1 == decisionData.postureSensorMode) {
            postureData.imuData.yaw = values[KEY_POSTURE][KEY_POSTURE_IMU]["Yaw"].asDouble();
          }
          else if(2 == decisionData.postureSensorMode){
            postureData.imuData.yaw = values[KEY_POSTURE][KEY_POSTURE_GPS]["Yaw"].asDouble();

std::cout<<"using gps"<<values[KEY_POSTURE][KEY_POSTURE_GPS]["Yaw"].asDouble()<<std::endl;
          }
          else{
            postureData.imuData.yaw = values[KEY_POSTURE][KEY_POSTURE_IMU]["Yaw"].asDouble();
          }
          postureData.imuData.b_gpsValid = values[KEY_POSTURE][KEY_POSTURE_IMU]["GpsValid"].asInt();
//          if (true == postureData.imuData.b_gpsValid){
//            postureData.gpsData.longitude = values[KEY_POSTURE][KEY_POSTURE_IMU]["GaussX"].asDouble();
//            postureData.gpsData.latitude = values[KEY_POSTURE][KEY_POSTURE_IMU]["GaussY"].asDouble();
//            postureData.gpsData.b_isDifferential = true;
//          }

          if (0.05 > postureData.imuData.velocity) {
            postureData.imuData.velocity = 0.05;
          }
          else;
          
//          if (true == postureData.imuData.b_gpsValid) {
        printf("current sensor mode %d \n",decisionData.postureSensorMode)  ;
if (0 == decisionData.postureSensorMode || 2 == decisionData.postureSensorMode) {
            postureData.gpsData.longitude = values[KEY_POSTURE][KEY_POSTURE_GPS]["GaussX"].asDouble();
            postureData.gpsData.latitude = values[KEY_POSTURE][KEY_POSTURE_GPS]["GaussY"].asDouble();
          }
          else if (1 == decisionData.postureSensorMode){
            postureData.gpsData.longitude = values[KEY_POSTURE][KEY_POSTURE_IMU]["GaussX"].asDouble();
            postureData.gpsData.latitude = values[KEY_POSTURE][KEY_POSTURE_IMU]["GaussY"].asDouble();
          }
          else{
            postureData.gpsData.longitude = values[KEY_POSTURE][KEY_POSTURE_GPS]["GaussX"].asDouble();
            postureData.gpsData.latitude = values[KEY_POSTURE][KEY_POSTURE_GPS]["GaussY"].asDouble();
          }
          
          postureData.gpsData.b_isDifferential = values[KEY_POSTURE][KEY_POSTURE_GPS]["IsDifferential"].asInt();
//          std::cout<<"!!!!!!!! posture  "<<postureData.gpsData.longitude<<"  "<<postureData.gpsData.latitude<<"  "<<postureData.imuData.yaw<<std::endl;
//          }
//          else;
          
          //          std::cout << "Longitude: " << postureData.gpsData.longitude << ", Latitude: " << postureData.gpsData.latitude << std::endl;
          
          //          if (true == postureData.b_isAlive && true == postureData.gpsData.b_isDifferential){
          //            gaussConvert(postureData.gpsData.longitude, postureData.gpsData.latitude, postureData.gpsData.longitude, postureData.gpsData.latitude);
          //          }
          //          else;
          
        }
        else;

        actuatorData.b_isAlive = values[KEY_ACTUATOR]["IsAlive"].asInt();
        if (true == actuatorData.b_isAlive) {
        	actuatorData.vehicleSpeed = values[KEY_ACTUATOR]["VehicleSpeed"].asDouble();
        }
        
        uiData.b_isAlive = values[KEY_UI]["IsAlive"].asInt();
        if (true == uiData.b_isAlive) {
          uiData.b_isValid = values[KEY_UI]["IsValid"].asInt();
          uiData.b_stopFlag = values[KEY_UI]["StopFlag"].asInt();
          uiData.b_speedOff = values[KEY_UI]["SpeedOff"].asInt();
          uiData.b_steeringOff = values[KEY_UI]["SteeringOff"].asInt();
          uiData.b_config1 = values[KEY_UI]["Config1"].asInt();
//          uiData.pathNumber = values[KEY_UI]["PathNumber"].asInt();
          uiData.pathNumber = values[KEY_UI]["PathNumber"].asInt();
//          uiData.b_smallCircleSelected = values[KEY_UI]["SmallCircleSelected"].asInt();
//          uiData.lastId = values[KEY_UI]["LastId"].asInt();
//          uiData.nextId = values[KEY_UI]["NextId"].asInt();
        }
        else;
//        uiData.b_isValid = true;
//        uiData.b_stopFlag = false;
        
        //iov Old
//        iovData.b_isAlive = values[KEY_IOV]["IsAlive"].asInt();
//        if (true == iovData.b_isAlive) {
//          iovData.b_isValid = values[KEY_IOV]["IsValid"].asInt();
//          iovData.phase = values[KEY_IOV]["Phase"].asInt();
//          iovData.time  = values[KEY_IOV]["Time"].asInt();
//          iovData.advSpd = values[KEY_IOV]["AdvSpd"].asDouble();
//        }
//        else;
        
//        std::cout << "Traffic Light: " << iovData.phase << ", Time Left: " << iovData.time << std::endl;;
        
        iovData.b_isAlive = values[KEY_IOV]["IsAlive"].asInt();
        if (true == iovData.b_isAlive){
        	iovData.b_isValid = values[KEY_IOV]["IsValid"].asInt();
  				iovData.vehicleId = values[KEY_IOV]["Vehicle"]["VehicleID"].asInt();
				  iovData.vehicleType = values[KEY_IOV]["Vehicle"]["vehicleType"].asInt();

  				iovData.vehicleGaussX = values[KEY_IOV]["Vehicle"]["GaussX"].asDouble();
  				iovData.vehicleGaussY = values[KEY_IOV]["Vehicle"]["GaussY"].asDouble();
  				iovData.vehicleSpeed = values[KEY_IOV]["Vehicle"]["Speed"].asDouble();
  				iovData.vehicleTime = values[KEY_IOV]["Vehicle"]["Time"].asDouble();
  
  				iovData.lightPhase = values[KEY_IOV]["Light"]["Phase"].asInt();
  				iovData.lightTime = values[KEY_IOV]["Light"]["Time"].asDouble();
  				iovData.lightAdvisedSpeed = values[KEY_IOV]["Light"]["AdvisedSpeed"].asDouble();
        }
        
        cvData.b_isAlive = values[KEY_CV]["IsAlive"].asInt();
        if (true == cvData.b_isAlive){
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
        }
        
        // Start
        startSignal.b_isValid = values[KEY_START]["IsValid"].asInt();
        startSignal.data = values[KEY_START]["Data"].asInt();
        
//        std::cout << "Got the data of \"DataFusion\": " << std::endl;
        
        Json::FastWriter fastWriter;
        std::string jsonString = fastWriter.write(values);
//        std::cout << "Values: " << jsonString << std::endl;
      }
      else{
        std::cout << "Parse error  :(" << std::endl;
      }
    }
  }
  
//  if (items [1].revents & ZMQ_POLLIN) {
//    int size = zmq_recv (rSocketList[1], msg, 204800, 0);
//    if (size != -1) {
//        // vision
//      std::cout << "Got Vision Information." << std::endl;
//    }
//  }
  
  
//      while(1){sleep(1);}

  timer1.expires_at(timer1.expires_at() + boost::posix_time::milliseconds(TIMER1_PERIOD));
  timer1.async_wait(strand1.wrap(boost::bind(&Jobs::subscriber, this)));
}

void Jobs::handler(){
//  std::cout << "Handling ..." << std::endl;
  

  timer2.expires_at(timer2.expires_at() + boost::posix_time::milliseconds(TIMER2_PERIOD));
  timer2.async_wait(strand2.wrap(boost::bind(&Jobs::handler, this)));
}

void Jobs::publisher(){
//  std::cout << "Broadcasting ..." << std::endl;

  Json::Value values;

  values[KEY_DECISION]["PostureSensorMode"] = decisionData.postureSensorMode;
  values[KEY_DECISION]["TargetSteeringAngleOut"] = decisionData.targetSteeringAngle;
  values[KEY_DECISION]["TargetSpeedOut"] = decisionData.targetSpeed;
  values[KEY_DECISION]["TargetAccReq"]=decisionData.targetAccReq;//new output
  values[KEY_DECISION]["PathNumber"] = decisionData.pathNumber;
 //  if (uiData.b_stopFlag == false) {
//    values[KEY_DECISION]["TargetSpeedOut"] = decisionData.targetSpeed;
////    std::cout << "Running~" << std::endl;
//  }
//  else{
//    values[KEY_DECISION]["TargetSpeedOut"] = 0;
////    std::cout << "STOP!" << std::endl;
//  }
  
  if (uiData.b_speedOff == true && uiData.b_steeringOff == true) {
    decisionData.targetWorkMode = 0;
  }
  else if (uiData.b_speedOff == false && uiData.b_steeringOff == true){
    decisionData.targetWorkMode = 3;
  }
  else if (uiData.b_speedOff == false && uiData.b_steeringOff == false){
    decisionData.targetWorkMode = 1;
  }
  else if (uiData.b_speedOff == true && uiData.b_steeringOff == false){
    decisionData.targetWorkMode = 2;
  }
  else{
    decisionData.targetWorkMode = 0;
  }
  if (NO_UI_MODE == true) decisionData.targetWorkMode = 1;
  
  if (iovData.b_isAlive == false){
//    std:: cout << RED << "No IOV, NO control!\n"<< RESET;
//    decisionData.targetWorkMode = 0;
  }
  values[KEY_DECISION]["TargetWorkMode"] = decisionData.targetWorkMode;
  
//  values[KEY_DECISION]["GearPosition"] = decisionData.targetGearPosition;
//  values[KEY_DECISION]["AccLevel"] = decisionData.targetAccLevel;
//  values[KEY_DECISION]["AccLevel"] = decisionData.targetAccLevel;
  
  values[KEY_DECISION]["CurrentState"] = decisionData.currentState;
  values[KEY_DECISION]["CurrentId"] = decisionData.currentId;
  values[KEY_DECISION]["CurrentIndex"] = decisionData.currentIndex;
  values[KEY_DECISION]["IsValid"] = true;
  values[KEY_DECISION]["PostureSensorMode"] = decisionData.postureSensorMode;

  values[KEY_DECISION]["CurrentTotalIndex"] = decisionData.currentTotalIndex;
  values[KEY_DECISION]["CurrentPreviewTotalIndex"] = decisionData.previewPoint.totalIndex;
  values[KEY_DECISION]["RedLightStopped"] = decisionData.b_redLightStopped;
  values[KEY_DECISION]["TakingOver"] = decisionData.b_takingOver;


  Json::FastWriter fastWriter;
  std::string jsonString = fastWriter.write(values);
  zmq_send(sSocketList[0], jsonString.c_str(), strlen(jsonString.c_str()), 0);
//  std::cout << "Sent: " << jsonString << std::endl;

  timer3.expires_at(timer3.expires_at() + boost::posix_time::milliseconds(TIMER3_PERIOD));
  timer3.async_wait(strand3.wrap(boost::bind(&Jobs::publisher, this)));
}



void Jobs::processor(){
//  std::cout << "Processing ..." << std::endl;
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<std::endl;
  decisionData.currentState = STATE_INITIAL;
  // make sure vehicle stops if no dgps nor lidar nor...
//  decisionData.targetSpeed = SPEED_ZERO;
  
//  STATE MACHINE
//  decisionData.currentState = 1;
//  if (0 == decisionData.currentState) {
//    if (1 == startSignal.b_isValid && 1 == startSignal.data){
//      decisionData.currentState = 1;
//    }
//    else;
//  }
//  else if ( 1 == decisionData.currentState)
  
  pathSelection(decisionData, uiData, pathNumberLists.size()-1);
// DGPS
//  std::cout << "here !! " << postureData.b_isAlive << postureData.gpsData.b_isValid << postureData.gpsData.b_isDifferential << lidarData.b_isAlive << lidarData.b_isValid << std::endl;
  if (1 == postureData.b_isAlive &&
//      1 == postureData.gpsData.b_isValid &&
//      1 == postureData.gpsData.b_isDifferential &&
//      1 == lidarData.b_isAlive &&
//      1 == lidarData.b_isValid &&
      1 == FOLLOW_ROAD_MAP_FLAG){
      // tbd follow path

    
    // steering angle
    followRoadMap(postureData.gpsData.longitude, postureData.gpsData.latitude, postureData.imuData.yaw, actuatorData.vehicleSpeed, roadPointss, decisionData, pathNumberLists);
   std::cout <<"posture yaw::"<< postureData.imuData.yaw << std::endl;
    
    //      decisionData.targetSteeringAngle = followRoadMap(postureData.gpsData.longitude, postureData.gpsData.latitude, postureData.imuData.yaw, postureData.imuData.velocity, roadPointss);

    // end of steering angle
    
    
    // velocity
//    velocityDecision(lidarData, uiData.b_stopFlag, postureData, decisionData);
    if (decisionData.currentState == STATE_FOLLOW_ROADMAP) {
      std::cout << GREEN << "following gps" << RESET << std::endl;
    }
    else {
printf("current sensor mode %d \n",decisionData.postureSensorMode)  ;
      std::cout << RED << "no road point!" << RESET << std::endl;
    }
    
//    if (lidarData.zone.yTop > OBSTACLE_DIS_MAX){
//      decisionData.targetSpeed = SPEED_HIGH;
//      decisionData.targetAccLevel = ACC_MID;
//    }
//    else if ( lidarData.zone.yTop > OBSTACLE_DIS_MID && lidarData.zone.yTop < OBSTACLE_DIS_MAX){
//      decisionData.targetSpeed = SPEED_MID;
//      decisionData.targetAccLevel = ACC_MID;
//    }
//    else if ( lidarData.zone.yTop > OBSTACLE_DIS_MIN && lidarData.zone.yTop < OBSTACLE_DIS_MID){
//      decisionData.targetSpeed = SPEED_LOW;
//      decisionData.targetAccLevel = ACC_MID;
//    }
//    else {
//      decisionData.targetSpeed = SPEED_ZERO;
//      decisionData.targetAccLevel = ACC_MID;
//    }
//    // tbd
////      decisionData.targetSpeed = SPEED_TEST;
//  
////    IOV stuff
////    if (true == iovData.b_isValid) {
////      if (decisionData.targetSpeed > iovData.advSpd) {
////        decisionData.targetSpeed = iovData.advSpd;
////        std::cout << "red light." << std::endl;
////      }
////      else;
////    }
////    else;
////    end of IOV
//    
//    if (true == uiData.b_stopFlag) {
//      decisionData.targetSpeed = SPEED_ZERO;
//      decisionData.targetAccLevel = ACC_MID;
//      std::cout << "stopped. " << std::endl;
//    }
//    else{
//      std::cout << "driving. " << std::endl;
//    }
    

    // End of Velocity
      

  }
  else;
  
  /*
  // Vision + Lidar
  
  if (STATE_INITIAL == decisionData.currentState &&
      1 == visionData.b_isAlive &&
           (0 != visionData.laneData.laneStatus[1] ||
           0 != visionData.laneData.laneStatus[2] )&&
           1 == lidarData.b_isAlive &&
           1 == lidarData.b_isValid &&
           true == FOLLOW_VISION_AND_LIDAR_FLAG
           ){
    followLidarAndLane(lidarData, visionData, postureData, decisionData);
//    velocityDecision (lidarData, uiData.b_stopFlag, postureData, decisionData);
  }
  
  // Vision
  else if (STATE_INITIAL == decisionData.currentState &&
           1 == visionData.b_isAlive &&
           (0 != visionData.laneData.laneStatus[1] ||
           0 != visionData.laneData.laneStatus[2] )&&
           1 == FOLLOW_VISION_FLAG){
    std::cout << "following vision" << std::endl;
//    followLane(postureData.imuData.yaw, postureData.imuData.velocity, lidarData.zone.pathAngle, decisionData);
    followLane(visionData, postureData, decisionData);
//    velocityDecision (lidarData, uiData.b_stopFlag, postureData, decisionData);
    
    
  }
  
  // Lidar
  else if (STATE_INITIAL == decisionData.currentState &&
           1 == lidarData.b_isAlive &&
           1 == lidarData.b_isValid &&
           1 == FOLLOW_LIDAR_FLAG){
    std::cout << "following lidar" << std::endl;
    followLidar (lidarData.zone.pathAngle, postureData, decisionData);
  }

  else {
//    std::cout << "State Error!!" << std::endl;
  }
   */
  

  
  
  
  
// general velocity decision

  
//  velocityDecision (lidarData, uiData.b_stopFlag, postureData, decisionData);
  
//  if(decisionData.roadType == 1){
//    decisionData.targetSpeed = SPEED_HIGH;
//  }
//  else if (decisionData.roadType == 2){
//    decisionData.targetSpeed = SPEED_MID;
//  }
//  else if (decisionData.roadType == 8){
//    decisionData.targetSpeed = 2.0;
//  }
//  else {
//    decisionData.targetSpeed = SPEED_LOW;
//  }
  
//  if (actuatorData.wsuSceneStatus == 2 ) {
//    if (actuatorData.wsuAlarmLevelStatus == 1 || actuatorData.wsuAlarmLevelStatus == 2) {
//      decisionData.targetSpeed = SPEED_ZERO;
//    }
//    else;
//  }
//  else;
  
// target speed decision for intersection
  std::cout << "Speed After Follow: " <<  decisionData.targetSpeed << std::endl;
  trafficLightJudge (postureData, cvData, stopPoints, decisionData);
//  stopPointJudge(postureData, cvData, stopPoints, decisionData);
  std::cout << "Speed After Light : " << decisionData.targetSpeed << std::endl;
  mapJudge(decisionData, cvData, visionData);
  
  std::cout << "Speed After Judge : " << decisionData.targetSpeed << std::endl;
  //curve related
//  if ((decisionData.currentId <= 2 && decisionData.currentIndex <= 45) ||
//      (decisionData.currentId >= 8 && decisionData.currentIndex >= 40) ||
//      (decisionData.currentId >= 9) ||
//      (decisionData.currentId <= 1)){
//      generateCurvePreviewPoint(decisionData, postureData);
//      if (true == curvesLidarCheck(decisionData.curveList[0], lidarData.blockedAreaIndex)) {
//        decisionData.targetSpeed = SPEED_ZERO;
//      }else;
//  }
//  else;
  
  //curve work
//  bool b_takeOver = false;
//  // huan dao lu duan
//  if (decisionData.currentId == 14){
//    b_takeOver = true;
//  }
//  else{
//    b_takeOver = false;
//    decisionData.b_takingOver = false;
//  }


if(true == LIDAR_INSTALLED_FLAG && true == lidarData.b_isAlive){
  if(true == decisionData.b_onRoadPoint){
//    lidarJudge(decisionData, postureData, lidarData);
    lidarJudgeLongerPreview(decisionData, lidarDecisionData, postureData, lidarData, roadPointss);

    std::cout << "[LL] Target Speed:" << std::endl;
    std::cout << "To Preview Point: " << decisionData.targetSpeed << std::endl;
    std::cout << "To Longer Lidar Range: " << lidarDecisionData.targetSpeed << std::endl;

//    decisionData.targetSpeed = decisionData.targetSpeed < lidarDecisionData.targetSpeed? decisionData.targetSpeed: lidarDecisionData.targetSpeed;
  }
}
  
  
//  if (true == curvesLidarCheck(decisionData.curveList[0], lidarData.blockedAreaIndex)) {
//    decisionData.targetSpeed = SPEED_ZERO;
//    std::cout << "Obstacle Detected!" << std::endl;
//  }
//  else{
//      std::cout << "No Obstacles!" << std::endl;
//  };
  
  clearCurveList(decisionData.curveList);
  
  
// terminal point
//  static uint32_t roadPointssSize = static_cast<uint32_t>(roadPointss.size()) - 1;
//  static uint32_t lastRoadPointsSize = static_cast<uint32_t>(roadPointss[roadPointssSize].size()) - 1;
//  stopPointJudge (postureData, roadPointss[roadPointssSize][lastRoadPointsSize], decisionData);
 
  //ACC

  if (true == iovData.b_isAlive && uiData.b_config1 == false){
//    decisionData.targetSpeed = 20.0;
    double accSpeed = cruiseController(postureData, iovData, actuatorData);
    std::cout << "ACC Target Speed: " << accSpeed << std::endl;
    if (accSpeed > SPEED_ACC_MAX){
      accSpeed = SPEED_ACC_MAX;
      std::cout << "! ACC Speed Limited" << std::endl;
    }
    else;
//    decisionData.targetSpeed = MIN(decisionData.targetSpeed, accSpeed);
    decisionData.targetSpeed = accSpeed;    
  }else;
  
  //ui
  static double lastLongitude = 0;
  static double lastLatitude = 0;
  if (uiData.b_stopFlag == false || NO_UI_MODE == true) {
    lastLongitude = postureData.gpsData.longitude;
    lastLatitude = postureData.gpsData.latitude;
  }
  else{
    decisionData.targetSpeed = 0.0;
    double errorX = lastLongitude - postureData.gpsData.longitude;
    double errorY = lastLatitude - postureData.gpsData.latitude;
    //decisionData.targetAccLevel = ACC_UISTOP;
    std::cout << "Last Posture: " << errorX << ", " << errorY << std::endl;
    std::cout << "Stopped because of UI." << std::endl;
  }
 
//  if (uiData.b_stopFlag == false){
//    ;
//  }
//  else {
//    decisionData.targetSpeed = 0.0;
//  }

  //decisionData.targetAccLevel = ACC_HIGH;          // 20160604pm added;  


//  if (decisionData.targetSpeed > ALMOST_ZERO) {
//
//    std::cout << "targetSpeed: " << decisionData.targetSpeed << std::endl;
//  }
//  else;
  
 if (decisionData.targetSpeed< DEAD_ZONE) 
  {decisionData.targetSpeed=0;}

else if(decisionData.targetSpeed <5 )
  {decisionData.targetSpeed=150;}
else 
  {decisionData.targetSpeed=250;}
 std::cout << "Speed control out: " << decisionData.targetSpeed << std::endl;
  //this part is to transform target speed into targetaccrequest
//double speed_error = 0;
//speed_error=decisionData.targetSpeed-actuatorData.velocity;
//if (decisionData.targetSpeed >DEAD_ZONE )
//{decisionData.targetAccReq=200;}
//else
//{decisionData.targetAccReq=0;}

 // std::cout << "Speed Out: " << decisionData.targetSpeed << std::endl;
 // std::cout << "Targt Acc: " << decisionData.targetAccLevel << std::endl;
  //std::cout <<"Target Acc Req" <<decisionData.targetAccReq <<std::endl;


  timer4.expires_at(timer4.expires_at() + boost::posix_time::milliseconds(TIMER4_PERIOD));
  timer4.async_wait(strand4.wrap(boost::bind(&Jobs::processor, this)));
}

int main(){

  
  void* context = zmq_ctx_new ();

// zmq receiving sockets
  void* fusionSocket = zmq_socket (context, ZMQ_SUB);
//  void* visionSocket = zmq_socket (context, ZMQ_SUB);
  
  zmq_connect (fusionSocket, "tcp://127.0.0.1:6931"); // ego-car

//  int rc = zmq_connect (fusionSocket, "ipc:///tmp/feeds/6");
  
//  zmq_connect (fusionSocket, "tcp://192.168.1.21:6969");
  zmq_setsockopt (fusionSocket, ZMQ_SUBSCRIBE, "", 0);
//  zmq_connect (visionSocket, "tcp://127.0.0.1:5556");
//  zmq_setsockopt (visionSocket, ZMQ_SUBSCRIBE, "", 0);

  
  std::vector<void*> receivingSocketList;
  receivingSocketList.push_back(fusionSocket);
//  receivingSocketList.push_back(visionSocket);

// zmq sending sockets

  void* publisherSocket = zmq_socket (context, ZMQ_PUB);
  zmq_bind(publisherSocket, "tcp://*:6970");
  
  std::vector<void*> sendingSocketList;
  sendingSocketList.push_back(publisherSocket);


  
  
  
// load road net
  //for test:
  std::vector<std::vector<RoadPoint>> rawRoadPointss;
//  std::vector<RoadPoint> roadPoints;
//  RoadPoint roadPoint;
//  
  rawRoadPointss.reserve(10000);
  uint32_t rawRoadPointssSize = loadRoadMapSize(ROADMAP_FILE_NAME);
std::cout << "mapsize:" << rawRoadPointssSize << std::endl;
//  uint32_t rawRoadPointssSize = 10;
  rawRoadPointss.resize(rawRoadPointssSize+1);
//  rawRoadPointss.resize(100);
  for (uint32_t i = 0; i < rawRoadPointssSize + 1; i++) {
//    rawRoadPointss.push_back(roadPoints);
    rawRoadPointss[i].reserve(1000);
//    rawRoadPointss[i].resize(1000);
//    for (uint32_t j = 0; j < 1000; j++){
//      rawRoadPointss[i].push_back(roadPoint);
//    }
  }
  
//  loadRoadMap(ROADMAP_FILE_NAME, rawRoadPointss);
  
//  std::cout << rawRoadPointss[1][0].longitude << ", " << rawoadPointss[1][0].latitude << std::endl;
  
//  std::cout << "SIZE: " << rawRoadPointss.size() << std::endl;
//  for (uint32_t i = 0; i < rawRoadPointss.size(); i++) {
//    std::cout << "sub size: " << rawRoadPointss[i].size() << std::endl;
//    for (uint32_t j = 0; j < rawRoadPointss[i].size(); j++) {
//      std::cout << rawRoadPointss[i][j].longitude << ", " << rawRoadPointss[i][j].latitude << std::endl;
//      gaussConvert(rawRoadPointss[i][j].longitude, rawRoadPointss[i][j].latitude, rawRoadPointss[i][j].longitude, rawRoadPointss[i][j].latitude);
//      std::cout << rawRoadPointss[i][j].longitude << ", " << rawRoadPointss[i][j].latitude << std::endl;
//    }
//  }
 
  loadRoadMapConverted(ROADMAP_FILE_NAME, rawRoadPointss, OFFSET_X, OFFSET_Y);
  
  std::cout << "Map loading done." << std::endl;
  
  
  // load stop point
  std::vector<RoadPoint> rawStopPoints;
//  rawStopPoints.reserve(100);
  uint32_t rawStopPointsSize = loadStopPointsSize(STOP_POINT_FILE_NAME);
  rawStopPoints.reserve(rawStopPointsSize + 1);
  
  loadStopPoints(STOP_POINT_FILE_NAME, rawStopPoints, OFFSET_X, OFFSET_Y);
  
//  for (uint32_t i = 0; i < rawStopPoints.size(); i++) {
//    gaussConvert(rawStopPoints[i].longitude, rawStopPoints[i].latitude, rawStopPoints[i].longitude, rawStopPoints[i].latitude);
//  }
  std::cout << "Stop Points loading done." << std::endl;
  
//  RoadPoint terminalPoint;
//  loadTerminalPoint(TERMINAL_POINT_FILE_NAME, terminalPoint);
//  gaussConvert(terminalPoint.longitude, terminalPoint.latitude, terminalPoint.longitude, terminalPoint.latitude);
//  std::cout << "Terminal point loading done." << std::endl;
  
  buildFinalRoadMap(rawRoadPointss, rawStopPoints);
  
  //  std::vector<std::vector<uint32_t>> pathNumberss;
  //  pathNumberss.reserve(10);
  //  pathNumberss.resize(10);
  //  for (int i = 0; i < 10; i++) {
  //    pathNumberss[i].reserve(1000);
  //  }
  
//  std::vector<std::array<uint32_t, MAX_PATH_NUMBER>> pathNumbersList;
//  pathNumbersList.resize(MAX_PATH);
//  loadPathNumber(PATH_NUMBER_FILE_NAME, pathNumbersList);
  std::vector<std::vector<uint32_t>> pathNumberLists;
  pathNumberLists.reserve(MAX_PATH);
  pathNumberLists.resize(MAX_PATH);
  for ( int i = 0; i < MAX_PATH; i ++){
    pathNumberLists.reserve(MAX_PATH_NUMBER);
  }
  loadPathNumber(PATH_NUMBER_FILE_NAME, pathNumberLists);
  
//  std::cout << "size2: " << rawRoadPointss.size() << std::endl;
  //  std::cout << rawRoadPointss[2][1].latitude << std::endl;
//  std::vector<Point> curve;
//  curve.reserve(50);
//  generateCurve(rawRoadPointss[2][1], rawRoadPointss[2][15], curve);
  //end test.
// thread initial
  
  boost::asio::io_service io;
std::cout << "message size:  " << rawRoadPointss.size() << std::endl;
  Jobs jobs(io, receivingSocketList, sendingSocketList, rawRoadPointss, rawStopPoints, pathNumberLists);
  boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
  io.run();
  t.join();

  return 0;
}


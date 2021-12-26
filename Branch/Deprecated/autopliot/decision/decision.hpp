#pragma once

#include <iostream>
#include <sstream>
#include <assert.h>
#include <unistd.h>
// for thread and timer:
#include <iostream>
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

#include "functions.hpp"



class Jobs{
  public:
  explicit Jobs (boost::asio::io_service& io, std::vector<void*>& receivingSocketList, std::vector<void*>& sendingSocketList, std::vector<std::vector<RoadPoint>>& roadpointss, std::vector<RoadPoint>& stopPoints, std::vector<std::vector<uint32_t>>& pathNumberLists);
    virtual ~Jobs();

    virtual void subscriber();
    virtual void handler();
    virtual void publisher();
    virtual void processor();

  private:
    boost::asio::strand strand1;
    boost::asio::strand strand2;
    boost::asio::strand strand3;
    boost::asio::strand strand4;
    boost::asio::deadline_timer timer1;
    boost::asio::deadline_timer timer2;
    boost::asio::deadline_timer timer3;
    boost::asio::deadline_timer timer4;
    std::vector<void*> rSocketList;
    std::vector<void*> sSocketList;
    std::vector<std::vector<RoadPoint>> roadPointss;
    std::vector<RoadPoint> stopPoints;
//    RoadPoint terminalPoint;
    std::vector<std::vector<uint32_t>> pathNumberLists;

    LidarData lidarData;
    VisionData visionData;
    PostureData postureData;
    ActuatorData actuatorData;
    UiData uiData;
    DecisionData decisionData;
    DecisionData lidarDecisionData;
    IovData iovData;
    CvData cvData;
    StartSignal startSignal;

//    uint32_t stateMachine = 0;
};


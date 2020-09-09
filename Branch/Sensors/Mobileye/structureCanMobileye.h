/*********************************************************************
//  created:    2011/06/03 - 18:00
//  filename:   structureCanMobileye.h
//
//  author:     Paul George
//              Copyright Heudiasyc UMR UTC/CNRS 6599
// 
//  version:    $Id: $
//
//  purpose:    Description of the CAN structures of Mobileye
*********************************************************************/

#ifndef __STRUCTURECANMOBILEYE_H__
#define __STRUCTURECANMOBILEYE_H__



#define MOBILEYE_MAX_OBSTACLE_COUNT 10
#define MOBILEYE_MAX_OBSTACLE_DATA 20

#define MOBILEYE_MAX_ID 63

#define OBSTACLE_MAX_X 250.0
#define OBSTACLE_MAX_Y 31.93
typedef uint64_t time_clock;
typedef int32_t timerange_clock;
#include <msgpack.hpp>

struct CanFrame
{

    uint32_t id;
    uint8_t dlc;
    uint8_t data [8];
	MSGPACK_DEFINE(id,dlc,data);
};

/// CAN structure with timestamping
struct TimestampedCanFrame
{
    CanFrame frame;
    time_clock time;
    timerange_clock timerange;
	MSGPACK_DEFINE(frame,time,timerange);

};
enum LaneValidStatus
{
	LANE_UNDEFINED = 0,
	LANE_NEW = 1,
	LANE_VALID = 2,
	LANE_INVALID = 3
};

// corresponding CAN frame = 0x731,0x732
typedef struct {
	int LaneValid;// 0 : undefined, 1 : new lane, 2 : valid lane, 3 : invalid lane
	double LaneCurvature;	//
	double LaneHeading;		//
	double LaneOffset;		//
	int LaneConf;			// Confidence : 0 -> 3
	MSGPACK_DEFINE(LaneValid,LaneCurvature,LaneHeading,LaneOffset,LaneConf);

}StructMobileyeLane;

typedef struct{
	time_clock time;
	timerange_clock timerange;
	StructMobileyeLane d;
	MSGPACK_DEFINE(time,timerange,d);

}TimestampedStructMobileyeLane;

// corresponding CAN frame = 0x7FF
typedef struct {
	int obstacleCount;			// Nombre d'obstacles
	MSGPACK_DEFINE(obstacleCount);

}StructMobileyeObstaclesHeader;

typedef struct{
	time_clock time;
	timerange_clock timerange;
	StructMobileyeObstaclesHeader d;
	MSGPACK_DEFINE(time,timerange,d);

}TimestampedStructMobileyeObstaclesHeader;

enum ObstacleType
{
	OBSTACLE_VEHICLE = 0,
	OBSTACLE_TRUCK = 1,
	OBSTACLE_BIKE = 2,
	OBSTACLE_PEDESTRIAN = 3,
	OBSTACLE_BICYCLE = 4,
  OBSTACLE_UNKNOWN = 99
};

enum ObstacleStatus
{
	OBSTACLE_UNDEFINED = 0,
	OBSTACLE_STANDING = 1,
	OBSTACLE_STOPPED = 2,
	OBSTACLE_MOVING = 3,
	OBSTACLE_ONCOMING = 4,
	OBSTACLE_PARKED = 5
};

enum ObstacleValidity
{
  OBSTACLE_NEW_VALID = 1,
  OBSTACLE_OLD_VALID = 2
};

// corresponding CAN frame = 0x739,0x73C,0x73F,0x742,0x745,0x748,0x74B,0x74E,0x751,0x754
typedef struct {
	int id;			//
	double x;	//
	double y;	//
	int type;		//
	int status;	// 0 : Undefined, 1 : Standing, 2 : Stopped, 3 : Moving, 4 : Oncoming, 5 : Parked
  int valid;    // 1 : new valid, 2 : old valid
	MSGPACK_DEFINE(id,x,y,type,status,valid);
}StructMobileyeObstacle1;

typedef struct{
	time_clock time;
	timerange_clock timerange;
	StructMobileyeObstacle1 d;
	MSGPACK_DEFINE(time,timerange,d);

}TimestampedStructMobileyeObstacle1;

// corresponding CAN frame = 0x73A,0x73D,0x740,0x743,0x746,0x749,0x74C,0x74F,0x752,0x755
typedef struct {
	double width;	//
	int age;		//
		MSGPACK_DEFINE(width,age);

}StructMobileyeObstacle2;

typedef struct{
	time_clock time;
	timerange_clock timerange;
	StructMobileyeObstacle2 d;
		MSGPACK_DEFINE(time,timerange,d);

}TimestampedStructMobileyeObstacle2;

typedef struct {
	StructMobileyeLane rightLane; // 40 bytes
	StructMobileyeLane leftLane;  // 40 bytes 
	StructMobileyeObstaclesHeader obstaclesHeader; // 8 bytes
	StructMobileyeObstacle1 obstacles1[MOBILEYE_MAX_OBSTACLE_DATA]; // 10 * 32 = 320 bytes
	StructMobileyeObstacle2 obstacles2[MOBILEYE_MAX_OBSTACLE_DATA]; // 10 * 16 = 160 bytes  ;  total = 568 bytes
	MSGPACK_DEFINE(rightLane,leftLane,obstaclesHeader,obstacles1,obstacles2);

} MobileyeDatas;

typedef struct{
	unsigned int LaneType;
	unsigned int LaneQuality;
	unsigned int LaneDegree;
	double LanePositionC0;
	//double LaneHeadingC1;
	double LaneCurvatureC2;
	double LaneCurvatureDerivC3;
	time_clock TimeStamp;
	MSGPACK_DEFINE(LaneType,LaneQuality,LaneDegree,LanePositionC0,LaneCurvatureC2,LaneCurvatureDerivC3,TimeStamp);

} LaneInfoPart1;

typedef struct{
	double LaneHeadingC1;
	time_clock TimeStamp;
	MSGPACK_DEFINE(LaneHeadingC1,TimeStamp);
} LaneInfoPart2;

typedef struct {
	LaneInfoPart1 LanePart1;
	LaneInfoPart2 LanePart2;
	MSGPACK_DEFINE(LanePart1,LanePart2);

} MobileyeLaneData;

#endif

#ifndef UDP_PROTOCOL_H
#define UDP_PROTOCOL_H
#include <msgpack.hpp>

//#include "vector"
#include <vector>
#include <string.h>

#define EthBufferMaxSize 5888u

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef float     float32;

struct OBJ_EQ
{
public:
    uint8 OBJ_ID;
    uint8 OBJ_Object_Class;

    float32 OBJ_Class_Probability;
    float32 OBJ_Lat_Distance;
    float32 OBJ_Long_Distance;
    float32 OBJ_Relative_Lat_Velocity;
    float32 OBJ_Relative_Long_Velocity;
    float32 OBJ_Relative_Long_Acc;
    float32 OBJ_Abs_Lat_Acc;
    float32 OBJ_Abs_Long_Acc;
    float32 OBJ_Abs_Lat_Velocity;
    float32 OBJ_Abs_Long_Velocity;
    float32 OBJ_Abs_Acceleration;
    float32 OBJ_Height;
    float32 OBJ_Height_STD;
    float32 OBJ_Length;
    float32 OBJ_Length_STD;
    float32 OBJ_Width;
    float32 OBJ_Width_STD;
    float32 OBJ_Existence_Probability;

    uint8 OBJ_Age_Seconds;
    uint8 OBJ_Motion_Orientation;
    uint8 OBJ_Motion_Status;
    uint8 OBJ_Measuring_Status;
    uint8 OBJ_Motion_Category;

    uint16 OBJ_Object_Age;
    MSGPACK_DEFINE(OBJ_ID, OBJ_Object_Class, OBJ_Class_Probability, OBJ_Lat_Distance,
                 OBJ_Long_Distance, OBJ_Relative_Lat_Velocity, OBJ_Relative_Long_Velocity, 
                 OBJ_Relative_Long_Acc,OBJ_Abs_Lat_Acc, OBJ_Abs_Long_Acc, OBJ_Abs_Lat_Velocity, 
                 OBJ_Abs_Long_Velocity, OBJ_Abs_Acceleration, OBJ_Height, OBJ_Height_STD, OBJ_Length, 
                 OBJ_Length_STD, OBJ_Width, OBJ_Width_STD, OBJ_Existence_Probability, OBJ_Age_Seconds, 
                 OBJ_Motion_Orientation, OBJ_Motion_Status, OBJ_Measuring_Status, OBJ_Motion_Category, OBJ_Object_Age);
};
// "OBJ_Object_Age" missed?

struct OBJ_40_EQ
{
    //Header header;
    OBJ_EQ obj_s[40];
    MSGPACK_DEFINE(obj_s);
};

struct OBJ_RSDA
{
public:
    uint8 RSDA_ID;
    uint16 RSDA_xPosition; 
    float32 rsda_dx;
    uint16 RSDA_yPosition;
    float32 rsda_dy;
    uint16 RSDA_xSpeed;
    float32 rsda_vx;
    uint16 RSDA_ySpeed;
    float32 rsda_vy;

    bool RSDA_ObjMovStatic;

    uint8 RSDA_SNR;
    //float rsda_snr;
    uint8 RSDA_obj_Class;

    MSGPACK_DEFINE(RSDA_ID, RSDA_xPosition, rsda_dx, RSDA_yPosition, rsda_dy, RSDA_xSpeed, 
                    rsda_vx, RSDA_ySpeed, rsda_vy, RSDA_ObjMovStatic, RSDA_SNR, RSDA_obj_Class);
};

struct OBJ_32_RSDA
{
    OBJ_RSDA objs_RSDA[32];
    MSGPACK_DEFINE(objs_RSDA);
    /* data */
};

struct OBJ_FSDA
{
public:
    uint8 FSDA_ID;
    uint16 FSDA_xPosition;
    float32 fsda_dx;
    uint16 FSDA_yPosition;
    float32 fsda_dy;
    uint16 FSDA_xSpeed;
    float32 fsda_vx;
    uint16 FSDA_ySpeed;
    float32 fsda_vy;
    bool FSDA_ObjMovStatic;
    uint8 FSDA_SNR;
    uint8 FSDA_obj_Class;

    MSGPACK_DEFINE(FSDA_ID, FSDA_xPosition, fsda_dx, FSDA_yPosition, fsda_dy, FSDA_xSpeed, fsda_vx,
                    FSDA_ySpeed, fsda_vy, FSDA_ObjMovStatic, FSDA_SNR, FSDA_obj_Class);
};

struct OBJ_32_FSDA
{
    OBJ_FSDA objs_FSDA[32];
    MSGPACK_DEFINE(objs_FSDA);
    /* data */
};

struct OBJ_LRR
{
public:
    uint8 LRR_ID;
    uint16 LRR_dx;
    float32 lrr_dx;
    uint16 LRR_dy;
    float32 lrr_dy;
    uint16 LRR_Vx;
    float32 lrr_vx;
    uint16 LRR_Vy;
    float32 lrr_vy;
    uint8 LRR_Sync;
    uint8 LRR_wExist;
    float32 lrr_wexist;
    uint8 LRR_wObstacle;
    float32 lrr_wobstacle;

    MSGPACK_DEFINE(LRR_ID, LRR_dx, lrr_dx, LRR_dy, lrr_dy, LRR_Vx, lrr_vx, LRR_Vy, lrr_vy, 
                    LRR_Sync, LRR_wExist, lrr_wexist, LRR_wObstacle, lrr_wobstacle);
};

struct OBJ_32_LRR
{
    OBJ_LRR objs_LRR[32];
    MSGPACK_DEFINE(objs_LRR);
    /* data */
};

/*stHeader*/
struct stHeader
{
public:
    uint8 FrameID;
    uint16 Counter;
    float32 Timestamp[2];

    MSGPACK_DEFINE(FrameID, Counter, Timestamp);
};

/*
typedef struct _tag_Header
{
    int seq;         //计数变量
    Stamp stamp;     //时间戳
    string frame_id; //坐标系名称
} Header;

typedef struct _tag_Stamp
{
    int secs;  //机器时间，秒数
    int nsecs; //秒数小数点后，剩余的纳秒数
} Stamp;
*/

struct Sematic_lines_info
{
public:
    /*sematic lines*/
    bool SL_Close_To_Junc[10];

    /*lanes num*/

    uint8 SLD_Num_Of_Lanes_Close_Left;
    uint8 SLD_Num_Of_Lanes_Close_Right;

    MSGPACK_DEFINE(SL_Close_To_Junc, SLD_Num_Of_Lanes_Close_Left, SLD_Num_Of_Lanes_Close_Right);
};

/*Free Space*/
struct 	Freespace_point
{
public:
	float32		FSP_Range;
	float32		FSP_Range_STD;
	float32		FSP_Azimuth_Angle;

    MSGPACK_DEFINE(FSP_Range,FSP_Range_STD,FSP_Azimuth_Angle);
};

struct FSP_72
{
    public:
    Freespace_point Freespace_pt[72];
    MSGPACK_DEFINE(Freespace_pt);
    /* data */
};

/* typedef struct _tag_Map{
    Header header;
    bool in_junction;
    std::vector<int> exit_lane_index; //依附全局路径规划，例：在下一个路口左转，则须将左转车道的序号放在此处
    std::vector<Lane> lanes; //从右侧数起，最右车道为0
    std::vector<Lane> virtual_lanes; //路口中的虚拟车道线
    Polygon drivable_area; //如在路口中，给出其边界多边形
    Polygon next_drivable_area;  //依附全局路径规划，下一个路口的边界多边形
    std::vector<Lane> next_lanes; //依附全局路径规划，在进入下一个路口后，下一步将要进入的路段的车道信息
    int next_road_id;
}Map; */

/*traffic lights*/

uint8	TFL_Number_Of_Objects;

struct 	Trafic_light
{
public:
	uint8	TFL_Object_ID;
	uint8 	TFL_LightBox_ID;
	float32	TFL_Long_Distance;
	float32	TFL_Long_Distance_STD;
	uint8	TFL_Color;
	uint8 	TFL_Mode;
	float32	TFL_Existence_Probability;

    MSGPACK_DEFINE(TFL_Object_ID,TFL_LightBox_ID,TFL_Long_Distance,TFL_Long_Distance_STD,
                    TFL_Color,TFL_Mode,TFL_Existence_Probability);
};

struct TFL_10
{
    public:
    Trafic_light TF_Lt[10];

    uint8 TFL_num;
    MSGPACK_DEFINE(TF_Lt,TFL_num);
    /* data */
};

/*Path_preds*/

struct Path_pred
{

public:
    float32 LS_Path_Prediction_VR_End;
        /* Description: The end view range of the center lane polynomial  */
        /* model (last Z that is allowed to use in the polynomial).  */
    float32 LS_Path_Pred_Half_Width;
    /* Description: Width from center line to lane border (of    */
    /* the pathPrediction model)                                 */
    float32 LS_Path_Pred_Conf;
    float64 LS_Path_Pred_C3;
    float64 LS_Path_Pred_C2;
    float32 LS_Path_Pred_C1;
    float32 LS_Path_Pred_C0;
    uint32 LS_Path_Pred_CRC;
    /* Description: CRC over the Path_Pred signals (including    */
    /* the LSA_Header_Buffer)                                    */
    bool LS_Path_Pred_Available;
    /* Description: If output is to be used                      */
    bool LS_Is_Highway_Exit_Right;
    /* Description: Highway exit on the right was found          */
    bool LS_Is_Highway_Exit_Left;
    /* Description: Highway exit on the left was found           */
    bool LS_Is_Highway_Merge_Right;
    /* Description: Highway merging from the right was found     */
    bool LS_Is_Highway_Merge_Left;
    /* Description: Highway merging from the left was found      */
    bool LS_Is_Exit_Right_Valid;
    /* Description: Highway exit on the right is valid           */
    bool LS_Is_Exit_Left_Valid;
    /* Description: Highway exit on the left is valid            */
    bool LS_Is_Merge_Right_Valid;
    /* Description: Highway merging from the right is valid      */
    bool LS_Is_Merge_Left_Valid;
    /* Description: Highway merging from the left info is valid  */
    bool LS_Exit_Merge_Available;
    /* Description: Is info available                            */
    bool LS_CA_Is_Construction_Area;
    /* Description: True if this is a construction area scene.   */
    /* Road responsibility (includes information from TSR for    */
    /* cones)                                                    */
    MSGPACK_DEFINE(LS_Path_Prediction_VR_End,LS_Path_Pred_Half_Width,LS_Path_Pred_Conf,LS_Path_Pred_C3,
                    LS_Path_Pred_C2,LS_Path_Pred_C1,LS_Path_Pred_C0,LS_Path_Pred_CRC,LS_Path_Pred_Available,
                    LS_Is_Highway_Exit_Right,LS_Is_Highway_Exit_Left,LS_Is_Highway_Merge_Right,
                    LS_Is_Highway_Merge_Left,LS_Is_Exit_Right_Valid,LS_Is_Exit_Left_Valid,LS_Is_Merge_Right_Valid,
                    LS_Is_Merge_Left_Valid,LS_Exit_Merge_Available,LS_CA_Is_Construction_Area);
};

/*Road_Edges*/

struct 	Road_Edge
{
    public:
	float32		LS_Road_Edge_Line_C0;
	float32		LS_Road_Edge_Line_C0_STD;
	float32		LS_Road_Edge_Line_C1_STD;
	float32		LS_Road_Edge_Line_C1;
	float64		LS_Road_Edge_Line_C2;
	float32		LS_Road_Edge_Line_C2_STD;
	float64		LS_Road_Edge_Line_C3_STD;
	float64		LS_Road_Edge_Line_C3;
	float32		LS_Road_Edge_Measured_VR_End;
    uint8       LS_Road_Edge_Type;

    MSGPACK_DEFINE(LS_Road_Edge_Line_C0,LS_Road_Edge_Line_C0_STD,LS_Road_Edge_Line_C1_STD,LS_Road_Edge_Line_C1,LS_Road_Edge_Line_C2,
                    LS_Road_Edge_Line_C2_STD,LS_Road_Edge_Line_C3_STD,LS_Road_Edge_Line_C3,LS_Road_Edge_Measured_VR_End,LS_Road_Edge_Type);
};

uint32 LS_Road_Edge_CRC;

struct Road_Edge_4
{
    public:
    Road_Edge Rd_Eg[4];
    uint32 LS_Rd_Eg_CRC;
    MSGPACK_DEFINE(Rd_Eg,LS_Rd_Eg_CRC);
    /* data */
};

/* Lane Support */

struct 	Host_Line
{
    public:
	float64		LS_Host_Line_C3;
	float64		LS_Host_Line_C3_STD;
	float64		LS_Host_Line_C2_STD;
	float64		LS_Host_Line_C2;
	float32		LS_Host_Line_C1;
	float32		LS_Host_Line_C1_STD;
	float32		LS_Host_Line_C0_STD;
	float32		LS_Host_Line_C0;
	float32		LS_Host_Measured_VR_End;
	uint8		LS_Host_Type_Classification;

    MSGPACK_DEFINE(LS_Host_Line_C3,LS_Host_Line_C3_STD,LS_Host_Line_C2_STD,LS_Host_Line_C2,LS_Host_Line_C1,LS_Host_Line_C1_STD,
                    LS_Host_Line_C0_STD,LS_Host_Line_C0,LS_Host_Measured_VR_End,LS_Host_Type_Classification);
};

struct Host_Line_2
{
    public:
    Host_Line Ht_Ln[2];
    float32	LS_Host_Est_Width;
    MSGPACK_DEFINE(Ht_Ln,LS_Host_Est_Width);
    /* data */
};

struct 	Adjacent_Line
{
    public:
	float32		LS_Adjacent_Line_C0_STD;
	float32		LS_Adjacent_Line_C0;
	float32		LS_Adjacent_Line_C1;
	float32		LS_Adjacent_Line_C1_STD;
	float64		LS_Adjacent_Line_C2_STD;
	float64		LS_Adjacent_Line_C2;
	float64		LS_Adjacent_Line_C3;
	float64		LS_Adjacent_Line_C3_STD;
	float32		LS_Adjacent_Measured_VR_End;
	uint8		LS_Adjacent_Type_Classification;

    MSGPACK_DEFINE(LS_Adjacent_Line_C0_STD,LS_Adjacent_Line_C0,LS_Adjacent_Line_C1,LS_Adjacent_Line_C1_STD,LS_Adjacent_Line_C2_STD,
                     LS_Adjacent_Line_C2,LS_Adjacent_Line_C3,LS_Adjacent_Line_C3_STD,LS_Adjacent_Measured_VR_End,LS_Adjacent_Type_Classification);
};

struct Adjacent_Line_4
{public:
    Adjacent_Line Adj_Ln[4];
    MSGPACK_DEFINE(Adj_Ln);
    /* data */
};


float32	LS_Host_Estimated_Width;
bool   	LS_CA_Is_Construction_Area_Lane_Support;

/* 
typedef struct _tag_Lane{
    int index; //车道序号，从右侧数起，最右车道为0
    float speed_limit;
    float length;
    float width;
    bool bidirectional; //是否允许双向通行，一般为False
    int stop_state; //车道末端停车要求（红绿灯），0未知，1绿灯，2无交通灯，3黄灯，4红灯
    std::vector<LanePoint> central_path_points; //车道中心线点列
    std::vector<float> central_path_coeffs; //车道中心线解析式参数（如无，可置空）
    int central_path_type; //0无解析式，1直线，2 conic，3 polynomial，4 Bezier
    std::vector<LaneBoundary> left_boundaries //左侧车道边界（车道线）
    std::vector<LaneBoundary> right_boundaries;
    std::vector<LaneSituation> situations; //该车道上存在的其他情况，例：存在人行横道，需标注出人行横道所在位置
    std::vector<float> traffic_light_pos; //该车道上红绿灯所在的位置（纵向距离，如100m处）
}Lane;

typedef struct _tag_LaneBoundary{
    LanePoint boundary_point;
    int boundary_type; //0未知，1白色虚线，2黄色虚线，3白色实线，4黄色实线，6道路实体边界（无车道线，以路沿护栏等为车道边界）
    float confidence; //一般面向车载感知检测，可置空
}LaneBoundary;

typedef struct _tag_LaneSituation{
    float s; //在纵向上的起作用的起始位置
    float length; //影响范围，纵向的长度
    int situation_type; //类型，1静态障碍（如施工等），2人行横道，3需要减速的情形（如路边存在学校）
    float reduced_max_speed; //situation导致的限速，若需要停车（如静态障碍，施工），则为0
    string comments; //文字说明，可选
}LaneSituation;

typedef struct _tag_LanePoint{
    Point position;
    float s; //纵向位置，参见OpenDRIVE道路坐标系
    float slope; //该点处坡度，可选
    float curvature; //该点处俯视图下的曲率，可选
    float tangent; //该点处俯视图下的方向角（弧度）
    float width; //该点处车道宽度（若此LanePoint为车道边界点，则置空；如果该车道宽度始终不变，亦可置空，因Lane中已给出宽度）
}LanePoint;

typedef struct _tag_Polygon{
    std::vector<Point> points;
}Polygon;

typedef struct _tag_Point{
    float x;
    float y;
    float z;
}Point; */

#endif
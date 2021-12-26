#ifndef __Marker_H__
#define __Marker_H__

#include <OpenICV/structure/header.h>
#include <OpenICV/structure/Pose.h>
#include <OpenICV/structure/Vector3.h>
#include <OpenICV/structure/ColorRGBA.h>
#include <msgpack.hpp>
#include <vector>

namespace icv{

  namespace  data{

//Basic structure of Marker
struct Marker
{
    uint8 ARROW=0;
    uint8 CUBE=1;
    uint8 SPHERE=2;
    uint8 CYLINDER=3;
    uint8 LINE_STRIP=4;
    uint8 LINE_LIST=5;
    uint8 CUBE_LIST=6;
    uint8 SPHERE_LIST=7;
    uint8 POINTS=8;
    uint8 TEXT_VIEW_FACING=9;
    uint8 MESH_RESOURCE=10;
    uint8 TRIANGLE_LIST=11;
    uint8 ADD=0;
    uint8 MODIFY=0;
    uint8 DELETE=2;
    uint8 DELETEALL=3;

    Header header;

    string ns;

    int32 id;
    int32 type;
    int32 action;
    Pose pose;
    Vector3 scale;
    ColorRGBA color;
    duration lifetime;
    bool frame_locked;

    std:: vector<Point> points;
   // geometry_msgs/Point[] points

    std:: vector<ColorRGBA>colors;
    // std_msgs/ColorRGBA[] colors
    string text;
    string mesh_resource;
    bool mesh_use_embedded_materials;


   MSGPACK_DEFINE( ARROW,CUBE, SPHERE,CYLINDER,LINE_STRIP,LINE_LIST,
  CUBE_LIST, SPHERE_LIST,POINTS,TEXT_VIEW_FACING,MESH_RESOURCE,TRIANGLE_LIST,
  ADD,MODIFY,DELETE,DELETEALL,header,ns, id, type,action,pose,scale,color,lifetime,
  frame_locked,points,colors,text,mesh_resource,mesh_use_embedded_materials);
};
  }
}

#endif 

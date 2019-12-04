/*
*   Autonomous Driving Navigation Library
*   Coordinates Converter Class
*   Li Bing, QYJY, 2018.7
*/

#ifndef ADNCOORDCONVERTER_H
#define ADNCOORDCONVERTER_H

#include "ADNType.h"
#include "ADNXShareEngine.h"

// Coordinates Converter Class
class ADNCoordConverter
{
public:
    ADNCoordConverter();

    // set screen size
    void SetScreenSize(int w,int h);

    // set parameters for 3D transform
    // center:   car view distance   (m)
    // near:     near view distance  (m)
    // far:      far view distance   (m)
    // width:    near view width     (m)
    // vanish:   vanish line height  (pix)
    void Set3DParam(double center,double near,double far,double width,int vanish);

    // local point to 3D screen position
    void LocalTo3D(ADNPoint& pt);
    // local line segment to 3D screen position
    bool LocalTo3D(ADNLine& line);

    // set car position and heading
    // pt:   car geography position  (longitude,latitude)
    // ang:  car heading north angle (degree)
    void SetCenter(ADNPoint pt,double ang);

    // geography point to local coordinates
    void GeoToLocal(ADNPoint& pt);   
    // geography point and heading to local coordinates
    void GeoToLocal(ADNPosition& pos);

    // get object screen height
    // p:    local coordinates   (m,m)
    // h0:   raw height          (m)
    double Get3DHeight(ADNPoint p,double h0);

    // geography coordinates to aerial view map
    void GeoToMap(double& x,double& y);

private:
    ADNPoint m_Center;      //car geography coordinates (longitude,latitude)
    ADNDouble m_Heading;    //car heading north angle (degree)
    ADNInt m_ScreenWidth;   //screen width  (pixel points)
    ADNInt m_ScreenHeight;  //screen height (pixel points)

    double m_CenterDistance;    //car view distance   (m)
    double m_NearDistance;      //near view distance   (m)
    double m_FarDistance;       //far view distance   (m)
    double m_NearWidth;         //near end view width   (m)
    double m_VanishingHeight;   //vanishing line height  (pixel points)

    GeoRect m_ClipRect;         //clip rectangle for 3d coordinate transform
};

#endif // ADNCOORDCONVERTER_H

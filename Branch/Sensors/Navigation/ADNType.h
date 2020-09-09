/*
*   Autonomous Driving Navigation Library
*   Basic Data Types
*   Li Bing, QYJY, 2018.7
*/

#ifndef ADNTYPE_H
#define ADNTYPE_H

typedef double ADNDouble;
typedef int    ADNInt;

// point
struct ADNPoint
{
    ADNDouble X;
    ADNDouble Y;

    ADNPoint();
    ADNPoint(double x,double y);
};

// position: location point and heading
struct ADNPosition
{
    ADNPoint Center;    //location point
    ADNDouble Heading;  //heading angle (degree)

    ADNPosition();
    ADNPosition(ADNPoint center,ADNDouble heading);
};

// rectangle
struct ADNRect
{
    ADNDouble X;    //left
    ADNDouble Y;    //top
    ADNDouble W;    //width
    ADNDouble H;    //height
};

// curve
struct ADNCurve
{
    ADNPoint *Points;   //points buffer
    ADNInt Count;       //points count
};

// line segment
struct ADNLine
{
    ADNPoint Source;    //source point
    ADNPoint Target;    //target point

    ADNLine();
    ADNLine(ADNPoint s,ADNPoint t);
};

// road line
struct ADNRoadLine
{
    ADNInt Id;          //id
    ADNInt Direction;   //direction
    ADNRect BBox;       //bound box
    ADNCurve Curve;     //shape curve
};

// object model
struct ADNObject
{
    ADNCurve Outline;   //outline curve (in local coordinates)
    ADNDouble Height;   //height
};

#endif // ADNTYPE_H

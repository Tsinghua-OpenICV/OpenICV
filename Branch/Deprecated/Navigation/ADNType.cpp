/*
*   Autonomous Driving Navigation Library
*   Basic Data Types
*   Li Bing, QYJY, 2018.7
*/

#include "ADNType.h"

ADNPoint::ADNPoint()
{
}

ADNPoint::ADNPoint(double x,double y)
{
    X=x;
    Y=y;
}

ADNPosition::ADNPosition()
{
}

ADNPosition::ADNPosition(ADNPoint center, ADNDouble heading)
{
    Center=center;
    Heading=heading;
}

ADNLine::ADNLine()
{
}

ADNLine::ADNLine(ADNPoint s, ADNPoint t)
{
    Source=s;
    Target=t;
}

/*
*   Autonomous Driving Navigation Library
*   Map Utility Functions from XShare Enginge
*   Li Bing, QYJY, 2018.7
*/

#include "ADNXShareEngine.h"

GeoLineSeg::GeoLineSeg()
: m_Beg(0,0)
, m_End(0,0)
{
    ;
}

GeoLineSeg::GeoLineSeg( Int32 xBeg,Int32 yBeg,Int32 xEnd,Int32 yEnd )
: m_Beg(xBeg,yBeg)
, m_End(xEnd,yEnd)
{
    ;
}

GeoLineSeg::GeoLineSeg( const GeoPoint& ptBeg, const GeoPoint& ptEnd )
: m_Beg( ptBeg )
, m_End( ptEnd )
{
    ;
}

GeoLineSeg::GeoLineSeg( const GeoLineSeg& other )
: m_Beg( other.m_Beg )
, m_End( other.m_End )
{
    ;
}

bool GeoLineSeg::operator==(const GeoLineSeg& other) const
{
    return m_Beg==other.m_Beg && m_End==other.m_End;
}

bool GeoLineSeg::operator!=(const GeoLineSeg& other) const
{
    return !(*this == other);
}



GeoPoint::GeoPoint()
{
    X = 0;
    Y = 0;
}

GeoPoint::GeoPoint(Int32 x, Int32 y)
{
    X = x;
    Y = y;
}

GeoPoint::GeoPoint( const GeoPoint& other )
{
    X = other.X;
    Y = other.Y;
}

bool GeoPoint::operator == (const GeoPoint& other) const
{
    return X==other.X && Y == other.Y;
}

bool GeoPoint::operator != (const GeoPoint& other) const
{
    return !(*this == other);
}



GeoRect::GeoRect()
: m_bottomLeft(0,0)
, m_upperRight(0,0)
{
    ;
}

GeoRect::GeoRect( Int32 left, Int32 bottom, Int32 right, Int32 up )
: m_bottomLeft(left,bottom)
, m_upperRight(right,up)
{
    ;
}

GeoRect::GeoRect( const GeoPoint& ptBottomLeft, const GeoPoint& ptUpRight )
: m_bottomLeft( ptBottomLeft )
, m_upperRight( ptUpRight )
{
    ;
}

GeoRect::GeoRect( const GeoPoint& centerPoint,Int32 halfSideLength )
{
    m_bottomLeft.X = centerPoint.X - halfSideLength;
    m_bottomLeft.Y = centerPoint.Y - halfSideLength;
    m_upperRight.X = centerPoint.X + halfSideLength;
    m_upperRight.Y = centerPoint.Y + halfSideLength;
}

GeoRect::GeoRect( const GeoRect& other )
: m_bottomLeft( other.m_bottomLeft )
, m_upperRight( other.m_upperRight )
{
    ;
}

bool GeoRect::operator==(const GeoRect& other) const
{
    return m_bottomLeft==other.m_bottomLeft && m_upperRight==other.m_upperRight;
}

bool GeoRect::operator!=(const GeoRect& other) const
{
    return !(*this == other);
}

Int32 GeoRect::Square() const
{
    return (m_upperRight.X - m_bottomLeft.X) * (m_upperRight.Y - m_bottomLeft.Y);
}

void GeoRect::Normalize()
{
    Int32 xmin = XMin(m_bottomLeft.X, m_upperRight.X);
    Int32 ymin = XMin(m_bottomLeft.Y, m_upperRight.Y);
    Int32 xmax = XMax(m_bottomLeft.X, m_upperRight.X);
    Int32 ymax = XMax(m_bottomLeft.X, m_upperRight.Y);

    m_bottomLeft.X = xmin;
    m_bottomLeft.Y = ymin;
    m_upperRight.X = xmax;
    m_upperRight.Y = ymax;
}

bool Contain( const GeoRect& rect, const GeoPoint& point )
{
    bool isContain = false;
    if ( point.X >= rect.m_bottomLeft.X
        && point.X < rect.m_upperRight.X
        && point.Y >= rect.m_bottomLeft.Y
        && point.Y < rect.m_upperRight.Y )
    {
        isContain = true;
    }

    return isContain;
}

GeoRect IntersectRect( const GeoRect& rect1,const GeoRect& rect2 )
{
    Int32 left = XMax( rect1.m_bottomLeft.X, rect2.m_bottomLeft.X );
    Int32 right = XMin( rect1.m_upperRight.X, rect2.m_upperRight.X );
    Int32 bottom = XMax( rect1.m_bottomLeft.Y, rect2.m_bottomLeft.Y );
    Int32 top = XMin( rect1.m_upperRight.Y, rect2.m_upperRight.Y );
    if( left>right || bottom>top )
    {
        return INVALIDATE_RECT;
    }
    else
    {
        return GeoRect(left,bottom,right,top);
    }
}

////求矩形相减
//bool RectSubtract(const GeoRect& rect1,const GeoRect& rect2, FrameRects& frameRects)
//{
//    XBEGIN_FUNC;
//    bool bl,tr;

//    bl = (rect1.m_bottomLeft.X <= rect2.m_bottomLeft.X)&&(rect1.m_bottomLeft.Y <= rect2.m_bottomLeft.Y);
//    tr = (rect1.m_upperRight.X >= rect2.m_upperRight.X)&&(rect1.m_upperRight.Y >= rect2.m_upperRight.Y);

//    if(!(bl&&tr)) return false;
//    else
//    {
//        frameRects.up = GeoRect(rect1.m_bottomLeft.X,rect2.m_upperRight.Y,rect1.m_upperRight.X,rect1.m_upperRight.Y);
//        frameRects.left = GeoRect(rect1.m_bottomLeft.X,rect2.m_bottomLeft.Y,rect2.m_bottomLeft.X,rect2.m_upperRight.Y);
//        frameRects.right = GeoRect(rect2.m_upperRight.X,rect2.m_bottomLeft.Y,rect1.m_upperRight.X,rect2.m_upperRight.Y);
//        frameRects.down = GeoRect(rect1.m_bottomLeft.X,rect1.m_bottomLeft.Y,rect1.m_upperRight.X,rect2.m_bottomLeft.Y);
//        return true;
//    }
//}

////求矩形相减(1、2有相交部分)
//bool RectSubtract(const GeoRect& rect1,const GeoRect& rect2, RectPair& pairRects)
//{
//    XBEGIN_FUNC;
//    Int32 horiz_left,horiz_mid,horiz_right;
//    Int32 vert_up,vert_mid,vert_down;

//    horiz_left = rect1.m_bottomLeft.X;
//    horiz_mid = rect2.m_bottomLeft.X;
//    horiz_right = rect1.m_upperRight.X;
//    bool left_mov = (rect1.m_upperRight.X > rect2.m_upperRight.X);
//    if (left_mov) horiz_mid = rect2.m_upperRight.X;

//    vert_down = rect1.m_bottomLeft.Y;
//    vert_mid = rect2.m_bottomLeft.Y;
//    vert_up = rect1.m_upperRight.Y;
//    bool down_mov = (rect1.m_upperRight.Y > rect2.m_upperRight.Y);
//    if (down_mov) vert_mid = rect2.m_upperRight.Y;

//    if(left_mov)
//    {
//        pairRects.m_First = GeoRect(horiz_mid,vert_down,horiz_right,vert_up);
//        if(down_mov) pairRects.m_Second = GeoRect(horiz_left,vert_mid,horiz_mid,vert_up);
//        else pairRects.m_Second = GeoRect(horiz_left,vert_down,horiz_mid,vert_mid);
//    }

//    else
//    {
//        pairRects.m_First = GeoRect(horiz_left,vert_down,horiz_mid,vert_up);
//        if(down_mov) pairRects.m_Second = GeoRect(horiz_mid,vert_mid,horiz_right,vert_up);
//        else pairRects.m_Second = GeoRect(horiz_mid,vert_down,horiz_right,vert_mid);
//    }

//    return true;
//}

////求旋转后的矩形的外接矩形
//GeoRect RotatedRectCircum(const GeoRect& rect, Int32 angle, const GeoPoint& centre)
//{
//    XBEGIN_FUNC;

//    GeoPoint leftupper(rect.m_bottomLeft.X,rect.m_upperRight.Y);
//    GeoPoint rightbottom(rect.m_upperRight.X,rect.m_bottomLeft.Y);

//    GeoPoint p1,p2,p3,p4;

//    p1 = GeoPointRotate(rect.m_bottomLeft,centre,angle);
//    p2 = GeoPointRotate(leftupper,centre,angle);
//    p3 = GeoPointRotate(rect.m_upperRight,centre,angle);
//    p4 = GeoPointRotate(rightbottom,centre,angle);

//    Int32 left,right,up,down,temp;

//    left = XMin(p1.X,p2.X);
//    temp = XMin(p3.X,p4.X);
//    left = XMin(left,temp);

//    temp = XMax(p1.X,p2.X);
//    right = XMax(p3.X,p4.X);
//    right = XMax(right,temp);

//    down = XMin(p1.Y,p2.Y);
//    temp = XMin(p3.Y,p4.Y);
//    down = XMin(down,temp);

//    temp = XMax(p1.Y,p2.Y);
//    up = XMax(p3.Y,p4.Y);
//    up = XMax(up,temp);

//    GeoRect Circum(left,down,right,up);

//    return Circum;

//}

////求两个矩形的差补框
//Int32 DeltaRect(const GeoRect& rawR,const GeoRect& newR,FrameRects& finalDelta)
//{
//    GeoRect interR = IntersectRect(rawR,newR);

//    FrameRects newDelta,rawDelta;

//    RectSubtract(newR,interR,newDelta);

//    RectSubtract(rawR,interR,rawDelta);

//    Int32 addIndex = 0;
//    Int32 subIndex = 3;

//    if(newDelta.up.Square()!=0) finalDelta.SetRect(newDelta.up,addIndex++);
//    else finalDelta.SetRect(rawDelta.up,subIndex--);

//    if(newDelta.left.Square()!=0) finalDelta.SetRect(newDelta.left,addIndex++);
//    else finalDelta.SetRect(rawDelta.left,subIndex--);

//    if(newDelta.right.Square()!=0) finalDelta.SetRect(newDelta.right,addIndex++);
//    else finalDelta.SetRect(rawDelta.right,subIndex--);

//    if(newDelta.down.Square()!=0) finalDelta.SetRect(newDelta.down,addIndex++);
//    else finalDelta.SetRect(rawDelta.down,subIndex--);

//    return addIndex;

//}

//void FrameRects::SetRect(const GeoRect& valueR, Int32 rectIndex)
//{
//    if(rectIndex<=0) up = GeoRect(valueR);
//    else if(rectIndex==1) left = GeoRect(valueR);
//    else if(rectIndex==2) right = GeoRect(valueR);
//    else down = GeoRect(valueR);

//}

//利用矩形框裁剪线段(Liang_Barsky)
GeoLineSeg RectTrimLineSeg(const GeoRect& rect,const GeoLineSeg& lineSeg)
{
    Int64 p[4];
    Int64 q[4];
    Int64 u;
    GeoLineSeg LS(0,0,0,0);

    p[0] = -(lineSeg.m_End.X - lineSeg.m_Beg.X);
    p[1] = - p[0];
    p[2] = -(lineSeg.m_End.Y - lineSeg.m_Beg.Y);
    p[3] = - p[2];

    q[0] = lineSeg.m_Beg.X - rect.m_bottomLeft.X;
    q[1] = rect.m_upperRight.X - lineSeg.m_Beg.X;
    q[2] = lineSeg.m_Beg.Y - rect.m_bottomLeft.Y;
    q[3] = rect.m_upperRight.Y - lineSeg.m_Beg.Y;

    Int64 zoomV = 1000000;
    Int64 umin = 0;
    Int64 umax = zoomV;

    for (int i=0; i<4; i++)
    {
        if(p[i]==0)
        {
            if(q[i]<0) return LS;
        }
        else if(p[i]<0)
        {
            u = q[i] * zoomV / p[i];
            if(umin<u) umin = u;
        }
        else
        {
            u = q[i] * zoomV / p[i];
            if(umax>u) umax = u;
        }
    }

    if(umax>=umin)
    {
        LS.m_Beg.X = lineSeg.m_Beg.X + umin * p[1]/zoomV;
        LS.m_Beg.Y = lineSeg.m_Beg.Y + umin * p[3]/zoomV;

        LS.m_End.X = lineSeg.m_Beg.X + umax * p[1]/zoomV;
        LS.m_End.Y = lineSeg.m_Beg.Y + umax * p[3]/zoomV;
    }

    return LS;
}

////利用矩形框裁剪曲线
//Int16 RectTrimCurve(const GeoRect& rect,const GeoCurve& rawCurve, GeoCurve& newCurve)
//{
//    if (rawCurve.Size()<=1)
//    {
//        return -1;
//    }

//    bool lastStatus,preStatus;
//    GeoPoint lastP,preP;
//    Int16 passNum = 0;

//    lastP = rawCurve.Front();
//    lastStatus = Contain(rect,lastP);
//    if(lastStatus) newCurve.PushBack(lastP);

//    for (int i=1; i<rawCurve.Size(); i++)
//    {
//        preP = rawCurve[i];
//        preStatus = Contain(rect,preP);

//        if (lastStatus==true&&preStatus==false)
//        {
//            GeoLineSeg insideP = RectTrimLineSeg(rect,GeoLineSeg(lastP,preP));
//            passNum++;
//            if(insideP.m_Beg==lastP)
//            {
//                newCurve.PushBack(insideP.m_End);
//            }
//            else
//            {
//                newCurve.PushBack(insideP.m_Beg);
//            }
//        }
//        else if (lastStatus==true&&preStatus==true)
//        {
//            newCurve.PushBack(preP);
//        }
//        else if(lastStatus==false&&preStatus==false)
//        {
//            GeoLineSeg insideP = RectTrimLineSeg(rect,GeoLineSeg(lastP,preP));
//            if (insideP!=GeoLineSeg(0,0,0,0))
//            {
//                passNum += 1;
//                if (GeoPointDist(insideP.m_Beg,lastP)<GeoPointDist(insideP.m_End,lastP))
//                {
//                    newCurve.PushBack(insideP.m_Beg);
//                    newCurve.PushBack(insideP.m_End);
//                }
//                else
//                {
//                    newCurve.PushBack(insideP.m_End);
//                    newCurve.PushBack(insideP.m_Beg);
//                }
//            }

//        }
//        else
//        {
//            GeoLineSeg insideP = RectTrimLineSeg(rect,GeoLineSeg(lastP,preP));
//            if (insideP.m_End==preP)
//            {
//                newCurve.PushBack(insideP.m_Beg);
//                newCurve.PushBack(insideP.m_End);
//            }
//            else
//            {
//                newCurve.PushBack(insideP.m_End);
//                newCurve.PushBack(insideP.m_Beg);
//            }

//        }
//        lastP = preP;
//        lastStatus = preStatus;
//    }

//    if (Contain(rect,rawCurve.Back()))
//    {
//        passNum++;
//    }

//    return passNum;
//}

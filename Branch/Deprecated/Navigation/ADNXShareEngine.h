/*
*   Autonomous Driving Navigation Library
*   Map Utility Functions from XShare Enginge
*   Li Bing, QYJY, 2018.7
*/

#ifndef ADNXSHAREENGINE_H
#define ADNXSHAREENGINE_H

//from XShare map engine

typedef int Int32;
typedef long long Int64;

//求两个数中较小的数
template<typename T>
inline T XMin(T x, T y)
{
    return x<y ? x:y;
}

//求两个数中较大的数
template<typename T>
inline T XMax(T x, T y)
{
    return x>y ? x:y;
}

#define INVALIDATE_RECT	GeoRect(0x7fffffff,0x7fffffff,0x7fffffff,0x7fffffff)

struct GeoPoint
{
    Int32 X;
    Int32 Y;

    GeoPoint();
    GeoPoint(Int32 x, Int32 y);
    GeoPoint( const GeoPoint& other );

    bool operator == (const GeoPoint& other) const;
    bool operator != (const GeoPoint& other) const;
};


struct GeoRect
{
    GeoPoint m_bottomLeft;			// 矩形左下角地理位置点
    GeoPoint m_upperRight;			// 矩形右上角地理位置点

    GeoRect();
    GeoRect( Int32 left, Int32 bottom, Int32 right, Int32 up );
    GeoRect( const GeoPoint& ptBottomLeft, const GeoPoint& ptUpRight );
    // 构造正方形
    GeoRect( const GeoPoint& centerPoint,Int32 halfSideLength );
    GeoRect( const GeoRect& other );

    bool operator==(const GeoRect& other) const;
    bool operator!=(const GeoRect& other) const;

    Int32 Square() const;

    void Normalize();
};

//线段
struct GeoLineSeg
{
    GeoPoint m_Beg;					// 线段起点
    GeoPoint m_End;					// 线段终点

    GeoLineSeg();
    GeoLineSeg( Int32 xBeg,Int32 yBeg,Int32 xEnd,Int32 yEnd );
    GeoLineSeg( const GeoPoint& ptBeg, const GeoPoint& ptEnd );
    GeoLineSeg( const GeoLineSeg& other );

    bool operator==(const GeoLineSeg& other) const;
    bool operator!=(const GeoLineSeg& other) const;
};

GeoLineSeg RectTrimLineSeg(const GeoRect& rect,const GeoLineSeg& lineSeg);

bool Contain( const GeoRect& rect, const GeoPoint& point );


#endif // ADNXSHAREENGINE_H

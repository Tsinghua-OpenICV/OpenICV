/*
*   Autonomous Driving Navigation Library
*   Renderer for Dynamic Display
*   Li Bing, QYJY, 2018.7
*/

#include "ADNRenderer.h"
#include <QPainter>
#include <QTime>

#define MAX_TRACK_POINTS 2048

void renderRotatedImage(QPainter *painter, QImage *im, int x, int y, int rotate)
{
    painter->translate(x,y);
    painter->rotate(rotate);
    painter->drawImage(-im->width()/2,-im->height()/2,*im);
    painter->rotate(-rotate);
    painter->translate(-x,-y);
}

ADNImage::ADNImage()
{
    m_Buffer=NULL;
}

ADNImage::ADNImage(int w, int h)
{
    Create(w,h);
}

ADNImage::~ADNImage()
{
    if(m_Buffer!=NULL)free(m_Buffer);
}

void ADNImage::Create(int w, int h)
{
    m_Width=w;
    m_Height=h;
    m_Buffer=(char*)calloc(w*h*4,sizeof(char));
}

void ADNImage::Create(int w, int h,char* buffer)
{
    m_Width=w;
    m_Height=h;
    m_Buffer=buffer;
}

char* ADNImage::GetBuffer()
{
    return m_Buffer;
}

void ADNImage::SetBuffer(char *buffer)
{
    memcpy(m_Buffer,buffer,m_Width*m_Height*4);
}

int ADNImage::Width()
{
    return m_Width;
}

int ADNImage::Height()
{
    return m_Height;
}

void ADNImage::Load(const char *path)
{
    QImage image(path);
    image=image.convertToFormat(QImage::Format_RGBA8888);
    Create(image.width(),image.height());
    memcpy(m_Buffer,image.bits(),image.byteCount());
}

ADNColor::ADNColor()
{
    R=G=B=A=255;
}

ADNColor::ADNColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
    R=r;
    G=g;
    B=b;
    A=a;
}

QPen GetPen(ADNColor color,int width,ADNRenderer::LineStyle style)
{
    QPen pen;
    pen.setColor(QColor(color.R,color.G,color.B,color.A));
    pen.setWidth(width);
    if(style==ADNRenderer::Dash)
    {
        pen.setStyle(Qt::DashLine);
    }
    else if(style==ADNRenderer::Dot)
    {
        pen.setStyle(Qt::DotLine);
    }
    return pen;
}

ADNRenderer::ADNRenderer()
{
    m_CountLayers=0;
    m_ObjectCount=0;
    m_ObjectArray=NULL;

    m_TrackBuffer=(double*)calloc(MAX_TRACK_POINTS*4,sizeof(double));
}

ADNRenderer::ADNRenderer(int width,int height)
{
    m_CountLayers=0;
    m_ObjectCount=0;
    m_ObjectArray=NULL;

    m_TrackBuffer=(double*)calloc(MAX_TRACK_POINTS*4,sizeof(double));

    Create(width,height);
}

ADNRenderer::~ADNRenderer()
{
    if(m_TrackBuffer!=NULL)free(m_TrackBuffer);
}

void ADNRenderer::Create(int width,int height)
{
    m_Width=width;
    m_Height=height;
    m_Canvas.Create(width,height);
    m_Converter.SetScreenSize(width,height);
}

void ADNRenderer::Create(int width,int height,char* buffer)
{
    m_Width=width;
    m_Height=height;
    m_Canvas.Create(width,height,buffer);
    m_Converter.SetScreenSize(width,height);
}

void ADNRenderer::Set3DParam(double center,double near,double far,double width,int vanish)
{
    m_Converter.Set3DParam(center,near,far,width,vanish);
    m_VanishHeight=vanish;
}

void ADNRenderer::AddMapLayer(ADNMapData* mapLayer,ADNColor color,int width,LineStyle style)
{
    m_MapLayers[m_CountLayers].Data=mapLayer;
    m_MapLayers[m_CountLayers].Type=Line;
    SetLayerStyle(m_CountLayers,color,width,style);
    m_CountLayers++;
}

void ADNRenderer::SetLayerStyle(int layer,ADNColor color,int width,LineStyle style)
{
    m_MapLayers[layer].Color=color;
    m_MapLayers[layer].Width=width;
    m_MapLayers[layer].Style=style;
}

char* ADNRenderer::GetImageBuffer()
{
    return m_Canvas.GetBuffer();
}

ADNCoordConverter& ADNRenderer::CoordConverter()
{
    return m_Converter;
}

void ADNRenderer::SetPosition(ADNPosition pos)
{
    m_Center=pos.Center;
    m_Heading=pos.Heading;
    m_Converter.SetCenter(m_Center,m_Heading);
}

void ADNRenderer::Refresh()
{
    QImage image((uchar*)m_Canvas.GetBuffer(),m_Canvas.Width(),m_Canvas.Height(),QImage::Format_RGBA8888);
    QPainter painter;
    if(painter.begin(&image))
    {
        QTime t;
        t.start();

        painter.setRenderHint(QPainter::Antialiasing);

        //render background
        renderBackGround(&painter);

//        renderVanishLine(&painter);

        renderLayers(&painter);

        renderTrack(&painter);

        renderTopBackGround(&painter);

        renderCar(&painter);

        renderObjects(&painter);

        renderMap(&painter);

        renderSteering(&painter);

        renderSpeedMeter(&painter);

//        QFont font;
//        font.setPointSize(10);
//        painter.setFont(font);
//        painter.drawText(5,15,QString::number(t.elapsed())+" ms");
    }
    painter.end();
}

void ADNRenderer::renderBackGround(void *painter)
{
//    ((QPainter*)painter)->fillRect(QRect(0,0,m_Width,m_Height),QBrush(Qt::black));
    QImage imBg((uchar*)m_ImageBg.GetBuffer(),m_ImageBg.Width(),m_ImageBg.Height(),QImage::Format_RGBA8888);
    ((QPainter*)painter)->drawImage(0,0,imBg);
}

void ADNRenderer::renderTopBackGround(void *painter)
{
    QImage im((uchar*)m_ImageTop.GetBuffer(),m_ImageTop.Width(),m_ImageTop.Height(),QImage::Format_RGBA8888);
    ((QPainter*)painter)->drawImage(0,0,im);
}

void ADNRenderer::renderLayers(void *painter)
{
    for(int iLayer=0;iLayer<m_CountLayers;iLayer++)
    {
        ((QPainter*)painter)->setPen(GetPen(m_MapLayers[iLayer].Color,m_MapLayers[iLayer].Width,m_MapLayers[iLayer].Style));
        for(int i=0;i<m_MapLayers[iLayer].Data->LineCount();i++)
        {
            ADNRoadLine road_line=m_MapLayers[iLayer].Data->GetLine(i);
            for(int k=0;k<road_line.Curve.Count-1;k++)
            {
                ADNPoint p1=road_line.Curve.Points[k];
                ADNPoint p2=road_line.Curve.Points[k+1];
                m_Converter.GeoToLocal(p1);
                m_Converter.GeoToLocal(p2);
                ADNLine line(p1,p2);
                if(m_Converter.LocalTo3D(line))
                {
                    ((QPainter*)painter)->drawLine(line.Source.X,line.Source.Y,line.Target.X,line.Target.Y);
                }
            }
        }
    }
}

void ADNRenderer::renderVanishLine(void *painter)
{
    ((QPainter*)painter)->setPen(QPen(QColor(60,60,60,255),2));
    int h=m_Height-m_VanishHeight;
    int w=m_Width;
    ((QPainter*)painter)->drawLine(0,h,w,h);
}

void ADNRenderer::LoadCarResource(const char *path)
{
    m_ImageCar.Load(path);
}

void ADNRenderer::LoadMapResource(const char *path)
{
    m_ImageMap.Load(path);
}

void ADNRenderer::LoadBackGround(const char *path)
{
    m_ImageBg.Load(path);
}

void ADNRenderer::LoadTopBackGround(const char *path)
{
    m_ImageTop.Load(path);
}

void ADNRenderer::LoadSteeringImage(const char *path)
{
    m_ImageSteering.Load(path);
}

void ADNRenderer::LoadSpeedMeterImage(char *pathMeter,char *pathPointer)
{
    m_ImageSpeedMeter.Load(pathMeter);
    m_ImageSpeedPointer.Load(pathPointer);
}

void ADNRenderer::LoadMapFrameImage(const char *path)
{
    m_ImageMapFrame.Load(path);
}

void ADNRenderer::renderCar(void *painter)
{
    ADNPoint pt(0,0);
    m_Converter.LocalTo3D(pt);
    int x=pt.X-m_ImageCar.Width()/2;
    int y=pt.Y-m_ImageCar.Height()/2;

    QImage imCar((uchar*)m_ImageCar.GetBuffer(),m_ImageCar.Width(),m_ImageCar.Height(),QImage::Format_RGBA8888);
    ((QPainter*)painter)->drawImage(x,y,imCar);
}

void ADNRenderer::SetMapImage(void *buffer)
{
    if(m_ImageMap.GetBuffer()==NULL)m_ImageMap.Create(m_Width,m_Height);
    QImage im((uchar*)buffer,m_Width,m_Height,QImage::Format_RGB16);
    QImage map(m_ImageMap.Width(),m_ImageMap.Height(),QImage::Format_RGBA8888);
    QPainter p(&map);

    int mapWidth=m_ImageMapFrame.Width();
    int mapHeight=m_ImageMapFrame.Height();

    p.drawImage(QRect(0,0,mapWidth,mapHeight),im,QRect(m_Width/2-mapWidth/2,m_Height/2-mapHeight/2,mapWidth,mapHeight),Qt::AvoidDither);
//    p.drawImage(QRect(0,0,mapWidth,mapHeight),im,QRect(m_Width/2-mapWidth*0.6,m_Height/2-mapHeight*0.6,mapWidth*1.2,mapHeight*1.2),Qt::AvoidDither);
    p.end();
    m_ImageMap.SetBuffer((char*)map.bits());
}

void ADNRenderer::renderMap(void *painter)
{
    int x0=m_Width-m_ImageMapFrame.Width();
    int y0=m_Height-m_ImageMapFrame.Height();

    QImage imMap((uchar*)m_ImageMap.GetBuffer(),m_ImageMap.Width(),m_ImageMap.Height(),QImage::Format_RGBA8888);
    ((QPainter*)painter)->drawImage(x0,y0,imMap);

    QImage imMapFrame((uchar*)m_ImageMapFrame.GetBuffer(),m_ImageMapFrame.Width(),m_ImageMapFrame.Height(),QImage::Format_RGBA8888);
    ((QPainter*)painter)->drawImage(x0,y0,imMapFrame);

//    ((QPainter*)painter)->setPen(QPen(QColor(80,80,80,255),1));
//    ((QPainter*)painter)->setBrush(Qt::NoBrush);
//    ((QPainter*)painter)->drawRect(m_Width-m_ImageMap.Width()-1,m_Height-m_ImageMap.Height()-1,m_ImageMap.Width(),m_ImageMap.Height());

//    ADNPoint *points=(ADNPoint*)m_TrackBuffer;
//    int count=0;
//    for(int i=0;i<m_TrackCount;i++)
//    {
//        ADNPoint pt=m_TrackData[i].Center;
//        m_Converter.GeoToMap(pt.X,pt.Y);
//        points[i]=pt;
//        count++;
//    }
//    ((QPainter*)painter)->setPen(QPen(QColor(0,200,200,255),7.5));
//    ((QPainter*)painter)->drawPolyline((QPointF*)points,count);

//    ((QPainter*)painter)->setPen(QPen(QColor(0,0,128,255)));
//    ((QPainter*)painter)->setBrush(QBrush(QColor(255,0,0,255)));
//    double x=m_Center.X;
//    double y=m_Center.Y;
//    m_Converter.GeoToMap(x,y);
//    double r=12;
//    ((QPainter*)painter)->drawEllipse(x-r/2,y-r/2,r,r);
}

void ADNRenderer::SetObjects(ADNObject *objects, int count)
{
    m_ObjectArray=objects;
    m_ObjectCount=count;
}

void ADNRenderer::renderObjects(void *painter)
{
    ((QPainter*)painter)->setPen(QPen(Qt::magenta,1));
    for(int iObj=0;iObj<m_ObjectCount;iObj++)
    {
        ADNObject obj=m_ObjectArray[iObj];
        double h0=obj.Height;
        int count=obj.Outline.Count;
        for(int k=0;k<count;k++)
        {
            ADNPoint p1=obj.Outline.Points[k];
            if(abs(p1.Y)>100||abs(p1.X)>50)break;
            ADNPoint p2=obj.Outline.Points[(k+1)%count];
            double h1=m_Converter.Get3DHeight(p1,h0);
            double h2=m_Converter.Get3DHeight(p2,h0);
            m_Converter.LocalTo3D(p1);
            m_Converter.LocalTo3D(p2);

            ((QPainter*)painter)->drawLine(p1.X,p1.Y,p2.X,p2.Y);
            ((QPainter*)painter)->drawLine(p1.X,p1.Y-h1,p2.X,p2.Y-h2);
            ((QPainter*)painter)->drawLine(p1.X,p1.Y,p1.X,p1.Y-h1);
            ((QPainter*)painter)->drawLine(p2.X,p2.Y,p2.X,p2.Y-h2);
        }
    }
}

void ADNRenderer::SetTrack(ADNPosition *track, int count)
{
    m_TrackData=track;
    m_TrackCount=count;
}

void ADNRenderer::renderTrack(void *painter)
{
    ((QPainter*)painter)->setPen(Qt::NoPen);
    ((QPainter*)painter)->setBrush(QBrush(QColor(0,255,40,128)));

    double width=50;
    double w0=m_Converter.Get3DHeight(ADNPoint(0,0),1);
    int count=0;
    ADNPoint *points=(ADNPoint*)m_TrackBuffer;
    for(int i=0;i<m_TrackCount;i++)
    {
        ADNPoint pt=m_TrackData[i].Center;
        m_Converter.GeoToLocal(pt);
        if(pt.Y<-5)
        {
            continue;
        }
        double w=m_Converter.Get3DHeight(pt,1)/w0*width;
        m_Converter.LocalTo3D(pt);
        points[MAX_TRACK_POINTS-count]=pt;
        points[MAX_TRACK_POINTS-count].X-=w/2;
        points[MAX_TRACK_POINTS+count+1]=pt;
        points[MAX_TRACK_POINTS+count+1].X+=w/2;
        count++;
    }
    ((QPainter*)painter)->drawPolygon((QPointF*)&points[MAX_TRACK_POINTS-count+1],2*count);
}

void ADNRenderer::SetVehicleStatus(int speed, int accel, int brake, int steering)
{
    m_Speed=speed;
    m_Accel=accel;
    m_Brake=brake;
    m_Steering=steering;
}

void ADNRenderer::renderSteering(void *painter)
{
    int x=m_Width*0.7,y=m_Height*0.18;
    QImage im((uchar*)m_ImageSteering.GetBuffer(),m_ImageSteering.Width(),m_ImageSteering.Height(),QImage::Format_RGBA8888);
    renderRotatedImage((QPainter*)painter,&im,x,y,m_Steering);
}

void ADNRenderer::renderSpeedMeter(void *painter)
{
    int x=m_Width*0.8,y=m_Height*0.04;
    int h=m_ImageSpeedMeter.Height()*0.88;
    int h0=m_ImageSpeedMeter.Height()*0.92;
    int h1=m_Brake*h/100;
    int h2=m_Accel*h/100;
    int w=m_ImageSpeedMeter.Width()/2;
    QImage im((uchar*)m_ImageSpeedMeter.GetBuffer(),m_ImageSpeedMeter.Width(),m_ImageSpeedMeter.Height(),QImage::Format_RGBA8888);
    ((QPainter*)painter)->fillRect(QRect(x,y+h0-h1,w,h1),QBrush(QColor(255,0,0,180)));
    ((QPainter*)painter)->fillRect(QRect(x+im.width()-w,y+h0-h2,w,h2),QBrush(QColor(0,255,0,180)));
    ((QPainter*)painter)->drawImage(x,y,im);

    int rotate=(m_Speed-99)*100/80;
    QImage imPointer((uchar*)m_ImageSpeedPointer.GetBuffer(),m_ImageSpeedPointer.Width(),m_ImageSpeedPointer.Height(),QImage::Format_RGBA8888);
    renderRotatedImage((QPainter*)painter,&imPointer,x+im.width()/2,y+imPointer.height()/2,rotate);

    int dx=m_ImageSpeedMeter.Width()*0.55;
    int dy=m_ImageSpeedMeter.Height()*0.82;
    int hfont=m_ImageSpeedMeter.Height()*0.08;
    QFont font;
    font.setPointSize(hfont);
    ((QPainter*)painter)->setFont(font);
    ((QPainter*)painter)->setPen(QColor(60,110,250));
    ((QPainter*)painter)->drawText(x+dx,y+dy,"km/h");
    font.setPointSize(hfont*2);
    ((QPainter*)painter)->setFont(font);
    QFontMetricsF fontMetrics(font);
    QString str=QString::number(m_Speed);
    qreal width = fontMetrics.width(str);
    ((QPainter*)painter)->drawText(x+dx-width,y+dy,str);
}

/*
*   Autonomous Driving Navigation Library
*   Renderer for Dynamic Display
*   Li Bing, QYJY, 2018.7
*/

#ifndef ADNRENDERER_H
#define ADNRENDERER_H

#include "ADNMapData.h"
#include "ADNCoordConverter.h"

#define MAX_LAYER_COUNT 8

// Image Class
class ADNImage
{
public:
    ADNImage();
    // construct image with width and height
    ADNImage(int w,int h);
    ~ADNImage();

    // create image buffer with width and height
    void Create(int w,int h);
    void Create(int w,int h,char* buffer);

    // load image from file
    void Load(const char *path);

    // get image bits buffer
    char* GetBuffer();

    void SetBuffer(char *buffer);

    // get width
    int Width();

    // get height
    int Height();

private:
    int m_Width;    //image width
    int m_Height;   //image height
    char* m_Buffer; //image bits buffer
};

// rgba color struct
struct ADNColor
{
    unsigned char R;    //red
    unsigned char G;    //green
    unsigned char B;    //blue
    unsigned char A;    //alpha

    ADNColor();
    ADNColor(unsigned char r,unsigned char g,unsigned char b,unsigned char a=255);
};

// renderer class
class ADNRenderer
{
public:
    // shape type
    enum ShapeType
    {
        Line    =0,
        Polygon =1
    };
    // line render style
    enum LineStyle
    {
        Solid   =0,
        Dash    =1,
        Dot     =2
    };

private:
    // map layer info
    struct MapLayer
    {
        ADNMapData* Data;   //data object
        ShapeType   Type;   //shape type
        ADNColor    Color;  //line color
        int         Width;  //line width
        LineStyle   Style;  //line style
    };

public:
    ADNRenderer();
    // construct renderer
    ADNRenderer(int width,int height);
    ~ADNRenderer();

public:
    // create with width and height
    void Create(int width,int height);
    void Create(int width,int height,char* buffer);

    // set parameters for 3D transform
    // center:   car view distance   (m)
    // near:     near view distance  (m)
    // far:      far view distance   (m)
    // width:    near view width     (m)
    // vanish:   vanish line height  (pix)
    void Set3DParam(double center,double near,double far,double width,int vanish);

    // add map layer
    void AddMapLayer(ADNMapData* mapLayer,ADNColor color=ADNColor(0,255,0),int width=1,LineStyle style=Solid);

    // set layer style
    void SetLayerStyle(int layer,ADNColor color,int width=1,LineStyle style=Solid);

    // get canvas bits buffer
    char* GetImageBuffer();

    // get coordinate converter
    ADNCoordConverter& CoordConverter();

    // set car position
    void SetPosition(ADNPosition pos);

    // set objects for display
    void SetObjects(ADNObject *objects,int count);

    // set track for distplay
    void SetTrack(ADNPosition *track, int count);


    void SetVehicleStatus(int speed, int accel, int brake, int steering);

    // refresh canvas
    void Refresh();

    // load car image resource
    void LoadCarResource(const char *path);

    // load map image resource
    void LoadMapResource(const char *path);

    void LoadBackGround(const char *path);
    void LoadTopBackGround(const char *path);

    void SetMapImage(void* buffer);
    void LoadMapFrameImage(const char *path);

    void LoadSteeringImage(const char *path);
    void LoadSpeedMeterImage(char *pathMeter,char *pathPointer);

private:
    void renderBackGround(void *painter);
    void renderTopBackGround(void *painter);
    void renderLayers(void *painter);
    void renderVanishLine(void *painter);
    void renderCar(void *painter);
    void renderMap(void *painter);
    void renderObjects(void *painter);
    void renderTrack(void *painter);

    void renderSteering(void *painter);
    void renderSpeedMeter(void *painter);

private:
    ADNImage   m_Canvas;
    int     m_Width;
    int     m_Height;
    int     m_VanishHeight;
    ADNPoint m_Center;
    ADNDouble m_Heading;
    MapLayer m_MapLayers[MAX_LAYER_COUNT];
    int     m_CountLayers;
    ADNCoordConverter m_Converter;
    ADNImage    m_ImageCar;
    ADNImage    m_ImageMap;
    ADNImage    m_ImageBg;
    ADNImage    m_ImageTop;

    ADNObject*  m_ObjectArray;
    int         m_ObjectCount;
    ADNPosition* m_TrackData;
    int         m_TrackCount;
    double*     m_TrackBuffer;

    int     m_Steering;
    int     m_Speed;
    int     m_Accel;
    int     m_Brake;
    ADNImage    m_ImageSteering;
    ADNImage    m_ImageSpeedMeter;
    ADNImage    m_ImageSpeedPointer;
    ADNImage    m_ImageMapFrame;
};

#endif // ADNRENDERER_H

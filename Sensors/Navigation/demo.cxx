/*
*   Autonomous Driving Navigation Window Demo
* JIANGK, , 2018.7
*/

#ifndef _Navifunction_H
#define _Navifunction_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
//#include "OpenICV/Data/icvPrimitiveData.hxx"
#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
//#include "QtWidgets/qapplication.h"

#include "ADNavi.h"
// #include "ADNavi/src/ADNType.h"
// #include "ADNavi/src/ADNMapData.h"
// #include "ADNavi/src/ADNRenderer.h"
// #include "ADNavi/src/ADNWindow.h"


using namespace icv;
using namespace core;
using namespace std;


class Navifunction : public icvFunction
{
public:
    Navifunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(0, 0, info) {
	             // load map layer
                 string data1="map/LAN.dat";
                 string data2="map/LBR.dat";
                 string data3="map/BAC.dat";

    dataLAN_p=boost::shared_ptr<ADNMapData>(new ADNMapData(data1.c_str()));    //lane central line
    dataLBR_p=boost::shared_ptr<ADNMapData>(new ADNMapData(data2.c_str()));    //lane boundary 
    dataBAC_p=boost::shared_ptr<ADNMapData>(new ADNMapData(data3.c_str()));  //road boundary
    //ADNMapData dataLBR("map/LBR.dat");    //lane boundary
    //ADNMapData dataBAC("map/BAC.dat");    //road boundary
    ICV_LOG_INFO<<dataLAN_p->LineCount();
    ICV_LOG_INFO<<dataLBR_p->LineCount();
    // set renderer
    //ADNRenderer renderer(width,height);
    render_p=boost::shared_ptr<ADNRenderer>(new ADNRenderer(width,height)); 
    render_p->Set3DParam(8,4,150,15,300);
    render_p->AddMapLayer(dataLAN_p.get(),ADNColor(0,255,0),1,ADNRenderer::Dot);
    render_p->AddMapLayer(dataLBR_p.get(),ADNColor(0,255,255));
//    renderer.AddMapLayer(&dataBAC,ADNColor(100,100,100),2);
                 string image1="res/car.png";
                 string image2="res/map.png";
    render_p->LoadCarResource(image1.c_str());  //car image resource
    render_p->LoadMapResource(image2.c_str());  //map image resource

    // load track for test
    trackBuffer=new ADNPosition[1024];
      trackCount=load_track(trackBuffer);
     count=trackCount-delta;

    render_p->SetTrack(trackBuffer,trackCount);

    // define view window
   // window_p=boost::shared_ptr<ADNWindow>(new ADNWindow(render_p->GetImageBuffer(),width,height)) ;

	}
    Navifunction() : Navifunction(nullptr) {
	
	
	}
int load_track(ADNPosition *track)
{
    FILE* file=fopen("track.txt","r");
    int n=0;
    if(file==NULL)
    ICV_LOG_DEBUG<<"File open error";
    else
    {
    double x=0.01;double y=0.01;double a=0.01;

    while (fscanf(file,"%lf,%lf,%lf\n",&x,&y,&a)>0) 
    {
    track[n]=ADNPosition(ADNPoint(x,y),a);// 
    // ICV_LOG_DEBUG<<x<<", "<<y<<","<<a;
       n++;
    }

    }

    return n;
}

// get object data
ADNObject getObject(ADNPosition pos,ADNCoordConverter& converter)
{
    double car_shape[]={-0.9,-2,0.9,-2,0.9,2,-0.9,2};
    converter.GeoToLocal(pos);
    double ang=pos.Heading/180*3.1415927;
    for(int i=0;i<4;i++)
    {
        double x=car_shape[2*i];
        double y=car_shape[2*i+1];
        buffer[i].X=pos.Center.X+x*cos(ang)+y*sin(ang);
        buffer[i].Y=pos.Center.Y+y*cos(ang)-x*sin(ang);
    }
    ADNObject obj;
    obj.Height=1.2;
    obj.Outline.Count=4;
    obj.Outline.Points=buffer;
    return obj;
}


  // virtual void ConfigurateInput(std::vector<icvNodeInput*>& inputPorts) override 			
  //   {
  //       ini_Input(inputPorts,0);

  //      // CheckDataType<icvDoubleData>(inputPorts[0]);
  //      // CheckDataType<icvDoubleData>(inputPorts[1]);

  //   }
  // virtual void ConfigurateOutput(std::vector<icvNodeOutput*>& outputPorts) override 
  //   {
  //     //  CheckDataType<icv::opencv::icvCvMatData>(outputPorts[0]);
	// 	        ini_Output(outputPorts,0);

		
  //   }
    virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) override
    {
buff_Input(inputPorts);
buff_Output(outputPorts); 
        
         if(pWindow==NULL){
        pWindow=new ADNWindow(render_p->GetImageBuffer(),width,height);
    }
        
        // set car position
        render_p->SetPosition(trackBuffer[i%count]);
        // set object position
        //ICV_LOG_DEBUG<<1;
        obj_=getObject(trackBuffer[i%count+delta],render_p->CoordConverter());
        render_p->SetObjects(&obj_,1);
        // refresh
        int track_points=count-i%count;
        if(track_points>300)track_points=300;
        render_p->SetTrack(&trackBuffer[i%count],track_points);



        render_p->Refresh();
       // if(i%10==3)
        pWindow->Refresh();
        //window_p->Refresh();
      //   m_App->processEvents();

        usleep(30*1000);
        i++;
    
    }
private:
ADNPoint buffer[4];
//ADNObject obj;
 int width=1000,height=500;
    int i=0;
    int delta=30;
    int count_=1;
    int trackCount,count;
    ADNWindow *pWindow=NULL;

    boost::shared_ptr<ADNRenderer> render_p;
    ADNPosition *trackBuffer;
    boost::shared_ptr<ADNWindow> window_p;
    boost::shared_ptr<ADNMapData> dataLAN_p;
    boost::shared_ptr<ADNMapData> dataLBR_p;
    boost::shared_ptr<ADNMapData> dataBAC_p;

    ADNObject obj_;
   // QApplication *m_App;

};



ICV_REGISTER_FUNCTION(Navifunction)

#endif 



#ifndef __GPS_FRAMES_H__
#define __GPS_FRAMES_H__

#include "structure_gps.h"




template <typename T>
class GpsFrame {
public:
  T* getFrameData() { return &frame_; }
  const T* getFrameData() const { return &frame_; }
  uint32_t getRoadTime() const { return rdt_; }
  void setRoadTime(uint32_t rdt) { rdt_ = rdt; }
private:
  T frame_;
  uint32_t rdt_;
};



class  GpsGgaFrame
  : public GpsFrame<trame_gga_dbl>
{
public:
  typedef trame_gga_dbl frameType;
  //typedef GenericObservable<GpsGgaFrame>::ObserverType ObserverType;
};

class  GpsGsaFrame
  : public GpsFrame<trame_gsa>
{
public:
  typedef trame_gsa frameType;
   //typedef GenericObservable<GpsGsaFrame>::ObserverType ObserverType;
};

class  GpsGstFrame
  : public GpsFrame<trame_gst>
{
public:
  typedef trame_gst frameType;
   //typedef GenericObservable<GpsGstFrame>::ObserverType ObserverType;
};

class  GpsGsvFrame
  : public GpsFrame<trame_gsv>
{
public:
  typedef trame_gsv frameType;
   //typedef GenericObservable<GpsGsvFrame>::ObserverType ObserverType;
};

class  GpsHdtFrame
  : public GpsFrame<trame_hdt>
{
public:
  typedef trame_hdt frameType;
   //typedef GenericObservable<GpsHdtFrame>::ObserverType ObserverType;
};

class  GpsRmcFrame
  : public GpsFrame<trame_rmc>
{
public:
  typedef trame_rmc frameType;
   //typedef GenericObservable<GpsRmcFrame>::ObserverType ObserverType;
};

class  GpsRotFrame
  : public GpsFrame<trame_rot>
{
public:
  typedef trame_rot frameType;
   //typedef GenericObservable<GpsRotFrame>::ObserverType ObserverType;
};

class  GpsVtgFrame
  : public GpsFrame<trame_vtg>
{
public:
  typedef trame_vtg frameType;
   //typedef GenericObservable<GpsVtgFrame>::ObserverType ObserverType;
};

class  GpsZdaFrame
  : public GpsFrame<trame_zda>
{
public:
  typedef trame_zda frameType;
   //typedef GenericObservable<GpsZdaFrame>::ObserverType ObserverType;
};

class  GpsSynchro
  : public GpsFrame<GpsSynchroFrame>
{
public:
  typedef GpsSynchroFrame frameType;
   //typedef GenericObservable<GpsSynchro>::ObserverType ObserverType;
};

class  GpsPps
  : public GpsFrame<uint32_t>
{
public:
  typedef uint32_t frameType;
   //typedef GenericObservable<GpsPps>::ObserverType ObserverType;
};

class GpsFrames {
public:
  GpsGgaFrame ggaFrame;
  GpsGsaFrame gsaFrame;
  GpsGstFrame gstFrame;
  GpsGsvFrame gsvFrame;
  GpsHdtFrame hdtFrame;
  GpsRmcFrame rmcFrame;
  GpsRotFrame rotFrame;
  GpsVtgFrame vtgFrame;
  GpsZdaFrame zdaFrame;
  GpsSynchro  synchroFrame;
  GpsPps      ppsFrame;
};


#endif

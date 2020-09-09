#include "sensor/dataFrame.h"


#include <string.h>


DataFrame::DataFrame( const char * frame, int length, road_time_t frameTime, road_timerange_t frameTimerange ) 
: length_ (length), time_ (frameTime), timerange_(frameTimerange) 
{  
  data_ = new char[length]; 
  memcpy(data_, frame, length); 
}



DataFrame::~DataFrame () 
{
  if (data_ != NULL)
    delete[] data_; 
}

/*
int DataFrame::getFrame(const char * frame)
{
 return 0; 
}


road_time_t DataFrame::getTime()
{
  return 0; 
}


road_timerange_t DataFrame::getTimerange()
{
  return 0; 
}
*/

/*  QString getFrame() const { return data; }
  road_time_t getFrameTime() const { return time; }
  road_timerange_t getFrameTimerange() const { return timerange; }
  void setFrame(QString const frame) { data = frame; }  
  void setFrameTime(road_time_t const t) { time = t;}
  void setFrameTimerange(road_timerange_t const tr) { timerange = tr; }
*/
/*

DataFrame& DataFrame::operator= (const DataFrame& frame)
{
  

 // f->time_ = time_; 
/*

  char * data;  = new char[frame.getFrame()] ; 

  f->data_*/
  
  

/*
  const string & string:: opérator =(const string &s) {

delete [] mystr;
mylen=s.mylen;
mystr=new char [mylen+1];
strcpy (mystr,s.mystr);
return *this;
}
}


//dest = source;*/
#include "dataFrame.h"


#include <string.h>


DataFrame::DataFrame( const char * frame, int length, uint32_t frameTime, uint32_t frameTimerange ) 
: length_ (length)
{  
  data_ = new char[length]; 
  memcpy(data_, frame, length); 
  time_=frameTime;
  timerange_=frameTimerange;
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


uint32_t DataFrame::getTime()
{
  return 0; 
}


uint32_t DataFrame::getTimerange()
{
  return 0; 
}
*/

/*  std::string getFrame() const { return data; }
  uint32_t getFrameTime() const { return time; }
  uint32_t getFrameTimerange() const { return timerange; }
  void setFrame(std::string const frame) { data = frame; }  
  void setFrameTime(uint32_t const t) { time = t;}
  void setFrameTimerange(uint32_t const tr) { timerange = tr; }
*/
/*

DataFrame& DataFrame::operator= (const DataFrame& frame)
{
  

 // f->time_ = time_; 
/*

  char * data;  = new char[frame.getFrame()] ; 

  f->data_*/
  
  

/*
  const string & string:: op�rator =(const string &s) {

delete [] mystr;
mylen=s.mylen;
mystr=new char [mylen+1];
strcpy (mystr,s.mystr);
return *this;
}
}


//dest = source;*/
//! Definition of the DataFrame class 
/*!
* Description
* 
* @author G�rald Dherbomez
* @date 28/09/2005
*/

#ifndef _DATAFRAME_H_
#define _DATAFRAME_H_


#include "icvTime.h"


class DataFrame
{
public:
  DataFrame() {data_ = NULL; length_ = 0; time_ = 0; timerange_ = 0;}
  DataFrame( const char * frame, int length, road_time_t frameTime, road_timerange_t frameTimerange );
  ~DataFrame (); 

// a faire : definir l'operateur egal
//  DataFrame& operator= (const DataFrame& frame); 

  /*
  int getFrame(const char * frame); 
  road_time_t getTime(); 
  road_timerange_t getTimerange();
  */

  int length_; 
  char * data_; 
  road_time_t time_; 
  road_timerange_t timerange_;




};

#endif
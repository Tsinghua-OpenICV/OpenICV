//! Definition of the DataFrame class 
/*!
* Description
* 
* @author G�rald Dherbomez
* @date 28/09/2005
*/

#ifndef _DATAFRAME_H_
#define _DATAFRAME_H_


#include "OpenICV/Core/icvTime.h"


class DataFrame
{
public:
  DataFrame() {data_ = NULL; length_ = 0; time_ = 0; timerange_ = 0;}
  DataFrame( const char * frame, int length, uint32_t frameTime, uint32_t frameTimerange );
  ~DataFrame (); 

// a faire : definir l'operateur egal
//  DataFrame& operator= (const DataFrame& frame); 

  /*
  int getFrame(const char * frame); 
  uint32_t getTime(); 
  uint32_t getTimerange();
  */

  int length_; 
  char * data_; 
  uint32_t time_; 
  uint32_t timerange_;




};

#endif
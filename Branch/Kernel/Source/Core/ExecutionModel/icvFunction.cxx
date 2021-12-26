#include "OpenICV/Core/icvFunction.h"

namespace icv { namespace core
{
   
    
    void icvFunction::RegisterSuspend(const icv_function<void(Duration64)>& func)
    {
        _sleepFunc = func;
    }


    
long icvFunction::get_loop_time(bool nanoseconds){
        count_time++;
       // ICV_LOG_INFO<<"count"<<count_time;

       if (nanoseconds) time_seq.insert( time_seq.begin(), icvTime::time_ns());
       else time_seq.insert( time_seq.begin(), icvTime::time_us());
                  long gap_once=long(time_seq.front()-time_seq.back());

        if(count_time>1)
        {
       
            
            loop_seq.insert(loop_seq.begin(), gap_once);

            time_seq.pop_back();

            long time_sum_;
             if(count_time>10)
            //  if(false)
            {
            time_sum_=vectorSum(loop_seq);

            long gap_ave=time_sum_/long(loop_seq.size());

             //ICV_LOG_INFO<<"JK5.3  "<<time_sum_/loop_seq.size();
            loop_seq.pop_back();
            return gap_ave;


             }
             else
             {
 

                ///time_sum_=vectorSum(loop_seq);

                
                return gap_once ;
            
            }
            



        }
        else   return time_t(0);



    }
















}}

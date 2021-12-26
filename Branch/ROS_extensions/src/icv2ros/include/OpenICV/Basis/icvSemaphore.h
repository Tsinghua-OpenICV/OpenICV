#ifndef icvSemaphore_h
#define icvSemaphore_h

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvDataObject.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#define ICV_FUNCTION
#include "OpenICV/Core/icvConfig.h"
#undef ICV_FUNCTION

#define ICV_SLEEP_CYCLE 10000

namespace icv { 

using namespace std;
    class icvSemaphore 
    {
    public:
        explicit icvSemaphore(int count=1) : count_(count),threadnum_(count) {

        }


         void Lock() {

        boost::unique_lock<boost::mutex> lock(mutex_);
        if(count_==0) condition_.wait(lock);
        if(count_>0)--count_;

        };
         void Lock(int ms) {
            assert(ms>0);
        boost::unique_lock<boost::mutex> lock(mutex_);
        if(!(count_>0))
        
        {

            boost::cv_status stat= condition_.wait_for(lock,boost::chrono::milliseconds(ms));
        
            if(stat==boost::cv_status::timeout)
            {

                count_=threadnum_;
                condition_.notify_all();

            }

         }
        

        };
         void Release(){
        boost::unique_lock<boost::mutex> lock(mutex_);

        if(count_<threadnum_)++count_;
        condition_.notify_one();

        } ;
        void Release_all(){
        boost::unique_lock<boost::mutex> lock(mutex_);

        count_=threadnum_;
        condition_.notify_all();

        } ;

    private:

    int count_;
    int threadnum_;
    boost::mutex mutex_;
    boost::condition_variable condition_;
    };
}

#endif // icvAtomicNode_h

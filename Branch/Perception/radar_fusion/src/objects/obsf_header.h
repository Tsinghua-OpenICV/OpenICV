#ifndef OBSF_HEADER_H
#define OBSF_HEADER_H

#include <string>

namespace tsinghua{
namespace dias {
namespace fusion{

class OBSFTime{
public:
    double _sec;
    double _nsec;
    const double to_second()const {
        return (double)_sec + (double)_nsec / (double)(1000000000);
    }
    const double to_millisecond()const {
        return (double)_sec * 1000 + (double)_nsec / (double)(1000000);
    }
};

class OBSFHeader
{
public:
    unsigned int _seq;
    OBSFTime _stamp;
    std::string _frame_id; 
};

}//
}//
}//


#endif
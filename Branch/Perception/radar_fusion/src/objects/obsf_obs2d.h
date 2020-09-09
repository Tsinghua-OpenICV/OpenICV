#ifndef OBSF_OBS2D_H
#define OBSF_OBS2D_H

#include "obsf_header.h"

namespace tsinghua{
namespace dias{
namespace fusion{


class OBSFObs2d{
public:
    OBSFObs2d(){}
    ~OBSFObs2d(){}
    
public:
    OBSFHeader _header;
    int   _obs_id;
    
    float _bbox_left;
    float _bbox_top;
    float _bbox_right;
    float _bbox_bottom; 
    
    float _truncated;
    
    short _occluded;
    float _life;
    
    short _classification;
};

}//
}//
}//

#endif
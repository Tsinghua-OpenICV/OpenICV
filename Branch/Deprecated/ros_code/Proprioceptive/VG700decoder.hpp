#ifndef VG700DECODER_HPP
#define VG700DECODER_HPP

#include <deque> 
#include <vector>

#include "kernel/road_time.h"
#include "structure/structureXbow.h"
#include "XbowComponentConfig.h"

class XBOWCOMPONENT_API VG700decoder
{
public:
    std::vector<VG700dataframe> decode(char *msg, size_t msg_len, road_time_t time);

private:
    class timestamped_uchar {
    public:
        timestamped_uchar(unsigned char c_, double t_)
            : c(c_)
            , t(t_)
        {}

        unsigned char c;
        double t;
    };
    std::deque<timestamped_uchar> databuf;
};

#endif // VG700DECODER_HPP

#include "VG700decoder.hpp"

#include <deque> 
#include <vector>

using namespace std;

vector<VG700dataframe> VG700decoder::decode(char *msg, size_t msg_len, road_time_t time)
{
    // Vector of decoded frames
    vector<VG700dataframe> frames;
    
    // VG700 dataframe to store decoded values
    VG700dataframe frame;
    
    // Add received bytes to the processing buffer
    for (size_t i = 0; i < msg_len; ++i) {
        databuf.push_back(timestamped_uchar((unsigned char) msg[i], time));
    }
    
    // Process bytes in the buffer
    while (!databuf.empty())
    {
        // A message must start by 0xFF, so we discard other leading bytes
        if (databuf[0].c != 0xFF)
        {
            databuf.pop_front();
            continue;
        }
        // Message length is 22 bytes, we stop if there is not enough data to process
        if (databuf.size() < 22)
            break;
        
        // Time of message is the header's receive time
        frame.message_time = databuf[0].t;
        // Discards the first byte (which is 0xFF)
        databuf.pop_front();
        
        // Compute checksum
        // There is a mistake in the VG700 user manual :
        //    the checksum is obviously computed as a modulo 256 sum
        //    and not modulo 255 as stated in the manual.
        unsigned char chksum = 0;
        for (unsigned int i=0; i<20; i++)
            chksum += databuf[i].c;
        // We probably found a packet
        if (chksum == databuf[20].c)
        {
            // Scaling factors
            const double angleScale = 180./32768.;
            const double rateScale  = 200.*1.5/32768.;
            const double accelScale = 4.0*1.5/32768.;
            
            // Store decoded values in the frame structure
            frame.rollAngle  = angleScale * (short)(((unsigned short)databuf[0].c)<<8 | databuf[1].c);
            frame.pitchAngle = angleScale * (short)(((unsigned short)databuf[2].c)<<8 | databuf[3].c);
            frame.rollRate  = rateScale * (short)(((unsigned short)databuf[4].c)<<8 | databuf[5].c);
            frame.pitchRate = rateScale * (short)(((unsigned short)databuf[6].c)<<8 | databuf[7].c);
            frame.yawRate   = rateScale * (short)(((unsigned short)databuf[8].c)<<8 | databuf[9].c);
            frame.accelX  = accelScale * (short)(((unsigned short)databuf[10].c)<<8 | databuf[11].c);
            frame.accelY  = accelScale * (short)(((unsigned short)databuf[12].c)<<8 | databuf[13].c);
            frame.accelZ  = accelScale * (short)(((unsigned short)databuf[14].c)<<8 | databuf[15].c);
            frame.temp = 44.4 * (5./4096. * (short)(((unsigned short)databuf[16].c)<<8 | databuf[17].c) - 1.375);
            frame.embedded_time = ((unsigned short)databuf[18].c)<<8 | databuf[19].c;
            
            // Push frame in the return value vector
            frames.push_back(frame);
                            
            // Remove decoded packet from buffer
            for (unsigned int i=0; i<21; i++)
                databuf.pop_front();
        }
    }
    return frames;
}


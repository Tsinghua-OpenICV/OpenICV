#ifndef OBSF_OBS_FRAME_H
#define OBSF_OBS_FRAME_H
#include <vector>
#include <map>
#include "obsf_obs2d.h"
#include "obsf_obs3d.h"
#include "obsf_define.h"

namespace tsinghua{
namespace dias{
namespace fusion{

template<typename Type>
class OBSFObsFrame{
public:
    typedef Type OBSType;
    
public:
    OBSFObsFrame(){
    }
    ~OBSFObsFrame(){
    }
    
    inline void set_obstacles(const std::vector<OBSType>& obstacles){
        _obstacles = obstacles;
    }

    inline std::vector<OBSType>& get_obstacles() {
        return _obstacles;
    }
    
    inline const std::vector<OBSType>& get_obstacles() const {
        return _obstacles;
    }
    
    OBSType* get_obstacle(int obs_id){
        std::map<int, int>::iterator it = _obs_id_g2l.find(obs_id);
        if (it == _obs_id_g2l.end()){
            return nullptr;
        }
        int local_id = it->second;
        return &(_obstacles[local_id]);
    }
    
    const OBSType* get_obstacle(int obs_id) const {
        std::map<int, int>::iterator it = _obs_id_g2l.find(obs_id);
        if (it == _obs_id_g2l.end()){
            return nullptr;
        }
        
        int local_id = it->second;
        return &(_obstacles[local_id]);
    }
    
    inline void set_header(const OBSFHeader &header)
    {
        _header = header;
    }

    inline OBSFHeader& get_header()
    {
        return _header;
    }

    inline const OBSFHeader& get_header() const
    {
        return _header;
    }
    
    void initialize(OBSFHeader &header, std::vector<OBSType>& obstacles){
        _header = header;
        _obstacles.assign(obstacles.begin(), obstacles.end());
        build_g2l_map();    
    }
    
    void build_g2l_map() {
        _obs_id_g2l.clear();
        for (int i = 0; i < (int)_obstacles.size(); i++){
            _obs_id_g2l[_obstacles[i]._obs_id] = i;
        }
    }
    
private:
    OBSFHeader _header;
    std::vector<OBSType>    _obstacles;
    
    /*map from global obs id to local index*/
    std::map<int, int>      _obs_id_g2l;    
    
    //OBSFPose                _pose;
};

typedef OBSFObsFrame<OBSFObs2d>                OBSFObs2dFrame;
typedef OBSFObsFrame<OBSFObs3d>                OBSFObs3dFrame;

}//
}//
}//
#endif
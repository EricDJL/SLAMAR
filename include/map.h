#ifndef MAP_INCLUDE_H
#define MAP_INCLUDE_H
#include "common_include.h"
#include "mappoint.h"
#include "frame.h"
namespace slamAR
{
    class Map
    {
    public:
        unordered_map<unsigned long,Frame::Ptr> m_keyframe;
        unordered_map<unsigned long,MapPoint::Ptr> m_mappoint;
        
        typedef shared_ptr<Map> Ptr;
        
        Map();
        
        void insertKeyFrame(Frame::Ptr frame);
        void insertMapPoint(MapPoint::Ptr mappoint);
        
    };
}
#endif

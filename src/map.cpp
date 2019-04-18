#include "map.h"
namespace slamAR
{
    Map::Map(){}
    void Map::insertKeyFrame(Frame::Ptr frame)
    {
        cout<<"keyframe size:"<<m_keyframe.size()<<endl;
        if(m_keyframe.find(frame->getId())==m_keyframe.end())
        {
            m_keyframe.insert(make_pair(frame->getId(),frame));
        }
        else
        {
            m_keyframe[frame->getId()]=frame;
        }
        //cout<<"keyframe size:"<<m_keyframe.size()<<endl;
    }
    void Map::insertMapPoint(MapPoint::Ptr mappoint)
    {
        if(m_mappoint.find(mappoint->getId())==m_mappoint.end())
        {
            m_mappoint.insert(make_pair(mappoint->getId(),mappoint));
        }
        else
        {
            m_mappoint[mappoint->getId()]=mappoint;
        }
    }
}

#include "mappoint.h"
namespace slamAR
{
    MapPoint::MapPoint():
    m_id(-1),m_pw(Vector3d(0,0,0)),m_pn(Vector3d(0,0,0)),m_observedtimes(0),m_correcttimes(0)
    {
        
    }
    MapPoint::MapPoint(long id,Vector3d pw,Vector3d pn)
    {
        
    }
    MapPoint::MapPoint(long id ,Vector3d pw,Vector3d pn,int observedtimes,int correcttimes):
    m_id(id),m_pw(pw),m_pn(pn),m_observedtimes(observedtimes),m_correcttimes(correcttimes)
    {
        
    }
        
    MapPoint::Ptr MapPoint::createMapPoint()
    {
        long id=0;
        return MapPoint::Ptr(new MapPoint(id++,Vector3d(0,0,0),Vector3d(0,0,0)));
    }
        
    void MapPoint::setId(long id)
    {
        m_id=id;
    }
    long MapPoint::getId()
    {
        return m_id;
    }
    void MapPoint::setPw(Vector3d pw)
    {
        m_pw=pw;
    }
    Vector3d MapPoint::getPw()
    {
        return m_pw;
    }
    void MapPoint::setPn(Vector3d pn)
    {
        m_pn=pn;
    }
    Vector3d MapPoint::getPn()
    {
        return m_pn;
    }
    void MapPoint::setDescriptor(Mat descriptor)
    {
        m_descriptor=descriptor;
    }
    Mat MapPoint::getDescriptor()
    {
        return m_descriptor;
    }
    void MapPoint::setObservedTimes(int observedtimes)
    {
        m_observedtimes=observedtimes;
    }
    int MapPoint::getObservedTimes()
    {
        return m_observedtimes;
    }
    void MapPoint::setCorrectTimes(int correcttimes)
    {
        m_correcttimes=correcttimes;
    }
    int MapPoint::getCorrectTimes()
    {
        return m_correcttimes;
    }
}

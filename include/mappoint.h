#ifndef MAPPOINT_INCLUDE_H
#define MAPPOINT_INCLUDE_H
#include "common_include.h"
namespace slamAR
{
    class MapPoint
    {
    private:
        unsigned long m_id;
        Vector3d m_pw;
        Vector3d m_pn;
        Mat m_descriptor;
        int m_observedtimes;
        int m_correcttimes;
        
    public:
        typedef shared_ptr<MapPoint> Ptr;
        
        MapPoint();
        MapPoint(long id,Vector3d pw,Vector3d pn);
        MapPoint(long id ,Vector3d pw,Vector3d pn,int observedtimes,int correcttimes);
        
        MapPoint::Ptr createMapPoint();
        
        void setId(long id);
        long getId();
        void setPw(Vector3d pw);
        Vector3d getPw();
        void setPn(Vector3d pn);
        Vector3d getPn();
        void setDescriptor(Mat descriptor);
        Mat getDescriptor();
        void setObservedTimes(int observedtimes);
        int getObservedTimes();
        void setCorrectTimes(int correcttimes);
        int getCorrectTimes();
    };
}
#endif

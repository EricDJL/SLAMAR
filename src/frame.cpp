#include "frame.h"
namespace slamAR
{
    Frame::Frame():m_id(-1),m_timestamp(-1),m_camera(nullptr){}
    Frame::Frame(long id):m_id(id){}
    Frame::Frame(long id,double timestamp,SE3 Tcw,Camera::Ptr camera,Mat color,Mat depth):
    m_id(id),m_timestamp(timestamp),m_camera(camera),m_color(color),m_depth(depth){}
    Frame::~Frame(){}
        
    //create frame
    Frame::Ptr Frame::createFrame()
    {
        static long factory_id=0;
        return Frame::Ptr(new Frame(factory_id++));
    }
        
    //find depth
    double Frame::findDepth(const cv::KeyPoint kp)
    {
        int x=cvRound(kp.pt.x);
        int y=cvRound(kp.pt.y);
        ushort d=m_depth.ptr<ushort>(y)[x];
        if(d!=0)
            return double(d)/m_camera->getDepthScale();
        else
        {
            int dx[4]={-1,0,1,0};
            int dy[4]={0,1,0,-1};
            for(int i=0;i<4;i++)
            {
                d=m_depth.ptr<ushort>(y+dy[i])[x+dx[i]];
                if(d!=0)
                    return double(d)/m_camera->getDepthScale();
            }
        }
        return -1.0;
    }
        
    //get center camera
    Vector3d Frame::getCenterCam()
    {
        return m_Tcw.inverse().translation();
    }
        
    //judge whether in frame
    bool Frame::isInFrame(const Vector3d pw)
    {
        Vector3d pc=m_camera->world2cam(pw,m_Tcw);
        if(pc(2,0)>=0)
            return true;
        Vector2d p=m_camera->world2piexl(pw,m_Tcw);
        if(p(0,0)>0&&p(1,0)>0&&p(0,0)<m_color.cols&&p(1,0)<m_color.rows)
            return true;
        return false;
    }
    
    //Data encapsulation
    void Frame::setId(const long id)
    {
        m_id=id;
    }
    long Frame::getId()
    {
        return m_id;
    }
    void Frame::setTimeStamp(const float timestamp)
    {
        m_timestamp=timestamp;
    }
    double Frame::getTimeStamp()
    {
        return m_timestamp;
    }
    void Frame::setTcw(const SE3 Tcw)
    {
        m_Tcw=Tcw;
    }
    SE3 Frame::getTcw()
    {
        return m_Tcw;
    }
    void Frame::setColor(const Mat color)
    {
        m_color=color;
    }
    Mat Frame::getColor()
    {
        return m_color;
    }
    void Frame::setDepth(const Mat depth)
    {
        m_depth=depth;
    }
    Mat Frame::getDepth()
    {
        return m_depth;
    }
}

#ifndef FRAME_INCLUDE_H
#define FRAME_INCLUDE_H
#include "common_include.h"
#include "camera.h"
namespace slamAR
{
    class Frame
    {
    private:
        unsigned long m_id;
        double m_timestamp;
        SE3 m_Tcw;
        Mat m_color,m_depth;
        
    public:
        typedef std::shared_ptr<Frame> Ptr;
        Camera::Ptr m_camera;
        Frame();
        Frame(long id);
        Frame(long id,double timestamp,SE3 Tcw,Camera::Ptr camera,Mat color,Mat depth);
        ~Frame();
        
        //create frame
        static Frame::Ptr createFrame();
        
        //find depth
        double findDepth(const cv::KeyPoint kp);
        
        //get center camera
        Vector3d getCenterCam();
        
        //judge whether in frame
        bool isInFrame(const Vector3d pw);
        
        //Data encapsulation
        void setId(const long id);
        long getId();
        void setTimeStamp(const float timestamp);
        double getTimeStamp();
        void setTcw(const SE3 Tcw);
        SE3 getTcw();
        void setColor(const Mat color);
        Mat getColor();
        void setDepth(const Mat depth);
        Mat getDepth();
    };
}
#endif

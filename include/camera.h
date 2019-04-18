#ifndef CAMERA_INCLUDE_H
#define CAMERA_INCLUDE_H
#include "common_include.h"
namespace slamAR
{
    class Camera
    {
    private:
        float m_fx,m_fy,m_cx,m_cy,m_depthscale;
    public:
        typedef std::shared_ptr<Camera> Ptr;
        
        //struct function
        Camera();
        ~Camera();
        
        //Data encapsulation
        void setFx(float fx);
        float getFx();
        void setFy(float fy);
        float getFy();
        void setCx(float cx);
        float getCx();
        void setCy(float cy);
        float getCy();
        void setDepthScale(float depth);
        float getDepthScale();
        
        //coordinate trans
        Vector3d world2cam(const Vector3d pw,const SE3 Tcw);
        Vector3d cam2world(const Vector3d pc,const SE3 Tcw);
        Vector2d cam2piexl(const Vector3d pc);
        Vector3d piexl2cam(const Vector2d p,const double depth);
        Vector3d piexl2world(const Vector2d p,const SE3 Tcw,const double depth);
        Vector2d world2piexl(const Vector3d pw,const SE3 Tcw);
        
    };
}
#endif

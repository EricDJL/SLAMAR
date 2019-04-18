#include "camera.h"
#include "config.h"
namespace slamAR
{
    Camera::Camera()
    {
        m_fx=Config::get<float>("camera.fx");
        m_fy=Config::get<float>("camera.fy");
        m_cx=Config::get<float>("camera.cx");
        m_cy=Config::get<float>("camera.cy");
        m_depthscale=Config::get<float>("camera.depth_scale");
    }
    Camera::~Camera(){}
    void Camera::setFx(float fx)
    {
        m_fx=fx;
    }
    float Camera::getFx()
    {
        return m_fx;
    }
    void Camera::setFy(float fy)
    {
        m_fy=fy;
    }
    float Camera::getFy()
    {
        return m_fy;
    }
    void Camera::setCx(float cx)
    {
        m_cx=cx;
    }
    float Camera::getCx()
    {
        return m_cx;
    }
    void Camera::setCy(float cy)
    {
        m_cy=cy;
    }
    float Camera::getCy()
    {
        return m_cy;
    }
    void Camera::setDepthScale(float depthscale)
    {
        m_depthscale=depthscale;
    }
    float Camera::getDepthScale()
    {
        return m_depthscale;
    }
        
        //coordinate trans
    Vector3d Camera::world2cam(const Vector3d pw,const SE3 Tcw)
    {
        return Tcw*pw;
    }
    Vector3d Camera::cam2world(const Vector3d pc,const SE3 Tcw)
    {
        return Tcw.inverse()*pc;
    }
    Vector2d Camera::cam2piexl(const Vector3d pc)
    {
        return Vector2d((m_fx*pc(0,0)/pc(2,0)+m_cx),(m_fy*pc(1,0)/pc(2,0)+m_cy));
    }
    Vector3d Camera::piexl2cam(const Vector2d p,const double depth)
    {
        return Vector3d((p(0,0)-m_cx)*depth/m_fx,(p(1,0)-m_cy)*depth/m_fy,depth);
    }
    Vector3d Camera::piexl2world(const Vector2d p,const SE3 Tcw,const double depth)
    {
        return cam2world(piexl2cam(p,depth),Tcw);
    }
    Vector2d Camera::world2piexl(const Vector3d pw,const SE3 Tcw)
    {
        cam2piexl(world2cam(pw,Tcw));
    }
    
}

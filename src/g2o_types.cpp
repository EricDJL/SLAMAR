#include "g2o_types.h"

namespace slamAR
{
  
    void EdgeProjectXYZ2UVPoseOnly::computeError()
    {
        const g2o::VertexSE3Expmap* pose=static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        _error=_measurement-m_camera->cam2piexl(pose->estimate().map(m_point));
    }
    
    void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose=static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
        g2o::SE3Quat p(pose->estimate());
        Vector3d trans_point=p.map(m_point);
        
        double x=trans_point[0];
        double y=trans_point[1];
        double z=trans_point[2];
        double z_2=z*z;
        
        float fx=m_camera->getFx();
        float fy=m_camera->getFy();
        
        _jacobianOplusXi(0,0)=fx*x*y/z_2;
        _jacobianOplusXi(0,1)=-(fx+fx*x*x/z_2);
        _jacobianOplusXi(0,2)=fx*y/z;
        _jacobianOplusXi(0,3)=-fx/z;
        _jacobianOplusXi(0,4)=0;
        _jacobianOplusXi(0,5)=fx*x/z_2;
        
        _jacobianOplusXi(1,0)=fy+fy*y*y/z_2;
        _jacobianOplusXi(1,1)=-fy*x*y/z_2;
        _jacobianOplusXi(1,2)=-fy*x/z;
        _jacobianOplusXi(1,3)=0;
        _jacobianOplusXi(1,4)=-fy/z;
        _jacobianOplusXi(1,5)=fy*y/z_2;
    }
    

}

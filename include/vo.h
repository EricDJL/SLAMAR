#ifndef V0_INCLUDE_H
#define VO_INCLUDE_H
#include "common_include.h"
#include "map.h"
using namespace cv;
namespace slamAR
{
    class VO
    {
    public:
        typedef shared_ptr<VO> Ptr;
        enum VOState{
            Initialization=-1,
            OK=1,
            Lost
        };
        
        VOState m_state;
        Map::Ptr m_map;
        Frame::Ptr m_ref;
        Frame::Ptr m_curr;
        
        cv::Ptr<ORB> m_orb;
        vector<Point3f> m_ref3d;
        vector<KeyPoint> m_curr_keypoint;
        Mat m_ref_descriptor;
        Mat m_curr_descriptor;
        vector<cv::DMatch> m_feature_match;
        
        SE3 m_Tcw_estimated;
        int m_num_inliers;
        int m_num_lost;
        
        int m_num_features;
        double m_scale_factor;
        int m_level;
        float m_match_ratio;
        int m_max_numlost;
        int m_min_inliers;
        
        double m_min_frame_rot;
        double m_min_frame_trans;
        
        VO();
        ~VO();
        bool addFrame(Frame::Ptr frame);
        
    protected:
        void extractKeyPoints();
        void computerDescriptor();
        void featureMatch();
        void poseEstimatePnP();
        void setRef3DPoints();
        
        bool isEstimatePose();
        bool isKeyFrame();
        void addKeyFrame();
        
    };
}
#endif

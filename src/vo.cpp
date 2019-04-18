#include "vo.h"
#include "config.h"
#include "g2o_types.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>
#include <chrono>
using namespace cv;
namespace slamAR
{
    VO::VO():m_state(Initialization),m_map(new Map),
    m_ref(nullptr),m_curr(nullptr),m_num_inliers(0),m_num_lost(0)
    {
        m_num_features=Config::get<int>("number_of_features");
        m_scale_factor=Config::get<double>("scale_factor");
        m_level=Config::get<int>("level");
        m_match_ratio=Config::get<float>("match_ratio");
        m_max_numlost=Config::get<int>("max_num_lost");
        m_min_inliers=Config::get<int>("min_inliers");
        
        double m_min_frame_rot=Config::get<double>("min_frame_rot");
        double m_min_frame_trans=Config::get<double>("min_frame_trans");
        m_orb=cv::ORB::create(m_num_features,m_scale_factor,m_level);
    }
    VO::~VO()
    {
        
    }
    
    void VO::extractKeyPoints()
    {
        m_orb->detect(m_curr->getColor(),m_curr_keypoint);
    }
    void VO::computerDescriptor()
    {
        m_orb->compute(m_curr->getColor(),m_curr_keypoint,m_curr_descriptor);
    }
    void VO::featureMatch()
    {
        /*
        vector<DMatch> matches;
        BFMatcher matcher(NORM_HAMMING);
        matcher.match(m_ref_descriptor,m_curr_descriptor,matches);
        
        double min_dist=1000;
        double max_dist=0;
        for(int i=0;i<m_ref_descriptor.rows;i++)
        {
            double dist=matches[i].distance;
            if(dist<min_dist)
                min_dist=dist;
            if(dist>max_dist)
                max_dist=dist;
            
            for(int i=0;i<m_ref_descriptor.rows;i++)
            {
                if(matches[i].distance<max(2*min_dist,30.0))
                    m_feature_match.push_back(matches[i]);
            }
            cout<<"good matches"<<m_feature_match.size()<<endl;
        }
        */
        vector<cv::DMatch> matches;
        cv::BFMatcher matcher ( cv::NORM_HAMMING );
        matcher.match ( m_ref_descriptor, m_curr_descriptor, matches );
        float min_dis = std::min_element (
        matches.begin(), matches.end(),
        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
        {
            return m1.distance < m2.distance;
        } )->distance;

        m_feature_match.clear();
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*m_match_ratio, 30.0 ) )
            {
                m_feature_match.push_back(m);
            }
        }
        cout<<"good matches: "<<m_feature_match.size()<<endl;
    }
    void VO::poseEstimatePnP()
    {
        vector<Point3f> pts3d;
        vector<Point2f> pts2d;
        for(DMatch m:m_feature_match)
        {
            pts3d.push_back(m_ref3d[m.queryIdx]);
            pts2d.push_back(m_curr_keypoint[m.trainIdx].pt);
        }
        Mat K=(Mat_<double>(3,3)<<
        m_ref->m_camera->getFx(),0,m_ref->m_camera->getCx(),
        0,m_ref->m_camera->getFy(),m_ref->m_camera->getCy(),
        0,0,1
        );
        Mat r,t,inliers;
        solvePnPRansac(pts3d,pts2d,K,Mat(),r,t,false,100,4.0,0.99,inliers);
        m_num_inliers=inliers.rows;
        cout<<"pnp inliers:"<<m_num_inliers<<endl;
        m_Tcw_estimated=Sophus::SE3(
            Sophus::SO3(r.at<double>(0,0),r.at<double>(1,0),r.at<double>(2,0)),
            Vector3d(t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0))
        );
        
       typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> block;
       block::LinearSolverType* linearSolver=new g2o::LinearSolverDense<block::PoseMatrixType>();
       block* solver_ptr=new block(linearSolver);
       
       g2o::OptimizationAlgorithmLevenberg* solver=new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
       g2o::SparseOptimizer optimizer;
       optimizer.setAlgorithm(solver);
       
       g2o::VertexSE3Expmap* pose=new g2o::VertexSE3Expmap();
       pose->setEstimate(g2o::SE3Quat(m_Tcw_estimated.rotation_matrix(),m_Tcw_estimated.translation()));
       pose->setId(0);
       
       optimizer.addVertex(pose);
       
       for(int i=0;i<inliers.rows;i++)
       {
           EdgeProjectXYZ2UVPoseOnly* edge=new EdgeProjectXYZ2UVPoseOnly();
           edge->setVertex(0,pose);
           edge->setId(i);
           
           edge->m_camera=m_curr->m_camera.get();
           int index=inliers.at<int>(i,0);
           edge->m_point=Vector3d(pts3d[index].x,pts3d[index].y,pts3d[index].z);
           
           edge->setMeasurement(Vector2d(pts2d[index].x,pts2d[index].y));
           edge->setInformation(Eigen::Matrix2d::Identity());
           
           optimizer.addEdge(edge);
       }
       
       chrono::steady_clock::time_point t1=chrono::steady_clock::now();
       optimizer.initializeOptimization();
       optimizer.optimize(100);
       chrono::steady_clock::time_point t2=chrono::steady_clock::now();
       chrono::duration<double> time=chrono::duration_cast<chrono::duration<double>>(t2-t1);
       cout<<"optimize time:"<<time.count()<<endl;
       
       m_Tcw_estimated=SE3(pose->estimate().rotation(),pose->estimate().translation());
       
    }
    void VO::setRef3DPoints()
    {
        m_ref3d.clear();
        m_ref_descriptor=Mat();
        
        for(size_t i=0;i<m_curr_keypoint.size();i++)
        {
            double d=m_ref->findDepth(m_curr_keypoint[i]);
            if(d>0)
            {
                //cout<<"d="<<d<<endl;
                Vector3d pc=m_ref->m_camera->piexl2cam(Vector2d( m_curr_keypoint[i].pt.x,m_curr_keypoint[i].pt.y),d);
                
                m_ref3d.push_back(cv::Point3f(pc(0,0),pc(1,0),pc(2,0)));
                //cout<<"m_ref3d="<<m_ref3d<<endl;
                m_ref_descriptor.push_back(m_curr_descriptor.row(i));
            }
        }
    }
        
    bool VO::addFrame(Frame::Ptr frame)
    {
        switch(m_state)
        {
            case Initialization:
            {
                cout<<"state is Initialization"<<endl;
                m_state=OK;
                m_curr=m_ref=frame;
                m_map->insertKeyFrame(frame);
                
                extractKeyPoints();
                computerDescriptor();
                setRef3DPoints();
                break;
            }
            case OK:
            {
                cout<<"state is OK"<<endl;
                m_curr=frame;
                extractKeyPoints();
                computerDescriptor();
                featureMatch();
                poseEstimatePnP();
                if(isEstimatePose()==true)
                {
                    m_curr->setTcw(m_Tcw_estimated*m_ref->getTcw());
                    m_ref=m_curr;
                    setRef3DPoints();
                    m_num_lost=0;
                    if(isKeyFrame())
                        addKeyFrame();
                }
                else
                {
                    m_num_lost++;
                    if(m_num_lost>m_max_numlost)
                        m_state=Lost;
                    return false;
                }
                cout<<"OK"<<endl;
                break;
            }
            case(Lost):
            {
                cout<<"vo lost"<<endl;
                break;
            }
        }
        return true;
    }
    
    bool VO::isEstimatePose()
    {
        if(m_num_inliers<m_min_inliers)
        {
            cout<<"inlier is too small"<<m_num_inliers<<endl;
            return false;
        }
        Sophus::Vector6d d=m_Tcw_estimated.log();
        if(d.norm()>5.0)
        {
            cout<<"motion is too large:"<<d.norm()<<endl;
            return false;
        }
        return true;
    }
    
    bool VO::isKeyFrame()
    {
        Sophus::Vector6d d=m_Tcw_estimated.log();
        Vector3d trans=d.head<3>();
        Vector3d rot=d.tail<3>();
        if(trans.norm()>m_min_frame_trans||rot.norm()<m_min_frame_rot)
            return true;
        return false;

    }
    
    void VO::addKeyFrame()
    {
        cout<<"add a keyframe"<<endl;
        m_map->insertKeyFrame(m_curr);
    }
}

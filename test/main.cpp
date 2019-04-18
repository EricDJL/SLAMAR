#include <iostream>
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <chrono>
#include "config.h"
#include "vo.h"

#include<GL/glut.h>
# include <stdlib.h>

using namespace std;
using namespace slamAR; 

/*
void init ( void )
{
    GLfloat mat_specular [ ] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shininess [ ] = { 50.0 };
    GLfloat light_position [ ] = { 1.0, 1.0, 1.0, 0.0 };
    glClearColor ( 0.0, 0.0, 0.0, 0.0 );
    glShadeModel ( GL_SMOOTH );
    glMaterialfv ( GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv ( GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv ( GL_LIGHT0, GL_POSITION, light_position);
    glEnable (GL_LIGHTING);
    glEnable (GL_LIGHT0);
    glEnable (GL_DEPTH_TEST);
}
//调用GLUT函数，绘制一个球
void display ( void )
{
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glutSolidSphere (1.0, 40, 50);
    glFlush ();
}
*/

int main(int arg,char** argv)
{
    /*
    //GLUT环境初始化
    glutInit (&arg, argv);
    //显示模式初始化
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    //定义窗口大小 
    glutInitWindowSize (300, 300);
    //定义窗口位置 
    glutInitWindowPosition (100, 100);
    //显示窗口，窗口标题为执行函数名 
    glutCreateWindow ( argv [ 0 ] );
    //调用OpenGL初始化函数 
    init ( );
    //注册OpenGL绘图函数 
    glutDisplayFunc ( display );
    //进入GLUT消息循环，开始执行程序 
    glutMainLoop( );
    return 0;
    */
    
    
    if(arg!=2)
    {
        cout<<"no parameter file"<<endl;
        return 0;
    }
    Config::setParameterFile(argv[1]);
    
    string datadir=Config::get<string>("datadir");
    cout<<"datadir:"<<datadir<<endl;
    ifstream fin(datadir+"/associate.txt");
    if(!fin)
    {
        cout<<"associate file dont have"<<endl;
        return 1;
    }
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( datadir+"/"+rgb_file );
        depth_files.push_back ( datadir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }
    cout<<"rgb and depth have read"<<endl;
    Camera::Ptr camera ( new Camera);
    
    cv::viz::Viz3d vis("VO");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos( 0, 1.0, 1.0), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );

    cout<<"read pic total "<<rgb_files.size() <<endl;
    
    VO::Ptr votest(new VO);
    for ( int i=0; i<rgb_files.size(); i++ )
    {
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        Frame::Ptr pFrame = Frame::createFrame();
        pFrame->m_camera = camera;
        pFrame->setColor(color);
        pFrame->setDepth(depth);
        pFrame->setTimeStamp(rgb_times[i]);

        chrono::steady_clock::time_point t1=chrono::steady_clock::now();
        votest->addFrame ( pFrame );
        chrono::steady_clock::time_point t2=chrono::steady_clock::now();
        chrono::duration<double> time=chrono::duration_cast<chrono::duration<double>>(t2-t1);
        cout<<"VO time:"<<time.count()<<endl;
        
        if ( votest->m_state == VO::VOState::Lost )
            break;
        SE3 Tcw = pFrame->getTcw().inverse();
         
        cv::Affine3d M(
            cv::Affine3d::Mat3( 
                Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
                Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
                Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
            )
        );
        cv::imshow("color image",color);
        cv::imshow("depth image", depth );
        cv::waitKey(1);
        vis.setWidgetPose( "Camera", M);
        vis.spinOnce(1, false);
    }
    return 0;
    
    
}

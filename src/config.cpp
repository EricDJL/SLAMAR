#include "config.h"
namespace slamAR
{
    Config::Config(){}
    
    Config::~Config()
    {
     if(m_file.isOpened())
        m_file.release();
    }
    
    void Config::setParameterFile(string filename)
    {
        if(Ptr==nullptr)
            Ptr=shared_ptr<Config>(new Config);
        Ptr->m_file=cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
        
        if(Ptr->m_file.isOpened()==false)
        {
            cout<<"file does not exsit"<<endl;
            Ptr->m_file.release();
            return;
        }
    }
    shared_ptr<Config> Config::Ptr=nullptr;
}

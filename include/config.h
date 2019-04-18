#ifndef CONFIG_INCLUDE_H
#define CONFIG_INCLUDE_H
#include "common_include.h"
namespace slamAR
{
    class Config
    {
    private:
        static shared_ptr<Config> Ptr;
        cv::FileStorage m_file;
        
        Config();
    public:
        ~Config();
        static void setParameterFile(string filename);
        
        template<typename T>
        static T get(string key)
        {
            return T(Config::Ptr->m_file[key]);
        }
        
    };
}
#endif

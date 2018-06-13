#ifndef CONFIG_H
#define CONFIG_H

#include "common_include.h"

namespace myvo 
{
  
  class Config 
  {
  private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;
    
    Config() {} //singleton, only one instance, via static method to create instance and access
    
  public:
    ~config();
    
    static void setParameterFile(const std::string& filename);
    
    template<typename T>
    static T get(const std::string& key)
    {
      return T(Config::config_->file_[key]);
    }
  };
}

#endif //CONFIG_H
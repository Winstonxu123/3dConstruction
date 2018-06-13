#include "../include/config.h"
namespace myvo
{
void Config::setParameterFile(const string& filename)
{
  if (config_ == nullptr)
    config_ = shared_ptr<Config>(new Config);
  
  config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
  if (config_->file_.isOpened() == false)
  {
    std::cerr << "parameter file" << filename << "doesn't exist." << std::endl;
    config_->file_.release();
    return;
  }

}

Config::~config()
{
  if (file_.isOpened())
    file_.release();

}

shared_ptr<Config> Config::config_ = nullptr;

}
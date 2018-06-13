#include <DBoW3/DBoW3.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <string>
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
  // read the image
  std::cout << "reading images..." << std::endl;
  vector<cv::Mat> images;
  
  for(int i=0; i<10; i++)
  {
    string path = "../data/" + to_string(i+1) + ".png";
    images.push_back(cv::imread(path));
    
  }
  
  // detect ORB features
  std::cout << "detecting ORB features..." << std::endl;
  Ptr<Feature2D> detector = ORB::create();
  vector<Mat> descriptors;
  
  for(Mat& image:images)
  {
    vector<KeyPoint> keypoints;
    Mat descriptor;
    detector->detectAndCompute(image, Mat(), keypoints, descriptor);
    descriptors.push_back(descriptor);
  }
  
  //create vocabulary
  cout << "creating vocabulary..." << endl;
  DBoW3::Vocabulary vocab;
  vocab.create(descriptors);
  
  cout << "vocabulary info: " << vocab << endl;
  vocab.save("vocabulary.yml.gz");
  cout << "done!" << endl;
  return 0;  
}
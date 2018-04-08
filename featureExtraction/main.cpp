#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
using namespace std;
using namespace cv;
/**
 * ORB here
 * SIFT or SURF in laptop 
 **/

int main(int argc, char **argv) {
    if (argc != 3)
    {
      cout<<"Usage: featureExtraction img1 img2"<<endl;
      return 0;
    }
    
    Mat img1=imread(argv[1]);
    Mat img2=imread(argv[2]);
    
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    std::vector<DMatch> goodMatches;
    Mat R,t;
    
    //define function
    void findFeature(Mat img1, Mat img2, vector<KeyPoint>& keypoints_1, vector<KeyPoint>& keypoints_2, 
		 vector<DMatch>& matches, vector<DMatch>& goodMatches);
    void pose_estimation_2d2d (vector<KeyPoint> keypoints_1,vector<KeyPoint> keypoints_2,
  vector<DMatch> matches, Mat& R, Mat& t);
    
    //call functions
    findFeature(img1, img2, keypoints_1, keypoints_2, matches, goodMatches);
    waitKey(0);
    printf("Feature Extraction done!!! Then calculate POSE...\n");
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t); //here using matches, we can use goodMatches as well, which I think better.
    
    //convert R to Eigen::Matrix3d, then convert to Quanterniond
    Eigen::Matrix3d rotation_matrix;
    cv2eigen(R,rotation_matrix);
    Eigen::Quaterniond q(rotation_matrix);
    
    //io processing, write data to file
    ofstream fout("/home/xu/3weiConstruction/featureExtraction/pose.txt", ios::out | ios::app);
    double translate[3][1] = {0};
    for(int i=0; i<t.rows; i++)
    {
      translate[i][0] = t.ptr<double>(i)[0];
    }
    fout << translate[0][0] << " " << translate[1][0] << " " << translate[2][0] << " " <<
    q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    fout.close();
    
    
    
    
    
    
    
    return 0;
}
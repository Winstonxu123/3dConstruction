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

void triangulation(const vector<KeyPoint>& keypoint_1, const vector<KeyPoint> keypoint_2,
		   const vector<DMatch>& matches, const Mat& R, const Mat& t, vector<Point3d>& points);

void findFeature(Mat img1, Mat img2, vector<KeyPoint>& keypoints_1, vector<KeyPoint>& keypoints_2, 
		 vector<DMatch>& matches, vector<DMatch>& goodMatches);

void pose_estimation_2d2d (vector<KeyPoint> keypoints_1,vector<KeyPoint> keypoints_2,
  vector<DMatch> matches, Mat& R, Mat& t);

Point2f pixel2cam (const Point2d& p, const Mat& K);

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
    
    //call functions
    findFeature(img1, img2, keypoints_1, keypoints_2, matches, goodMatches);
    waitKey(0);
    printf("Feature Extraction done!!! Then calculate POSE...\n");
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t); //here using matches, we can use goodMatches as well, which I think better.
    
    vector<Point3d> points;
    triangulation(keypoints_1, keypoints_2, matches, R, t, points);
    
    Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    ofstream fout("/home/xu/3weiConstruction/triangulation/KeyPointDepth.txt", ios::out | ios::app);
    for(int i=0; i<matches.size(); i++)
    {
      Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, K);
      Point2d pt1_cam_3d(points[i].x / points[i].z, points[i].y / points[i].z);
      cout << "point in the first camera frame: " << pt1_cam << endl;
      cout << "point projected from 3D " << pt1_cam_3d << ",d=" << points[i].z << endl;
      if ((i+1) % 5 != 0)
	fout << points[i].z << " ";
      else
	fout << points[i].z << endl;
      Point2f pt2_cam = pixel2cam(keypoints_2[matches[i].trainIdx].pt, K);
      Mat pt2_trans = R*(Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z) + t;
      pt2_trans /= pt2_trans.at<double>(2,0);
      
      cout << "point in the sceond camera frame: " << pt2_cam << endl;
      cout << "point reprojected from second frame: " << pt2_trans.t() << endl;
    }
    fout.close();
    return 0;
}

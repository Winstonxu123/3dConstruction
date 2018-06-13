#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace std;
using namespace cv;

void findFeature(Mat img1, Mat img2, vector<KeyPoint>& keypoints_1, vector<KeyPoint>& keypoints_2, 
		 vector<DMatch>& matches, vector<DMatch>& goodMatches);

Point2f pixel2cam (const Point2d& p, const Mat& K);

void bundleAdjustment(const vector<Point3f> points_3d, const vector<Point2f> points_2d, const Mat& K, Mat& R, Mat& t);

int main(int argc, char **argv) {
     if (argc != 5)
    {
      cout<<"Usage: featureExtraction img1 img2 depthImg1 depthImg2"<<endl;
      return 0;
    }
    
    Mat img1 = imread(argv[1]);
    Mat img2 = imread(argv[2]);
    Mat depth = imread(argv[3], 0); // depth image is unint16, one channel
    
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    std::vector<DMatch> goodMatches;
    Mat R,t;
    findFeature(img1, img2, keypoints_1, keypoints_2, matches, goodMatches);
    waitKey(0);
    printf("Feature Extraction done!!! Then calculate POSE...\n");
    
    Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    
    for(DMatch m:goodMatches)
    {
      ushort d = depth.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)]; //get depth value
      if(d == 0) continue;
      float dd = d/1000.0;
      Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
      pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
      pts_2d.push_back(keypoints_2[m.trainIdx].pt);
      
    }
    cout << "3d-2d pairs: " << pts_3d.size() << endl;
    Mat r; 	//r is a rotation vector
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false, SOLVEPNP_EPNP);
    Rodrigues(r,R); //convert rotation vector to rotation matrix
    
    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t <<endl;
    
    //use bundleAdjustment to optimize
    cout << "calling bundle adjustment" << endl;
    bundleAdjustment(pts_3d, pts_2d, K, R, t);
    return 0;
}

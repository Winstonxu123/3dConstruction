#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace std;
using namespace cv;

void findFeature(Mat img1, Mat img2, vector<KeyPoint>& keypoints_1, vector<KeyPoint>& keypoints_2, 
		 vector<DMatch>& matches, vector<DMatch>& goodMatches);

Point2f pixel2cam (const Point2d& p, const Mat& K);

void bundleAdjustment(const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t);

void pose_estimation_3d3d(const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t);
int main(int argc, char **argv) {
    if (argc != 5)
    {
      cout<<"Usage: featureExtraction img1 img2 depthImg1 depthImg2"<<endl;
      return 0;
    }
    
    Mat img1 = imread(argv[1]);
    Mat img2 = imread(argv[2]);
    Mat depth1 = imread(argv[3], 0); // depth image is unint16, one channel
    Mat depth2 = imread(argv[4], 0);
    
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    std::vector<DMatch> goodMatches;
    Mat R,t;
    findFeature(img1, img2, keypoints_1, keypoints_2, matches, goodMatches);
    waitKey(0);
    printf("Feature Extraction done!!! Then calculate POSE...\n");
    
    Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts1;
    vector<Point3f> pts2;
    for ( DMatch m:matches )
    {
        ushort d1 = depth1.ptr<unsigned short> ( int ( keypoints_1[m.queryIdx].pt.y ) ) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        ushort d2 = depth2.ptr<unsigned short> ( int ( keypoints_2[m.trainIdx].pt.y ) ) [ int ( keypoints_2[m.trainIdx].pt.x ) ];
        if ( d1==0 || d2==0 )   // bad depth
            continue;
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        Point2d p2 = pixel2cam ( keypoints_2[m.trainIdx].pt, K );
        float dd1 = float ( d1 ) /1000.0;
        float dd2 = float ( d2 ) /1000.0;
        pts1.push_back ( Point3f ( p1.x*dd1, p1.y*dd1, dd1 ) );
        pts2.push_back ( Point3f ( p2.x*dd2, p2.y*dd2, dd2 ) );
    }
    cout << "3d-3d pairs: " << pts1.size() << endl;
    
    pose_estimation_3d3d(pts1, pts2, R, t );
    cout << "ICP via SVD results: " << endl;
    cout << "R = " << R << endl;
    cout << "t = " << t << endl;
    //note R is the 2nd picture warp to the 1st picture, so we need to transpose R and t.
    cout << "R_inv = " << R.t() << endl;
    cout << "t_inv = " << -R.t()*t << endl;
    
    cout << "calling bundle adjustment" << endl;

    bundleAdjustment(pts1, pts2, R, t);
    cout << endl;
    //verify p1 = R*p2+t
    for(int i=0; i<5; i++)
    {
      cout << "p1 = " << pts1[i] << endl;
      cout << "p2 = " << pts2[i] << endl;
      cout << "(R*p2+t) = " << R*(Mat_<double>(3,1) << pts2[i].x, pts2[i].y, pts2[i].z) + t << endl;
      cout << endl;
    }
    
    return 0;
}

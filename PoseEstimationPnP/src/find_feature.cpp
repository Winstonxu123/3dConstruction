#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

Point2f pixel2cam (const Point2d& p, const Mat& K)
{
    return Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
    );
}

void findFeature(Mat img1, Mat img2, vector<KeyPoint>& keypoints_1, vector<KeyPoint>& keypoints_2, 
		 vector<DMatch>& matches, vector<DMatch>& goodMatches)
{
    Mat descriptors_1, descriptors_2;
    Ptr<ORB> orb = ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);  //default parameters(500 features)
    
    //find keypoint
    orb->detect(img1,keypoints_1);
    orb->detect(img2,keypoints_2);
    
    //calculate BRIEF descriptor
    orb->compute(img1, keypoints_1, descriptors_1);
    orb->compute(img2, keypoints_2, descriptors_2);
    
    //draw result
    Mat outimg1;
    drawKeypoints(img1,keypoints_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imshow("ORB Features",outimg1);
    
    //Brute Force match between couple images
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_1,descriptors_2,matches);
    
    //filting points
      //find max_distance and min_distance, which represent the most and least similarity between couple points
    double min_dist = 10000, max_dist = 0;
    for (int i=0; i<descriptors_1.rows; i++)
    {
      double dist = matches[i].distance;
      if (dist < min_dist) min_dist = dist;
      if (dist > max_dist) max_dist = dist;
    }
    
    printf("****** Max distance: %f \n", max_dist);
    printf("****** Min distance: %f \n", min_dist);
    
    // filt by experience: if distance > 2*min_dist, then give up.
    for (int i=0; i<descriptors_1.rows; i++)
    {
      if (matches[i].distance <= max(2*min_dist, 30.0)) //30.0 is the lower limit, make it >30.0
       goodMatches.push_back(matches[i]);
    }
    
    //draw matching result
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img1, keypoints_1, img2, keypoints_2, matches, img_match);
    drawMatches(img1, keypoints_1, img2, keypoints_2, goodMatches, img_goodmatch);
    imshow("all points matching", img_match);
    imshow("after optimization", img_goodmatch);
}
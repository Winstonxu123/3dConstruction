#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
int main(int argc, char **argv) {
    cv::Mat image;
    image=cv::imread("/home/xu/3weiTest/depth.png");
    for (int v=0; v<image.rows;v++)
      for (int u=0;u<image.cols;u++)
      {
	int d = image.at<unsigned short>(v,u);
        printf("d= %d\n",d);
      }
    cv::imshow("img",image);
    cv::waitKey(0);
    return 0;
}

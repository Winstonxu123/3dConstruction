#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
int main(int argc, char **argv) {
    cv::Mat image;
    image=cv::imread("/home/xu/3weiConstruction/test/1.jpg");
    cv::imshow("img",image);
    cv::waitKey(0);
    return 0;
}

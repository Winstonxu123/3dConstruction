#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Core>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/io/ply_io.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
using namespace std;

void convert_ply_to_txt(string filename, string txtname);
int get_rows(string filename);

int main(int argc, char **argv) {
    cv::Mat colorImg = cv::imread(argv[1]);
    cv::Mat depthImg = cv::imread(argv[2], -1);
    string saveStr = argv[3];
    string saveStr_copy = saveStr;
    if (argc != 4)
    {
      cout << "Usage: convert_to_pointCloud colorImg depthImg filename" << endl;
      return 0;
    }
    
    //calculate point cloud map
    //camera intrinsic matrix [fx 0 cx; 0 fy cy; 0 0 1]
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 100.0; //that depends
    cout << "transforming images to point cloud..." << endl;
    
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    //new one point_cloud
    PointCloud::Ptr pointCloud(new PointCloud);
    
    for (int v=0; v<colorImg.rows; v++)
      for (int u=0; u<colorImg.cols; u++)
      {
	unsigned int d = depthImg.ptr<unsigned short>(v)[u];
	if (d == 0) continue;
	
	//convert pixel coordinate to camera coordinate
	Eigen::Vector3d point;
	point[2] = double(d) / depthScale;
	point[1] = (v-cy) * point[2] / fy;
	point[0] = (u-cx) * point[2] / fx;
	
	PointT p;
	p.x = point[0];
	p.y = point[1];
	p.z = point[2];
	
	pointCloud->push_back(p);
      }
    
    pointCloud->is_dense = true;
    cout << "The total number of points is " << pointCloud->size() << endl;
    string plySuffix = ".ply";
    string txtSuffix = ".txt";
    string plyTemp = saveStr.append(plySuffix);
    string txtTemp = saveStr_copy.append(txtSuffix);
    
//     string filename("map.ply");
//     string txtname("map.txt");
    pcl::io::savePLYFile(plyTemp, *pointCloud);
    convert_ply_to_txt(plyTemp, txtTemp);
    return 0;
}

void convert_ply_to_txt(string filename, string txtname)
{
  int rows = get_rows(filename);
  ifstream fin;
  ofstream fout;
  fin.open(filename, ifstream::in);
  fout.open(txtname);
  
  if (!fin.is_open())
  {
    cout << "ERROR: Unable to read ply file" << endl;
    exit(-1);
  }
  
  int skipCount = 30; //ply header
  
  char buf[200];
  for (int i=0; i<skipCount; i++)
  {
    fin.getline(buf, sizeof(buf));
  }
   
   double x, y, z;
   fout << rows << endl;
  for (int i=0; i<(rows - 1); i++)
  {
    fin >> x >> y >> z;
    fout << x << " " << y << " " << z << endl;
  }
  
  fin.close();
  fout.close();
  

}


int get_rows(string filename)
{
  ifstream fin;
  fin.open(filename, ifstream::in);
  if (!fin.is_open())
  {
    cout << "ERROR: Unable to read ply file" << endl;
    exit(-1);
  }
  
  int line = 0;
  char c;
  while (fin.get(c))
  {
    if (c == '\n')
      line++;
  }
  line -= 31; //header
  fin.close();
  return line;

}


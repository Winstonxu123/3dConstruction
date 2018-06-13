#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include "./include/icpPointToPlane.h"
using namespace std;

int loadPointCloud(string FName, double* M);
int point_num(string FName, int & N);
Eigen::MatrixXd Get_transform(Matrix optR, Matrix optT);
int writePointCloud(string dataFName, string regFName, Eigen::MatrixXd transform);

int main(int argc, char **argv) {
    Eigen::MatrixXd pre_transform(4, 4);
    Eigen::MatrixXd current_transform(4, 4);
    pre_transform << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    
    string modelFName;         //target_point_cloud
    string templateFName;      //source_point_cloud
    string regFName;
    string outputFName;
    
    if (argc != 5)
    {
      cout << "Usage: icp modelFrame templateFrame regFName outputFName" << endl;
      return 0;
    }
    
    for (int i=0; i<1; i++)
    {
//       sprintf(modelFName, "/home/xu/3weiConstruction/lib-icp/icp/Frame0.txt");
//       sprintf(templateFName, "/home/xu/3weiConstruction/lib-icp/icp/Frame1.txt");
//       sprintf(regFName, "/home/xu/3weiConstruction/lib-icp/icp/result.txt");
//       sprintf(outputFName,"/home/xu/3weiConstruction/lib-icp/icp/transform.txt");
      string prefix = "/home/xu/3weiConstruction/convert_to_pointcloud/build/icp-data/";
      modelFName = prefix + argv[1];
      templateFName = prefix + argv[2];
      regFName = argv[3];
      outputFName = argv[4];
      
      int32_t dim = 3;
      int mod_num;
      int tem_num;
      
      point_num(templateFName, tem_num);
      point_num(modelFName, mod_num);
      
      //allocate memory for model and template, saving all points' information 
      double* M = (double*)calloc(3 * mod_num, sizeof(double));
      double* T = (double*)calloc(3 * tem_num, sizeof(double));
      
      loadPointCloud(modelFName, M);
      loadPointCloud(templateFName, T);
      
      //start with identity as initial transformation
      Matrix R = Matrix::eye(3);
      Matrix t(3, 1);
      
      //run point-to-plane ICP (-1 represents no outlier threshold)
      cout << "Running ICP (point-to-plane, no outliers)" << endl;
      IcpPointToPlane icp(M, mod_num, dim);
      //IcpPointToPlane icp;
      icp.fit(T, tem_num, R, t, -1);
      
      //results
      cout << "--------------------------------" << endl << "transformation results:" << endl;
      
      current_transform = pre_transform * Get_transform(R, t);
      cout << current_transform << endl;
      writePointCloud(templateFName, regFName, current_transform); //transform template to model
      pre_transform = current_transform;
      
      ofstream ofile;
      ofile.open(outputFName);
      ofile << current_transform << endl;
      ofile.close();
      //free memory
      free(M);
      free(T);

    }
    return 0;
}


int loadPointCloud(string FName, double* M)
{
  int i, N;
  double x, y, z;
  ifstream ifile;
  
  ifile.open(FName, ifstream::in);
  if (!ifile.is_open())
  {
    cout << "ERROR: Unable to open point file" << endl;
    exit(-1);
  }
  
  ifile >> N;  // First line has number of points to follow
  
  for (int i=0; i<N; i++)
  {
    ifile >> x >> y >> z;
    M[i * 3 + 0] = x;
    M[i * 3 + 1] = y;
    M[i * 3 + 2] = z;
  }
  ifile.close();
  
  return 0;

}


int point_num(string FName, int& N)
{
  ifstream ifile;
  ifile.open(FName, ifstream::in);
  
  if (!ifile.is_open())
  {
    cout << "ERROR: Unable to open point file" << endl;
    exit(-1);
  }
  
  ifile >> N;
  ifile.close();
  
  return 0;
}


Eigen::MatrixXd Get_transform(Matrix optR, Matrix optT)
{
  Eigen::MatrixXd transform(4, 4);
  transform << optR.val[0][0], optR.val[0][1], optR.val[0][2], optT.val[0][0],
	       optR.val[1][0], optR.val[1][1], optR.val[1][2], optT.val[1][0],
	       optR.val[2][0], optR.val[2][1], optR.val[2][2], optT.val[2][0],
	             0       ,        0      ,        0      ,      1        ;
  return transform;

}


int writePointCloud(string dataFName, string regFName, Eigen::MatrixXd transform)
{
  ifstream datafile;
  ofstream outfile;
  outfile.open(regFName);
  datafile.open(dataFName, ifstream::in);
  
  int V;
  if (!datafile.is_open())
  {
    cout << "ERROR: Unable to open point file" << endl;
    exit(-1);
  }
  
  datafile >> V;
  outfile << V << endl;
  
  double x, y, z;
  for (int j=0; j<V; j++)
  {
    datafile >> x >> y >> z;
    Eigen::Vector4d v_4d(x, y, z, 1);
    v_4d = transform * v_4d;
    outfile << v_4d(0) << " " << v_4d(1) << " " << v_4d(2) << endl;
    
  }
  datafile.close();
  outfile.close();
  return 0;

}





























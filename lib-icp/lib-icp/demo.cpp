/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libicp.
Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

// Demo program showing how libicp can be used

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "icpPointToPlane.h"

using namespace std;

int loadPointCloud(char* FName, double* M);
int point_num(char* FName, int & N);
Eigen::MatrixXd Get_transform(Matrix optR, Matrix optT);
int writePointCloud(char* dataFName, char* regFName, Eigen::MatrixXd transform);

int main(int argc, char** argv) {

	Eigen::MatrixXd pre_transform(4, 4);
	Eigen::MatrixXd current_transform(4, 4);
	pre_transform << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

	char modelFName[128];         //target_point_cloud
	char templateFName[128];      //source_point_cloud
	char regFName[128]; 
	char outputFName[128];


	int STARTFRAME =3;
	int ENDFRAME = 2;

	for (int i = STARTFRAME; i> ENDFRAME; i--)
	{
		/*sprintf_s(modelFName, "F:\\WZC\\mine\\data\\pcd_data\\viewer08_txt\\view08_Frame%d.txt", i);
		sprintf_s(templateFName, "F:\\WZC\\mine\\data\\pcd_data\\viewer08_txt\\view08_Frame%d.txt", i- 1);
		sprintf_s(outputFName, "F:\\WZC\\mine\\data\\pcd_data\\viewer08_txt_R_t\\view08_R_t_Frame%d.txt", i - 1);
		sprintf_s(regFName, "F:\\WZC\\mine\\data\\pcd_data\\viewer08_txt_reg4\\view08_Frame_reg%d.txt", i - 1);*/

		sprintf_s(modelFName, "F:\\WZC\\mine\\data\\data\\view08_Frame10.txt");
		sprintf_s(templateFName, "F:\\WZC\\mine\\data\\data\\view08_Frame4.txt");
		sprintf_s(regFName, "F:\\WZC\\mine\\data\\data\\444.txt");
		sprintf_s(outputFName, "F:\\WZC\\mine\\data\\data\\output.txt");


		int32_t dim = 3;
		int mod_num;
		int tem_num;

		point_num(templateFName, tem_num);
		point_num(modelFName, mod_num);


		// allocate model and template memory
		double* M = (double*)calloc(3 * mod_num, sizeof(double));
		double* T = (double*)calloc(3 * tem_num, sizeof(double));

		loadPointCloud(modelFName, M);
		loadPointCloud(templateFName, T);



		// start with identity as initial transformation
		// in practice you might want to use some kind of prediction here
		Matrix R = Matrix::eye(3);
		Matrix t(3, 1);

		// run point-to-plane ICP (-1 = no outlier threshold)
		cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
		IcpPointToPlane icp(M, mod_num, dim);
		icp.fit(T, tem_num, R, t, -1);

		// results
		cout << endl << "Transformation results:" << endl;

		current_transform = pre_transform*Get_transform(R, t);
		cout << current_transform << endl;
		writePointCloud(templateFName, regFName, current_transform);
		pre_transform = current_transform;

	

		ofstream ofile;
		ofile.open(outputFName, ofstream::out);
		ofile << current_transform << endl;
		ofile.close();
		// free memory
		free(M);
		free(T);
	}
	// success
	return 0;
}





int loadPointCloud(char* FName,  double* M)
{
	int i , N;
	double x, y, z;
	ifstream ifile;

	ifile.open(FName, ifstream::in);
	if (!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	ifile >> N; // First line has number of points to follow
	
	for (i = 0; i < N; i++)
	{
		ifile >> x >> y >> z;
		M[i * 3 + 0] = x;
		M[i * 3 + 1] = y;
		M[i * 3 + 2] = z;
		
		
	}

	ifile.close();

	return 0;
}

int point_num(char* FName, int & N)
{
	
	ifstream ifile;

	ifile.open(FName, ifstream::in);
	if (!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	ifile >> N; // First line has number of points to follow
	
	ifile.close();

	return 0;
}

Eigen::MatrixXd Get_transform(Matrix optR, Matrix optT)
{
	Eigen::MatrixXd transform(4, 4);
	transform << optR.val[0][0], optR.val[0][1], optR.val[0][2], optT.val[0][0], optR.val[1][0], optR.val[1][1], optR.val[1][2], optT.val[1][0], optR.val[2][0], optR.val[2][1], optR.val[2][2], optT.val[2][0],
		0, 0, 0, 1;
	return transform;
}



int writePointCloud(char* dataFName, char* regFName, Eigen::MatrixXd transform)
{
	ifstream datafile;
	
	ofstream outfile;
	outfile.open(regFName);
	datafile.open(dataFName, ifstream::in);

	int V;

	if (!datafile.is_open())
	{
		cout << "Unable to open point file'" << dataFName << "'" << endl;
		exit(-1);
	}
	datafile >> V;
	outfile << V << endl;

	double x, y, z;
	for (int j = 0; j < V; j++)
	{
		datafile >> x >> y >> z;
		Eigen::Vector4d v_4d(x, y, z, 1);
		v_4d = transform*v_4d;
		outfile << v_4d(0) << " " << v_4d(1) << " " << v_4d(2) << endl;
	}

	datafile.close();
	outfile.close();

	return 0;
}
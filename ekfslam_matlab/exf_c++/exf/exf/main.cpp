#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<boost/algorithm/string.hpp>
#include<Eigen/Core>


using namespace Eigen;
using namespace std;
using namespace boost;
int main()
{
	vector<string> splits;
	vector<double> landmarks;
	MatrixXd landmarkMatrix;
	string s;
	ifstream is("C://Users//Arthas//Desktop//ekfslam//exf_c++//landmark.txt");
	getline(is, s);
	double value = 0.0;
	split(splits, s, is_any_of("\t"));
	for (int i = 0; i < splits.size(); i++)
	{
		stringstream ss(splits[i]);
		ss >> value;
		value /= 10;
		landmarks.push_back(value);
		//cout << value << " ";
	}
	int landmardSize = landmarks.size() / 2;
	landmarkMatrix.resize(2, landmardSize);
	for (int i = 0; i < landmarks.size(); i++)
	{
		int row = i / landmardSize;
		landmarkMatrix(row,i)=landmarks[i];
	}
	cout << endl;
	system("pause");
}
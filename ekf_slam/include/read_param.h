#ifndef READ_PARAM_H_
#define READ_PARAM_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;

//read the landmarks and way points from file
void ReadParamFromFile(string lmfilename,string wpfilename,MatrixXd& lm,MatrixXd& wp);







#endif
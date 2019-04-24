#ifndef UPDATE_H_
#define UPDATE_H_
#include<eigen3/Eigen/Dense>
using namespace Eigen;
double Compute_W(Vector3d xtrue,MatrixXd wp,int& iwp,int minSquareDistance,double dt);

Vector3d Vehicle_model(Vector3d xtrue,double vel,double w,double dt);

void AddControlNoise(double& vel,double& w,Matrix2d Q);

void Predict(MatrixXd& x,MatrixXd& P,double vel,double w,Matrix2d Q,double dt);

void GetObservation(Vector3d truePose,MatrixXd lm,double rMax,double radiansMin,MatrixXd& z);

void AddObserveNoise(MatrixXd z,Matrix2d R);

void Observe_Model(MatrixXd x,int idf,Matrix<double,2,1>& z,MatrixXd& H);//get the predict observation of specified landmark

void Compute_Assosiciation(MatrixXd x,MatrixXd P,Matrix<double,2,1> z,Matrix2d R,int idf,double* result);

void DataAssociation(MatrixXd x,MatrixXd P,MatrixXd z,Matrix2d R,MatrixXd& zn,int* idf,MatrixXd& ze);

void Update(MatrixXd& x,MatrixXd& P,MatrixXd z,Matrix2d R,int *idf);

void Add_State(MatrixXd& x,MatrixXd& P,MatrixXd z,Matrix2d R);
#endif
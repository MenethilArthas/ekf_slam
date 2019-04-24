#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "read_param.h"
#include "update.h"
#include "config.h"
#include "ros_plot.h"
using namespace std;
using namespace Eigen;

int main( int argc, char** argv )
{
	ros::init(argc, argv, "exfslam_sim");
	ros::NodeHandle n;
	ros::Rate r(50);
	Plot plot(n);
	Vector3d lastXTrue(0,0,0);
	Vector3d curXTrue(0,0,0);
	MatrixXd z;//observed landmarks	
	MatrixXd P=MatrixXd::Zero(3,3);//covariance matrix
	MatrixXd x=MatrixXd::Zero(1,3);//states vector
	Matrix2d Q;
	Matrix2d R;
	int iwp=0;
	double dt=CONTROL_DURATION_TIME;
	double dtSum=0.0;
	double vel=0.0;
	double w=0.0;
	//set control and observation noise
	Q<<pow(SIGMA_V,2),0.0,0.0,pow(SIGMA_W,2);
	R<<pow(SIGMA_R,2),0.0,0.0,pow(SIGMA_B,2);
	//read in the landmarks and waypoints from txt file
	string landmarksFile="/home/arthas/exf_slam/landmark.txt";
	string waypointsFile="/home/arthas/exf_slam/waypoint.txt";
	MatrixXd landmarks;
	MatrixXd waypoints;
	MatrixXd zn,ze;
	ReadParamFromFile(landmarksFile,waypointsFile,landmarks,waypoints);
	while (ros::ok())
	{
		w=Compute_W(lastXTrue,waypoints,iwp,AT_WAYPOINT,dt);
		curXTrue=Vehicle_model(lastXTrue,VEL, w,dt);
		vel=VEL;
		AddControlNoise(vel,w,Q);
		//EKF predict step
		Predict( x, P, vel, w, Q, dt);
		dtSum+=dt;
		if(dtSum>=OBSERVE_DURATION_TIME)
		{
			dtSum=0.0;
			//get the observable landmarks
			GetObservation( curXTrue, landmarks, MAX_RANGE,MIN_RADIANS, z);
			//add observe noise
			AddObserveNoise(z,R);
			//cout<<"observed landmarks num="<<z.cols()<<endl;
			//get data association
			int idf[z.cols()];			
			DataAssociation(x, P, z,R,zn,idf,ze);

// 			cout<<"z number="<<z.cols()<<"  zn number="<<zn.cols()<<"  ze number="<<ze.cols()<<endl;
// 			cout<<"before update:"<<endl;
// 			cout<<P<<"\n"<<endl;
			Update(x,P,ze,R,idf);
// 			cout<<"after update:"<<endl;
// 			cout<<P<<"\n"<<endl;
			Add_State(x,P, zn, R);
// 			cout<<"after add_state:"<<endl;
// 			cout<<P<<"\n"<<endl;
			plot.PlotObserve(zn,ze,x.block<1,3>(0,0));
		}
		plot.PlotLmUncertainty(x,&P);
		plot.PlotXPose(x);
		plot.PlotEnvironment(landmarks,waypoints);
		
		lastXTrue=curXTrue;
		r.sleep();
	}
}


#ifndef ROS_PLOT_H_
#define ROS_PLOT_H_

#include"ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
class Plot
{
public:
	Plot(ros::NodeHandle& n);
	void PlotEnvironment(MatrixXd rLandmarks,MatrixXd rWaypoints);
	void PlotXPose(Vector3d LastTruePose,Vector3d CurTruePose);
	void PlotXPose(MatrixXd x);
	void PlotObserve(MatrixXd zn,MatrixXd ze,MatrixXd x);
	void PlotLmUncertainty(MatrixXd x,MatrixXd* P);
private:
	ros::Publisher marker_pub;
	ros::Publisher markerArray_pub;
	visualization_msgs::Marker	xPose;
	int poseCount;
};




#endif
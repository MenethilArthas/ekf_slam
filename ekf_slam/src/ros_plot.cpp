#include "ros_plot.h"

Plot::Plot(ros::NodeHandle& n)
{
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	markerArray_pub= n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
	poseCount=0;
}

void Plot::PlotEnvironment(MatrixXd rLandmarks, MatrixXd rWaypoints)
{
	visualization_msgs::Marker	lmPoints;
	visualization_msgs::Marker	wpPoints;
	visualization_msgs::Marker	line_strip;
	lmPoints.header.frame_id =wpPoints.header.frame_id=line_strip.header.frame_id= "/my_frame";
	lmPoints.ns =wpPoints.ns=line_strip.ns= "lm_and_wp";
	lmPoints.action=wpPoints.action=line_strip.action= visualization_msgs::Marker::ADD;
	lmPoints.pose.orientation.w = wpPoints.pose.orientation.w=line_strip.pose.orientation.w=1.0;
	lmPoints.id = 0;
	wpPoints.id = 1;
	line_strip.id = 2;

	lmPoints.type =  visualization_msgs::Marker::POINTS;
	wpPoints.type =  visualization_msgs::Marker::POINTS;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	lmPoints.scale.x = 0.1;
	lmPoints.scale.y = 0.1;
	// Set the color -- be sure to set alpha to something non-zero!
	lmPoints.color.g = 1.0f;
	lmPoints.color.a = 1.0f;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
	wpPoints.scale.x = 0.2;
	wpPoints.scale.y = 0.2;
	line_strip.scale.x = 0.05;
	line_strip.scale.y = 0.05;
	// Set the color -- be sure to set alpha to something non-zero!
	wpPoints.color.r = 1.0f;
	wpPoints.color.a = 1.0f;
	line_strip.color.b = 1.0f;
	line_strip.color.a = 1.0f;
	lmPoints.header.stamp = wpPoints.header.stamp =line_strip.header.stamp = ros::Time::now();

	lmPoints.lifetime = ros::Duration();
	for(int i=0;i<rLandmarks.cols();i++)
	{
		geometry_msgs::Point p;
		p.x=rLandmarks(0,i);
		p.y=rLandmarks(1,i);
		lmPoints.points.push_back(p);
	}
	wpPoints.lifetime = ros::Duration();
	line_strip.lifetime=ros::Duration();
	for(int i=0;i<rWaypoints.cols();i++)
	{
		geometry_msgs::Point p;
		p.x=rWaypoints(0,i);
		p.y=rWaypoints(1,i);
		wpPoints.points.push_back(p);
		line_strip.points.push_back(p);
	}
	geometry_msgs::Point p;
	p.x=rWaypoints(0,0);
	p.y=rWaypoints(1,0);
	line_strip.points.push_back(p);
	// Publish the marker
// 	while (marker_pub.getNumSubscribers() < 1)
// 	{
// 		if (!ros::ok())
// 		{
// 			return 0;
// 		}
// 		ROS_WARN_ONCE("Please create a subscriber to the marker");
// 		sleep(1);
// 	}
	marker_pub.publish(lmPoints);
	marker_pub.publish(wpPoints);
	marker_pub.publish(line_strip);	
}
void Plot::PlotXPose(MatrixXd x)
{
	poseCount++;
	xPose.header.frame_id="/my_frame";
	xPose.ns= "lm_and_wp";
	xPose.action=visualization_msgs::Marker::ADD;
	xPose.pose.orientation.w =1.0;
	xPose.id = 3;
	xPose.type = visualization_msgs::Marker::LINE_STRIP;
	xPose.color.b=1.0f;
	xPose.color.g=1.0f;
	xPose.color.a=1.0f;
	xPose.scale.x=0.05;
	xPose.scale.y=0.05;
	xPose.header.stamp=ros::Time::now();
	xPose.lifetime=ros::Duration();

	geometry_msgs::Point p2;
	p2.x=x(0);
	p2.y=x(1);
// 	xPose.points.push_back(p1);
	if(poseCount==80)
		xPose.points.erase(xPose.points.begin());
	xPose.points.push_back(p2);
	marker_pub.publish(xPose);
}
void Plot::PlotXPose(Vector3d LastTruePose,Vector3d CurTruePose)
{
	poseCount++;
	xPose.header.frame_id="/my_frame";
	xPose.ns= "lm_and_wp";
	xPose.action=visualization_msgs::Marker::ADD;
	xPose.pose.orientation.w =1.0;
	xPose.id = 3;
	xPose.type = visualization_msgs::Marker::LINE_STRIP;
	xPose.color.b=1.0f;
	xPose.color.g=1.0f;
	xPose.color.a=1.0f;
	xPose.scale.x=0.05;
	xPose.scale.y=0.05;
	xPose.header.stamp=ros::Time::now();
	xPose.lifetime=ros::Duration();
	geometry_msgs::Point p1;
	p1.x=LastTruePose(0);
	p1.y=LastTruePose(1);
	geometry_msgs::Point p2;
	p2.x=CurTruePose(0);
	p2.y=CurTruePose(1);
// 	xPose.points.push_back(p1);
	if(poseCount==80)
		xPose.points.erase(xPose.points.begin());
	xPose.points.push_back(p2);
	marker_pub.publish(xPose);
}
void Plot::PlotLmUncertainty(MatrixXd x,MatrixXd* P)
{
	int lmLength=(x.cols()-3)/2;	
	visualization_msgs::MarkerArray ellipseArray;

	for(int i=0;i<lmLength;i++)
	{	
		visualization_msgs::Marker	uncertaintyEllipse;
		uncertaintyEllipse.header.frame_id="/my_frame";
		uncertaintyEllipse.ns= "lm_and_wp";
		uncertaintyEllipse.action=visualization_msgs::Marker::ADD;
		uncertaintyEllipse.pose.orientation.w =1.0;
		uncertaintyEllipse.id = 5+i;
		uncertaintyEllipse.type = visualization_msgs::Marker::SPHERE;
		uncertaintyEllipse.color.b=1.0f;
		uncertaintyEllipse.color.g=1.0f;
		uncertaintyEllipse.color.a=1.0f;
		uncertaintyEllipse.scale.x=(*P)(3+2*i,3+2*i)*20;
		uncertaintyEllipse.scale.y=(*P)(3+2*i+1,3+2*i+1)*20;
		uncertaintyEllipse.scale.z=0.0;
		uncertaintyEllipse.pose.position.x=x(0,3+2*i);
		uncertaintyEllipse.pose.position.y=x(0,3+2*i+1);
		uncertaintyEllipse.header.stamp=ros::Time::now();
		uncertaintyEllipse.lifetime=ros::Duration();
		ellipseArray.markers.push_back(uncertaintyEllipse);
	}
	if(lmLength>0)
		markerArray_pub.publish(ellipseArray);
}
void Plot::PlotObserve(MatrixXd zn, MatrixXd ze,MatrixXd x)
{
	visualization_msgs::Marker	line_list;
	line_list.header.frame_id="/my_frame";
	line_list.ns= "lm_and_wp";
	line_list.action=visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w =1.0;
	line_list.id = 4;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.color.r=1.0f;
	line_list.color.a=1.0f;
	line_list.scale.x=0.05;
	line_list.scale.y=0.05;
	line_list.header.stamp=ros::Time::now();
	line_list.lifetime=ros::Duration();
	geometry_msgs::Point p1;
	p1.x=x(0);
	p1.y=x(1);
	
	geometry_msgs::Point p2;
	for(int i=0;i<zn.cols();i++)
	{
		double r=zn(0,i);
		double angle=zn(1,i);
		double s=sin(x(2)+angle);
		double c=cos(x(2)+angle);
		p2.x=x(0)+r*c;
		p2.y=x(1)+r*s;
		line_list.points.push_back(p1);
		line_list.points.push_back(p2);
	}
	for(int i=0;i<ze.cols();i++)
	{
		double r=ze(0,i);
		double angle=ze(1,i);
		double s=sin(x(2)+angle);
		double c=cos(x(2)+angle);
		p2.x=x(0)+r*c;
		p2.y=x(1)+r*s;
		line_list.points.push_back(p1);
		line_list.points.push_back(p2);
	}
	marker_pub.publish(line_list);
}

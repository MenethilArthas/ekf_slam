#include<eigen3/Eigen/Dense>
#include"update.h"
#include"config.h"
#include<math.h>
#include <iostream>
#include <boost/concept_check.hpp>
#include<vector>
using namespace std;
using namespace Eigen;

double Compute_W(Vector3d xtrue,MatrixXd wp,int& iwp,int minSquareDistance,double dt)
{
	Vector2d cwp=wp.block<2,1>(0,iwp);//get current way point 2*1
	double squareDistance=pow(xtrue(0)-cwp(0),2)+pow(xtrue(1)-cwp(1),2);
	if(squareDistance<minSquareDistance)
	{
			iwp++;
			if(iwp==wp.cols()-1)
				iwp=0;
			cwp=wp.block<2,1>(0,iwp);//get current way point 2*1
	}
	double dy=xtrue(1)-cwp(1);
	double dx=xtrue(0)-cwp(0);
	double deltaAngle=NormalizedAngle(atan2(cwp(1)-xtrue(1),cwp(0)-xtrue(0))-xtrue(2));
	double w=deltaAngle/dt;
	if(fabs(w)>MAX_W)
		w=MAX_W*(w/fabs(w));
	return w;
}

Vector3d Vehicle_model(Vector3d xtrue,double vel,double w,double dt)
{
	Vector3d newPose;
	double rotateR=(vel/w);
	newPose(0)=xtrue(0)-rotateR*sin(xtrue(2))+rotateR*sin(xtrue(2)+w*dt);
	newPose(1)=xtrue(1)+rotateR*cos(xtrue(2))-rotateR*cos(xtrue(2)+w*dt);
	newPose(2)=NormalizedAngle(xtrue(2)+w*dt);
	return newPose;
}
//generate random number from normal distribution
double sampleNormal()
{
	double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
	double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
	double r = u * u + v * v;
	if (r == 0 || r > 1) return sampleNormal();
	double c = sqrt(-2 * log(r) / r);
	return u * c;
}
void AddControlNoise(double& vel,double& w,Matrix2d Q)
{
	vel=vel+sampleNormal()*sqrt(Q(0,0));
	w=w+sampleNormal()*sqrt(Q(1,1));
}
void AddObserveNoise(MatrixXd z,Matrix2d R)
{
	int size=z.cols();
	for(int i=0;i<size;i++)
	{
		z(0,i)=z(0,i)+sampleNormal()*sqrt(R(0,0));
		z(1,i)=z(1,i)+sampleNormal()*sqrt(R(1,1));
	}
}
void Predict(MatrixXd& x,MatrixXd& P,double vel,double w,Matrix2d Q,double dt)
{
	double rotateR=vel/w;
	double xita=x(2);
	double a=rotateR * ( -cos(xita) + cos(xita + w*dt ));
	double b=rotateR * ( -sin(xita) + sin(xita + w*dt ));
	Matrix3d Gt;
	Matrix<double,3,2> Vt;
	//jacobians
	Gt<<1,0,a,0,1,b,0,0,1;
	Vt(0,0)=(-sin(xita)+sin(xita+w*dt))/w;
	Vt(0,1)=vel*(sin(xita)-sin(xita+w*dt))/pow(w,2)+vel*cos(xita+w*dt)*dt/w;
	Vt(1,0)=(cos(xita)-cos(xita+w*dt))/w;
	Vt(1,1)=-vel*(cos(xita)-cos(xita+w*dt))/pow(w,2)+vel*sin(xita+w*dt)*dt/w;
	Vt(2,0)=0;
	Vt(2,1)=dt;
	
	//predict covariance
	P.block<3,3>(0,0)=Gt*P.block<3,3>(0,0)*Gt.transpose()+Vt*Q*Vt.transpose();
	int dim=P.cols()-3;	
 	if(P.cols()>3)
	{
		P.block(0,3,3,dim)=Gt*P.block(0,3,3,dim);
		P.block(3,0,dim,3)=P.block(0,3,3,dim).transpose();
	}
	//predict state
	x(0)=x(0)+rotateR * ( -sin(xita) + sin(xita + w*dt ));
	x(1)=x(1)+rotateR * ( cos(xita) - cos(xita + w*dt ));
	x(2)=NormalizedAngle(x(2)+w*dt);
}

void GetObservation(Vector3d truePose,MatrixXd lm,double rMax,double radiansMin,MatrixXd& z)
{
	int lmSize=lm.cols();
	double phi=truePose(2);
	vector<double> lmList;
	for(int i=0;i<lmSize;i++)
	{
		double x=lm(0,i);
		double y=lm(1,i);
		double dx=x-truePose(0);
		double dy=y-truePose(1);
		double length1=sqrt(pow(cos(phi),2)+pow(sin(phi),2));
		double length2=sqrt(pow(dx,2)+pow(dy,2));
		double xita=acos((cos(phi)*dx+sin(phi)*dy) /(length1*length2) );
		
		if(fabs(dx)<rMax&&fabs(dy)<rMax&&
			(pow(dx,2)+pow(dy,2)<pow(rMax,2))&&
			(xita<radiansMin))
		{
			lmList.push_back(sqrt(pow(dx,2)+pow(dy,2)));
			lmList.push_back(atan2(dy,dx)-phi);			
		}
	}
	z.resize(2,lmList.size()/2);
	for(int i=0;i<lmList.size()/2;i++)
	{
		z(0,i)=lmList[2*i];
		z(1,i)=lmList[2*i+1];
	}
}
void Observe_Model(MatrixXd x,int idf,Matrix<double,2,1>& z,MatrixXd& H)
{
	int Nxv=3;
	int pos=Nxv+idf*2;//idf start from 0
	H=MatrixXd::Zero(2,x.cols());
	double dx=x(pos)-x(0);
	double dy=x(pos+1)-x(1);
	double d2=pow(dx,2)+pow(dy,2);
	double d=sqrt(d2);
	double xd=dx/d;
	double yd=dy/d;
	double xd2=dx/d2;
	double yd2=dy/d2;
	//predict z
	z(0)=d;
	z(1)=NormalizedAngle(atan2(dy,dx)-x(2)) ;
	//calculate H
// 	cout<<"before update:"<<H<<endl;
	H.block<2,3>(0,0)<<-xd,-yd,0,yd2,-xd2,-1;
	H.block<2,2>(0,pos)<<xd,yd,-yd2,xd2;
// 	cout<<"after update:"<<H<<endl;
}
void Compute_Assosiciation(MatrixXd x,MatrixXd P,Matrix<double,2,1> z,Matrix2d R,int idf,double* result)
{
	Matrix<double,2,1> zp;
	MatrixXd H;
	Observe_Model(x,idf,zp,H);
	Matrix<double,2,1> v=z-zp;
// 	cout<<x<<endl;
// 	cout<<idf<<"\t"<<"v="<<v<<endl;
	double r=z(0);
	double angle=z(1);
	double s=sin(x(2)+angle);
	double c=cos(x(2)+angle);
// 	cout<<"newly observerd Point ("<<x(0)+r*c<<","<<x(1)+r*s<<")";
// 	cout<<"already exsitence Point("<<x(3+idf*2)<<","<<x(3+2*idf+1)<<")"<<endl;
	v(1)=NormalizedAngle(v(1));

	Matrix2d S=H*P*H.transpose()+R;
	result[0]=v.transpose()*S.inverse()*v;
	result[1]=result[0]+log(S.determinant());
	result[2]=fabs(v(0));
	result[3]=fabs(v(1));
//  	cout<<result[0]<<"\t"<<result[1]<<endl;
}
void DataAssociation(MatrixXd x,MatrixXd P,MatrixXd z,Matrix2d R,MatrixXd& zn,int* idf,MatrixXd& ze)
{
	int Nxv=3;
	int Ne=(x.cols()-Nxv)/2;//number of landmarks already in map
	int newLandMarkSize=z.cols();//number of landmarks newly observed
	ze.resize(2,0);
	zn.resize(2,0);
	vector<double> vectorZE,vectorZN;
	double result[4];
	int matchCount=0;
	for(int i=0;i<newLandMarkSize;i++)
	{
		int jbest=0;
		int nbest=INF;
		int outer=INF;
		for(int j=0;j<Ne;j++)
		{
			Compute_Assosiciation(x,P,z.block<2,1>(0,i),R,j,result);
			if(result[0]<GATE_REJECT&&result[1]<nbest&&result[2]<GATE_REJECT&&result[3]<GATE_RAD)
			{
				nbest= result[1];
				jbest= j;
			}
			else if(result[0]<outer)
				outer=result[0];
		}
		if(jbest!=0)
		{
			vectorZE.push_back(z(0,i));
			vectorZE.push_back(z(1,i));
			idf[matchCount]=jbest;
			matchCount++;
		}
		else if(outer>GATE_AUGMENT)
		{
			vectorZN.push_back(z(0,i));
			vectorZN.push_back(z(1,i));
		}
	}
	int zeSize=vectorZE.size()/2;
	int znSize=vectorZN.size()/2;
	ze.resize(2,zeSize);
	zn.resize(2,znSize);
	for(int i=0;i<zeSize;i++)
	{
		ze(0,i)=vectorZE[2*i];
		ze(1,i)=vectorZE[2*i+1];
	}
	for(int j=0;j<znSize;j++)
	{
		zn(0,j)=vectorZN[2*j];
		zn(1,j)=vectorZN[2*j+1];
	}
}
void Update(MatrixXd& x,MatrixXd& P,MatrixXd ze,Matrix2d R,int *idf)
{
	int length=ze.cols();
	for(int i=0;i<length;i++)
	{
		Matrix<double,2,1> zp;
		MatrixXd H;
		Observe_Model(x,idf[i],zp,H);//获得第idf[i]个路标的预计观测值zp
		Matrix<double,2,1> v;
		v(0)=ze(0,i)-zp(0);
		v(1)=NormalizedAngle(ze(1,i)-zp(1));
		MatrixXd PHt=P*H.transpose();
		MatrixXd Ins=(H*PHt+R).inverse();
		MatrixXd K=PHt*Ins;
// 		cout<<"length="<<length<<endl;
// 		cout<<K<<"\n"<<endl;
		x=x+(K*v).transpose();
		MatrixXd I=MatrixXd::Identity(x.cols(),x.cols());
		P=(I-K*H)*P;
	}
}
void Add_State(MatrixXd& x,MatrixXd& P,MatrixXd z,Matrix2d R)
{
	int originalLength=x.cols();
	int appendLength=z.cols()*2;
	MatrixXd backupX=x;
	MatrixXd backupP=P;
	x.resize(1,originalLength+appendLength);
	P.resize(originalLength+appendLength,originalLength+appendLength);
	x.block(0,0,1,originalLength)=backupX;
	P.block(0,0,originalLength,originalLength)=backupP;

	for(int i=0;i<appendLength/2;i++)
	{
		//append state vector
		double r=z(0,i);
		double angle=z(1,i);
		double s=sin(x(2)+angle);
		double c=cos(x(2)+angle);
		x(originalLength+2*i)=x(0)+r*c;
		x(originalLength+2*i+1)=x(1)+r*s;
		// jacobians
		Matrix<double,2,3> Gv;
		Gv<<1,0,-r*s,0,1,r*c;
		Matrix2d Gz;
		Gz<<c,-r*s,s,r*c;
// 		if(P.cols()>10)
// 		{
// 			MatrixXd test1=Gv*P.block<3,3>(0,0)*Gv.transpose();
// 			cout<<P.block<3,3>(0,0)<<endl;
// 			cout<<test1<<"\n"<<endl;
// 			MatrixXd test2=Gz*R*Gz.transpose();
// 			cout<<test2<<endl;
// 		}
		//append covariance 
		P.block<2,2>(originalLength+2*i,originalLength+2*i)=Gv*P.block<3,3>(0,0)*Gv.transpose()+Gz*R*Gz.transpose();
// 		cout<<P<<"\n"<<endl;
		P.block<2,3>(originalLength+2*i,0)=Gv*P.block<3,3>(0,0);
// 		cout<<P<<"\n"<<endl;
		P.block<3,2>(0,originalLength+2*i)=P.block<2,3>(originalLength+2*i,0).transpose();
// 		cout<<P<<"\n"<<endl;
		if(originalLength>3)
		{
			P.block(originalLength+2*i,3,2,2*i+originalLength-3)=Gv*P.block(0,3,3,2*i+originalLength-3);
// 			cout<<P<<"\n"<<endl;
			P.block(3,originalLength+2*i,2*i+originalLength-3,2)=P.block(originalLength+2*i,3,2,2*i+originalLength-3).transpose();
// 			cout<<P<<"\n"<<endl;
		}	
	}
}
double NormalizedAngle(double rAngle)
{
	while(rAngle<-PI)
		rAngle+=2*PI;
	while(rAngle>PI)
		rAngle-=2*PI;
	return rAngle;
}
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <vector>
#include "read_param.h"

using namespace boost;
void ReadParamFromFile(string lmfilename,string wpfilename,MatrixXd& lm,MatrixXd& wp)
{
	string slm;
	string swp;
	vector<string> veclm;
	vector<string> vecwp;
	double value=0.0;
	ifstream lmStream(lmfilename.c_str());
	ifstream wpStream(wpfilename.c_str());
	getline(lmStream,slm);
	getline(wpStream,swp);
	split(veclm,slm,is_any_of("\t"));
	int lmSize = veclm.size() / 2;
	lm.resize(2, lmSize);
	for(int i=0;i<veclm.size();i++)
	{
		stringstream ss(veclm[i]);
		ss>>value;
		value/=static_cast<double>(10);
		int row=i/lmSize;
		int col=i%lmSize;
		lm(row,col)=value;
	}
	split(vecwp,swp,is_any_of("\t"));
	int wpSize = vecwp.size() / 2;
	wp.resize(2, wpSize);
	for(int i=0;i<vecwp.size();i++)
	{
		stringstream ss(vecwp[i]);
		ss>>value;
		value/=static_cast<double>(10);
		int row=i/wpSize;
		int col=i%wpSize;
		wp(row,col)=value;
	}
}


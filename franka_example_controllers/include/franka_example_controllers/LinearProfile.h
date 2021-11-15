#pragma once
#include <cmath>
#include <iostream>

using namespace std; 

class LinearProfile {

private:
	double init_time; 
	double dq_c, tf, ddq_c; 
	double qi, dqi; 
public:
	LinearProfile(void);
	void init(double qi, double dqi, double dq_c, double tf, double init_time = 0); 
	void getPosFromPlanner(double curr_time, double &q, double &dq, double &ddq);
	void getPosFromPlanner(double curr_time, double &q); 

};





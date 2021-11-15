#pragma once
#include <cmath>
#include <iostream>

using namespace std; 

class TrapezoidalProfile {

private:
	double init_time; 
	double tc, tf, ddq_c; 
	double qi, qf; 
public:
	TrapezoidalProfile(void);
	void init(double qi, double qf, double tf, double init_time = 0); 
	void getPosFromPlanner(double curr_time, double &q, double &dq, double &ddq);
	void getPosFromPlanner(double curr_time, double &q); 

};





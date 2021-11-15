
#include <franka_example_controllers/TrapezoidalProfile.h>



TrapezoidalProfile::TrapezoidalProfile(void){
	init_time = 0; 
	qi = 0; 
	qf = 0; 
	ddq_c = 0; 
	tf = 0; 
	tc = 0; 
};


void TrapezoidalProfile::init(double qi, double qf, double tf, double init_time){
	cout << "Planner: qi = " << qi << " qf= "<<qf<<" tf = "<< tf <<endl; 
	double dq_c = 2*(qf-qi)/(tf*1.3);
	tc = (qi - qf + dq_c*tf)/dq_c;
	ddq_c = pow(dq_c,2)/(qi - qf + dq_c*tf);
	this->qi = qi; 
	this->qf = qf; 
	this->init_time = init_time; 
	this->tf = tf; 
}



void TrapezoidalProfile::getPosFromPlanner(double curr_time, double &q){
	double t = curr_time - init_time; 

	if(t>=0 && t<=tc){
	    q = qi + 0.5 * ddq_c * pow(t,2);

	} else if (t>tc && t<=tf-tc){
		q = qi + ddq_c*tc*(t - 0.5*tc);
		
	} else if( t>tf-tc && t<=tf){
		q = qf - 0.5*ddq_c*pow(tf-t, 2);
		
	} else{
		q = qf; 
		
	}

	if(abs(qi-qf)<0.0001){
		q = qi;
		
	}
	//cout << "t = " << t <<"qi = " << qi << " qf= "<<qf<<" q "<< q<<endl; 
}


void TrapezoidalProfile::getPosFromPlanner(double curr_time, double &q, double &dq, double &ddq){
	double t = curr_time - init_time; 

	if(t>=0 && t<=tc){
	    q = qi + 0.5 * ddq_c * pow(t,2);
	    dq = ddq_c * t;
	    ddq = ddq_c;
	} else if (t>tc && t<=tf-tc){
		q = qi + ddq_c*tc*(t - 0.5*tc);
		dq = ddq_c*tc;
		ddq = 0;
	} else if( t>tf-tc && t<=tf){
		q = qf - 0.5*ddq_c*pow(tf-t, 2);
		dq = -ddq_c*t + ddq_c*tf;
		ddq = -ddq_c;
	} else{
		q = qf; 
		dq = 0; 
		ddq = 0; 

	}

	if(abs(qi-qf)<0.0001){
		q = qi;
		dq = 0;
		ddq = 0;
	}

}





























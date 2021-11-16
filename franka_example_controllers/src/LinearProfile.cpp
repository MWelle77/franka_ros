
#include <franka_example_controllers/LinearProfile.h>



LinearProfile::LinearProfile(void){
	init_time = 0; 
	qi = 0; 
	ddq_c = 0; 
	tf = 0; 
	dq_c = 0; 
};


void LinearProfile::init(double qi, double dqi, double dq_c, double tf, double init_time){
	//cout << "Planner: dqi = " << dqi << " dqc= "<<dq_c <<" tf = "<< tf <<endl; 

	ddq_c=(dq_c-dqi)/tf;
	//cout << "ddq_c = " << ddq_c  <<endl; 


	this->qi = qi;  
	this->dqi = dqi; 
	this->dq_c = dq_c; 
	this->init_time = init_time; 
	this->tf = tf;

}



void LinearProfile::getPosFromPlanner(double curr_time, double &q){
	double t = curr_time - init_time; 


	if(t>=0 && t<=tf){
	    q = qi + 0.5 * ddq_c * pow(tf,2) + dqi*tf;	

	}
	else{
		double q_tf=qi + 0.5 * ddq_c * pow(tf,2) + dqi*tf;	
		q = q_tf + dq_c*t;  
		
	}

	if(abs(dqi-dq_c)<0.0001){
		q = qi;
	}
	
	//cout << "t = " << t <<"qi = " << qi << " qf= "<<qf<<" q "<< q<<endl; 
}

void LinearProfile::getPosFromPlanner(double curr_time, double &q, double &dq, double &ddq){
	double t = curr_time - init_time; 

	

	if(t>=0 && t<=tf){
	    ddq = ddq_c;	
	    dq = ddq_c * t +dqi;
	    q = qi + 0.5 * ddq_c * pow(t,2) + dqi*t;
	    
	} else{
		ddq = 0;
		dq = dq_c;  
		double q_tf=qi + 0.5 * ddq_c * pow(tf,2) + dqi*tf;		
		q = q_tf + dq_c*t; 
		
		

	}

	if(abs(dqi-dq_c)<0.0001){
		q = qi;
		dq = dq_c;
		ddq = 0;
	}
	//cout << "dqi= " << dqi << " dq_c= "<<dq_c <<" ddq_c = "<< ddq_c << " t= " << t <<endl; 
	//cout << "Planner get pos: q = " << qi << " dq= "<<dq <<" ddq = "<< ddq << " t= " << t <<endl; 


}






























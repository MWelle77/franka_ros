
#include <franka_example_controllers/PolinomialProfile.h>



PolinomialProfile::PolinomialProfile(void){
	init_time = 0; 
	qi = 0; 
	qf = 0; 
	tf = 0; 
	};


void PolinomialProfile::init(double qi, double qf, double tf, double init_time){
	cout << "Planner: qi = " << qi << " qf= "<<qf<<" tf = "<< tf <<endl; 

	this->qi = qi; 
	this->qf = qf; 
	this->init_time = init_time; 
	this->tf = tf; 

	Eigen::VectorXd qHat(6); 
	qHat << qi, 0, 0, qf, 0, 0;
    
    Eigen::MatrixXd A(6, 6);

    A << 0, 0, 0, 0, 0, 1,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 2, 0, 0,
        pow(tf,5), pow(tf,4), pow(tf,3), pow(tf,2), tf, 1,
        5*pow(tf,4), 4*pow(tf,3), 3*pow(tf,2), 2*tf, 1, 0,
        20*pow(tf,3), 12*pow(tf,2), 6*tf, 2, 0, 0; 

    coeff = A.inverse()*qHat;
    cout << "PLANNER: "<<coeff.transpose() <<endl; 

}



void PolinomialProfile::getPosFromPlanner(double curr_time, double &q, double &dq, double &ddq){

    double t = curr_time - init_time; 
    
    if(t >= tf){        
        q=qf;
        dq=0;
        ddq=0;
    } else{
        q = coeff(0)*pow(t,5) + coeff(1)*pow(t,4) + coeff(2)*pow(t,3) + coeff(3)*pow(t,2) + coeff(4)*t + coeff(5);
        dq = 5*coeff(0)*pow(t,4) + 4*coeff(1)*pow(t,3) + 3*coeff(2)*pow(t,2) + 2*coeff(3)*t + coeff(4);
        ddq = 20*coeff(0)*pow(t,3) + 12*coeff(1)*pow(t,2) + 6*coeff(2)*t + 2*coeff(3);
    }

    if(abs(qi-qf)<0.0001 ){        
        q=qi;
        dq=0;
        ddq=0;
    }

}






























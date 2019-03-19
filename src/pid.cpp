#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );
	void setDt(float dt);
	void setValues(double dt, double max, double min, double Kp, double Kd, double Ki);

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};

PID::PID( double dt, double max, double min, double kp, double kd, double ki )
{
    pimpl = new PIDImpl(dt,max,min,kp,kd,ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}

PID::~PID() 
{
    delete pimpl;
}

void PID::setVals(double dt, double max, double min, double Kp, double Kd, double Ki ){
	pimpl->setValues(dt, max, min, Kp, Kd, Ki);
}

/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

void PIDImpl::setValues(double dt, double max, double min, double Kp, double Kd, double Ki){
	_dt = dt;
	_max = max;
	_min = min;
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
}

void PIDImpl::setDt(float dt){
	_dt = dt;
}


double PIDImpl::calculate( double setpoint, double pv )
{
    
	bool d = 0;
	if(d)cout<<"set = "<<setpoint<<" actual = "<<pv<<"\n";

	// Calculate error
    double error = setpoint - pv;
	if(d)std::cout<<"error = "<<error<<"\n";

    // Proportional term
    double Pout = _Kp * error;
    if(d)std::cout<<"p term = "<<_Kp<<" Pout = "<<Pout<<"\n";

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;
    if(d)std::cout<<"i term = "<<_Ki<<" Iout = "<<Iout<<"\n";

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;
    if(d)std::cout<<"d term = "<<_Kd<<" Dout = "<<Dout<<"\n";

    // Calculate total output
    double output = Pout + Iout + Dout;
    if(0||d) cout<<"output = "<<output<<"\n";

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif

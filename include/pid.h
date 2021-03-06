#ifndef PID_H_
#define PID_H_


class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt, double max, double min, double Kp, double Kd, double Ki, bool* d );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
	void setDt(float dt);
	void setVals(double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif

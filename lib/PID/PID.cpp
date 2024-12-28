#include"PID.h"

PID::PID(float kp, float ki, float kd, int max_out, int min_out)
     :_kp(kp),_ki(ki),_kd(kd),_max_out(max_out),_min_out(min_out),_integral(0),_prev_error(0), _lastTime(0){}

int PID::compute(float input, float setpoint){
    float error = setpoint - input;
    unsigned long now = millis();
    float delta_time = (now - _lastTime)/1000.00;

    if(_lastTime == 0){
        _lastTime = now;
        return 0;
    } 
     
     _integral += error*delta_time;
     float differentiate = (error - _prev_error)/delta_time;
     float out = _kp*error + _ki*_integral + _kd*differentiate ;

     if(out > _max_out) out = _max_out;
     if(out < _min_out) out = _min_out;
     
     _lastTime = now;
     _prev_error = error;

     return out;


}     

void PID::set_Kp(float kp) {_kp = kp;}  
void PID::set_Ki(float ki) {_ki = ki;}  
void PID::set_Kd(float kd) {_kd = kd;} 

void PID::reset()
{
    _integral = 0;
    _prev_error = 0;
    _lastTime = millis();
}
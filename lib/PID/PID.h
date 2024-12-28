#ifndef PID_H
#define PID_H

#include<Arduino.h>

class PID{
    public:
      PID(float kp, float ki, float kd, int max_out, int min_out);

      int compute(float input, float setPoint);

      void set_Kp(float kp);
      void set_Ki(float ki);
      void set_Kd(float kd);

      void reset();

    private:
      float _kp;
      float _ki;
      float _kd;

      int _max_out;
      int _min_out;

      float _integral;
      float _prev_error;
      unsigned long _lastTime;

};


#endif 
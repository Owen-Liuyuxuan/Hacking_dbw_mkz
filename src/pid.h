#ifndef _PID_H
#define _PID_H

#include <cmath>

namespace pid_utils{
    float clip(float x, float max, float min)
    {
        if(x > max)
            return max;
        if(x < min)
            return min;

        return x;
    }
}

class PID
{
  public:
    float Kp;
    float Ki;
    float Kd;
    float period;

    float integrate_max;
    float output_max;
    float output_min;

    float integral;
    float last_time;
    bool is_initialized;
    PID(float period, float Kp, float Ki, float Kd, float integrate_max = 10, float output_max = 1, float output_min = -1)
    {
        this->period = period;
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->integrate_max = integrate_max;
        this->output_max = output_max;
        this->output_min = output_min;
        this->integral = 0;
        this->is_initialized = false;
    }
    float step(float delta)
    {
        float diff;
        if (this->is_initialized)
        {
            diff = (delta - this->last_time)/this->period;
        }
        else
        {
            diff = 0;
            this->is_initialized = true;
        }
        this->integral += delta * this->integral;
        this->integral = pid_utils::clip(this->integral, this->integrate_max, -this->integrate_max);
        this->last_time = delta;
        float output = this->Kp * delta + this->Ki * this->integral + this->Kd * diff;
        output = pid_utils::clip(output, this->output_max, this->output_min);
        return output;
    }
};

#endif // !_PID_H

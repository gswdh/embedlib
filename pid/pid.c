#include "pid.h"

void pid_init(pid_t * pid, pid_real_t set_point, pid_real_t kp, pid_real_t ki, pid_real_t kd)
{
    // Set point
    pid->set_point = set_point;

    // Coefficients
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    // Init states
    pid->integral = 0;
    pid->derivative = 0;
}

void pid_set_point(pid_t * pid, pid_real_t set_point)
{
    pid->set_point = set_point;
}

void pid_set_kp(pid_t * pid, pid_real_t kp)
{
    pid->kp = kp;
}

void pid_set_ki(pid_t * pid, pid_real_t ki)
{
    pid->ki = ki;
}

void pid_set_kd(pid_t * pid, pid_real_t kd)
{
    pid->kd = kd;
}

pid_real_t pid_calculate(pid_t * pid, pid_real_t feedback)
{
    // Calc the error
    pid_real_t error = pid->set_point - feedback;

    // Calculate the proportional term
    pid_real_t p_term = pid->kp * error;

    // Integral term. TODO: implement limits on the integrals value
    pid->integral += error;
    pid_real_t i_term = pid->ki * pid->integral;

    // Calc derivative
    pid_real_t d_term = pid->kd * (pid->derivative - feedback);
    pid->derivative = feedback;

    // Sum and return the result
    return p_term + i_term + d_term;
}
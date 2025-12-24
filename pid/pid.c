#include "pid.h"

#include <stddef.h>

pid_error_t
pid_init(pid_t *pid, const double set_point, const double kp, const double ki, const double kd)
{
    if (pid == NULL)
    {
        return PID_ERROR_NULL_POINTER;
    }

    // Set point
    pid->set_point = set_point;

    // Coefficients
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    // Init states
    pid->integral   = 0;
    pid->derivative = 0;

    return PID_OK;
}

pid_error_t pid_set_point(pid_t *pid, const double set_point)
{
    if (pid == NULL)
    {
        return PID_ERROR_NULL_POINTER;
    }
    pid->set_point = set_point;
    return PID_OK;
}

pid_error_t pid_set_kp(pid_t *pid, const double kp)
{
    if (pid == NULL)
    {
        return PID_ERROR_NULL_POINTER;
    }
    pid->kp = kp;
    return PID_OK;
}

pid_error_t pid_set_ki(pid_t *pid, const double ki)
{
    if (pid == NULL)
    {
        return PID_ERROR_NULL_POINTER;
    }
    pid->ki = ki;
    return PID_OK;
}

pid_error_t pid_set_kd(pid_t *pid, const double kd)
{
    if (pid == NULL)
    {
        return PID_ERROR_NULL_POINTER;
    }
    pid->kd = kd;
    return PID_OK;
}

pid_error_t pid_calculate(pid_t *pid, const double feedback, double *output)
{
    if ((pid == NULL) || (output == NULL))
    {
        return PID_ERROR_NULL_POINTER;
    }

    // Calc the error
    const double error = pid->set_point - feedback;

    // Calculate the proportional term
    const double p_term = pid->kp * error;

    // Integral term. TODO: implement limits on the integrals value
    pid->integral += error;
    const double i_term = pid->ki * pid->integral;

    // Calc derivative
    const double d_term = pid->kd * (pid->derivative - feedback);
    pid->derivative     = feedback;

    // Sum and return the result
    *output = p_term + i_term + d_term;
    return PID_OK;
}

char *pid_error_string(const pid_error_t error_code)
{
    switch (error_code)
    {
    case PID_OK:
        return "PID_OK";
    case PID_ERROR_NULL_POINTER:
        return "PID_ERROR_NULL_POINTER";
    default:
        return "Unknown error";
    }
}
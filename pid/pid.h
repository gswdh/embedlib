#ifndef _PID_H_
#define _PID_H_

typedef struct
{
    double set_point;
    double kp, ki, kd;
    double integral, derivative;
} pid_t;

typedef enum
{
    PID_OK = 0,
    PID_ERROR_NULL_POINTER,
} pid_error_t;

pid_error_t
pid_init(pid_t *pid, const double set_point, const double kp, const double ki, const double kd);
pid_error_t pid_set_point(pid_t *pid, const double set_point);
pid_error_t pid_set_kp(pid_t *pid, const double kp);
pid_error_t pid_set_ki(pid_t *pid, const double ki);
pid_error_t pid_set_kd(pid_t *pid, const double kd);
pid_error_t pid_calculate(pid_t *pid, const double feedback, double *output);

char *pid_error_string(const pid_error_t error_code);

#endif
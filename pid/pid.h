#ifndef _PID_H_
#define _PID_H_

#ifndef PID_REAL_TYPE
#define PID_REAL_TYPE double
#endif

typedef PID_REAL_TYPE pid_real_t;

typedef struct
{
    pid_real_t set_point;
    pid_real_t kp, ki, kd;
    pid_real_t integral, derivative;
} pid_t;

void pid_init(pid_t * pid, pid_real_t set_point, pid_real_t kp, pid_real_t ki, pid_real_t kd);
void pid_set_point(pid_t * pid, pid_real_t set_point);
void pid_set_kp(pid_t * pid, pid_real_t kp);
void pid_set_ki(pid_t * pid, pid_real_t ki);
void pid_set_kd(pid_t * pid, pid_real_t kd);
pid_real_t pid_calculate(pid_t * pid, pid_real_t feedback);

#endif
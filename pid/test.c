#include <stdio.h>

#include "pid.h"

#define PLANT_MODEL_HISTORY_LEN (100)

pid_real_t plant_model_history[PLANT_MODEL_HISTORY_LEN] = {0};

pid_real_t plant_model(pid_real_t drive)
{
    return drive * 0.2;
}

int main(void)
{
    printf("********** Running PID Tests **********\n");

    pid_t pid = {0};
    pid_init(&pid, 0, 0, 0, 0);

    printf("********** Finished PID Tests **********\n");

    return 0;
}
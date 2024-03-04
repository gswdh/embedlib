#include "stats.h"

#include <math.h>

double stats_mean(double *data, unsigned long len)
{
    double sum = 0;

    for (unsigned long i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return sum / (double)len;
}

double stats_stdev(double *data, unsigned long len)
{
    double mean = stats_mean(data, len);

    double stdev = 0;

    for (unsigned long i = 0; i < len; i++)
    {
        stdev += pow(data[i] - mean, 2);
    }

    return (double)sqrt(stdev / (double)len);
}
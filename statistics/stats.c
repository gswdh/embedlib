#include "stats.h"

#include <math.h>
#include <stdint.h>

/* ---- double ---- */

double stats_mean(const double *data, const uint32_t len)
{
    if (len == 0U)
    {
        return 0.0;
    }

    double sum = 0.0;
    for (uint32_t i = 0U; i < len; i++)
    {
        sum += data[i];
    }
    return sum / (double)len;
}

double stats_stdev(const double *data, const uint32_t len)
{
    if (len <= 1U)
    {
        return 0.0;
    }

    const double mean     = stats_mean(data, len);
    double       variance = 0.0;
    for (uint32_t i = 0U; i < len; i++)
    {
        const double diff = data[i] - mean;
        variance += diff * diff;
    }
    return sqrt(variance / (double)(len - 1U));
}

double stats_linear_interp(const double a, const double b, double coef)
{
    if (coef < 0.0)
    {
        coef = 0.0;
    }
    else if (coef > 1.0)
    {
        coef = 1.0;
    }
    else
    {
        /* coef is within range */
    }
    return a + (coef * (b - a));
}

/* ---- float ---- */

float stats_mean_f32(const float *data, const uint32_t len)
{
    if (len == 0U)
    {
        return 0.0F;
    }

    float sum = 0.0F;
    for (uint32_t i = 0U; i < len; i++)
    {
        sum += data[i];
    }
    return sum / (float)len;
}

float stats_stdev_f32(const float *data, const uint32_t len)
{
    if (len <= 1U)
    {
        return 0.0F;
    }

    const float mean     = stats_mean_f32(data, len);
    float       variance = 0.0F;
    for (uint32_t i = 0U; i < len; i++)
    {
        const float diff = data[i] - mean;
        variance += diff * diff;
    }
    return sqrtf(variance / (float)(len - 1U));
}

float stats_linear_interp_f32(const float a, const float b, float coef)
{
    if (coef < 0.0F)
    {
        coef = 0.0F;
    }
    else if (coef > 1.0F)
    {
        coef = 1.0F;
    }
    else
    {
        /* coef is within range */
    }
    return a + (coef * (b - a));
}

/* ---- int32_t ---- */

double stats_mean_i32(const int32_t *data, const uint32_t len)
{
    if (len == 0U)
    {
        return 0.0;
    }

    double sum = 0.0;
    for (uint32_t i = 0U; i < len; i++)
    {
        sum += (double)data[i];
    }
    return sum / (double)len;
}

double stats_stdev_i32(const int32_t *data, const uint32_t len)
{
    if (len <= 1U)
    {
        return 0.0;
    }

    const double mean     = stats_mean_i32(data, len);
    double       variance = 0.0;
    for (uint32_t i = 0U; i < len; i++)
    {
        const double diff = (double)data[i] - mean;
        variance += diff * diff;
    }
    return sqrt(variance / (double)(len - 1U));
}

double stats_linear_interp_i32(const int32_t a, const int32_t b, double coef)
{
    if (coef < 0.0)
    {
        coef = 0.0;
    }
    else if (coef > 1.0)
    {
        coef = 1.0;
    }
    else
    {
        /* coef is within range */
    }
    return (double)a + (coef * ((double)b - (double)a));
}

/* ---- uint32_t ---- */

double stats_mean_u32(const uint32_t *data, const uint32_t len)
{
    if (len == 0U)
    {
        return 0.0;
    }

    double sum = 0.0;
    for (uint32_t i = 0U; i < len; i++)
    {
        sum += (double)data[i];
    }
    return sum / (double)len;
}

double stats_stdev_u32(const uint32_t *data, const uint32_t len)
{
    if (len <= 1U)
    {
        return 0.0;
    }

    const double mean     = stats_mean_u32(data, len);
    double       variance = 0.0;
    for (uint32_t i = 0U; i < len; i++)
    {
        const double diff = (double)data[i] - mean;
        variance += diff * diff;
    }
    return sqrt(variance / (double)(len - 1U));
}

double stats_linear_interp_u32(const uint32_t a, const uint32_t b, double coef)
{
    if (coef < 0.0)
    {
        coef = 0.0;
    }
    else if (coef > 1.0)
    {
        coef = 1.0;
    }
    else
    {
        /* coef is within range */
    }
    return (double)a + (coef * ((double)b - (double)a));
}

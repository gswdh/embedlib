#ifndef __STS__
#define __STS__

#include <stdint.h>

/* ---- double ---- */

/**
 * @brief Arithmetic mean of a double dataset.
 * @param data Pointer to array of values.
 * @param len  Number of elements.
 * @return Mean, or 0.0 if len is 0.
 */
double stats_mean(const double *data, uint32_t len);

/**
 * @brief Sample standard deviation of a double dataset (N-1 denominator).
 * @param data Pointer to array of values.
 * @param len  Number of elements.
 * @return Standard deviation, or 0.0 if len <= 1.
 */
double stats_stdev(const double *data, uint32_t len);

/**
 * @brief Linear interpolation between two doubles.
 * @param a    Value at coef = 0.
 * @param b    Value at coef = 1.
 * @param coef Interpolation coefficient, clamped to [0, 1].
 * @return Interpolated value.
 */
double stats_linear_interp(double a, double b, double coef);

/* ---- float ---- */

/**
 * @brief Arithmetic mean of a float dataset.
 */
float stats_mean_f32(const float *data, uint32_t len);

/**
 * @brief Sample standard deviation of a float dataset (N-1 denominator).
 */
float stats_stdev_f32(const float *data, uint32_t len);

/**
 * @brief Linear interpolation between two floats.
 */
float stats_linear_interp_f32(float a, float b, float coef);

/* ---- int32_t ---- */

/**
 * @brief Arithmetic mean of an int32_t dataset.
 * @return Mean as double.
 */
double stats_mean_i32(const int32_t *data, uint32_t len);

/**
 * @brief Sample standard deviation of an int32_t dataset (N-1 denominator).
 * @return Standard deviation as double.
 */
double stats_stdev_i32(const int32_t *data, uint32_t len);

/**
 * @brief Linear interpolation between two int32_t values.
 * @return Interpolated value as double.
 */
double stats_linear_interp_i32(int32_t a, int32_t b, double coef);

/* ---- uint32_t ---- */

/**
 * @brief Arithmetic mean of a uint32_t dataset.
 * @return Mean as double.
 */
double stats_mean_u32(const uint32_t *data, uint32_t len);

/**
 * @brief Sample standard deviation of a uint32_t dataset (N-1 denominator).
 * @return Standard deviation as double.
 */
double stats_stdev_u32(const uint32_t *data, uint32_t len);

/**
 * @brief Linear interpolation between two uint32_t values.
 * @return Interpolated value as double.
 */
double stats_linear_interp_u32(uint32_t a, uint32_t b, double coef);

#endif

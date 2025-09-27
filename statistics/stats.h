#ifndef __STS__
#define __STS__

/**
 * @brief Calculate the arithmetic mean of a dataset
 * @param data Pointer to array of double values
 * @param len Number of elements in the data array
 * @return Arithmetic mean of the dataset, or 0.0 if len is 0
 *
 * Calculates the sum of all values divided by the number of elements.
 * Returns 0.0 if the length is zero to avoid division by zero.
 */
double stats_mean(double *data, unsigned long len);

/**
 * @brief Calculate the standard deviation of a dataset
 * @param data Pointer to array of double values
 * @param len Number of elements in the data array
 * @return Standard deviation of the dataset, or 0.0 if len is 0 or 1
 *
 * Calculates the square root of the variance using the formula:
 * sqrt(sum((x - mean)²) / (n - 1)) for sample standard deviation
 * Returns 0.0 if length is zero or one (insufficient data for standard deviation).
 */
double stats_stdev(double *data, unsigned long len);

/**
 * @brief Perform linear interpolation between two values
 * @param a First value (when coef = 0.0)
 * @param b Second value (when coef = 1.0)
 * @param coef Interpolation coefficient (0.0 to 1.0)
 * @return Interpolated value between a and b
 *
 * Performs linear interpolation using the formula:
 * result = a + coef * (b - a)
 *
 * When coef = 0.0, returns a
 * When coef = 1.0, returns b
 * When coef = 0.5, returns the midpoint between a and b
 */
double stats_linear_interp(double a, double b, double coef);

#endif
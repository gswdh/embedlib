#ifndef _ANGLES_H_
#define _ANGLES_H_

#include <math.h>

#define ANGLES_DEG_OF_RADIAN (180 / M_PI)
#define ANGLES_PI (M_PI)

void angles_xyz_to_ab(double x, double y, double z, double *a, double *b);
double angles_rad_to_deg(double rad);
double angles_deg_to_rad(double deg);

#endif

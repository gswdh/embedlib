#include "angles.h"

double angles_rad_to_deg(double rad)
{
    return rad * ANGLES_DEG_OF_RADIAN;
}

double angles_deg_to_rad(double deg)
{
    return deg / ANGLES_DEG_OF_RADIAN;
}

void angles_xyz_to_ab(double x, double y, double z, double *a, double *b)
{
    double theta = angles_rad_to_deg(atan(x / (sqrt(pow(y, 2) + pow(z, 2)))));
    double psi = angles_rad_to_deg(atan(y / (sqrt(pow(x, 2) + pow(z, 2)))));
    double phi = angles_rad_to_deg(atan(z / (sqrt(pow(x, 2) + pow(y, 2)))));

    // Calculate the a angle
    if (phi > 0)
    {
        *a = theta;
    }

    else
    {
        if (theta > 0)
        {
            *a = 180 - theta;
        }

        else
        {
            *a = -180 - theta;
        }
    }

    // Calculate the b angle
    if (phi > 0)
    {
        *b = psi;
    }

    else
    {
        if (psi > 0)
        {
            *b = 180 - psi;
        }

        else
        {
            *b = -180 - psi;
        }
    }
}
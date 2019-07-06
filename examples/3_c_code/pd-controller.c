/*
 * Build with:
 *
 * Windows: cl.exe /D_USRDLL /D_WINDLL pd-controller.c /link /DLL /OUT:windows\lib-pd-controller.dll
 *
 * Linux/macOS: gcc -shared -fPIC -o linux/pd-controller pd-controller.c
 *
 */

#include <math.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

Nothing qdiff(const double a[4], const double b[4], double c[4])
{
    c[0] = -a[3]*b[0] - a[2]*b[1] + a[1]*b[2] + a[0]*b[3];
    c[1] =  a[2]*b[0] - a[3]*b[1] - a[0]*b[2] + a[1]*b[3];
    c[2] = -a[1]*b[0] + a[0]*b[1] - a[3]*b[2] + a[2]*b[3];
    c[3] =  a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

double q2aa(const double q[4], double r[3])
{
    double half_theta = 0.;
    double theta = 0.;
    double one_over_sin_half_theta = 0.;
    double r_mag = 0.;
    int k = 0;
    half_theta = q[3];
    if (half_theta >  1.) { half_theta =  1.; };
    if (half_theta < -1.) { half_theta = -1.; };
    half_theta = acos(half_theta);
    if (half_theta != 0.)
    {
        theta = 2. * half_theta;
        one_over_sin_half_theta = 1./sin(half_theta);
        for (k = 0; k <= 2; ++k)
        {
            r[k] = q[k] * one_over_sin_half_theta;
            r_mag += r[k] * r[k];
        }
        r_mag = sqrt(r_mag);
        for (k = 0; k <= 2; ++k)
        {
            r[k] /= r_mag;
        }
    }
    else
    {
        theta = 0.;
        r[0] = 1.; r[1] = 0.; r[2] = 0.;
    }
    return theta;
}

EXPORT Nothing pd_controller(const double gains[2], const double q_TI[4], const double q_BI[4], const double w_BI_B[3], double f_B[3], double tau_B[3])
{
    double q_TB[4] = {0., 0., 0., 0.};
    double r_B[3]  = {0., 0., 0.};
    int k = 0;
    double theta = 0.;
    qdiff(q_TI, q_BI, q_TB);
    theta = q2aa(q_TB, r_B);
    for (k = 0; k <= 2; ++k)
    {
        tau_B[k] = gains[0] * theta * r_B[k] - gains[1] * w_BI_B[k];
        f_B[k] = 0.;
    }
}

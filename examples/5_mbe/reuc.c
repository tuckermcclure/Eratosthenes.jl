/*
 * Build with:
 *
 * Windows: cl.exe /D_USRDLL /D_WINDLL reuc.c /link /DLL /OUT:windows\lib-reuc.dll
 *
 * Linux/macOS: gcc -shared -fPIC -o linux/reuc reuc.c
 *
 */

#include <math.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

double PI = 3.141592653589793;

Nothing qdiff(const double a[4], const double b[4], double c[4])
{
    c[0] = -a[3]*b[0] - a[2]*b[1] + a[1]*b[2] + a[0]*b[3];
    c[1] =  a[2]*b[0] - a[3]*b[1] - a[0]*b[2] + a[1]*b[3];
    c[2] = -a[1]*b[0] + a[0]*b[1] - a[3]*b[2] + a[2]*b[3];
    c[3] =  a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

typedef struct {
    double real;
    double imag;
} Complex;

Complex cadd(Complex a, Complex b)
{
    Complex c;
    c.real = a.real + b.real;
    c.imag = a.imag + b.imag;
    return c;
}

Complex csub(Complex a, Complex b)
{
    Complex c;
    c.real = a.real - b.real;
    c.imag = a.imag - b.imag;
    return c;
}

Complex cscale(double a, Complex b)
{
    Complex c;
    c.real = a * b.real;
    c.imag = a * b.imag;
    return c;
}

Complex cconj(Complex a)
{
    Complex c = {a.real, -a.imag};
    return c;
}

Complex cmult(Complex a, Complex b)
{
    Complex c = {a.real * b.real - a.imag * b.imag, 
                 a.imag * b.real + a.real * b.imag};
    return c;
}
Complex cdiv(Complex a, Complex b) // a / b
{
    double scale = 1./(b.real * b.real + b.imag * b.imag);
    Complex c = {(a.real * b.real + a.imag * b.imag) * scale, 
                 (a.imag * b.real - a.real * b.imag) * scale};
    return c;
}

Complex csquare(Complex a)
{
    Complex c = {a.real * a.real - a.imag * a.imag, 
                 2. * a.imag * a.real};
    return c;
}

double cmag2(Complex a)
{
    return a.real * a.real + a.imag * a.imag;
}

Complex cmultim(Complex a)
{
    Complex c = {-a.imag, a.real};
    return c;
}

EXPORT Nothing reuc(double I, double kappa_c, double mu_c, double rho, double alpha, const double q_TI[4], const double q_BI[4], const double w_BI_B[3], double tau_B[3])
{
    double q[4] = {0., 0., 0., 0.};
    Complex omega = {0., 0.};
    Complex w = {0., 0.};
    double z = 0.;
    Complex w_dot = {0., 0.};
    double v = 0.;
    double eta = 0.;
    double temp = 0.;
    double kappa = 0.;
    double mu = 0.;
    Complex omega_d = {0., 0.};
    Complex e = {0., 0.};
    double eta_dot = 0.;
    Complex e_over_w = {0., 0.};
    double kappa_dot = 0.;
    double mu_dot = 0.;
    Complex omega_d_dot = {0., 0.};
    Complex u = {0., 0.};

    // Parameterize the rotation rate. Note that ω_BI_B[3] is presumed to be 0.
    omega.real = w_BI_B[0];
    omega.imag = w_BI_B[1];

    // Turn the target orientation and current orientation into the z-w parameters.
    qdiff(q_BI, q_TI, q);
    z = 2. * atan2(q[2], q[3]);
    w.real = (q[1] * q[2] + q[3] * q[0]) / (q[3]*q[3] + q[2]*q[2]);
    w.imag = (q[3] * q[1] - q[0] * q[2]) / (q[3]*q[3] + q[2]*q[2]);

    // Calculate its rate of change (Eq. 13a).
    w_dot = cadd(cscale(0.5, omega), cscale(0.5, cmult(cconj(omega), csquare(w))));

    // Form the ratio that separates the phase space into good and bad regions (Eq. 17).
    v   = cmag2(w);
    eta = z/v;

    // Form the gains (Eq. 21a and b).
    temp  = atan(rho * (1 - eta * eta));
    kappa = 2. * kappa_c / PI * temp;
    mu    = mu_c / PI * temp + 0.5 * mu_c;

    // Form the desired rate (Eq. 19).
    omega_d.imag = -mu * z;
    omega_d = cdiv(omega_d, cconj(w));
    omega_d = cadd(cscale(-kappa, w), omega_d);

    // Form the rate error (Eq. 40).
    e = csub(omega, omega_d);

    // Eq. 42
    e_over_w = cdiv(e, w);
    eta_dot = -mu * eta + kappa * (1. + v) * eta + e_over_w.imag - (1. + v) * eta * e_over_w.real;

    // Eq. 37a
    temp = (1. - eta*eta);
    temp = rho * eta * eta_dot / (1. + rho*rho * temp*temp);
    kappa_dot = -4. * kappa_c / PI * temp;
    mu_dot    = -2. * mu_c / PI * temp;

    // Calculate the rate of change of the desired rate (Eq. 36).
    omega_d_dot = cscale(-mu_dot * eta - mu * eta_dot, w);
    omega_d_dot = cadd(omega_d_dot, cscale(-mu * eta, w_dot));
    omega_d_dot = cmultim(omega_d_dot); // Multiply by im.
    omega_d_dot = cadd(cadd(cscale(-kappa_dot, w), cscale(-kappa, w_dot)), omega_d_dot);

    // Create the feedback control torque (Eq. 41).
    u = cmultim(cscale(mu * eta, w));
    u = cadd(cscale(kappa, w), u);
    u = cadd(omega, u);
    u = cadd(omega_d_dot, cscale(-alpha, u));

    // Turn the torque parameterization into a torque command.
    tau_B[0] = I * u.real;
    tau_B[1] = I * u.imag;
    tau_B[2] = 0.;
}

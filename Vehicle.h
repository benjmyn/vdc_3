
#ifndef VEHICLE_H
#define VEHICLE_H
#include "Log.h"

class Vehicle {
public:
    // Tire Data
    mat fxdata;
    mat fydata;
    mat mzdata;
    // Tires
    vec p94x, p94xs;
    vec p94y, p94ys;
    vec p94z, p94zs;
    // Static Alignment
    double camf, camr; // Static camber [deg]
    double toef, toer; // Static toe [deg]
    double kingpin, caster, trail, scrub;
    // Dynamic Alignment (NEW)
    double ack; // Ackermann ratio [deg/deg]
    double steer_ratio; // Steering ratio [deg/deg]
    double anti_lift, anti_dive, anti_squat; // Anti-effects [-]
    double cam_gain_f, cam_gain_r; // Camber gain in heave [deg/m]
    double toe_gain_f, toe_gain_r; // Toe gain in heave [deg/m]
    // Mass-Inertia
    double m; // Mass [kg]
    double izz; // Inertia [kg-m2]
    // Center of Gravity
    double h; // CG height [m]
    double fwt; // Front weight [-]
    // Footprint
    double l; // Wheelbase [m]
    double tf, tr; // Front and rear track widths [m]
    // Roll Centers
    double zf, zr; // Roll center heights [m]
    // Springing
    double ksf, ksr; // Spring stiffness [N/m]
    double kaf, kar; // ARB stiffness [N/m]
    double kpf, kpr; // Pneumatic stiffness [N/m]
    // Motion Ratios
    double mrsf, mrsr; // Sprung motion ratio [-]
    double mraf, mrar; // ARB motion ratio [-]
    // Drivetrain
    double fpb; // Front braking [-]
    double crr; // Rolling resistance [N/N]
    double Re; // Effective radius [m]
    double fpt; // Static front torque [-]
    // Torque vectoring
    bool tv_enabled; // For comparison use
    double trq_upper_limit; // (after gearing)
    double trq_lower_limit; // (after gearing, throttle only)
    double dTf_str, dTr_str; // Torque difference from steer (Nm/deg) - linear for now
    double dTf_yaw, dTr_yaw; // Torque difference from yaw (Nm/deg) - linear for now
    double dfpt_str; // Frontward torque bias with steer (-/deg) - linear for now
    // Aerodynamics
    double cxa, cza;
    vec cxe, cze;
    double cpx, cpz;


    // Calculated Attributes
    double a, b; // Front and rear axle to CG lengths [m]
    double krsf, krsr; // Sprung & ARB roll rates
    double krpf, krpr; // Pneumatic roll rates
    double khf, khr, kh; // Total heave rates
    double krf, krr, kr; // Total roll rates
    field<vec> pos_whl; // Wheel position vectors

    Vehicle();
    // Load base parameters from files then calculate leftovers
    void LoadSuspension();
    void LoadTiresLat(string filepath, double &pressure);
    void LoadTiresLong(string filepath, double &pressure);
    void LoadAero();
    void LoadPowertrain();
    void LoadCalculatedAttributes();
    // Dynamic Results (sideslip-frame)
    void RadiusYawMoment(LogYmd &log, const int &refines, const double &yaw, const double &steer, const double &R, const double &T);
    void VelocityYawMoment(LogYmd &log, int refines, const double &yaw, const double &steer, const double &v, const double &T);
    double GetAx(const double &yaw, const vec &fx, const vec &fy, const double &fxa) const;
    double GetAy(const double &yaw, const vec &fx, const vec &fy) const;
    double GetAa(const vec &fx, const vec &fy, const vec &mz) const;
    // Update either V or R (sideslip-frame)
    double GetV(const double &R, const double &ay) const;
    double GetR(const double &v, const double &ay) const;
    // Update tire inputs (tire-frame)
    vec GetSlip(const double &yaw, const vec &str, const double &R);
    vec GetTorque(const double &T, const double &R, const double &yaw, const double &steer, const double &v); // Torque Vectoring
    // Theoretical (pacejka)/Possible (tire) forces (tire-frame)
    vec PacejkaFx(const vec &fz, const vec &inc); // Just maximum value
    vec PacejkaFy(const vec &slip, const vec &fz, const vec &inc); // Based on slip, inclination
    vec PacejkaMz(const vec &slip, const vec &fz); // Based on slip, ??
    vec TireFx(const vec &Tw, const vec &fz, const vec &inc, uvec &fxflags); // Minimum of demanded vs avail
    vec TireFy(const vec &fxt, const vec &slip, const vec &fz, const vec &inc); // Based on fx
    vec TireMz(const vec &slip, const vec &fz, const vec &inc, const vec &fyt); // Scaled to proportion of ideal fy
    field<vec> ConvTireToCorner(const vec &fxt, const vec &fyt, const vec &str); // Tire forces -> Corner forces
    // Aero forces (car-frame)
    double AeroFx(const double &v, const double &yaw, const double &roll, const double &pitch, const double &heave);
    double AeroFz(const double &v, const double &yaw, const double &roll, const double &pitch, const double &heave);
    double AeroMy(const double &fxa); // Assume static CP for now...
    // Weight Transfer (car-frame)
    vec StaticLoad();
    vec AeroLoad(const double &fza, const double &mya);
    vec LongLoadTransfer(const vec &fx);
    vec LatLoadTransfer(const vec &fy, const double &pitch, const double &heave);
    vec SteerLoadTransfer(const double &steer);
    vec TotalLoad(const vec &fx, const vec &fy, const double &fza, const double &mya, const double &steer, const double &pitch, const double &heave);
    // Update car orientation (car-frame)
    double GetRoll(const vec &fy, const double &heave);
    double GetPitch(const vec &fx); // ADD JACKING FORCES
    double GetHeave(const vec &fz); // ADD JACKING FORCES
    // Update alignment (tire-frame)
    // NEED TO ADD COMPLIANCE!!!!!!!!
    vec GetBump(const double &roll, const double &pitch, const double &heave);
    vec GetInclination(const double &steer, const double &roll, const double &pitch, const double &heave);
    vec GetSteer(const double &steer, const double &roll, const double &pitch, const double &heave);
};

#endif //VEHICLE_H
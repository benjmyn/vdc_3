
#define ARMA_DONT_USE_BLAS
#define sind(x) (sin(x / 57.3))
#define atand(x) (57.3 * atan(x))
#include <armadillo>
using namespace std;
using namespace arma;
#include "xform.h"
#include "Vehicle.h"

Vehicle::Vehicle() {
    // Loaded Values
    m = 285;
    izz = 150;
    a = 0.8;
    b = 0.7;
    cpx = 0.1;
    cpz = 0.1;
    cxe = {0, 0, 0, 0};
    cze = {0, 0, 0, 0};
    h = 0.300;
    tf = 1.194;
    tr = 1.194;
    zf = 0.089;
    zr = 0.089;
    Re = 0.200;
    camf = 0;
    camr = 0;
    cam_gain_f = 50;
    cam_gain_r = 70;
    toef = -0.5;
    toer = 0.4;
    ack = 0.1;
    fpb = 0.75;
    fpt = 0.00;


    // Calculated Values
    l = a + b;
    pos_whl.set_size(4);
    pos_whl(0) = { a,  tf/2};
    pos_whl(1) = { a, -tf/2};
    pos_whl(2) = {-b,  tr/2};
    pos_whl(3) = {-b, -tr/2};

    p94x = {0.0, -0.674, 3.44};
    p94xs = {0.53};
    p94y = {1.250, -0.14, 2.65, 2500.0, 611.0, 0.05, 1.03, -2.39, 0.0, 0.0, 0.100, 0.0, 0.0, 0.0, -22, 0.003, 0.030, 0.0};
    p94ys = {0.53, 1.10};

    // Heave & roll stiffnesses
    mraf = 1.0;
    mrar = 1.0;
    mrsf = 1.0;
    mrsr = 1.0;
    kaf = 7000;
    kar = 7000;
    ksf = 35000;
    ksr = 35000;
    kpf = 125000;
    kpr = 125000;
    krsf = (pow(mrsf, -2) * ksf + pow(mraf, -2) * kaf) * pow(tf, 2) / 2;
    krsr = (pow(mrsr, -2) * ksr + pow(mrar, -2) * kar) * pow(tr, 2) / 2;
    krpf = kpf * pow(tf, 2) / 2;
    krpr = kpr * pow(tr, 2) / 2;
    khf = (pow(mrsf, -2) * ksf * kpf) / (pow(mrsf, -2) * ksf + kpf);
    khr = (pow(mrsr, -2) * ksr * kpr) / (pow(mrsr, -2) * ksr + kpr);
    kh = khf + khr;
    krf = (krsf * krpf) / (krsf + krpf);
    krr = (krsr * krpr) / (krsr + krpr);
    kr = krf + krr;
    cout << "Initialized Vehicle" << endl;
}
void Vehicle::LoadCalculatedAttributes() {
    l = a + b;
}
void Vehicle::RadiusYawMoment(LogYmd &log, const double &yaw, const double &steer, const double &R, const double &T) {
    // Reset local variables
    double v = 20; // Initial guess velocity (is this optimal??)
    double v_old = v;
    // Alignment
    double roll = 0;
    double pitch = 0;
    double heave = 0;
    vec inc = {0, 0, 0, 0};
    vec str = {0, 0, 0, 0};
    vec slip = {0, 0, 0, 0};
    // Tire forces/moments
    vec Tw = {0, 0, 0, 0};
    vec fxt = {0, 0, 0, 0};
    vec fyt = {0, 0, 0, 0};
    // Corner forces/moments
    field<vec> f(4);
    vec fx = {0, 0, 0, 0};
    vec fy = {0, 0, 0, 0};
    vec fz = {0, 0, 0, 0};
    vec mz = {0, 0, 0, 0};
    // Aero forces/moments
    double fxa = 0;
    double fza = 0;
    double mya = 0;
    // Vehicle dynamics (wow!)
    double ax = 0;
    double ay = 0;
    double aa = 0;
    // Convergence loop
    for (int iter = 0; iter < 4; ++iter) {
        // Update alignment
        roll = GetRoll(fy);
        pitch = GetPitch(fz, mya);
        heave = GetHeave(fz);
        inc = GetInclination(steer, roll, pitch, heave);
        str = GetSteer(steer, roll, pitch, heave);
        slip = GetSlip(yaw, str, R);
        // Update aerodynamic forces
        fxa = AeroFx(v, yaw, roll, pitch, heave);
        fza = AeroFz(v, yaw, roll, pitch, heave);
        mya = AeroMy(fxa);
        // Update normal forces
        fz = TotalLoad(fx, fy, fza, mya, pitch, heave);
        // Update tire forces
        Tw = GetTorque(T, R, yaw, steer, v);
        fxt = TireFx(Tw, fz, inc);
        fyt = TireFy(fxt, slip, fz, inc);
        // Update corner forces
        f = ConvTireToCorner(fxt, fyt, str);
        for (int i = 0; i < 4; ++i) {
            fx(i) = f(i)(0);
            fy(i) = f(i)(1);
        }
        mz = TireMz(slip, fz, inc, fyt);
        // Update acceleration
        ax = GetAx(yaw, fx, fy);
        ay = GetAy(yaw, fx, fy);
        aa = GetAa(fx, fy, mz);
        // Update velocity (iterative variable)
        v = 0.9 * sqrt(R * ay) + 0.1 * v_old;
        v_old = v;
    }
    log.aa = aa;
    log.ax = ax;
    log.ay = ay;
    log.v = v;
}
void Vehicle::VelocityYawMoment(LogYmd &log, const int refines, const double &yaw, const double &steer, const double &v, const double &T) {
    // Reset local variables
    double R = 1000; // Initial guess radius (is this optimal??)
    double R_old = R;
    // Alignment
    double roll = 0;
    double pitch = 0;
    double heave = 0;
    vec inc = {0, 0, 0, 0};
    vec str = {0, 0, 0, 0};
    vec slip = {0, 0, 0, 0};
    // Tire forces/moments
    vec Tw = {0, 0, 0, 0};
    vec fxt = {0, 0, 0, 0};
    vec fyt = {0, 0, 0, 0};
    // Corner forces/moments
    field<vec> f(4);
    vec fx = {0, 0, 0, 0};
    vec fy = {0, 0, 0, 0};
    vec fz = {0, 0, 0, 0};
    vec mz = {0, 0, 0, 0};
    // Aero forces/moments
    double fxa = 0;
    double fza = 0;
    double mya = 0;
    // Vehicle dynamics (wow!)
    double ax = 0;
    double ay = 0;
    double ay_old = 0;
    double ay_i = 0;
    double aa = 0;
    double aa_old = 0;
    // Convergence loop
    //const int ITER_TOTAL = floor(1500/pow(v,2) + 5);
    const int ITER_TOTAL = refines;
    for (int iter = 0; iter < ITER_TOTAL; ++iter) {
        // Update alignment
        roll = GetRoll(fy);
        pitch = GetPitch(fz, mya);
        heave = GetHeave(fz);
        inc = GetInclination(steer, roll, pitch, heave);
        str = GetSteer(steer, roll, pitch, heave);
        slip = GetSlip(yaw, str, R);
        // Update aerodynamic forces
        fxa = AeroFx(v, yaw, roll, pitch, heave);
        fza = AeroFz(v, yaw, roll, pitch, heave);
        mya = AeroMy(fxa);
        // Update normal forces
        fz = TotalLoad(fx, fy, fza, mya, pitch, heave);
        // Update tire forces
        Tw = GetTorque(T, R, yaw, steer, v);
        fxt = TireFx(Tw, fz, inc);
        fyt = TireFy(fxt, slip, fz, inc);
        // Update corner forces
        f = ConvTireToCorner(fxt, fyt, str);
        for (int i = 0; i < 4; ++i) {
            fx(i) = f(i)(0);
            fy(i) = f(i)(1);
        }
        mz = TireMz(slip, fz, inc, fyt);
        // Update acceleration
        ax = GetAx(yaw, fx, fy);
        ay = GetAy(yaw, fx, fy);
        aa = GetAa(fx, fy, mz);
        ay_i = 0.6 * ay + 0.4 * ay_old; // (P-CONTROLLER FOR CONVERGENCE)
        // Check if error OK
        if (iter == ITER_TOTAL - 1 && abs(1 - ay/ay_old) > 0.01) {
            static int err_total = 0;
            printf("Error #%i) %.1f%% at yaw = %.0f, steer = %.0f\n",
                ++err_total, 100 * (1 - ay/ay_old), yaw, steer);
        }
        if (iter >= 4 && abs(1 - ay/ay_old) < 0.001 && abs(1 - aa/aa_old) < 0.001) {
            iter = ITER_TOTAL; // break out
        }
        aa_old = aa;
        ay_old = ay;
        // Update radius (iterative variable) (P-CONTROLLER FOR CONVERGENCE)
        R = sign(ay_i) * (abs(0.6 * pow(v, 2) / ay_i) + abs(0.4 * R_old));
        R_old = R;
        //if (abs(R) > 10000) {
        //    R = INFINITY; // Band-aid
        //}
        //cout << "fy:\n" << fy << endl;
        //cout << "slip:\n" << slip << endl;
        //cout << "ay: " << ay << endl;
        //cout << "R: " << R << endl;

    }
    log.aa = aa;
    log.ax = ax;
    log.ay = ay;
    log.R = R;
    log.slip = slip;
    log.fxt = fxt;
    log.fyt = fyt;
    log.fz = fz;
    log.yaw = yaw;
    log.steer = steer;
}
double Vehicle::GetAx(const double &yaw, const vec &fx, const vec &fy) const {
    double ax = 0;
    for (int i = 0; i < 4; ++i) {
        vec fxb = xform(vec({fx(i),fy(i)}), yaw);
        ax += fxb(0) / m;
    }
    return ax;
}
double Vehicle::GetAy(const double &yaw, const vec &fx, const vec &fy) const {
    double ay = 0;
    for (int i = 0; i < 4; ++i) {
        ay += xform(vec({fx(i),fy(i)}), yaw)(1) / m;
    }
    return ay;
}
double Vehicle::GetAa(const vec &fx, const vec &fy, const vec &mz) const {
    double aa = 0;
    for (int i = 0; i < 4; ++i) {
        aa += (pos_whl(i)(0) * fy(i) - pos_whl(i)(1) * fx(i) + mz(i)) / izz;
    }
    return aa;
}
double Vehicle::GetV(const double &R, const double &ay) const {
    return sqrt(R * ay);
}
double Vehicle::GetR(const double &v, const double &ay) const {
    return pow(v, 2) / ay;
}
vec Vehicle::GetSlip(const double &yaw, const vec &str, const double &R) {
    vec beta_i(4);
    field<vec> R_to_pos(4);
    vec R_to_CG = xform(vec({0, R}), yaw);
    static const vec glob_y = {0, 1};
    for (int i = 0; i < 4; ++i) {
        R_to_pos(i) = R_to_CG - pos_whl(i);
        beta_i(i) = vecang(vec({0, 1}), R_to_pos(i));
        if (beta_i(i) < -90) {
            beta_i(i) += 180; // Band-aid fix
        }
    }
    return beta_i + 57.3 * vec({0, 0, b/R, b/R}) - str;
}
vec Vehicle::GetTorque(const double &T, const double &R, const double &yaw, const double &steer, const double &v) {
    double fpt_tv;
    double c_prld_f, c_ramp_f, c_prld_r, c_ramp_r;
    if (T >= 0) { // On-throttle
        //Tw = {0, 0, T/2, T/2};
        fpt_tv = fpt;
        c_prld_f = sign(R) * 15 * 0.3;
        c_ramp_f = sign(R) * sind(40) * 0.3;
        c_prld_r = sign(R) * 15 * 0.3;
        c_ramp_r = sign(R) * sind(40) * 0.3;
    }
    else { // Braking
        //Tw = {fpb*T/2, fpb*T/2, (1-fpb)*T/2, (1-fpb)*T/2};
        fpt_tv = fpb;
        c_prld_f = 0;
        c_ramp_f = 0;
        c_prld_r = 0;
        c_ramp_r = 0;
    }
    double Tf = fpt_tv * T;
    double Tr = (1 - fpt_tv) * T;
    double dTf = Tf * c_ramp_f + c_prld_f + 0 * abs(yaw) * steer;
    double dTr = Tr * c_ramp_r + c_prld_r + 0 * abs(yaw) * steer;
    vec Tw = 0.5 * vec({Tf - dTf, Tf + dTf, Tr - dTr, Tr + dTr});
    // debug
    //Tw = vec({0, 0, 0, 0});
    return Tw;
}
field<vec> Vehicle::ConvTireToCorner(const vec &fxt, const vec &fyt, const vec &str) {
    field<vec> f(4);
    for (int i = 0; i < 4; ++i) {
        f(i) = xform(vec({fxt(i), fyt(i)}), str(i));
    }
    return f;
}
vec Vehicle::PacejkaFx(const vec &fz, const vec &inc){
    const vec fzn = -fz;
    const vec fzkn = fzn / 1000;
    return (p94x(2) * fzkn + p94x(1)) % (1 - 0.01 * abs(inc)) % fzn * p94xs(0);
}
vec Vehicle::PacejkaFy(const vec &slip, const vec &fz, const vec &inc) {
    const vec fzn = -fz;
    const vec fzkn = fzn / 1000;
    const vec C = slip * 0 + p94y(0);
    const vec D = fzn % (p94y(1) * fzkn + p94y(2)) % (1 - p94y(15) * pow(inc, 2));
    const vec BCD = p94y(3) * sind(2 * atand(fzkn / p94y(4))) % (1 - p94y(5) * abs(inc));
    const vec B = BCD / (C % D);
    const vec H = p94y(8) * fzkn + p94y(9) + p94y(10) * inc;
    const vec V = p94y(11) * fzn + p94y(12) + (p94y(13) * fzkn + p94y(14)) % inc % fzkn;
    const vec E = (p94y(6) * fzkn + p94y(7)) % (1 - (p94y(16) * inc  + p94y(17)));
    const vec Cs = C * p94ys(1);
    const vec Ds = D * p94ys(0);
    const vec Bs = B / (p94ys(0) * p94ys(1));
    const vec Vs = V * p94ys(0);
    const vec Hs = H * p94ys(0);
    const vec Bx1 = Bs % (slip + Hs);
    vec fy = Ds % sind(Cs % atand(Bx1 - E % (Bx1 - atand(Bx1)))) + Vs;
    return fy;
}
vec Vehicle::PacejkaMz(const vec &slip, const vec &fz) {
    return vec({0, 0, 0, 0}); // Placeholder!!!
}
vec Vehicle::TireFx(const vec &Tw, const vec &fz, const vec &inc) {
    vec fxp = PacejkaFx(fz, inc); // Tire capability
    vec fxT = Tw / Re; // Demanded torque
    vec fxt(4);
    for (int i = 0; i < 4; ++i) {
        if (abs(fxp(i)) > abs(fxT(i))) {
            // If demanded less than potential, use demanded
            fxt(i) = fxT(i);
        }
        else {
            // If demanded greater, use potential *** reduced for wheelspin
            fxt(i) = 0.7 * fxp(i) * sign(Tw(i));
        }
    }
    //fxt += 0.00 * fz; // Rolling resistance (fz is negative)
    return fxt;
}
vec Vehicle::TireFy(const vec &fxt, const vec &slip, const vec &fz, const vec &inc) {
    vec fxp = PacejkaFx(fz, inc);
    vec fyp = PacejkaFy(slip, fz, inc); // Based on pacejka equation
    vec fyt = fyp % sqrt(1 - square(fxt / (1.00*fxp))); // Based on traction ellipse (conservative approximation)
    return fyt;
}
vec Vehicle::TireMz(const vec &slip, const vec &fz, const vec &inc, const vec &fyt) {
    return vec({0, 0, 0, 0}); // Placeholder!!!
}
double Vehicle::AeroFx(const double &v, const double &yaw, const double &roll, const double &pitch, const double &heave) {
    // Drag equation
    double ex = prod(1 - cxe % vec({yaw, roll, pitch, heave}));
    return -0.5 * 1.225 * ex * cxa * pow(v, 2);
}
double Vehicle::AeroFz(const double &v, const double &yaw, const double &roll, const double &pitch, const double &heave) {
    // Downforce equation
    double ez = prod(1 - cze % vec({yaw, roll, pitch, heave}));
    return 0.5 * 1.225 * ez * cza * pow(v, 2);
}
double Vehicle::AeroMy(const double &fxa) {
    return (cpz + h) * fxa;
}
vec Vehicle::StaticLoad() {
    return -4.9 * m * vec({b, b, a, a}) / l;
}
vec Vehicle::AeroLoad(const double &fza, const double &mya) {
    // Distribute loads
    return 0.5 * fza * vec({b + cpx, b + cpx, a - cpx, a - cpx}) / l + 0.5 * mya / vec({-l, -l, l, l});
}
vec Vehicle::LongLoadTransfer(const vec &fx) {
    // Distribute long. loads from tires only (aero accounted for)
    return 0.5 * sum(fx) * h / vec({l, l, -l, -l});
}
vec Vehicle::LatLoadTransfer(const vec &fy, const double &pitch, const double &heave) {
    // LLTD equations
    const double qf = zf; // const for now
    const double qr = zr; // const for now
    const double q = fwt * qr + (1 - fwt) * qf;
    double yf = fy(0) + fy(1);
    double yr = fy(2) + fy(3);
    vec xi = {(krf / kr) * (h - q) + qf + (-qf / krpf) * (krf * krr / kr),
                    (krf / kr) * (h - q) + 0 + (qr / krpr) * (krf * krr / kr),
                    (krr / kr) * (h - q) + 0 +(qf / krpf) * (krf * krr / kr),
                    (krr / kr) * (h - q) + qr + (-qr / krpr) * (krf * krr / kr)};
    double dzf = (xi(0) * yf + xi(1) * yr) / tf;
    double dzr = (xi(2) * yf + xi(3) * yr) / tr;
    return vec({dzf, -dzf, dzr, -dzr});
}
vec Vehicle::TotalLoad(const vec &fx, const vec &fy, const double &fza, const double &mya, const double &pitch, const double &heave) {
    return StaticLoad() + AeroLoad(fza, mya) + LongLoadTransfer(fx) + LatLoadTransfer(fy, pitch, heave);
}
double Vehicle::GetRoll(const vec &fy) {
    // moment over angular roll stiffness
    return 0;
}
double Vehicle::GetPitch(const vec &fz, const double &mya) {
    // Difference between axle heaves over wheelbase (SAA)
    return 0;
}
double Vehicle::GetHeave(const vec &fz) {
    // (fz - fzs) / combined stiffness
    return 0;
}
vec Vehicle::GetBump(const double &roll, const double &pitch, const double &heave) {
    // Get individual wheel bumps by summing displacements from these 3
    vec bump_roll = roll/57.3 * vec({-tf/2, tf/2, -tr/2, tr/2});
    vec bump_pitch = pitch/57.3 * vec({a, a, -b, -b});
    vec bump_heave = heave * vec({1, 1, 1, 1});
    return bump_roll + bump_pitch + bump_heave;
}
vec Vehicle::GetInclination(const double &steer, const double &roll, const double &pitch, const double &heave) {
    // Get camber on each wheel (heavy assumptions being made here, camber will be rough)
    vec bump = GetBump(roll, pitch, heave);
    vec inc_bump = vec({-camf, camf, -camr, camr}) +
                        bump % vec({-cam_gain_f, cam_gain_f, -cam_gain_r, cam_gain_r});
    vec inc_steer = {0, 0, 0, 0};
    vec inc_comp = {0, 0, 0, 0};
    return inc_bump + inc_steer + inc_comp + roll;
}
vec Vehicle::GetSteer(const double &steer, const double &roll, const double &pitch, const double &heave) {
    // Get steering angles using bump steer (minimal but worth having), ackermann
    vec bump = GetBump(roll, pitch, heave);
    vec str_steer = {steer * (1 + ack/2 * sign(steer)),
                         steer * (1 - ack/2 * sign(steer)),
                         0,
                         0};
    vec str_bump = vec({-toef, toef, -toer, toer}) +
                        bump % vec({-toe_gain_f, toe_gain_f, -toe_gain_r, toe_gain_r});
    vec str_comp = {0, 0, 0, 0};
    return str_bump + str_steer + str_comp;
}


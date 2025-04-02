
#ifndef XFORM_H
#define XFORM_H

inline vec xform(const vec &input, const double &ang_deg) {
    const mat xform = {{cos(ang_deg/57.3), -sin(ang_deg/57.3)},
                         {sin(ang_deg/57.3), cos(ang_deg/57.3)}};
    return xform * input;
}

inline double vecang(const vec &a, const vec &b) {
    return 57.3 * (atan2(b(1),b(0)) - atan2(a(1),a(0)));
}

#endif //XFORM_H

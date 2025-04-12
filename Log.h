//
// Created by benjmyn on 3/29/25.
//

#ifndef LOG_H
#define LOG_H

struct LogYmd {
    double aa;
    double ax;
    double ay;
    double yaw;
    double steer;
    double v;
    double R;
    double T;
    double roll;
    double pitch;
    double heave;
    vec slip;
    vec bump;
    vec fxt;
    vec fyt;
    vec fz;
    vec mz;
    vec Tw;
    vec cam;
    vec toe;
    uvec fxflags;
};
struct LogLts {

};

#endif //LOG_H

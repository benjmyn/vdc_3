#ifndef VISUALYMD_H
#define VISUALYMD_H


inline field<LogYmd> VisualYmdCR(Vehicle &Car, const double &R, const double &T, const double &yaw_range, const double &steer_range, const int &yaw_ct, const int &steer_ct) {
    field<LogYmd> log(yaw_ct, steer_ct);
    vec yaw_vec = linspace(-yaw_range, yaw_range, yaw_ct);
    vec steer_vec = linspace(-steer_range, steer_range, steer_ct);
    for (int i = 0; i < yaw_ct; ++i) {
        for (int j = 0; j < steer_ct; ++j) {
            log(i, j).yaw = yaw_vec(i);
            log(i, j).steer = steer_vec(j);
            log(i, j).R = R;
            log(i, j).T = T;
            Car.RadiusYawMoment(log(i,j), log(i,j).yaw, log(i,j).steer, log(i,j).R, log(i,j).T);
        }
    }
    return log;
}
inline field<LogYmd> VisualYmdCV(Vehicle &Car, const int &refines, const double &v, const double &T, const double &yaw_range, const double &steer_range, const int &yaw_ct, const int &steer_ct) {
    field<LogYmd> log(yaw_ct, steer_ct);
    vec yaw_vec = linspace(-yaw_range, yaw_range, yaw_ct);
    vec steer_vec = linspace(-steer_range, steer_range, steer_ct);
    for (int i = 0; i < yaw_ct; ++i) {
        for (int j = 0; j < steer_ct; ++j) {
            log(i, j).yaw = yaw_vec(i);
            log(i, j).steer = steer_vec(j);
            log(i, j).v = v;
            log(i, j).T = T;
            Car.VelocityYawMoment(log(i,j), refines, log(i,j).yaw, log(i,j).steer, log(i,j).v, log(i,j).T);
        }
    }
    return log;
}

#endif //VISUALYMD_H

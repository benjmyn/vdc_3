#ifndef DIFF_H
#define DIFF_H

inline vec diff(const vec &x, const vec &y) {
    vec dydx(x.size());
    for (int i = 0; i < x.n_elem-1; ++i) {
        dydx(i) = (y(i+1) - y(i)) / (x(i+1) - x(i));
    }
    dydx(dydx.size()-1) = x(dydx.size()-2);
    return dydx;
}
inline void diffdb(double* dydx, const double* x, const double* y, size_t size) {
    for (size_t i = 0; i < size - 1; ++i) {
        dydx[i] = (y[i+1] - y[i]) / (x[i+1] - x[i]);
    }
    dydx[size-1] = dydx[size-2];
    //for (size_t i = 1; i < size; ++i) {
    //    dydx[i] =
    //}
}

#endif //DIFF_H

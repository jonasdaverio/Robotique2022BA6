#ifndef MACROS_H
#define MACROS_H

#define STANDARD_GRAVITY 9.80665f //in m^2/s
#define WHEEL_TRACK 0.0538f //in m, empirical
#define WHEEL_CIRCUMFERENCE 0.13f //in m, empirical
#define STEPNB 1000

#define MTOSTEP(f) f/WHEEL_CIRCUMFERENCE*STEPNB
#define STEPTOM(n) n*WHEEL_CIRCUMFERENCE/STEPNB

#endif //MACROS_H

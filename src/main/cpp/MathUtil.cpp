#include "MathUtil.h"
#include <cmath>

// this function doesnt work just check the sign of AngleDifference
bool AngleDirection(double current, double target) {
    double diff = fmod((current - target), 360.0);
    return false;
}

double AngleDifference(double a, double b) {
    a = fmod(a, 360.0);
    b = fmod(b, 360.0);
    double diff = a - b;
    if (diff > 180) {
        diff = -(360 - diff);
    }
    if (diff < -180) {
        diff = -(-360 - diff);
    }
    return diff;
}
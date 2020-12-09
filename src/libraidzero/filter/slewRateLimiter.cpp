#include "libraidzero/filter/slewRateLimiter.hpp"

#include "pros/rtos.hpp"

#include <algorithm>

SlewRateLimiter::SlewRateLimiter(double irateLimit, double iinitialValue)
    : rateLimit{irateLimit}, previousTime{pros::millis()}, 
      previousValue{iinitialValue}
{}

double SlewRateLimiter::calculate(double input) {
    std::uint32_t currentTime = pros::millis();
    double elapsedTime = (currentTime - previousTime) / 1000.0;
    previousValue += std::clamp(
        input - previousValue, -rateLimit * elapsedTime, rateLimit * elapsedTime
    );
    previousTime = currentTime;
    return previousValue;
}

void SlewRateLimiter::reset(double value) {
    previousValue = value;
    previousTime = pros::millis();
}

#include "libraidzero/filter/slewRateLimiter.hpp"

#include "pros/rtos.hpp"

#include <algorithm>

SlewRateLimiter::SlewRateLimiter(double irateIncreaseLimit, double irateDecreaseLimit, double iinitialValue)
    : rateIncreaseLimit{irateIncreaseLimit}, 
      rateDecreaseLimit{
          irateDecreaseLimit == 0 ? 10000000 : irateDecreaseLimit
      }, 
      previousTime{pros::millis()}, previousValue{iinitialValue}
{}

void SlewRateLimiter::setLimits(double irateIncreaseLimit, double irateDecreaseLimit) {
    rateIncreaseLimit = irateIncreaseLimit;
    rateDecreaseLimit = irateDecreaseLimit == 0 ? 10000000 : irateDecreaseLimit;
}

double SlewRateLimiter::calculate(double input) {
    std::uint32_t currentTime = pros::millis();
    double elapsedTime = (currentTime - previousTime) / 1000.0;
    previousValue += std::clamp(
        input - previousValue, -rateDecreaseLimit * elapsedTime, rateIncreaseLimit * elapsedTime
    );
    previousTime = currentTime;
    return previousValue;
}

void SlewRateLimiter::reset(double value) {
    previousValue = value;
    previousTime = pros::millis();
}

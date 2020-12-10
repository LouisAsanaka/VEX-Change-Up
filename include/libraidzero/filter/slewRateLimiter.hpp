#pragma once

#include <cstdint>

class SlewRateLimiter {
public:
    /**
     * Constructs a SlewRateLimiter.
     * 
     * @param irateIncreaseLimit rate limit when increasing in units/s
     * @param irateDecreaseLimit rate limit when decreasing in units/s
     * @param iinitialValue starting value in units
     */
    SlewRateLimiter(double irateIncreaseLimit, double irateDecreaseLimit = 0.0, double iinitialValue = 0.0);
    double calculate(double input);
    void reset(double value);
private:
    double rateIncreaseLimit, rateDecreaseLimit;
    std::uint32_t previousTime;
    double previousValue;
};